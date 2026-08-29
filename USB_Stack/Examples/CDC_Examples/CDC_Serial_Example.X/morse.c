/**
 * @file morse.c
 * @brief Minimal speed-invariant Morse code decoder - proof of concept.
 * @author Roman Senn
 * @date 29/08/2026
 *
 * ============================================================================
 * ALGORITHM
 * ============================================================================
 *
 * International Morse timing is defined only in ratios, not absolute time:
 * a dash is 3 dots long, the gap between elements of one character is 1 dot,
 * the gap between characters is 3 dots, and the gap between words is 7 dots.
 * None of that says how long a dot itself is - that's set entirely by how
 * fast the operator is keying. So instead of a fixed WPM/threshold table,
 * this decoder keeps a running estimate of "dot_ms" (an exponential moving
 * average, updated every time a mark gets classified as a dot) and derives
 * every other threshold from it:
 *
 *   mark   <= 2 * dot_ms  -> dot,  else -> dash          (dot=1, dash=3)
 *   gap    <= 2 * dot_ms  -> still inside one character  (element gap=1)
 *   gap    <= 5 * dot_ms  -> character finished          (char gap=3)
 *   gap    >  5 * dot_ms  -> word finished (emit ' ')    (word gap=7)
 *
 * (2x sits midway between the 1x/3x mark ratio; 5x sits midway between the
 * 3x/7x gap ratio - the usual way to place a decision boundary between two
 * known-good reference lengths.) Because only dot-classified marks feed the
 * average, a run of dashes doesn't drag the estimate upward, and a change in
 * keying speed converges again within a few dots.
 *
 * Each finished character's dot/dash sequence is folded into one byte using
 * the standard "leading 1" trick: start the accumulator at 1, then for every
 * element shift left and OR in 0 for a dot or 1 for a dash. The leading 1
 * doubles as a length marker, so up to 6 elements fit in a uint8_t with no
 * separate length field, and the whole alphabet + digits become a flat
 * (code, character) lookup table.
 *
 * ============================================================================
 */

#include "morse.h"

/* Startup guess for dot length before any real keying has been seen. 80ms
 * is roughly 15 WPM (PARIS timing: dot_ms = 1200 / WPM). Clamped so a wildly
 * wrong first sample (or a stuck key) can't send the estimate to 0 or to
 * infinity. */
#define MORSE_DOT_MS_INIT 80u
#define MORSE_DOT_MS_MIN  20u  /* ~60 WPM ceiling */
#define MORSE_DOT_MS_MAX 500u  /* ~2.4 WPM floor  */

/* A key edge shorter than this is treated as contact bounce, not a real
 * element/gap, and is swallowed rather than fed to the decoder. This is a
 * fixed floor, deliberately NOT derived from dot_ms: without it, a single
 * bounce glitch (a few ms) gets classified as a very short dot, drags the
 * dot_ms average down towards MORSE_DOT_MS_MIN, and once collapsed every
 * following pause reads as a character/word gap - a runaway feedback loop
 * that turns one glitch into a stream of garbage single-element letters.
 * 12ms sits comfortably under any legitimate hand-keyed dot (even ~60 WPM
 * is ~20ms) while catching ordinary tactile-switch bounce. */
#define MORSE_GLITCH_MS 12u

/* EWMA smoothing: new = old + (sample - old) / MORSE_SMOOTH_DIV. A small
 * divisor tracks speed changes fast; a larger one rides through occasional
 * sloppy keying without wobbling the whole alphabet's thresholds. */
#define MORSE_SMOOTH_DIV 4

#define MORSE_OUTBUF_SIZE 16u

/* Elements accumulate into m_code via the leading-1 scheme above; 1 itself
 * means "no elements yet". Once 6 elements are in (m_code >= 0x40), further
 * elements before the character boundary are dropped rather than shifted
 * out - a 7-element pattern isn't valid Morse anyway, and the lookup below
 * will simply report '?' for whatever got captured. */
#define MORSE_CODE_EMPTY 1u
#define MORSE_CODE_FULL  0x40u

typedef enum
{
    MORSE_GAP_INTRA = 0, /* still inside one character   */
    MORSE_GAP_CHAR  = 1, /* character just finished      */
    MORSE_GAP_WORD  = 2  /* word just finished (adds ' ') */
} morse_gap_kind_t;

static uint16_t m_dot_ms;
static uint8_t  m_code;
static uint8_t  m_key_down;
static uint8_t  m_have_release;
static uint32_t m_mark_start_ms;
static uint32_t m_release_start_ms;
static uint8_t  m_gap_state; /* highest morse_gap_kind_t already acted on */

static char    m_outbuf[MORSE_OUTBUF_SIZE];
static uint8_t m_out_head;
static uint8_t m_out_tail;

/* Encoder (text -> LED) state - see the ENCODER section near the bottom of
 * this file for the functions that use it. */
#define MORSE_LED_QUEUE_SIZE 32u

typedef enum
{
    LED_PHASE_IDLE = 0, /* nothing playing; look at the queue          */
    LED_PHASE_ON,       /* lit for the current element's dot/dash time */
    LED_PHASE_GAP        /* dark, between elements/characters/words     */
} led_phase_t;

static uint8_t    m_led_queue[MORSE_LED_QUEUE_SIZE];
static uint8_t    m_led_q_head;
static uint8_t    m_led_q_tail;
static led_phase_t m_led_phase;
static uint8_t    m_led_code;      /* current character's code, 0 while idle/word-gap */
static uint8_t    m_led_nbits;     /* element count of m_led_code                       */
static uint8_t    m_led_next_elem; /* index of the next element still to play           */
static uint32_t   m_led_deadline_ms;
static uint8_t    m_led_on;

/** code, per the leading-1 scheme, for every letter and digit - derived by
 *  hand from the standard International Morse alphabet (not copied from any
 *  existing table): start at 1, shift-and-OR 0 per dot / 1 per dash. E.g.
 *  'A' = .-  ->  1 -> (dot)2 -> (dash)5. Sorted by code for readability;
 *  lookup is a linear scan since this only runs once per character. */
static const struct { uint8_t code; char ch; } MORSE_TABLE[] =
{
    { 2,'E'}, { 3,'T'},
    { 4,'I'}, { 5,'A'}, { 6,'N'}, { 7,'M'},
    { 8,'S'}, { 9,'U'}, {10,'R'}, {11,'W'}, {12,'D'}, {13,'K'}, {14,'G'}, {15,'O'},
    {16,'H'}, {17,'V'}, {18,'F'}, {20,'L'}, {22,'P'}, {23,'J'}, {24,'B'}, {25,'X'},
    {26,'C'}, {27,'Y'}, {28,'Z'}, {29,'Q'},
    {32,'5'}, {33,'4'}, {35,'3'}, {39,'2'}, {47,'1'},
    {48,'6'}, {56,'7'}, {60,'8'}, {62,'9'}, {63,'0'}
};
#define MORSE_TABLE_LEN (sizeof(MORSE_TABLE) / sizeof(MORSE_TABLE[0]))

static char morse_lookup(uint8_t code)
{
    uint8_t i;
    for(i = 0; i < MORSE_TABLE_LEN; i++)
    {
        if(MORSE_TABLE[i].code == code) return MORSE_TABLE[i].ch;
    }
    return '?';
}

/* Reverse of morse_lookup(): character -> leading-1 code, same table.
 * Returns 0 (never a valid code - the leading 1 alone is MORSE_CODE_EMPTY)
 * for anything that isn't a Morse-encodable letter or digit. */
static uint8_t morse_encode_lookup(char c)
{
    uint8_t i;
    if(c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A'); /* case-insensitive */
    for(i = 0; i < MORSE_TABLE_LEN; i++)
    {
        if(MORSE_TABLE[i].ch == c) return MORSE_TABLE[i].code;
    }
    return 0;
}

/* Position of the leading-1 marker bit = number of elements encoded below
 * it, e.g. 'A' = 5 = 0b101 -> 2 elements. */
static uint8_t code_element_count(uint8_t code)
{
    uint8_t n = 0;
    while(code > 1u)
    {
        code >>= 1;
        n++;
    }
    return n;
}

static void push_out(char c)
{
    uint8_t next = (uint8_t)((m_out_head + 1u) % MORSE_OUTBUF_SIZE);
    if(next != m_out_tail) /* queue full: drop silently, PoC-level */
    {
        m_outbuf[m_out_head] = c;
        m_out_head = next;
    }
}

static void finalize_char(void)
{
    if(m_code != MORSE_CODE_EMPTY)
    {
        push_out(morse_lookup(m_code));
        m_code = MORSE_CODE_EMPTY;
    }
}

static void append_element(uint8_t dash)
{
    if(m_code < MORSE_CODE_FULL)
    {
        m_code = (uint8_t)((m_code << 1) | (dash ? 1u : 0u));
    }
}

static void update_dot_estimate(uint32_t dot_sample_ms)
{
    int32_t diff = (int32_t)dot_sample_ms - (int32_t)m_dot_ms;
    int32_t next = (int32_t)m_dot_ms + diff / MORSE_SMOOTH_DIV;
    if(next < (int32_t)MORSE_DOT_MS_MIN) next = MORSE_DOT_MS_MIN;
    if(next > (int32_t)MORSE_DOT_MS_MAX) next = MORSE_DOT_MS_MAX;
    m_dot_ms = (uint16_t)next;
}

static uint8_t classify_mark_is_dash(uint32_t mark_ms)
{
    return mark_ms > ((uint32_t)m_dot_ms * 2u);
}

static morse_gap_kind_t classify_gap(uint32_t gap_ms)
{
    uint32_t d = m_dot_ms;
    if(gap_ms <= d * 2u) return MORSE_GAP_INTRA;
    if(gap_ms <= d * 5u) return MORSE_GAP_CHAR;
    return MORSE_GAP_WORD;
}

/* Shared by the key-down edge (final gap length, since the key just closed
 * it) and morse_poll() (a still-growing gap while the key stays up). Each
 * gap_state transition fires exactly once per idle period. */
static void note_gap(uint32_t gap_ms)
{
    morse_gap_kind_t kind = classify_gap(gap_ms);
    if(kind >= MORSE_GAP_CHAR && m_gap_state < MORSE_GAP_CHAR)
    {
        finalize_char();
        m_gap_state = MORSE_GAP_CHAR;
    }
    if(kind == MORSE_GAP_WORD && m_gap_state < MORSE_GAP_WORD)
    {
        push_out(' ');
        m_gap_state = MORSE_GAP_WORD;
    }
}

void morse_init(void)
{
    m_dot_ms           = MORSE_DOT_MS_INIT;
    m_code             = MORSE_CODE_EMPTY;
    m_key_down         = 0;
    m_have_release     = 0;
    m_mark_start_ms    = 0;
    m_release_start_ms = 0;
    m_gap_state        = MORSE_GAP_WORD; /* nothing pending at startup */
    m_out_head         = 0;
    m_out_tail         = 0;

    m_led_q_head      = 0;
    m_led_q_tail      = 0;
    m_led_phase       = LED_PHASE_IDLE;
    m_led_code        = 0;
    m_led_nbits       = 0;
    m_led_next_elem   = 0;
    m_led_deadline_ms = 0;
    m_led_on          = 0;
}

void morse_key_edge(uint8_t key_down, uint32_t now_ms)
{
    if(key_down)
    {
        if(m_have_release)
        {
            uint32_t gap_ms = now_ms - m_release_start_ms;
            if(gap_ms < MORSE_GLITCH_MS)
            {
                /* Too short to be a real release - the key never actually
                 * came up. Swallow it: cancel the release, restore the
                 * "still down" state, and keep the original mark running
                 * (m_mark_start_ms is untouched, so the eventual key-up
                 * still measures the whole press, glitch included). */
                m_have_release = 0;
                m_key_down     = 1;
                return;
            }
            note_gap(gap_ms);
        }
        m_have_release = 0;
        m_gap_state     = MORSE_GAP_INTRA; /* nothing resolved yet this gap */
        m_mark_start_ms = now_ms;
        m_key_down      = 1;
    }
    else
    {
        uint32_t mark_ms = now_ms - m_mark_start_ms;
        if(mark_ms < MORSE_GLITCH_MS)
        {
            /* Too short to be a real press - swallow it and keep treating
             * the key as still down (m_key_down/m_mark_start_ms untouched). */
            return;
        }

        {
            uint8_t dash = classify_mark_is_dash(mark_ms);
            if(!dash)
            {
                update_dot_estimate(mark_ms);
            }
            append_element(dash);
        }

        m_release_start_ms = now_ms;
        m_have_release      = 1;
        m_key_down          = 0;
    }
}

void morse_poll(uint32_t now_ms)
{
    if(m_have_release && !m_key_down)
    {
        note_gap(now_ms - m_release_start_ms);
    }
}

uint8_t morse_available(void)
{
    return m_out_head != m_out_tail;
}

char morse_getch(void)
{
    char c = 0;
    if(m_out_head != m_out_tail)
    {
        c = m_outbuf[m_out_tail];
        m_out_tail = (uint8_t)((m_out_tail + 1u) % MORSE_OUTBUF_SIZE);
    }
    return c;
}


/* ****************************************************************** */
/* ENCODER (text -> LED blinks)                                       */
/* ****************************************************************** */

static void led_play_element(uint32_t now_ms)
{
    uint8_t  bit   = (uint8_t)((m_led_code >> (m_led_nbits - 1u - m_led_next_elem)) & 1u);
    uint16_t units = bit ? 3u : 1u; /* dash = 3 dots, dot = 1 */
    m_led_on          = 1;
    m_led_phase        = LED_PHASE_ON;
    m_led_deadline_ms = now_ms + (uint32_t)m_dot_ms * units;
}

void morse_led_send(char c)
{
    uint8_t next = (uint8_t)((m_led_q_head + 1u) % MORSE_LED_QUEUE_SIZE);
    if(next != m_led_q_tail) /* queue full: drop silently, PoC-level */
    {
        m_led_queue[m_led_q_head] = (uint8_t)c;
        m_led_q_head = next;
    }
}

void morse_led_task(uint32_t now_ms)
{
    if(m_led_phase != LED_PHASE_IDLE)
    {
        if((int32_t)(now_ms - m_led_deadline_ms) < 0) return; /* still mid-phase */

        if(m_led_phase == LED_PHASE_ON)
        {
            m_led_on = 0;
            m_led_next_elem++;
            m_led_phase = LED_PHASE_GAP;
            /* 1 dot between elements of one character, 3 dots once the
             * character (its last element) is done - the standard
             * inter-character gap. */
            m_led_deadline_ms = now_ms + (uint32_t)m_dot_ms *
                                 (m_led_next_elem < m_led_nbits ? 1u : 3u);
            return;
        }

        /* LED_PHASE_GAP just elapsed. */
        if(m_led_next_elem < m_led_nbits)
        {
            led_play_element(now_ms);
            return;
        }
        m_led_phase = LED_PHASE_IDLE;
    }

    /* Idle: pull the next queued character and start it. */
    if(m_led_q_head != m_led_q_tail)
    {
        char c = (char)m_led_queue[m_led_q_tail];
        m_led_q_tail = (uint8_t)((m_led_q_tail + 1u) % MORSE_LED_QUEUE_SIZE);

        if(c == ' ')
        {
            /* Word gap: no element to light, just a silent pause. */
            m_led_code        = 0;
            m_led_nbits       = 0;
            m_led_next_elem   = 0;
            m_led_phase       = LED_PHASE_GAP;
            m_led_deadline_ms = now_ms + (uint32_t)m_dot_ms * 7u;
            return;
        }

        {
            uint8_t code = morse_encode_lookup(c);
            if(code == 0) return; /* not valid Morse: drop it, stay idle */
            m_led_code      = code;
            m_led_nbits     = code_element_count(code);
            m_led_next_elem = 0;
            led_play_element(now_ms);
        }
    }
}

uint8_t morse_led_is_on(void)
{
    return m_led_on;
}

uint16_t morse_get_dot_ms(void)
{
    return m_dot_ms;
}
