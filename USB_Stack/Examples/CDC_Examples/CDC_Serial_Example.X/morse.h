/**
 * @file morse.h
 * @brief Minimal speed-invariant Morse code decoder - proof of concept.
 * @author Roman Senn
 * @date 29/08/2026
 *
 * Feed it raw key-down/key-up edges with a millisecond timestamp; it has no
 * idea what WPM it is being sent at. Instead it tracks a moving average of
 * how long a "short" key-down (a dot) actually lasted, and uses that
 * estimate - not a fixed timing table - to tell a dot from a dash and an
 * intra-character gap from an inter-character or inter-word gap. A change
 * in keying speed is absorbed within a handful of characters rather than
 * requiring the operator to match one fixed WPM.
 *
 * Pure logic, no hardware access: it doesn't read a pin or a timer itself,
 * so it's easy to reason about and to reuse against any board/timebase.
 */
#ifndef MORSE_H
#define MORSE_H

#include <stdint.h>

/** Reset the decoder to its startup state (default dot-length guess, empty
 *  output queue). */
void morse_init(void);

/** Call the instant the key changes state.
 *  key_down: 1 the instant the key goes down, 0 the instant it goes up.
 *  now_ms:   free-running millisecond timestamp of the edge. */
void morse_key_edge(uint8_t key_down, uint32_t now_ms);

/** Call periodically (at least every few ms) while the key is up, so a
 *  trailing character/word gets flushed even when there is no following
 *  key-down edge to trigger it. Safe to call at any time, key up or down. */
void morse_poll(uint32_t now_ms);

/** Non-zero if a decoded character is waiting in the output queue. */
uint8_t morse_available(void);

/** Pop the next decoded character (letter, digit, or ' ' for a word gap).
 *  Returns 0 if the queue is empty. Decoded but unrecognised element
 *  patterns come out as '?'. */
char morse_getch(void);


/* ****************************************************************** */
/* ENCODER (text -> LED blinks): the reverse direction. Queue characters
 * and pump the playback state machine; it plays each one back as
 * dot/dash timing at the *same* adaptively-learned speed morse_key_edge()
 * above is tracking, so keying in Morse and reading it blinked back both
 * happen at the operator's own pace.
 *
 * This module never touches a pin itself (see the file header note above)
 * - it only reports the LED level it currently wants via morse_led_is_on();
 * the caller is responsible for driving the actual GPIO from that. */
/* ****************************************************************** */

/** Queue one character for LED playback. Letters and digits become their
 *  dot/dash pattern; ' ' becomes a silent inter-word pause; anything else
 *  (punctuation, control characters, ...) is not valid Morse and is
 *  silently dropped - only valid Morse characters ever reach the LED. */
void morse_led_send(char c);

/** Call periodically (at least every few ms) to advance the LED playback
 *  state machine. */
void morse_led_task(uint32_t now_ms);

/** Non-zero if the LED should be lit right now. */
uint8_t morse_led_is_on(void);

/** Current adaptive dot length, in milliseconds - the same estimate
 *  morse_key_edge() maintains from the operator's own keying. */
uint16_t morse_get_dot_ms(void);

#endif
