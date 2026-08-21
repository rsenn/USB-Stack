/**
 * @file MIDI_Synth.c
 * @brief USB MIDI 4-voice synth: notes and control changes together (single TU).
 * @author Roman Senn
 * @date 20/08/2026
 *
 * USB uC - USB MIDI Synth Example.
 * Copyright (C) 2017-2026  John Izzard
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
============================================================================
WHAT THIS EXAMPLE IS
============================================================================

A class-compliant USB MIDI 1.0 sound module: the host sends Note On/Off
AND Control Change down the same bulk OUT endpoint, and both act on the
running voices at the same time - hold a chord and sweep a CC and you hear
it change under your fingers.

Sound engine: 4-voice polyphonic pulse wave, 8 kHz sample rate, 8-bit PWM
on CCP1. Each voice is a 16-bit phase accumulator; the CCs steer pulse
width, master volume, vibrato depth and pitch bend.

  Note On/Off       voice allocation, oldest-voice stealing when full
  Pitch Bend        +/- 2 semitones on every sounding voice
  CC 1  Mod Wheel   vibrato depth (LFO on pitch)
  CC 7  Volume      master volume
  CC 64 Sustain     hold notes after Note Off
  CC 74 Brightness  pulse width (thin and reedy -> square)
  CC 120 / 123      All Sound Off / All Notes Off

As with the controller example there is no usb_midi.c class driver: MIDI
1.0 has no class requests, so the class lives here on top of the core
endpoint API.

Endpoint map (EP1, bulk, 64 bytes, PINGPONG_0_OUT):
  EP1 OUT - host -> device only. A sound module has nothing to send back,
            so unlike MIDI_Controller.c this device declares a single
            endpoint and a single embedded jack.

Interrupt layout: the audio timer runs at high priority and USB at low
priority, so a long control transfer cannot stretch a sample period and
put a click in the output.

============================================================================
BUILDING
============================================================================

This file is one translation unit; it replaces main.c, usb_app.c and
usb_descriptors.c. Compile it together with the stack core (usb.c only)
and define MIDI_SYNTH_EXAMPLE so that usb_config.h picks the MIDI
endpoint/interface counts.

---- VERIFIED TOOLCHAINS --------------------------------------------------

Built for PIC18F25K50, PIC18F14K50 and PIC18F47J53 with each of:

  XC8  v2.31, v2.36, v4.00      pass
  SDCC 4.0.0, 4.1.0, 4.3.0rc1   pass
  XC8  v1.43, v1.45             cannot work, see below

All six working toolchains emit identical configuration words for all three
parts (XC8 additionally writes 0xFF to two unimplemented config locations).

XC8 v1.4x is out of reach and always was: its __at() accepts only a literal
address - even __at(0x400+4) is rejected - while every buffer address in
this stack is computed from EP0_SIZE and PINGPONG_MODE. The stack's own
HID_Mouse_Example fails on v1 in exactly the same way, so this is not
something the MIDI examples introduced.

---- XC8 (v2.x / v4.x, C90 mode) -----------------------------------------

  xc8-cc -mcpu=18F25K50 -std=c90 -O2 \
         -DMIDI_SYNTH_EXAMPLE \
         -I../../USB -I../../Hardware \
         -o MIDI_Synth.hex \
         MIDI_Synth.c ../../USB/usb.c

If you use the USB uC bootloader, add -mcodeoffset=0x2000.

XC8 2.4x and later no longer ship PIC18F25K50 support in the compiler; it
lives in the PIC18F-K device family pack. If you get

  ::: error: (2103) no device-support files specified; use the -mdfp option

point -mdfp at the pack's xc8 subdirectory (not the pack root):

  -mdfp=/opt/microchip/mplabx/v5.35/packs/Microchip/PIC18F-K_DFP/1.3.84/xc8

Packs downloaded from packs.download.microchip.com put that xc8 directory
at the pack root, so there -mdfp=<pack dir> is enough.

---- SDCC (pic16 port; tested with 4.0.0, 4.1.0 and 4.3.0rc1) -------------

SDCC compiles one source per invocation, so this is compile-each-then-link:

  sdcc -mpic16 -p18f25k50 --use-non-free -c -DMIDI_SYNTH_EXAMPLE \
       -I../../USB -I../../Hardware -o MIDI_Synth.o MIDI_Synth.c

  sdcc -mpic16 -p18f25k50 --use-non-free -c -DMIDI_SYNTH_EXAMPLE \
       -I../../USB -I../../Hardware -o usb.o ../../USB/usb.c

  sdcc -mpic16 -p18f25k50 --use-non-free -o MIDI_Synth.hex MIDI_Synth.o usb.o

The Makefile in this directory does all of the above:

  make COMPILER=sdcc DEVICE=18F25K50

gplink prints a few "Relocation symbol _cinit has no section (pass 0)"
warnings during the link. They resolve on the next pass and the hex is
produced normally.

Portability between the two compilers is handled by USB/usb_compiler.h -
register header, part macros, absolute-address placement, ISR syntax,
NOP() and the __delay_*() builtins. Nothing compiler specific is left in
this file. The XC8 output is unchanged by any of it, byte for byte.

Verified on 18F25K50, 18F14K50 and 18F47J53: both compilers emit identical
configuration words for all three.

============================================================================
HARDWARE (PIC18F2xK50 / PIC18F4xK50 defaults)
============================================================================

  RC2 / CCP1   PWM audio out (RC5 on the 20-pin PIC18F1xK50 parts). Feed
               it through an RC low-pass (e.g. 1k + 10nF, corner ~16 kHz)
               into an amplifier, or into headphones via a series resistor
               and a DC blocking capacitor. The PWM carrier is 187.5 kHz,
               well above the 4 kHz audio band.
  LED          from Hardware/config.h - lit while any voice is sounding.
*/

/* ************************************************************************** */
/* ************************** COMPILER SHIM ********************************* */
/* ************************************************************************** */

#include <stdint.h>
#include <stdbool.h>

/* Register header, part-macro mapping, __at() placement, ISR syntax and the
 * __delay_*() builtins all come from here - see USB/usb_compiler.h. */
#include "usb_compiler.h"

#include "fuses.h"   /* #pragma config words - works with both compilers */

#include "config.h"
#include "usb.h"
#include "usb_app.h"
#include "usb_ch9.h"

#if defined(_PIC14E)
#error "The MIDI examples need a 64-byte bulk buffer and PWM; target a PIC18 part."
#endif

/* ************************************************************************** */


/* ************************************************************************** */
/* ************************ USB MIDI 1.0 CONSTANTS ************************** */
/* ************************************************************************** */

/* Interface class/subclass codes (Audio Device Class 1.0). */
#define AUDIO_CLASS            0x01
#define SUBCLASS_AUDIOCONTROL  0x01
#define SUBCLASS_MIDISTREAMING 0x03

/* Class-specific descriptor types. */
#define CS_INTERFACE_DESC 0x24
#define CS_ENDPOINT_DESC  0x25

/* Class-specific descriptor subtypes. */
#define MS_HEADER     0x01
#define MIDI_IN_JACK  0x02
#define MIDI_OUT_JACK 0x03
#define MS_GENERAL    0x01

/* Jack types. */
#define JACK_EMBEDDED 0x01
#define JACK_EXTERNAL 0x02

/* Code Index Numbers (byte 0, low nibble). */
#define CIN_NOTE_OFF       0x08
#define CIN_NOTE_ON        0x09
#define CIN_CONTROL_CHANGE 0x0B
#define CIN_PROGRAM_CHANGE 0x0C
#define CIN_PITCH_BEND     0x0E

/* Controller numbers we act on. */
#define CC_MOD_WHEEL   1
#define CC_VOLUME      7
#define CC_SUSTAIN     64
#define CC_BRIGHTNESS  74
#define CC_ALL_SOUND_OFF 120
#define CC_ALL_NOTES_OFF 123

/* ************************************************************************** */


/* ************************************************************************** */
/* ************************* ENDPOINT HAL *********************************** */
/* ************************************************************************** */

#define MIDI_EP      EP1
#define MIDI_EP_SIZE EP1_SIZE
#define MIDI_UEPbits UEP1bits
#define MIDI_BD_OUT  BD1_OUT

#if PINGPONG_MODE != PINGPONG_0_OUT
#error "MIDI_Synth.c assumes PINGPONG_0_OUT (the usb_config.h default)."
#endif

/* EP0 uses three buffers in PINGPONG_0_OUT (out even, out odd, in). */
#define MIDI_EP_BUFFERS_STARTING_ADDR (EP_BUFFERS_STARTING_ADDR + (EP0_SIZE * 3))
#define MIDI_EP_OUT_BUFFER_BASE_ADDR   MIDI_EP_BUFFERS_STARTING_ADDR

#define MIDI_EP_OUT_DATA_TOGGLE_VAL g_usb_ep_stat[MIDI_EP][OUT].Data_Toggle_Val

/* Under SDCC the endpoint buffers are pointer macros into the USB RAM window
 * that usb.c reserves in one block - see USB/usb_compiler.h. */
#if USB_SDCC
#define g_midi_ep_out USB_ABS_ARR(uint8_t, MIDI_EP_OUT_BUFFER_BASE_ADDR)
#else
uint8_t g_midi_ep_out[MIDI_EP_SIZE] __at(MIDI_EP_OUT_BUFFER_BASE_ADDR);
#endif

/* USB interrupt priority bit - usb_hal.h gives us the enable/flag only. */
#if defined(_18F24K50) || defined(_18F25K50) || defined(_18F45K50)
#define USB_INTERRUPT_PRIORITY IPR3bits.USBIP
#else
#define USB_INTERRUPT_PRIORITY IPR2bits.USBIP
#endif

/* ************************************************************************** */


/* ************************************************************************** */
/* ***************************** DESCRIPTORS ******************************** */
/* ************************************************************************** */

/** Device Descriptor */
const ch9_device_descriptor_t g_device_descriptor =
{
    0x12,        // bLength
    DEVICE_DESC, // bDescriptorType
    0x0200,      // bcdUSB (2.0)
    0x00,        // bDeviceClass - defined at interface level
    0x00,        // bDeviceSubClass
    0x00,        // bDeviceProtocol
    EP0_SIZE,    // bMaxPacketSize0
    0x04D8,      // idVendor - Microchip
    0x00DE,      // idProduct
    0x0001,      // bcdDevice
    0x01,        // iManufacturer
    0x02,        // iProduct
    0x00,        // iSerialNumber
    0x01         // bNumConfigurations
};

/*
 * Flat byte array for the same reason as in MIDI_Controller.c: the MIDI
 * class descriptors are variable length and the length arithmetic is easier
 * to check when it is written out.
 *
 * Jack topology - receive only:
 *
 *   EP1 OUT --> [emb IN jack 1] --> [ext OUT jack 2]
 *
 * "External OUT jack 2" is the sound engine as far as the host is concerned.
 */

#define MS_CS_TOTAL_LEN  27u /* 7 + 6 + 9 + 5 */
#define CONFIG_TOTAL_LEN 72u /* 9 + 9 + 9 + 9 + MS_CS_TOTAL_LEN + 9 */

static const uint8_t config_descriptor0[CONFIG_TOTAL_LEN] =
{
    /* ---- Configuration Descriptor ------------------------------------- */
    9,                          // bLength
    CONFIGURATION_DESC,         // bDescriptorType
    (uint8_t)CONFIG_TOTAL_LEN,  // wTotalLength LSB
    0x00,                       // wTotalLength MSB
    0x02,                       // bNumInterfaces - AudioControl + MIDIStreaming
    0x01,                       // bConfigurationValue
    0x00,                       // iConfiguration
    0x80,                       // bmAttributes - bus powered, no remote wakeup
    50,                         // bMaxPower - 100 mA

    /* ---- Standard AudioControl Interface ------------------------------ */
    9,                          // bLength
    INTERFACE_DESC,             // bDescriptorType
    0x00,                       // bInterfaceNumber
    0x00,                       // bAlternateSetting
    0x00,                       // bNumEndpoints
    AUDIO_CLASS,                // bInterfaceClass
    SUBCLASS_AUDIOCONTROL,      // bInterfaceSubClass
    0x00,                       // bInterfaceProtocol
    0x00,                       // iInterface

    /* ---- Class-Specific AudioControl Header --------------------------- */
    9,                          // bLength
    CS_INTERFACE_DESC,          // bDescriptorType
    MS_HEADER,                  // bDescriptorSubtype - HEADER
    0x00, 0x01,                 // bcdADC (1.00)
    9, 0x00,                    // wTotalLength - just this header
    0x01,                       // bInCollection
    0x01,                       // baInterfaceNr(1) - interface 1

    /* ---- Standard MIDIStreaming Interface ----------------------------- */
    9,                          // bLength
    INTERFACE_DESC,             // bDescriptorType
    0x01,                       // bInterfaceNumber
    0x00,                       // bAlternateSetting
    0x01,                       // bNumEndpoints - bulk OUT only
    AUDIO_CLASS,                // bInterfaceClass
    SUBCLASS_MIDISTREAMING,     // bInterfaceSubClass
    0x00,                       // bInterfaceProtocol
    0x00,                       // iInterface

    /* ---- Class-Specific MIDIStreaming Header -------------------------- */
    7,                          // bLength
    CS_INTERFACE_DESC,          // bDescriptorType
    MS_HEADER,                  // bDescriptorSubtype
    0x00, 0x01,                 // bcdMSC (1.00)
    (uint8_t)MS_CS_TOTAL_LEN,   // wTotalLength LSB
    0x00,                       // wTotalLength MSB

    /* ---- MIDI IN Jack, Embedded (ID 1) -------------------------------- */
    6,                          // bLength
    CS_INTERFACE_DESC,          // bDescriptorType
    MIDI_IN_JACK,               // bDescriptorSubtype
    JACK_EMBEDDED,              // bJackType
    0x01,                       // bJackID
    0x00,                       // iJack

    /* ---- MIDI OUT Jack, External (ID 2) ------------------------------- */
    /* The sound engine. Fed by embedded IN jack 1.                        */
    9,                          // bLength
    CS_INTERFACE_DESC,          // bDescriptorType
    MIDI_OUT_JACK,              // bDescriptorSubtype
    JACK_EXTERNAL,              // bJackType
    0x02,                       // bJackID
    0x01,                       // bNrInputPins
    0x01,                       // baSourceID(1) - embedded IN jack 1
    0x01,                       // baSourcePin(1)
    0x00,                       // iJack

    /* ---- Standard Bulk OUT Endpoint ----------------------------------- */
    /* Audio class endpoints are 9 bytes, not the usual 7.                 */
    9,                          // bLength
    ENDPOINT_DESC,              // bDescriptorType
    0x01,                       // bEndpointAddress - EP1 OUT
    0x02,                       // bmAttributes - bulk
    MIDI_EP_SIZE, 0x00,         // wMaxPacketSize
    0x00,                       // bInterval - ignored for bulk
    0x00,                       // bRefresh
    0x00,                       // bSynchAddress

    /* ---- Class-Specific Bulk OUT Endpoint ----------------------------- */
    5,                          // bLength
    CS_ENDPOINT_DESC,           // bDescriptorType
    MS_GENERAL,                 // bDescriptorSubtype
    0x01,                       // bNumEmbMIDIJack
    0x01                        // baAssocJackID(1) - embedded IN jack 1
};

/** Configuration Descriptor Addresses Array */
const usb_desc_addr_t g_config_descriptors[] =
{
    USB_DESC_ADDR(config_descriptor0)
};

/** String Descriptors */
typedef struct
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wLANGID[1];
}string_zero_descriptor_t;

typedef struct
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bString[6];
}vendor_string_descriptor_t;

typedef struct
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bString[10];
}product_string_descriptor_t;

static const string_zero_descriptor_t string_zero_descriptor =
{
    sizeof(string_zero_descriptor_t),
    STRING_DESC,
    {0x0409}
};

static const vendor_string_descriptor_t vendor_string_descriptor =
{
    sizeof(vendor_string_descriptor_t),
    STRING_DESC,
    {'J','o','h','n','n','y'}
};

static const product_string_descriptor_t product_string_descriptor =
{
    sizeof(product_string_descriptor_t),
    STRING_DESC,
    {'M','I','D','I',' ','S','y','n','t','h'}
};

const usb_desc_addr_t g_string_descriptors[] =
{
    USB_DESC_ADDR(&string_zero_descriptor),
    USB_DESC_ADDR(&vendor_string_descriptor),
    USB_DESC_ADDR(&product_string_descriptor)
};

/* Number of string descriptors; usb.c range-checks bDescriptorIndex on it. */
const uint8_t g_size_of_sd = sizeof(g_string_descriptors) / sizeof(g_string_descriptors[0]);

/* ************************************************************************** */


/* ************************************************************************** */
/* ****************************** SOUND ENGINE ****************************** */
/* ************************************************************************** */

#define NUM_VOICES   4
#define SAMPLE_RATE  8000u

/* Timer0 runs from Fosc/4 = 12 MHz with no prescaler. */
#define TIMER0_RELOAD (65536u - (12000000u / SAMPLE_RATE)) /* 8 kHz */

/* Peak amplitude of one voice. Velocity (1..127) times volume (0..127) is
 * at most 16129, and >> 9 turns that into 0..31 with a single shift - which
 * is why master volume is folded into the voice amplitude here instead of
 * multiplying the mix in the audio interrupt. Four voices at 31 sum to
 * +/-124, just inside the PWM swing. */
#define VOICE_AMP_MAX 31

/*
 * Phase increments for MIDI notes 96..107 (C7..B7) at 8 kHz:
 *   inc = f * 65536 / SAMPLE_RATE
 * B7 is 3951 Hz, just under Nyquist, so this is the highest octave that
 * still fits in a 16-bit increment. Lower octaves are the same value shifted
 * right, which is exact - octaves are powers of two.
 */
static const uint16_t m_note_inc[12] =
{
    17146, /* C7  */
    18165, /* C#7 */
    19246, /* D7  */
    20390, /* D#7 */
    21602, /* E7  */
    22887, /* F7  */
    24248, /* F#7 */
    25690, /* G7  */
    27217, /* G#7 */
    28836, /* A7  */
    30551, /* A#7 */
    32367  /* B7  */
};

typedef struct
{
    uint16_t phase;      /* 16-bit phase accumulator                       */
    uint16_t inc;        /* phase increment actually used by the ISR       */
    uint16_t base_inc;   /* increment before pitch bend / vibrato          */
    uint8_t  note;       /* MIDI note number, 0xFF when the voice is free  */
    uint8_t  velocity;   /* as received, so CC 7 can rescale the voice     */
    uint8_t  amp;        /* current amplitude, ramps toward target_amp     */
    uint8_t  target_amp; /* 0 when releasing                               */
    uint8_t  age;        /* for oldest-voice stealing                      */
    bool     held;       /* key still down (vs. held by the sustain pedal) */
}voice_t;

static volatile voice_t m_voice[NUM_VOICES];

/* Controller state. Read by the ISR, written by the MIDI handler. */
static volatile uint8_t  m_pulse_width = 0x80; /* CC 74, 0x80 = square      */
static volatile uint8_t  m_volume      = 100;  /* CC 7,  0..127             */
static          uint8_t  m_mod_wheel   = 0;    /* CC 1,  vibrato depth      */
static          uint8_t  m_sustain     = 0;    /* CC 64                     */
static          int16_t  m_bend        = 0;    /* pitch bend, -8192..8191   */

/* Set by the audio ISR every 64 samples (125 Hz) to pace the LFO. */
static volatile bool m_lfo_tick = false;

static uint8_t m_age_counter = 0;
static uint8_t m_lfo_phase   = 0;

/**
 * Phase increment for a MIDI note.
 *
 * Octaves are exact power-of-two shifts of the top-octave table, so this is
 * a table lookup and a shift - no division, no floating point.
 */
/**
 * Voice amplitude for a velocity at the current master volume.
 *
 * Folding master volume in here rather than in the audio interrupt keeps the
 * interrupt free of multiplies: it is a shift, and it only runs when a note
 * starts or CC 7 moves.
 */
static uint8_t amp_from_velocity(uint8_t velocity)
{
    return (uint8_t)(((uint16_t)velocity * (uint16_t)m_volume) >> 9);
}

static uint16_t note_to_inc(uint8_t note)
{
    uint8_t octave;
    uint8_t semitone;

    if(note > 107) note = 107; /* above B7 we would alias past Nyquist */

    octave   = (uint8_t)(note / 12u);
    semitone = (uint8_t)(note % 12u);

    /* Table is octave 8 (notes 96..107). */
    return (uint16_t)(m_note_inc[semitone] >> (8u - octave));
}

/**
 * Apply pitch bend and vibrato to a voice's base increment.
 *
 * Deliberately runs in the main loop, not the audio ISR: it needs a 32-bit
 * multiply, and the ISR has a 1500 instruction budget per sample that is
 * better spent on the oscillators.
 *
 * Both modulations are linear approximations of the exponential pitch law.
 * Over the +/-2 semitones of pitch bend the error stays under 1 cent at the
 * centre and a few cents at the extremes - inaudible for a square wave.
 */
static void voice_apply_modulation(uint8_t v, int16_t mod)
{
    int32_t delta;

    /* mod is in 1/8192ths of the +/-12.5% span (2 semitones). */
    delta = (int32_t)m_voice[v].base_inc * (int32_t)mod;
    delta = delta / 65536; /* /8192 for the unit, /8 for the 12.5% span */

    m_voice[v].inc = (uint16_t)((int32_t)m_voice[v].base_inc + delta);
}

static void voice_start(uint8_t v, uint8_t note, uint8_t velocity)
{
    m_voice[v].note       = note;
    m_voice[v].base_inc   = note_to_inc(note);
    m_voice[v].inc        = m_voice[v].base_inc;
    m_voice[v].phase      = 0;
    m_voice[v].held       = true;
    m_voice[v].age        = m_age_counter++;
    m_voice[v].velocity   = velocity;
    m_voice[v].target_amp = amp_from_velocity(velocity);
}

static void voice_release(uint8_t v)
{
    m_voice[v].held = false;

    /* The pedal keeps the voice sounding until it is lifted. */
    if(!m_sustain) m_voice[v].target_amp = 0;
}

/**
 * Pick a voice for a new note: reuse the same note if it is already
 * sounding, else take a silent voice, else steal the oldest.
 */
static uint8_t voice_allocate(uint8_t note)
{
    uint8_t v;
    uint8_t oldest       = 0;
    uint8_t oldest_age   = 0;
    uint8_t age_now      = m_age_counter;

    for(v = 0; v < NUM_VOICES; v++)
    {
        if(m_voice[v].note == note) return v;
    }

    for(v = 0; v < NUM_VOICES; v++)
    {
        if(m_voice[v].amp == 0 && m_voice[v].target_amp == 0) return v;
    }

    for(v = 0; v < NUM_VOICES; v++)
    {
        uint8_t age = (uint8_t)(age_now - m_voice[v].age);
        if(age >= oldest_age)
        {
            oldest_age = age;
            oldest     = v;
        }
    }

    return oldest;
}

static void voices_all_off(bool immediate)
{
    uint8_t v;

    m_sustain = 0;

    for(v = 0; v < NUM_VOICES; v++)
    {
        m_voice[v].held       = false;
        m_voice[v].target_amp = 0;
        m_voice[v].note       = 0xFF;
        m_voice[v].velocity   = 0;
        if(immediate) m_voice[v].amp = 0;
    }
}

/* ************************************************************************** */


/* ************************************************************************** */
/* **************************** MIDI INTERPRETER **************************** */
/* ************************************************************************** */

static void handle_note_on(uint8_t note, uint8_t velocity)
{
    uint8_t v;

    /* Note On with velocity 0 is a Note Off - every host does this. */
    if(velocity == 0)
    {
        for(v = 0; v < NUM_VOICES; v++)
        {
            if(m_voice[v].note == note) voice_release(v);
        }
        return;
    }

    v = voice_allocate(note);
    voice_start(v, note, velocity);
}

static void handle_note_off(uint8_t note)
{
    uint8_t v;

    for(v = 0; v < NUM_VOICES; v++)
    {
        if(m_voice[v].note == note) voice_release(v);
    }
}

static void handle_control_change(uint8_t controller, uint8_t value)
{
    uint8_t v;

    switch(controller)
    {
        case CC_MOD_WHEEL:
            m_mod_wheel = value;
            break;

        case CC_VOLUME:
            m_volume = value;
            /* Rescale the sounding voices. They ramp to the new level rather
             * than jumping, so a volume sweep is smooth. */
            for(v = 0; v < NUM_VOICES; v++)
            {
                if(m_voice[v].target_amp) m_voice[v].target_amp = amp_from_velocity(m_voice[v].velocity);
            }
            break;

        case CC_BRIGHTNESS:
            /* 0..127 -> a pulse width from very thin to square. Below about
             * 8 the pulse gets short enough to disappear at 8 kHz, so the
             * range is clamped. */
            if(value < 8) value = 8;
            m_pulse_width = (uint8_t)(value << 1);
            break;

        case CC_SUSTAIN:
            m_sustain = (value >= 64u) ? 1u : 0u;
            if(!m_sustain)
            {
                /* Pedal lifted: release everything no longer held down. */
                for(v = 0; v < NUM_VOICES; v++)
                {
                    if(!m_voice[v].held) m_voice[v].target_amp = 0;
                }
            }
            break;

        case CC_ALL_SOUND_OFF:
            voices_all_off(true);
            break;

        case CC_ALL_NOTES_OFF:
            voices_all_off(false);
            break;

        default:
            break;
    }
}

/**
 * Handle one 4-byte USB-MIDI event packet.
 *
 * Notes and CCs arrive interleaved in the same packet burst and are applied
 * in order, so a CC sandwiched between two Note Ons takes effect on the
 * second one - which is what makes "notes and control changes at the same
 * time" actually sound right.
 */
static void midi_handle_event(const uint8_t* p_event)
{
    uint8_t cin   = (uint8_t)(p_event[0] & 0x0F);
    uint8_t data1 = p_event[2];
    uint8_t data2 = p_event[3];

    switch(cin)
    {
        case CIN_NOTE_ON:
            handle_note_on(data1, data2);
            break;

        case CIN_NOTE_OFF:
            handle_note_off(data1);
            break;

        case CIN_CONTROL_CHANGE:
            handle_control_change(data1, data2);
            break;

        case CIN_PITCH_BEND:
            /* 14-bit, LSB first, centre 0x2000. */
            m_bend = (int16_t)((((uint16_t)data2 << 7) | data1) - 8192);
            break;

        case CIN_PROGRAM_CHANGE:
        default:
            break;
    }
}

/* ************************************************************************** */


/* ************************************************************************** */
/* *************************** ENDPOINT SERVICE ***************************** */
/* ************************************************************************** */

static void midi_arm_ep_out(void)
{
    if(MIDI_EP_OUT_DATA_TOGGLE_VAL) g_usb_bd_table[MIDI_BD_OUT].STAT = _DTSEN | _DTS;
    else                            g_usb_bd_table[MIDI_BD_OUT].STAT = _DTSEN;
    g_usb_bd_table[MIDI_BD_OUT].CNT   = MIDI_EP_SIZE;
    g_usb_bd_table[MIDI_BD_OUT].STAT |= _UOWN;
}

static void midi_ep_out_tasks(void)
{
    uint8_t received = g_usb_bd_table[MIDI_BD_OUT].CNT;
    uint8_t i;

    MIDI_EP_OUT_DATA_TOGGLE_VAL ^= 1;

    /* USB-MIDI packets are always 4 bytes; ignore any ragged tail. */
    for(i = 0; (uint8_t)(i + 4u) <= received; i = (uint8_t)(i + 4u))
    {
        midi_handle_event(&g_midi_ep_out[i]);
    }

    midi_arm_ep_out();
}

/* ************************************************************************** */


/* ************************************************************************** */
/* *********************** USB APPLICATION CALLBACKS ************************ */
/* ************************************************************************** */

bool usb_service_class_request(void)
{
    /* MIDI 1.0 defines no class requests. Anything that arrives is bogus. */
    return false;
}

bool usb_get_class_descriptor(void)
{
    /* MIDI class descriptors only ever appear inside the configuration
     * descriptor, never fetched on their own. */
    return false;
}

void usb_app_init(void)
{
    usb_ram_set(0, g_midi_ep_out, MIDI_EP_SIZE);

    g_usb_bd_table[MIDI_BD_OUT].STAT = 0;
    g_usb_bd_table[MIDI_BD_OUT].ADR  = MIDI_EP_OUT_BUFFER_BASE_ADDR;

    MIDI_UEPbits.EPHSHK   = 1; // Handshaking enabled
    MIDI_UEPbits.EPCONDIS = 1; // Data only, no SETUP on this endpoint
    MIDI_UEPbits.EPOUTEN  = 1;
    MIDI_UEPbits.EPINEN   = 0; // Receive only

    g_usb_ep_stat[MIDI_EP][OUT].Halt = 0;
    MIDI_EP_OUT_DATA_TOGGLE_VAL = 0;

    /* A re-enumeration should not leave notes hanging. */
    voices_all_off(true);

    midi_arm_ep_out();
}

void usb_app_tasks(void)
{
    if(TRANSACTION_EP != MIDI_EP) return;
    if(TRANSACTION_DIR != OUT) return;

    midi_ep_out_tasks();
}

void usb_app_clear_halt(uint8_t bd_table_index, uint8_t ep, uint8_t dir)
{
    g_usb_ep_stat[ep][dir].Data_Toggle_Val = 0;

    if(g_usb_ep_stat[ep][dir].Halt)
    {
        g_usb_ep_stat[ep][dir].Halt         = 0;
        g_usb_bd_table[bd_table_index].STAT = 0;
    }

    if(ep == MIDI_EP && dir == OUT) midi_arm_ep_out();
}

bool usb_app_set_interface(uint8_t alternate_setting, uint8_t interface)
{
    /* Both interfaces have only alternate setting 0. */
    if(alternate_setting != 0) return false;
    if(interface >= NUM_INTERFACES) return false;

    MIDI_EP_OUT_DATA_TOGGLE_VAL = 0;
    return true;
}

bool usb_app_get_interface(uint8_t* alternate_setting_result, uint8_t interface)
{
    if(interface >= NUM_INTERFACES) return false;

    *alternate_setting_result = 0;
    return true;
}

bool usb_out_control_finished(void)
{
    return false;
}

/* ************************************************************************** */


/* ************************************************************************** */
/* ******************************** AUDIO *********************************** */
/* ************************************************************************** */

/**
 * Write an 8-bit sample to the CCP1 PWM duty cycle.
 *
 * PR2 = 63 with a 1:1 prescaler gives a 187.5 kHz carrier and exactly 8 bits
 * of duty resolution: the top 6 bits live in CCPR1L and the bottom 2 in
 * DC1B.
 */
#define PWM_WRITE(sample) do{                                \
        CCPR1L      = (uint8_t)((sample) >> 2);              \
        CCP1CONbits.DC1B = (uint8_t)((sample) & 0x03);       \
    }while(0)

/**
 * Audio interrupt: one sample, 8000 times a second.
 *
 * Runs at high priority so that USB - which can sit in a control transfer
 * for a while - cannot delay a sample and put a click in the output.
 * Everything here is 16-bit adds and compares; the amplitude ramp and the
 * volume multiply are the only arithmetic beyond that.
 */
static void audio_sample(void)
{
    static uint8_t amp_divider = 0;
    static uint8_t lfo_divider = 0;

    int16_t mix = 0;
    uint8_t v;
    uint8_t ramp;

    /* Ramp amplitudes every 8th sample: 1 kHz, so a voice fades in or out
     * over ~32 ms. Instant gating would click. */
    amp_divider++;
    ramp = ((amp_divider & 0x07u) == 0u) ? 1u : 0u;

    for(v = 0; v < NUM_VOICES; v++)
    {
        uint8_t amp = m_voice[v].amp;

        if(ramp)
        {
            if(amp < m_voice[v].target_amp)      amp++;
            else if(amp > m_voice[v].target_amp) amp--;
            m_voice[v].amp = amp;
        }

        if(amp == 0) continue;

        m_voice[v].phase = (uint16_t)(m_voice[v].phase + m_voice[v].inc);

        /* Pulse wave: compare the top 8 bits of the phase against the
         * pulse width. m_pulse_width == 0x80 gives a square. */
        if((uint8_t)(m_voice[v].phase >> 8) < m_pulse_width) mix -= (int16_t)amp;
        else                                                 mix += (int16_t)amp;
    }

    /* Centre on mid-rail. Master volume is already baked into each voice's
     * amplitude, so mix is bounded by +/-124 and this cannot wrap. */
    PWM_WRITE((uint8_t)(128 + mix));

    lfo_divider++;
    if((lfo_divider & 0x3Fu) == 0) m_lfo_tick = true; /* 125 Hz */
}

static void audio_init(void)
{
    uint8_t v;

    for(v = 0; v < NUM_VOICES; v++)
    {
        m_voice[v].phase      = 0;
        m_voice[v].inc        = 0;
        m_voice[v].base_inc   = 0;
        m_voice[v].note       = 0xFF;
        m_voice[v].amp        = 0;
        m_voice[v].target_amp = 0;
        m_voice[v].velocity   = 0;
        m_voice[v].age        = 0;
        m_voice[v].held       = false;
    }

    /* Timer2 + CCP1 -> 187.5 kHz, 8-bit PWM. */
    #if defined(_18F13K50) || defined(_18F14K50)
    TRISCbits.TRISC5 = 0; // CCP1 is on RC5 for the 20-pin parts
    #else
    TRISCbits.TRISC2 = 0; // CCP1 is on RC2
    #endif
    PR2      = 63;
    T2CON    = 0x04;  // prescaler 1:1, Timer2 on
    CCP1CON  = 0x0C;  // PWM mode
    PWM_WRITE(128);   // mid-rail = silence

    /* Timer0 -> 8 kHz sample clock, 16-bit, Fosc/4, no prescaler. */
    T0CON  = 0x08;    // 16-bit, internal clock, prescaler bypassed, off
    TMR0H  = (uint8_t)(TIMER0_RELOAD >> 8);
    TMR0L  = (uint8_t)(TIMER0_RELOAD & 0xFF);
    INTCON2bits.TMR0IP = 1; // high priority - audio must not be delayed
    INTCONbits.TMR0IF  = 0;
    INTCONbits.TMR0IE  = 1;
    T0CONbits.TMR0ON   = 1;
}

/**
 * LFO + pitch bend update, 125 times a second from the main loop.
 *
 * The LFO is a triangle built from the top bit of an 8-bit phase, which is
 * all the shape a vibrato needs and costs nothing.
 */
static void modulation_tasks(void)
{
    int16_t vibrato;
    int16_t mod;
    uint8_t v;

    /* ~5.2 Hz: 125 Hz tick, phase wraps every 256/11 ticks. */
    m_lfo_phase = (uint8_t)(m_lfo_phase + 11u);

    /* Triangle in the range -128..127. */
    if(m_lfo_phase < 128) vibrato = (int16_t)((int16_t)m_lfo_phase * 2 - 128);
    else                  vibrato = (int16_t)(384 - (int16_t)m_lfo_phase * 2);

    /* Mod wheel scales the vibrato to at most ~1/16 of the bend range,
     * which is about a quarter tone either side. */
    vibrato = (int16_t)((vibrato * (int16_t)m_mod_wheel) >> 7);
    vibrato = (int16_t)(vibrato << 2);

    mod = (int16_t)(m_bend + vibrato);

    for(v = 0; v < NUM_VOICES; v++)
    {
        if(m_voice[v].base_inc == 0) continue;
        voice_apply_modulation(v, mod);
    }
}

/* ************************************************************************** */


/* ************************************************************************** */
/* ******************************* STARTUP ********************************** */
/* ************************************************************************** */

static void example_init(void)
{
    /* Oscillator: 48 MHz for USB full speed. */
    #if defined(_18F13K50) || defined(_18F14K50)
    OSCTUNEbits.SPLLEN = 1;
    PLL_STARTUP_DELAY();

    #elif defined(_18F24K50) || defined(_18F25K50) || defined(_18F45K50)
    #if XTAL_USED == NO_XTAL
    OSCCONbits.IRCF = 7;
    #endif
    #if XTAL_USED != MHz_12
    OSCTUNEbits.SPLLMULT = 1;
    #endif
    OSCCON2bits.PLLEN = 1;
    PLL_STARTUP_DELAY();
    #if XTAL_USED == NO_XTAL
    ACTCONbits.ACTSRC = 1;
    ACTCONbits.ACTEN  = 1;
    #endif

    #elif defined(__J_PART)
    OSCTUNEbits.PLLEN = 1;
    PLL_STARTUP_DELAY();
    #endif

    LED_OFF();
    LED_OUPUT();
}

static void flash_led(void)
{
    uint8_t i;

    for(i = 0; i < 3; i++)
    {
        LED_ON();
        __delay_ms(150);
        LED_OFF();
        __delay_ms(150);
    }
}

USB_ISR_HIGH(isr_high)
{
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
        /* Reload before doing the work so the period stays constant. */
        TMR0H = (uint8_t)(TIMER0_RELOAD >> 8);
        TMR0L = (uint8_t)(TIMER0_RELOAD & 0xFF);
        INTCONbits.TMR0IF = 0;
        audio_sample();
    }
}

USB_ISR_LOW(isr_low)
{
    if(USB_INTERRUPT_ENABLE && USB_INTERRUPT_FLAG)
    {
        usb_tasks();
        USB_INTERRUPT_FLAG = 0;
    }
}

void main(void)
{
    uint8_t v;
    bool    sounding;

    example_init();
    flash_led();
    audio_init();

    usb_init();

    /* Two priority levels: audio high, USB low. */
    RCONbits.IPEN          = 1;
    USB_INTERRUPT_PRIORITY = 0;
    USB_INTERRUPT_FLAG     = 0;
    USB_INTERRUPT_ENABLE   = 1;
    INTCONbits.GIEL        = 1;
    INTCONbits.GIEH        = 1;

    while(1)
    {
        if(m_lfo_tick)
        {
            m_lfo_tick = false;
            modulation_tasks();

            sounding = false;
            for(v = 0; v < NUM_VOICES; v++)
            {
                if(m_voice[v].amp) sounding = true;

                /* Free a voice once it has fully faded out, so the
                 * allocator can see it as available again. */
                if(m_voice[v].amp == 0 && m_voice[v].target_amp == 0)
                {
                    m_voice[v].note     = 0xFF;
                    m_voice[v].base_inc = 0;
                }
            }

            if(sounding) LED_ON();
            else         LED_OFF();
        }

        /* All MIDI handling happens in the USB interrupt; the main loop
         * only paces modulation. */
    }
}

/* ************************************************************************** */
