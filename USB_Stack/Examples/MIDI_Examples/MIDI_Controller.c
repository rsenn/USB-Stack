/**
 * @file MIDI_Controller.c
 * @brief USB MIDI keyboard + control surface (single translation unit).
 * @author Roman Senn
 * @date 20/08/2026
 *
 * USB uC - USB MIDI Controller Example.
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

A class-compliant USB MIDI 1.0 device that is a keyboard and a control
surface at the same time: the same MIDI port carries Note On/Off from the
key switches AND Control Change from the pots and the sustain switch, all
interleaved in one bulk IN stream. It is bidirectional, so the host can
also send MIDI back (Note On/Off drives the LED, so you can drive the
device from a DAW track).

There is no usb_midi.c class driver in this stack, and none is needed:
MIDI 1.0 defines no class requests and no class descriptor GETs, so the
whole class lives in this file on top of the core endpoint API
(usb_arm_endpoint(), g_usb_bd_table, g_usb_ep_stat, the usb_app_* hooks).

Endpoint map (EP1, bulk, 64 bytes, PINGPONG_0_OUT):
  EP1 IN  - device -> host: notes + CCs generated here.
  EP1 OUT - host -> device: notes + CCs from the host.

USB-MIDI event packets are always 4 bytes:
  byte 0: [cable number : 4][code index number : 4]
  byte 1..3: the MIDI message, zero padded.

============================================================================
BUILDING
============================================================================

This file is one translation unit; it replaces main.c, usb_app.c and
usb_descriptors.c. Compile it together with the stack core (usb.c only -
no class driver is used) and define MIDI_CONTROLLER_EXAMPLE so that
usb_config.h picks the MIDI endpoint/interface counts.

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
         -DMIDI_CONTROLLER_EXAMPLE \
         -I../../USB -I../../Hardware \
         -o MIDI_Controller.hex \
         MIDI_Controller.c ../../USB/usb.c

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

  sdcc -mpic16 -p18f25k50 --use-non-free -c -DMIDI_CONTROLLER_EXAMPLE \
       -I../../USB -I../../Hardware -o MIDI_Controller.o MIDI_Controller.c

  sdcc -mpic16 -p18f25k50 --use-non-free -c -DMIDI_CONTROLLER_EXAMPLE \
       -I../../USB -I../../Hardware -o usb.o ../../USB/usb.c

  sdcc -mpic16 -p18f25k50 --use-non-free -o MIDI_Controller.hex MIDI_Controller.o usb.o

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
HARDWARE (PIC18F2xK50 / PIC18F4xK50 defaults - adjust the map below)
============================================================================

  RB0..RB7  8 key switches, active low, internal weak pull-ups.
            Notes C..G of the current octave (base note 60 = middle C).
  RA0..RA3  4 potentiometers (AN0..AN3) -> CC 1, 7, 10, 74.
  BUTTON    from Hardware/config.h -> sustain pedal, CC 64.
  LED       from Hardware/config.h -> lit while the host holds a note.
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
#error "The MIDI examples need two 64-byte bulk buffers; target a PIC18 part."
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

/* Code Index Numbers (byte 0, low nibble) for the messages we use. */
#define CIN_NOTE_OFF       0x08
#define CIN_NOTE_ON        0x09
#define CIN_POLY_KEYPRESS  0x0A
#define CIN_CONTROL_CHANGE 0x0B
#define CIN_PROGRAM_CHANGE 0x0C
#define CIN_CHAN_PRESSURE  0x0D
#define CIN_PITCH_BEND     0x0E

/* Cable number 0 in the high nibble. */
#define CABLE_0 0x00

/* MIDI status bytes (channel 1). */
#define MIDI_NOTE_OFF       0x80
#define MIDI_NOTE_ON        0x90
#define MIDI_CONTROL_CHANGE 0xB0

/* Controller numbers we emit. */
#define CC_MOD_WHEEL 1
#define CC_VOLUME    7
#define CC_PAN       10
#define CC_BRIGHTNESS 74
#define CC_SUSTAIN   64

/* ************************************************************************** */


/* ************************************************************************** */
/* ************************* ENDPOINT HAL *********************************** */
/* ************************************************************************** */

#define MIDI_EP      EP1
#define MIDI_EP_SIZE EP1_SIZE
#define MIDI_UEPbits UEP1bits
#define MIDI_BD_OUT  BD1_OUT
#define MIDI_BD_IN   BD1_IN

#if PINGPONG_MODE != PINGPONG_0_OUT
#error "MIDI_Controller.c assumes PINGPONG_0_OUT (the usb_config.h default)."
#endif

/* EP0 uses three buffers in PINGPONG_0_OUT (out even, out odd, in). */
#define MIDI_EP_BUFFERS_STARTING_ADDR (EP_BUFFERS_STARTING_ADDR + (EP0_SIZE * 3))
#define MIDI_EP_OUT_BUFFER_BASE_ADDR   MIDI_EP_BUFFERS_STARTING_ADDR
#define MIDI_EP_IN_BUFFER_BASE_ADDR   (MIDI_EP_BUFFERS_STARTING_ADDR + MIDI_EP_SIZE)

#define MIDI_EP_OUT_DATA_TOGGLE_VAL g_usb_ep_stat[MIDI_EP][OUT].Data_Toggle_Val
#define MIDI_EP_IN_DATA_TOGGLE_VAL  g_usb_ep_stat[MIDI_EP][IN].Data_Toggle_Val

/* Under SDCC the endpoint buffers are pointer macros into the USB RAM window
 * that usb.c reserves in one block - see USB/usb_compiler.h. */
#if USB_SDCC
#define g_midi_ep_out USB_ABS_ARR(uint8_t, MIDI_EP_OUT_BUFFER_BASE_ADDR)
#define g_midi_ep_in  USB_ABS_ARR(uint8_t, MIDI_EP_IN_BUFFER_BASE_ADDR)
#else
uint8_t g_midi_ep_out[MIDI_EP_SIZE] __at(MIDI_EP_OUT_BUFFER_BASE_ADDR);
uint8_t g_midi_ep_in[MIDI_EP_SIZE]  __at(MIDI_EP_IN_BUFFER_BASE_ADDR);
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
    0x00DD,      // idProduct
    0x0001,      // bcdDevice
    0x01,        // iManufacturer
    0x02,        // iProduct
    0x00,        // iSerialNumber
    0x01         // bNumConfigurations
};

/*
 * The configuration is emitted as a byte array rather than a struct tree.
 * The MIDI class descriptors are variable length (a MIDI OUT jack grows with
 * its pin count, a class-specific endpoint grows with its jack count), and a
 * flat array keeps the wTotalLength arithmetic visible and free of any
 * struct-packing assumptions.
 *
 * Jack topology (this is what a host shows you):
 *
 *   EP1 OUT --> [emb IN jack 1] --> [ext OUT jack 4]   host -> device
 *   EP1 IN  <-- [emb OUT jack 3] <-- [ext IN jack 2]   device -> host
 */

#define MS_CS_TOTAL_LEN  47u  /* 7 + 6 + 6 + 9 + 9 + 5 + 5 */
#define CONFIG_TOTAL_LEN 101u /* 9 + 9 + 9 + 9 + MS_CS_TOTAL_LEN + 9 + 9 */

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
    0x00,                       // bNumEndpoints - AC has no endpoints here
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
    0x01,                       // bInCollection - one streaming interface
    0x01,                       // baInterfaceNr(1) - interface 1

    /* ---- Standard MIDIStreaming Interface ----------------------------- */
    9,                          // bLength
    INTERFACE_DESC,             // bDescriptorType
    0x01,                       // bInterfaceNumber
    0x00,                       // bAlternateSetting
    0x02,                       // bNumEndpoints - bulk IN + bulk OUT
    AUDIO_CLASS,                // bInterfaceClass
    SUBCLASS_MIDISTREAMING,     // bInterfaceSubClass
    0x00,                       // bInterfaceProtocol
    0x00,                       // iInterface

    /* ---- Class-Specific MIDIStreaming Header -------------------------- */
    7,                          // bLength
    CS_INTERFACE_DESC,          // bDescriptorType
    MS_HEADER,                  // bDescriptorSubtype
    0x00, 0x01,                 // bcdMSC (1.00)
    (uint8_t)MS_CS_TOTAL_LEN,   // wTotalLength LSB - header + jacks + cs eps
    0x00,                       // wTotalLength MSB

    /* ---- MIDI IN Jack, Embedded (ID 1) -------------------------------- */
    /* Where the host's MIDI arrives inside the device.                    */
    6,                          // bLength
    CS_INTERFACE_DESC,          // bDescriptorType
    MIDI_IN_JACK,               // bDescriptorSubtype
    JACK_EMBEDDED,              // bJackType
    0x01,                       // bJackID
    0x00,                       // iJack

    /* ---- MIDI IN Jack, External (ID 2) -------------------------------- */
    /* Represents the keys/pots feeding the embedded OUT jack.             */
    6,                          // bLength
    CS_INTERFACE_DESC,          // bDescriptorType
    MIDI_IN_JACK,               // bDescriptorSubtype
    JACK_EXTERNAL,              // bJackType
    0x02,                       // bJackID
    0x00,                       // iJack

    /* ---- MIDI OUT Jack, Embedded (ID 3) ------------------------------- */
    /* Sourced from external IN jack 2; feeds the bulk IN endpoint.        */
    9,                          // bLength
    CS_INTERFACE_DESC,          // bDescriptorType
    MIDI_OUT_JACK,              // bDescriptorSubtype
    JACK_EMBEDDED,              // bJackType
    0x03,                       // bJackID
    0x01,                       // bNrInputPins
    0x02,                       // baSourceID(1) - external IN jack 2
    0x01,                       // baSourcePin(1)
    0x00,                       // iJack

    /* ---- MIDI OUT Jack, External (ID 4) ------------------------------- */
    /* Sourced from embedded IN jack 1; this is where host MIDI ends up.   */
    9,                          // bLength
    CS_INTERFACE_DESC,          // bDescriptorType
    MIDI_OUT_JACK,              // bDescriptorSubtype
    JACK_EXTERNAL,              // bJackType
    0x04,                       // bJackID
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
    0x01,                       // baAssocJackID(1) - embedded IN jack 1

    /* ---- Standard Bulk IN Endpoint ------------------------------------ */
    9,                          // bLength
    ENDPOINT_DESC,              // bDescriptorType
    0x81,                       // bEndpointAddress - EP1 IN
    0x02,                       // bmAttributes - bulk
    MIDI_EP_SIZE, 0x00,         // wMaxPacketSize
    0x00,                       // bInterval
    0x00,                       // bRefresh
    0x00,                       // bSynchAddress

    /* ---- Class-Specific Bulk IN Endpoint ------------------------------ */
    5,                          // bLength
    CS_ENDPOINT_DESC,           // bDescriptorType
    MS_GENERAL,                 // bDescriptorSubtype
    0x01,                       // bNumEmbMIDIJack
    0x03                        // baAssocJackID(1) - embedded OUT jack 3
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
    uint16_t bString[15];
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
    {'M','I','D','I',' ','C','o','n','t','r','o','l','l','e','r'}
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
/* ************************ CONTROLLER HARDWARE MAP ************************* */
/* ************************************************************************** */

#define NUM_KEYS 8

/* ADC setup below is written for the PIC18F2xK50/4xK50 family. On other
 * parts the example builds as keys + sustain only; add your part to
 * adc_init()/adc_read_7bit() and bump this to get the pots back. */
#if defined(_18F24K50) || defined(_18F25K50) || defined(_18F45K50)
#define NUM_POTS 4
#else
#define NUM_POTS 0
#endif

#define KEYS_PORT   PORTB
#define KEYS_TRIS   TRISB
#define KEYS_WPU    WPUB

/* Note assigned to key 0; the rest follow chromatically. 60 = middle C. */
#define BASE_NOTE 60

/* Fixed velocity - these are plain switches, there is nothing to measure. */
#define KEY_VELOCITY 100

/* Debounce time in SOF ticks (1 ms each). */
#define DEBOUNCE_MS 5

/* A pot must move by more than this (in 7-bit units) before we send a CC.
 * Without it, ADC noise floods the bulk IN endpoint. */
#define POT_HYSTERESIS 1

#if NUM_POTS > 0
/* Controller number sent for each pot. */
static const uint8_t m_pot_cc[NUM_POTS] =
{
    CC_MOD_WHEEL,
    CC_VOLUME,
    CC_PAN,
    CC_BRIGHTNESS
};
#endif

/* ************************************************************************** */


/* ************************************************************************** */
/* ****************************** MIDI QUEUE ******************************** */
/* ************************************************************************** */

/*
 * Events are produced in the SOF interrupt and consumed by the main loop, so
 * the queue is a single-producer/single-consumer ring: the producer only
 * moves head, the consumer only moves tail, and no critical section is
 * needed as long as both indices are byte sized (atomic on PIC18).
 */
#define TX_QUEUE_LEN 16u /* must be a power of two */
#define TX_QUEUE_MASK (TX_QUEUE_LEN - 1u)

typedef struct
{
    uint8_t header; /* cable number | code index number */
    uint8_t byte0;  /* status */
    uint8_t byte1;
    uint8_t byte2;
}midi_event_t;

static volatile midi_event_t m_tx_queue[TX_QUEUE_LEN];
static volatile uint8_t      m_tx_head = 0;
static volatile uint8_t      m_tx_tail = 0;

/* True when EP1 IN is free to be armed again. */
static volatile bool m_ep_in_ready = false;

/* Set by the SOF handler, cleared by the main loop: 1 ms elapsed. */
static volatile bool m_tick = false;

/* State of the notes the *host* has switched on, for the LED. */
static volatile uint8_t m_host_notes_on = 0;

static void midi_queue_event(uint8_t cin, uint8_t status, uint8_t data1, uint8_t data2)
{
    uint8_t next = (uint8_t)((m_tx_head + 1u) & TX_QUEUE_MASK);

    /* Full queue: drop the event rather than block the SOF interrupt. A
     * dropped Note Off would hang a note, so the queue is sized for the
     * worst case burst (8 keys + 4 pots + sustain in one tick). */
    if(next == m_tx_tail) return;

    m_tx_queue[m_tx_head].header = (uint8_t)(CABLE_0 | cin);
    m_tx_queue[m_tx_head].byte0  = status;
    m_tx_queue[m_tx_head].byte1  = data1;
    m_tx_queue[m_tx_head].byte2  = data2;
    m_tx_head = next;
}

static void midi_send_note_on(uint8_t note, uint8_t velocity)
{
    midi_queue_event(CIN_NOTE_ON, MIDI_NOTE_ON, note, velocity);
}

static void midi_send_note_off(uint8_t note)
{
    midi_queue_event(CIN_NOTE_OFF, MIDI_NOTE_OFF, note, 0);
}

static void midi_send_cc(uint8_t controller, uint8_t value)
{
    midi_queue_event(CIN_CONTROL_CHANGE, MIDI_CONTROL_CHANGE, controller, value);
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

static void midi_arm_ep_in(uint8_t cnt)
{
    if(MIDI_EP_IN_DATA_TOGGLE_VAL) g_usb_bd_table[MIDI_BD_IN].STAT = _DTSEN | _DTS;
    else                           g_usb_bd_table[MIDI_BD_IN].STAT = _DTSEN;
    g_usb_bd_table[MIDI_BD_IN].CNT   = cnt;
    g_usb_bd_table[MIDI_BD_IN].STAT |= _UOWN;
}

/**
 * Handle one 4-byte USB-MIDI event coming from the host.
 *
 * A control surface that ignores its input is only half a device: hosts use
 * the return path to light up feedback (a DAW echoing the track's notes, or
 * a controller map echoing CC values). Here we just track note count on the
 * LED, but this is the hook to extend.
 */
static void midi_handle_event(const uint8_t* p_event)
{
    uint8_t cin    = (uint8_t)(p_event[0] & 0x0F);
    uint8_t data1  = p_event[2];
    uint8_t data2  = p_event[3];

    switch(cin)
    {
        case CIN_NOTE_ON:
            /* Note On with velocity 0 is a Note Off - every host does this. */
            if(data2 == 0)
            {
                if(m_host_notes_on) m_host_notes_on--;
            }
            else m_host_notes_on++;
            break;

        case CIN_NOTE_OFF:
            if(m_host_notes_on) m_host_notes_on--;
            break;

        case CIN_CONTROL_CHANGE:
            /* CC 123 = All Notes Off, CC 120 = All Sound Off. */
            if(data1 == 123 || data1 == 120) m_host_notes_on = 0;
            break;

        default:
            break;
    }

    if(m_host_notes_on) LED_ON();
    else                LED_OFF();
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

static void midi_ep_in_tasks(void)
{
    MIDI_EP_IN_DATA_TOGGLE_VAL ^= 1;
    m_ep_in_ready = true;
}

/**
 * Move as many queued events as will fit into the bulk IN buffer and arm it.
 *
 * Notes and CCs share this one stream, which is exactly what "keyboard and
 * control surface at the same time" means at the wire level: a single MIDI
 * port, events interleaved in the order they happened.
 */
static void midi_flush_tx(void)
{
    uint8_t cnt = 0;

    if(!m_ep_in_ready) return;
    if(m_tx_tail == m_tx_head) return;

    while((m_tx_tail != m_tx_head) && (cnt <= (uint8_t)(MIDI_EP_SIZE - 4u)))
    {
        g_midi_ep_in[cnt]                  = m_tx_queue[m_tx_tail].header;
        g_midi_ep_in[(uint8_t)(cnt + 1u)]   = m_tx_queue[m_tx_tail].byte0;
        g_midi_ep_in[(uint8_t)(cnt + 2u)]   = m_tx_queue[m_tx_tail].byte1;
        g_midi_ep_in[(uint8_t)(cnt + 3u)]   = m_tx_queue[m_tx_tail].byte2;
        cnt = (uint8_t)(cnt + 4u);
        m_tx_tail = (uint8_t)((m_tx_tail + 1u) & TX_QUEUE_MASK);
    }

    m_ep_in_ready = false;
    midi_arm_ep_in(cnt);
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
    /* MIDI class descriptors are only ever returned inside the
     * configuration descriptor, never fetched on their own. */
    return false;
}

void usb_app_init(void)
{
    usb_ram_set(0, g_midi_ep_out, MIDI_EP_SIZE);
    usb_ram_set(0, g_midi_ep_in,  MIDI_EP_SIZE);

    g_usb_bd_table[MIDI_BD_OUT].STAT = 0;
    g_usb_bd_table[MIDI_BD_OUT].ADR  = MIDI_EP_OUT_BUFFER_BASE_ADDR;
    g_usb_bd_table[MIDI_BD_IN].STAT  = 0;
    g_usb_bd_table[MIDI_BD_IN].ADR   = MIDI_EP_IN_BUFFER_BASE_ADDR;

    MIDI_UEPbits.EPHSHK   = 1; // Handshaking enabled
    MIDI_UEPbits.EPCONDIS = 1; // Data only, no SETUP on this endpoint
    MIDI_UEPbits.EPOUTEN  = 1;
    MIDI_UEPbits.EPINEN   = 1;

    g_usb_ep_stat[MIDI_EP][OUT].Halt = 0;
    g_usb_ep_stat[MIDI_EP][IN].Halt  = 0;
    MIDI_EP_OUT_DATA_TOGGLE_VAL = 0;
    MIDI_EP_IN_DATA_TOGGLE_VAL  = 0;

    /* Drop anything queued while we were unconfigured. */
    m_tx_head = 0;
    m_tx_tail = 0;
    m_host_notes_on = 0;

    m_ep_in_ready = true;
    midi_arm_ep_out();
}

void usb_app_tasks(void)
{
    if(TRANSACTION_EP != MIDI_EP) return;

    if(TRANSACTION_DIR == IN) midi_ep_in_tasks();
    else                      midi_ep_out_tasks();
}

void usb_app_clear_halt(uint8_t bd_table_index, uint8_t ep, uint8_t dir)
{
    g_usb_ep_stat[ep][dir].Data_Toggle_Val = 0;

    if(g_usb_ep_stat[ep][dir].Halt)
    {
        g_usb_ep_stat[ep][dir].Halt    = 0;
        g_usb_bd_table[bd_table_index].STAT = 0;
    }

    if(ep != MIDI_EP) return;

    /* Re-arm whichever direction was just un-halted. */
    if(dir == IN) m_ep_in_ready = true;
    else          midi_arm_ep_out();
}

bool usb_app_set_interface(uint8_t alternate_setting, uint8_t interface)
{
    /* Both interfaces have only alternate setting 0. */
    if(alternate_setting != 0) return false;
    if(interface >= NUM_INTERFACES) return false;

    MIDI_EP_OUT_DATA_TOGGLE_VAL = 0;
    MIDI_EP_IN_DATA_TOGGLE_VAL  = 0;
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
/* ************************* CONTROL SURFACE SCAN *************************** */
/* ************************************************************************** */

static uint8_t m_key_state    = 0;    /* debounced, 1 = pressed */
static uint8_t m_key_sample   = 0;    /* last raw sample */
static uint8_t m_key_timer[NUM_KEYS]; /* per-key debounce countdown */

#if NUM_POTS > 0
static uint8_t m_pot_value[NUM_POTS]; /* last CC value sent, 0..127 */
static uint8_t m_pot_index  = 0;      /* round-robin ADC channel */
static bool    m_pot_primed = false;  /* first pass just seeds the values */
#endif

static uint8_t m_sustain_down = 0;
static uint8_t m_sustain_timer = 0;

#if NUM_POTS > 0
/**
 * Start a conversion and wait for it. At 12 MIPS one conversion is a few
 * microseconds, well inside the 1 ms SOF budget, so polling here is cheaper
 * than another interrupt.
 */
static uint8_t adc_read_7bit(uint8_t channel)
{
    ADCON0 = (uint8_t)((channel << 2) | 0x01); // select channel, ADON
    __delay_us(5);                             // acquisition time
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO) {}
    /* Right justified 10-bit result -> 7-bit MIDI value. */
    return (uint8_t)((((uint16_t)ADRESH << 8) | ADRESL) >> 3);
}

static void adc_init(void)
{
    ADCON1 = 0x00; // VREF+ = VDD, VREF- = VSS
    ADCON2 = 0xBE; // right justified, 20 TAD, Fosc/64
    ADCON0 = 0x01; // ADON
}
#endif

/**
 * One key scan pass. Called every millisecond from SOF.
 *
 * Debounce is per key: a change must survive DEBOUNCE_MS consecutive
 * milliseconds before it becomes a note. This matters more than usual here,
 * because a bounce turns into a spurious Note On/Off pair on the wire.
 */
static void scan_keys(void)
{
    uint8_t raw = (uint8_t)~KEYS_PORT; /* active low -> 1 = pressed */
    uint8_t i;

    for(i = 0; i < NUM_KEYS; i++)
    {
        uint8_t mask = (uint8_t)(1u << i);

        if((raw & mask) != (m_key_state & mask))
        {
            /* Restart the timer whenever the raw level moves again. */
            if((raw & mask) != (m_key_sample & mask)) m_key_timer[i] = DEBOUNCE_MS;
            else if(m_key_timer[i]) m_key_timer[i]--;

            if(m_key_timer[i] == 0)
            {
                m_key_state ^= mask;

                if(m_key_state & mask) midi_send_note_on((uint8_t)(BASE_NOTE + i), KEY_VELOCITY);
                else                   midi_send_note_off((uint8_t)(BASE_NOTE + i));
            }
        }
        else m_key_timer[i] = DEBOUNCE_MS;
    }

    m_key_sample = raw;
}

/**
 * One pot per millisecond, round-robin: 4 pots means each is refreshed every
 * 4 ms, which is far finer than a fader can be moved and keeps the ADC out
 * of the way of the key scan.
 */
static void scan_pots(void)
{
#if NUM_POTS > 0
    uint8_t value = adc_read_7bit(m_pot_index);
    uint8_t last  = m_pot_value[m_pot_index];
    uint8_t delta = (uint8_t)((value > last) ? (value - last) : (last - value));

    if(!m_pot_primed || (delta > (uint8_t)POT_HYSTERESIS))
    {
        m_pot_value[m_pot_index] = value;
        if(m_pot_primed) midi_send_cc(m_pot_cc[m_pot_index], value);
    }

    m_pot_index++;
    if(m_pot_index >= NUM_POTS)
    {
        m_pot_index  = 0;
        m_pot_primed = true; /* first full sweep only seeds the values */
    }
#endif
}

static void scan_sustain(void)
{
    uint8_t down = (BUTTON_PRESSED) ? 1u : 0u;

    if(down == m_sustain_down)
    {
        m_sustain_timer = DEBOUNCE_MS;
        return;
    }

    if(m_sustain_timer) m_sustain_timer--;
    if(m_sustain_timer) return;

    m_sustain_down = down;
    midi_send_cc(CC_SUSTAIN, down ? 127u : 0u);
    m_sustain_timer = DEBOUNCE_MS;
}

/* ************************************************************************** */


/* ************************************************************************** */
/* ******************************* STARTUP ********************************** */
/* ************************************************************************** */

static void example_init(void)
{
    uint8_t i;

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

    /* Keys: inputs, digital, weak pull-ups on. */
    KEYS_TRIS = 0xFF;
    #if defined(ANSELB)
    ANSELB = 0x00;
    #endif
    #if defined(_18F24K50) || defined(_18F25K50) || defined(_18F45K50)
    KEYS_WPU = 0xFF;
    INTCON2bits.nRBPU = 0;
    #elif defined(_18F13K50) || defined(_18F14K50)
    KEYS_WPU = 0xFF;
    INTCON2bits.nRABPU = 0;
    #endif

    /* Pots: RA0..RA3 analog inputs. */
    #if NUM_POTS > 0
    TRISA |= 0x0F;
    #if defined(ANSELA)
    ANSELA |= 0x0F;
    #endif
    adc_init();
    #endif

    /* Sustain switch (the board button from config.h). */
    #if defined(BUTTON_ANSEL)
    BUTTON_ANSEL &= ~(1 << BUTTON_ANSEL_BIT);
    #elif defined(BUTTON_ANCON)
    BUTTON_ANCON |= (1 << BUTTON_ANCON_BIT);
    #endif

    LED_OFF();
    LED_OUPUT();

    for(i = 0; i < NUM_KEYS; i++) m_key_timer[i] = DEBOUNCE_MS;
    m_sustain_timer = DEBOUNCE_MS;
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

/**
 * SOF fires once per millisecond whenever the bus is up, which is a better
 * time base for scanning than a spare timer: it costs nothing, and it stops
 * automatically when the host suspends.
 */
void usb_sof(void)
{
    m_tick = true;
}

USB_ISR(isr)
{
    if(USB_INTERRUPT_ENABLE && USB_INTERRUPT_FLAG)
    {
        usb_tasks();
        USB_INTERRUPT_FLAG = 0;
    }
}

void main(void)
{
    example_init();
    flash_led();

    usb_init();
    INTCONbits.PEIE      = 1;
    USB_INTERRUPT_FLAG   = 0;
    USB_INTERRUPT_ENABLE = 1;
    INTCONbits.GIE       = 1;

    while(1)
    {
        if(usb_get_state() != STATE_CONFIGURED) continue;

        if(m_tick)
        {
            m_tick = false;
            scan_keys();
            scan_pots();
            scan_sustain();
        }

        midi_flush_tx();
    }
}

/* ************************************************************************** */
