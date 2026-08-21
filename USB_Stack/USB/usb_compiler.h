/**
 * @file usb_compiler.h
 * @brief Compiler abstraction so the stack builds with both XC8 and SDCC.
 * @author Roman Senn
 * @date 21/08/2026
 *
 * USB uC - USB Stack.
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
 * Everything in here is a no-op under XC8 - the XC8 build is unchanged, byte
 * for byte. The macros exist so that a single source line satisfies both
 * compilers where they disagree:
 *
 *   - which register header to include
 *   - how the part is spelled in #if defined(...)
 *   - where an absolute-address qualifier goes, and that SDCC cannot place
 *     arrays at all
 *   - interrupt function syntax
 *   - NOP() and the __delay_*() builtins
 */

#ifndef USB_COMPILER_H
#define USB_COMPILER_H

#include <stdint.h>

#if defined(__SDCC) || defined(SDCC)
#define USB_SDCC 1
#else
#define USB_SDCC 0
#endif


/* ************************************************************************** */
/* *********************** REGISTER DEFINITIONS ***************************** */
/* ************************************************************************** */

#if USB_SDCC
#include <pic18fregs.h>
#else
#include <xc.h>
#endif

/* ************************************************************************** */


/* ************************************************************************** */
/* ************************ PART IDENTIFICATION ***************************** */
/* ************************************************************************** */

/*
 * The stack tests XC8's part macros (_18F25K50, __J_PART, ...). SDCC spells
 * the same part __18f25k50, so map its names onto XC8's and let every
 * existing #if in the stack keep working untouched.
 */
#if USB_SDCC

#if   defined(__18f13k50) && !defined(_18F13K50)
#define _18F13K50
#elif defined(__18f14k50) && !defined(_18F14K50)
#define _18F14K50
#elif defined(__18f24k50) && !defined(_18F24K50)
#define _18F24K50
#elif defined(__18f25k50) && !defined(_18F25K50)
#define _18F25K50
#elif defined(__18f45k50) && !defined(_18F45K50)
#define _18F45K50
#elif defined(__18f26j53) && !defined(_18F26J53)
#define _18F26J53
#elif defined(__18f27j53) && !defined(_18F27J53)
#define _18F27J53
#elif defined(__18f46j53) && !defined(_18F46J53)
#define _18F46J53
#elif defined(__18f47j53) && !defined(_18F47J53)
#define _18F47J53
#endif

/* XC8 predefines __J_PART for the J series; SDCC does not. */
#if !defined(__J_PART) && (defined(_18F26J53) || defined(_18F27J53) || \
                           defined(_18F46J53) || defined(_18F47J53))
#define __J_PART
#endif

#endif /* USB_SDCC */

/* ************************************************************************** */


/* ************************************************************************** */
/* ******************** ABSOLUTE ADDRESS PLACEMENT ************************** */
/* ************************************************************************** */

/*
 * XC8 and SDCC disagree about fixed-address objects in two ways:
 *
 *   1. XC8 wants the qualifier after the declarator, SDCC wants it in front:
 *        XC8:   ch9_setup_t g_usb_setup __at(SETUP_DATA_ADDR);
 *        SDCC:  __at(SETUP_DATA_ADDR) ch9_setup_t g_usb_setup;
 *
 *      USB_AT_PRE()/USB_AT_POST() go at both ends of the declaration, so one
 *      line of source satisfies either compiler.
 *
 *   2. SDCC's pic16 port silently drops __at on *array* declarations: it
 *      emits the `global` but never reserves the storage, and the link then
 *      fails with "Symbol not previously defined". Reproduced on SDCC 4.0.0,
 *      4.1.0 and 4.3.0rc1. Structs and scalars are placed correctly, so an
 *      absolute array is declared under SDCC as a one-member struct and the
 *      original name is #defined to that member - use sites are unchanged.
 *      USB_ABS_ARRAY()/USB_ABS_ARRAY_EXTERN() do that; because a macro
 *      cannot emit a #define, the accompanying alias is written out at each
 *      declaration site.
 */
#if USB_SDCC
#define USB_AT_PRE(addr)  __at(addr)
#define USB_AT_POST(addr)
#else
#define USB_AT_PRE(addr)
#define USB_AT_POST(addr) __at(addr)
#endif

/*
 * The stack also places several objects at the *same* address on purpose -
 * g_usb_setup, g_usb_get_descriptor, m_set_address and friends are all views
 * of the 8-byte SETUP packet. SDCC cannot express that: it merges same-address
 * declarations into one section and gpasm then rejects the repeated labels
 * ("Address label duplicated or different in second pass"). It also lays
 * absolute objects out in declaration order rather than at their stated
 * addresses once more than one lands in a region.
 *
 * So under SDCC nothing in USB RAM is a real object. Each name becomes a
 * pointer dereference at its fixed address, and one reservation block per
 * region keeps the linker from allocating anything else there. Use sites are
 * unchanged, and XC8 keeps its ordinary __at() objects.
 */
#if USB_SDCC
#define USB_ABS_OBJ(type, addr) (*(type *)(addr))
#define USB_ABS_ARR(type, addr) ((type *)(addr))
#define USB_RESERVE(name, bytes, addr) \
        struct {uint8_t r[bytes];} USB_AT_PRE(addr) name
#endif

/* ************************************************************************** */


/* ************************************************************************** */
/* ********************* DESCRIPTOR ADDRESS TABLES ************************** */
/* ************************************************************************** */

/*
 * usb.c reaches the configuration and string descriptors through tables of
 * their addresses. XC8 folds `(uint16_t)&descriptor` into a constant
 * initializer; SDCC rejects it ("Initializer element is not a constant
 * expression") and will only accept an address constant when the element
 * type is a pointer.
 *
 * usb_desc_addr_t is uint16_t under XC8 - so existing descriptor files that
 * spell the tables `const uint16_t g_config_descriptors[]` are unchanged and
 * still match - and a pointer under SDCC. usb.c casts every element before
 * use, so both spellings work there.
 */
#if USB_SDCC
/* SDCC's pic16 port places `const` objects in code space, so the table holds
 * __code pointers; without the qualifier it warns about indirection levels. */
typedef const void __code *usb_desc_addr_t;
#define USB_DESC_ADDR(x) ((const void __code *)(x))
#else
typedef uint16_t usb_desc_addr_t;
#define USB_DESC_ADDR(x) ((uint16_t)(x))
#endif

/* ************************************************************************** */


/* ************************************************************************** */
/* ***************************** INTERRUPTS ********************************* */
/* ************************************************************************** */

/*
 * USB_ISR        - single interrupt vector (IPEN off).
 * USB_ISR_HIGH   - high priority vector (IPEN on).
 * USB_ISR_LOW    - low priority vector (IPEN on).
 *
 * Used as:  USB_ISR(my_isr) { ... }
 */
#if USB_SDCC
#define USB_ISR(name)      void name(void) __interrupt(1)
#define USB_ISR_HIGH(name) void name(void) __interrupt(1)
#define USB_ISR_LOW(name)  void name(void) __interrupt(2)
#else
#define USB_ISR(name)      void __interrupt()              name(void)
#define USB_ISR_HIGH(name) void __interrupt(high_priority) name(void)
#define USB_ISR_LOW(name)  void __interrupt(low_priority)  name(void)
#endif

/* ************************************************************************** */


/* ************************************************************************** */
/* ****************************** BUILTINS ********************************** */
/* ************************************************************************** */

#if USB_SDCC

/*
 * XC8 spells active-low register bits nXXX, SDCC spells them NOT_XXX. These
 * expand inside the member access (INTCON2bits.nRBPU -> INTCON2bits.NOT_RBPU),
 * so the call sites stay in XC8 spelling.
 */
#define nRBPU  NOT_RBPU
#define nRABPU NOT_RABPU

#ifndef NOP
#define NOP() __asm nop __endasm
#endif

/*
 * XC8 provides cycle-accurate __delay_ms()/__delay_us() built on _XTAL_FREQ.
 * SDCC has no equivalent, so these are calibrated busy loops for the 48 MHz
 * (12 MIPS) USB clock every supported part runs at. They are only used for
 * start-up delays - PLL lock, LED flashing, ADC acquisition - so a few
 * percent of error does not matter. Both round up.
 */
#define USB_DELAY_LOOP(iterations)                       \
    do {                                                 \
        volatile uint16_t _usb_dly = (iterations);       \
        while(_usb_dly--) { NOP(); NOP(); NOP(); }       \
    } while(0)

/* ~12 cycles per loop iteration at 12 MIPS -> 1000 iterations per ms. */
#define __delay_ms(ms)                                   \
    do {                                                 \
        volatile uint16_t _usb_ms = (ms);                \
        while(_usb_ms--) USB_DELAY_LOOP(1000u);          \
    } while(0)

#define __delay_us(us) USB_DELAY_LOOP((uint16_t)(us))

#endif /* USB_SDCC */

/* ************************************************************************** */

#endif
