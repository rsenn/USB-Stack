/**
 * @file main.c
 * @brief Main C file.
 * @author John Izzard
 * @date 29/04/2021
 * 
 * CDC Serial Example.
 * Copyright (C) 2017-2021  John Izzard
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
 * USB uC BOOTLOADER INSTRUCTIONS
 * 
 * 1. SETUP PROJECT
 * Right click on your MPLABX project, and select Properties. 
 * Under XC8 global options, click XC8 linker. In the Option categories dropdown, 
 * select Additional options. In the Codeoffset input, you need to put an 
 * offset of 0x2000. (For PIC16F145X offset is in words, therefore 0x1000).
 * 
 * If you are using the a J Series bootloader:
 * In the Option categories dropdown, select Memory Model. In the ROM ranges 
 * input, you need to put a range starting from the Codeoffset (0x2000) to 1KB from last 
 * byte in flash. e.g. For X7J53, 2000-1FBFF is used. This makes sure your code 
 * isn't placed in the same Flash Page as the Config Words. That area is write 
 * protected.
 * 
 * PIC18FX6J53: 2000-FBFF.
 * PIC18FX7J53: 2000-1FBFF.
 * 
 * 2. DOWNLOAD FROM MPLABX
 * You can get MPLABX to download your code every time you press build. 
 * To set this up, right click on your MPLABX project, and select Properties. 
 * Under Conf: "PROCESSOR", click Building. Check the "Execute this line after 
 * build" box and place in this line of code (use the drive letter or name of 
 * your device depending on OS):
 * 
 * Windows Example: cp ${ImagePath} E:\ 
 *                  **Needs a space following "\".
 * 
 * OSX Example: cp ${ImagePath} /Volumes/PIC18FX7J53
 * 
 * Linux Example: cp ${ImagePath} /media/PIC18FX7J53
 * 
 * 3. START BOOTLOADER
 * If you have previously loaded a program, reset your device or insert the USB 
 * cable whilst holding down the bootloader button. The bootloader LED will 
 * turn on to indicate "bootloader mode" is active. If no program is present, 
 * just insert the USB cable.. Your PIC will now appear as a thumb drive.
 * 
 * 4. READ/ERASE
 * If you've previously loaded a program, PROG_MEM.BIN file will exist on the 
 * drive. You can use this file to view the raw binary of your program using a 
 * hex editor. If you wish to erase your program, just delete this file. After 
 * the erase completes, the bootloader will restart and you can load a new program.
 * 
 * 5. EEPROM READ/WRITE/ERASE
 * For PICs that have EEPROM, a EEPROM.BIN file will also exist on the drive. 
 * This file can be used to view your EEPROM and modify it's values. Open the 
 * file in a hex editor, and modify any values and save the file. You can also 
 * erase all the EEPROM values by deleting this file (the bootloader will restart, 
 * and the file will reappear with blank EEPROM).
 * 
 * 6. DOWNLOAD
 * To program, simply drag and drop your hex file or right click your hex file 
 * and select send to PIC18F25K50 (for example). The bootloader will close and 
 * instantly start running your code. Alternatively, as seen in step two, you 
 * can get MPLABX to download the file automatically after a build.
 * 
 */

#include "../../../USB/usb_compiler.h"
#include <stdint.h>
#include <stdbool.h>
#include "../../../Hardware/fuses.h"
#include "../../../Hardware/config.h"
#include "../../../USB/usb.h"
#include "../../../USB/usb_cdc.h"
#include "morse.h"

/*
 * Morse code demo (replaces the stock Hello World loop), both directions:
 *
 *  - Key -> serial: the board's own bootloader button doubles as a
 *    straight key. Hold it down for a short "dot" or a long "dash",
 *    release between elements/characters/words - the decoder in morse.c
 *    has no fixed WPM, it learns your keying speed from the dot lengths
 *    it sees and adapts as you speed up or slow down. Decoded text
 *    streams out over the CDC serial port as you key it.
 *
 *  - Serial -> LED: any valid Morse character (A-Z, 0-9, or a space for a
 *    word gap) sent to the CDC serial port is played back as dot/dash
 *    blinks on the user LED, at the same speed morse.c has learned from
 *    your own keying above. Anything that isn't valid Morse is dropped.
 */
#define DEBOUNCE_MS 20u

static void example_init(void);
static void flash_led(void);
static USB_ISR(isr);
static void serial_print_string(char* string);
static void serial_echo(void);
static void timer0_init(void);
static uint32_t get_ms_ticks(void);
static void morse_task(void);
static void serial_to_led_task(void);
static void led_sync_task(void);

static bool volatile m_serial_pkt_sent = true;
static bool volatile m_serial_pkt_rcv = false;

/* Written only by the Timer0 ISR, read only via get_ms_ticks() (which
 * briefly disables interrupts) - a bare volatile read here could tear, since
 * a 32-bit read/write is not atomic on an 8-bit core. */
static uint32_t volatile g_ms_ticks = 0;

void main(void)
{
    example_init();
    flash_led();
    morse_init();
    timer0_init();

    usb_init();
    INTCONbits.PEIE = 1;
    USB_INTERRUPT_FLAG = 0;
    USB_INTERRUPT_ENABLE = 1;
    INTCONbits.GIE = 1;

    while(1)
    {
        while(usb_get_state() < STATE_CONFIGURED){} // Pause if not configured or suspended.

        // Morse Code Demo: key -> decoded text out over serial.
        morse_task();

        // Morse Code Demo: text in over serial -> blinked out on the LED.
        serial_to_led_task();
        led_sync_task();

        // Loop-back Example
//        serial_echo();
    }

    return;
}

static USB_ISR(isr)
{
    if(USB_INTERRUPT_ENABLE && USB_INTERRUPT_FLAG)
    {
        usb_tasks();
        USB_INTERRUPT_FLAG = 0;
    }
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
        // 16-bit Timer0, Fosc/4, 1:32 prescale -> 375 counts/ms at the
        // stack's fixed 48MHz system clock (12MHz instruction clock).
        // Reload high byte first, then low - PIC18's buffered 16-bit
        // timer write commits both halves together on the low-byte write.
        TMR0H = 0xFE;
        TMR0L = 0x89;
        INTCONbits.TMR0IF = 0;
        g_ms_ticks++;
    }
}

static void timer0_init(void)
{
    T0CON = 0x04; // 16-bit, internal Fosc/4 clock, 1:32 prescale, timer off
    TMR0H = 0xFE;
    TMR0L = 0x89;
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
    T0CONbits.TMR0ON  = 1;
}

static uint32_t get_ms_ticks(void)
{
    uint32_t t;
    INTCONbits.GIE = 0;
    t = g_ms_ticks;
    INTCONbits.GIE = 1;
    return t;
}

static void morse_task(void)
{
    static uint8_t  candidate_down  = 0;
    static uint8_t  stable_down     = 0;
    static uint32_t candidate_since = 0;

    uint32_t now      = get_ms_ticks();
    uint8_t  raw_down = BUTTON_PRESSED ? 1u : 0u;

    if(raw_down != candidate_down)
    {
        candidate_down  = raw_down;
        candidate_since = now;
    }
    else if(candidate_down != stable_down && (now - candidate_since) >= DEBOUNCE_MS)
    {
        stable_down = candidate_down;
        morse_key_edge(stable_down, now);
    }

    morse_poll(now);

    while(morse_available())
    {
        char text[2];
        text[0] = morse_getch();
        text[1] = '\0';
        serial_print_string(text);
    }
}

/* Drains whatever the host just sent over CDC into the LED-playback queue,
 * then re-arms the OUT endpoint for the next packet. cdc_data_out() (below)
 * just raises m_serial_pkt_rcv - the received bytes themselves are already
 * sitting in g_cdc_dat_ep_out[0..g_cdc_num_data_out-1] by the time it fires. */
static void serial_to_led_task(void)
{
    if(m_serial_pkt_rcv)
    {
        uint8_t i;
        for(i = 0; i < g_cdc_num_data_out; i++)
        {
            morse_led_send((char)g_cdc_dat_ep_out[i]);
        }
        m_serial_pkt_rcv = false;
        cdc_arm_data_ep_out();
    }
}

/* Pumps the LED playback state machine and drives the physical LED to
 * match - only touching LED_ON()/LED_OFF() on an actual change, since
 * morse.c itself never touches hardware (see morse.h). */
static void led_sync_task(void)
{
    static uint8_t led_on = 0;
    uint8_t        want;

    morse_led_task(get_ms_ticks());

    want = morse_led_is_on();
    if(want != led_on)
    {
        led_on = want;
        if(want) LED_ON();
        else     LED_OFF();
    }
}

static void example_init(void)
{
    // Oscillator Settings.
    // PIC16F145X.
    #if defined(_PIC14E)
    #if XTAL_USED == NO_XTAL
    OSCCONbits.IRCF = 0xF;
    #endif
    #if XTAL_USED != MHz_12
    OSCCONbits.SPLLMULT = 1;
    #endif
    OSCCONbits.SPLLEN = 1;
    PLL_STARTUP_DELAY();
    #if XTAL_USED == NO_XTAL
    ACTCONbits.ACTSRC = 1;
    ACTCONbits.ACTEN = 1;
    #endif

    // PIC18F14K50.
    #elif defined(_18F13K50) || defined(_18F14K50)
    OSCTUNEbits.SPLLEN = 1;
    PLL_STARTUP_DELAY();
    
    // PIC18F2XK50.
    #elif defined(_18F24K50) || defined(_18F25K50) || defined(_18F45K50)
    #if XTAL_USED == NO_XTAL
    OSCCONbits.IRCF = 7;
    #endif
    #if (XTAL_USED != MHz_12)
    OSCTUNEbits.SPLLMULT = 1;
    #endif
    OSCCON2bits.PLLEN = 1;
    PLL_STARTUP_DELAY();
    #if XTAL_USED == NO_XTAL
    ACTCONbits.ACTSRC = 1;
    ACTCONbits.ACTEN = 1;
    #endif

    // PIC18F2XJ53 and PIC18F4XJ53.
    #elif defined(__J_PART)
    OSCTUNEbits.PLLEN = 1;
    PLL_STARTUP_DELAY();
    #endif

    
    // Make boot pin digital.
    #if defined(BUTTON_ANSEL) 
    BUTTON_ANSEL &= ~(1<<BUTTON_ANSEL_BIT);
    #elif defined(BUTTON_ANCON)
    BUTTON_ANCON |= (1<<BUTTON_ANCON_BIT);
    #endif

    
    // Apply pull-up.
    #ifdef BUTTON_WPU
    #if defined(_PIC14E)
    WPUA = 0;
    #if defined(_16F1459)
    WPUB = 0;
    #endif
    BUTTON_WPU |= (1 << BUTTON_WPU_BIT);
    OPTION_REGbits.nWPUEN = 0;
    
    #elif defined(_18F13K50) || defined(_18F14K50)
    WPUA = 0;
    WPUB = 0;
    BUTTON_WPU |= (1 << BUTTON_WPU_BIT);
    INTCON2bits.nRABPU = 0;
    
    #elif defined(_18F24K50) || defined(_18F25K50) || defined(_18F45K50)
    WPUB = 0;
    TRISE &= 0x7F;
    BUTTON_WPU |= (1 << BUTTON_WPU_BIT);
    INTCON2bits.nRBPU = 0;
    
    #elif defined(_18F26J53) || defined(_18F27J53)
    LATB = 0;
    BUTTON_WPU |= (1 << BUTTON_WPU_BIT);
    BUTTON_RXPU_REG &= ~(1 << BUTTON_RXPU_BIT);
    
    #elif defined(_18F46J53) || defined(_18F47J53)
    LATB = 0;
    LATD = 0;
    LATE = 0;
    BUTTON_WPU |= (1 << BUTTON_WPU_BIT);
    BUTTON_RXPU_REG &= ~(1 << BUTTON_RXPU_BIT);
    #endif
    #endif
    
    LED_OFF();
    LED_OUPUT();
}

static void flash_led(void)
{
    for(uint8_t i = 0; i < 3; i++)
    {
        LED_ON();
        __delay_ms(500);
        LED_OFF();
        __delay_ms(500);
    }
}

void cdc_set_control_line_state(void)
{

}

void cdc_set_line_coding(void)
{
    
}

void cdc_data_out(void)
{
    m_serial_pkt_rcv = true;
}

void cdc_data_in(void)
{
    m_serial_pkt_sent = true;
}

void cdc_notification(void)
{

}

static void serial_print_string(char* string)
{
    uint8_t i = 0;
    while(*string)
    {
        while(!m_serial_pkt_sent){}
        g_cdc_dat_ep_in[i++] = *string++;
        if(i == CDC_DAT_EP_SIZE)
        {
            m_serial_pkt_sent = false;
            cdc_arm_data_ep_in(CDC_DAT_EP_SIZE);
            i = 0;
        }
    }
    if(i)
    {
        while(!m_serial_pkt_sent){}
        m_serial_pkt_sent = false;
        cdc_arm_data_ep_in(i);
    }
}

static void serial_echo(void)
{
    if(m_serial_pkt_rcv && m_serial_pkt_sent)
    {
        usb_ram_copy(g_cdc_dat_ep_out, g_cdc_dat_ep_in, g_cdc_num_data_out);
        m_serial_pkt_sent = false;
        cdc_arm_data_ep_in(g_cdc_num_data_out);
        m_serial_pkt_rcv = false;
        cdc_arm_data_ep_out();
    }
}
