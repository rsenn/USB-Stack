/**
 * @file usb_app.c
 * @brief Contains application specific functions.
 * @author John Izzard
 * @date 05/06/2020
 * 
 * USB uC - USB Application file (This file is for the HID Mouse Example).
 * Copyright (C) 2017-2020  John Izzard
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
#include <stdbool.h>
#include "../../../../USB/usb.h"
#include "../../../../USB/usb_app.h"
#include "../../../../USB/usb_hid.h"


bool usb_service_class_request(void)
{
    return hid_class_request();
}


bool usb_get_class_descriptor(void)
{
    return hid_get_class_descriptor();
}


void usb_app_init(void)
{
    hid_init();
}


void usb_app_tasks(void)
{
    if(TRANSACTION_EP == HID_EP) hid_tasks();
}


void usb_app_clear_halt(uint8_t bd_table_index, uint8_t ep, uint8_t dir)
{
    hid_clear_halt(bd_table_index, ep, dir);
}


bool usb_app_set_interface(uint8_t alternate_setting, uint8_t interface)
{
#if NUM_ALT_INTERFACES != 0
    if(g_set_interface.Interface < NUM_INTERFACES) return false;
#else
    if(alternate_setting == 0 && interface == 0)
    {
        hid_clear_ep_toggle();
        return true;
    }
    else return false;
#endif

}


bool usb_app_get_interface(uint8_t* alternate_setting_result, uint8_t interface)
{
#if NUM_ALT_INTERFACES != 0
    if(g_get_interface.Interface < NUM_INTERFACES) return false;
#endif

    return false;
}


bool usb_out_control_finished(void)
{
    return false;
}
