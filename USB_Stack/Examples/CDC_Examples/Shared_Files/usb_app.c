/**
 * @file usb_app.c
 * @brief Contains application specific functions.
 * @author John Izzard
 * @date 29/04/2021
 * 
 * USB uC - USB Application file (This file is for the CDC Examples).
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
#include <stdbool.h>
#include "../../../USB/usb.h"
#include "../../../USB/usb_app.h"
#include "../../../USB/usb_cdc.h"


bool usb_service_class_request(void)
{
    return cdc_class_request();
}


bool usb_get_class_descriptor(void)
{
    return false;
}


void usb_app_init(void)
{
    cdc_init();
}


void usb_app_tasks(void)
{
    cdc_tasks();
}


void usb_app_clear_halt(uint8_t bd_table_index, uint8_t ep, uint8_t dir)
{
    g_usb_ep_stat[ep][dir].Halt = 0;
    g_usb_ep_stat[ep][dir].Data_Toggle_Val = 0;
    g_usb_bd_table[bd_table_index].STAT = 0;
}


bool usb_app_set_interface(uint8_t alternate_setting, uint8_t interface)
{
#if NUM_ALT_INTERFACES != 0
    if(g_set_interface.Interface < NUM_INTERFACES) return false;
#else
    if(alternate_setting == 0 && interface == 0)
    {
        CDC_COM_EP_IN_DATA_TOGGLE_VAL = 0;
        return true;
    }
    else if(alternate_setting == 0 && interface == 1)
    {
        CDC_DAT_EP_OUT_DATA_TOGGLE_VAL = 0;
        CDC_DAT_EP_IN_DATA_TOGGLE_VAL = 0;
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
    return cdc_out_control_tasks();
}
