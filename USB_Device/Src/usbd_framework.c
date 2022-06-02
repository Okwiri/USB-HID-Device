/*
 * usbd_framework.c
 *
 *  Created on: May 21, 2022
 *      Author: Oki Agaya Okwiri
 */
#include "usbd_framework.h"
#include "usbd_driver.h"

void usbd_initialize()
{
	//initialize_gpio_pins();
	//initialize_core();
	//connect();
}

void usbd_poll()
{
	usb_driver.poll();
}
