/*
 * usbd_framework.h
 * USB Device Framework Layer
 *
 *  Created on: May 21, 2022
 *      Author: Oki Agaya Okwiri
 */

#ifndef USBD_FRAMEWORK_H_
#define USBD_FRAMEWORK_H_
#include "stm32f4xx.h"
#include "usbd_driver.h"

void usbd_initialize();
void usbd_poll();

#endif /* USBD_FRAMEWORK_H_ */
