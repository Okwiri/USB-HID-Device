/*
 * usbd_driver.h
 *USB DEVICE driver Layer
 *  Created on: May 21, 2022
 *      Author: Oki Agaya Okwiri
 */

#ifndef USBD_DRIVER_H_
#define USBD_DRIVER_H_
#include "stm32f4xx.h"
#include "usb_standards.h"

#define USB_OTG_FS_GLOBAL ((USB_OTG_GlobalTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_GLOBAL_BASE))
#define USB_OTG_FS_DEVICE ((USB_OTG_DeviceTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_OTG_FS_PCGCCTL ((uint32_t *)USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE)) //power and clock control register

/*
 * \brief returns the structure that contains the structure that contains the registers of a specific IN end-point
 *
 */

inline static USB_OTG_INEndpointTypeDef * IN_ENDPOINT(uint8_t endPointNumber)
{
	return ((USB_OTG_INEndpointTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + endPointNumber));
}

/*
 * brief returns the structure that contains the registers for a specific OUT end-point
 */
inline static USB_OTG_OUTEndpointTypeDef * OUT_ENDPOINT(uint8_t endPointNumber)
{
	return ((USB_OTG_OUTEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + endPointNumber));
}

inline static __IO uint32_t *FIFO(uint8_t endPointNumber)
{
	return(__IO uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (endPointNumber * 0x1000));
}

/*
 * define number of IN and OUT endpoints
 */
#define ENDPOINT_COUNT	3

typedef struct
{
	void (*initialize_core)();
	void (*initialize_gpio_pins)();
	void (*connect)();
	void (*disconnect)();
	void (*flush_rxfifo)();
	void (*flush_txfifo)(uint8_t endPointNumber);
	void (*configure_in_endpoint)(uint8_t endpointNumber, UsbEndpointType usbEndPointType, uint16_t endpointSize);
	void (*read_packet)(void *buffer, uint16_t size);
	void (*write_packet)(uint8_t endpoint_number , void const *buffer, uint16_t size);
	void (*poll)();
	//ToDo add other pointers to friver functions
}USBDriver;

extern const USBDriver usb_driver; //extern keyword lets the compiler know that the definition will be done in a difefrent file


#endif /* USBD_DRIVER_H_ */
