/*
 * usbd_driver.c
 *
 *  Created on: May 21, 2022
 *      Author: user
 */
#include "usbd_driver.h"
#include "usb_standards.h"
#include "stdint.h"

static void initialize_gpio_pins()
{

	//enable clock to GPIOA SINCE DP->PA12, DM->PA11
		SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);

	//configure the Alternate Function Register
		MODIFY_REG(GPIOA->AFR[1],
					GPIO_AFRH_AFSEL12 | GPIO_AFRH_AFSEL11,
					_VAL2FLD(GPIO_AFRH_AFSEL12, 0xA) | _VAL2FLD(GPIO_AFRH_AFSEL11, 0xA)
				);
	//configure the PA12 and PA11 to work in Alternate function Mode
		//GPIOA->MODER
		MODIFY_REG(GPIOA->MODER,
				GPIO_MODER_MODE12 | GPIO_MODER_MODE11 ,
				_VAL2FLD(GPIO_MODER_MODE12,2) | _VAL2FLD(GPIO_MODER_MODE11,2)
				);
}
static void initialize_core()
{
	//Enable clock for USB core
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_OTGFSEN);
	//configures the USB core to run in device mode and to use the embedded full-speed PHY and set turn around time
	MODIFY_REG(USB_OTG_FS->GUSBCFG,
			USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_TRDT,
			USB_OTG_GUSBCFG_FDMOD | _VAL2FLD(USB_OTG_GUSBCFG_TRDT, 0x06)
			);
	//configure the device to run in full speed mode
	MODIFY_REG(USB_OTG_FS_DEVICE->DCFG,
			USB_OTG_DCFG_DSPD,
			_VAL2FLD(USB_OTG_DCFG_DSPD, 3)
			);

    //Enable VBUS sensing
	SET_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_VBUSBSEN);

	//configure the core interrupts
	MODIFY_REG(USB_OTG_FS->GINTMSK,
			USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_SOFM | USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM |
			USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_OEPINT,
			USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_SOFM | USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM |
			USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_OEPINT
			);
	//clear all pending CORE interrupts
	WRITE_REG(USB_OTG_FS->GINTSTS, 0xFFFFFFFF);

	//Unmask USB global interrupts
	SET_BIT(USB_OTG_FS->GAHBCFG, USB_OTG_GAHBCFG_GINT);

	//Unmask Transfer Completed Interrupts
	SET_BIT(USB_OTG_FS_DEVICE->DOEPMSK, USB_OTG_DOEPMSK_XFRCM);
	SET_BIT(USB_OTG_FS_DEVICE->DIEPMSK, USB_OTG_DIEPMSK_XFRCM);
}
/*
 * brief: connects device to the bus
 */
static void connect()
{
	//Power on the transceivers
	SET_BIT(USB_OTG_FS->GCCFG,USB_OTG_GCCFG_PWRDWN);

	//Connect device to the bus
	CLEAR_BIT(USB_OTG_FS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);
}

/*
 * brief disconnects the device from the bus
 */
static void disconnect()
{

	//disconnect device feom bus
	SET_BIT(USB_OTG_FS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);

	//Power off the transceivers
	CLEAR_BIT(USB_OTG_FS->GCCFG,USB_OTG_GCCFG_PWRDWN);
}
/** \brief Pops data from the RxFIFO and stores it in the buffer.
 * \param buffer Pointer to the buffer, in which the popped data will be stored.
 * \param size Count of bytes to be popped from the dedicated RxFIFO memory.
 */
static void read_packet(void *buffer, uint16_t size)
{
	//NOTE: There is only one RXFIFO
	uint32_t *fifo = FIFO(0);
	for(; size>=4; size -=4, buffer += 4)
	{
		//Pop one 32-bit piece of data from the FIFO until none remains
		uint32_t data = *fifo;

		//store piece of data in the buffer
		*((uint32_t*)buffer) = data;
	}
	if (size > 0)
	{
		//pop the last remaining bits of data that are less than one word in size
		uint32_t data = *fifo;

		for(; size >0; size-- , buffer++, data>>=8) // shift to the right by one byte
		{
			//store the data in the buffer after correct alignment
			*((uint8_t*)buffer)= 0xFF & data;

		}
	}
}


/** \brief Pushes a packet into the TxFIFO of an IN endpoint.
 * \param endpoint_number The number of the endpoint, to which the data will be written.
 * \param buffer Pointer to the buffer contains the data to be written to the endpoint.
 * \param size The size of data to be written in bytes.
 */
static void write_packet(uint8_t endpoint_number , void const *buffer, uint16_t size)
{
	uint32_t *fifo = FIFO(endpoint_number); //store a pointer to the address representig FIFO for a particular endpoint
	USB_OTG_INEndpointTypeDef *in_endpoint = IN_ENDPOINT(endpoint_number);

	//Configure the transmission (1 Packet of size bytes)
	MODIFY_REF(in_endpoint->DIEPTSIZ,
			USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
			_VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, size)
			);

	//Enable transmission after clearing NAK and STALL bits
	MODIFY_REG(in_endpoint->DIEPCTL,
			USB_OTG_DIEPCTL_STALL,
			USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA
			);

	//Get size in terms of 32 bit words (to avoid integer overflow in the loop)
	size = (size+3)/4;
	for(; size >0; size--, buffer +=4)
	{
		//Pushes the data to the TxFIFO
		*fifo = *((uint32_t *)buffer); //derefencing the pointer to write to the address of the TXFIFO
	}

}
static void reconfigure_fifo_start_addresses()
{
	/*
	 * Note RxFIFO will always start at address Zero
	 */

	//First changeable start address begins after the region of the RXFIFO
	uint16_t start_address = _FLD2VAL(USB_OTG_GRXFSIZ_RXFD,USB_OTG_FS->GRXFSIZ);

	//Updates the start address of the TXFIFO0
	MODIFY_REG(USB_OTG_FS->DIEPTXF0_HNPTXFSIZ,
			USB_OTG_TX0FSA,
			_VAL2FLD(USB_OTG_TX0FSA,start_address)
			);

	//the next start address is where TXFIFO0 ends
	start_address += _FLD2VAL(USB_OTG_TX0FD,USB_OTG_FS->DIEPTXF0_HNPTXFSIZ ) * 4;

	//updates the start address for the next TXFIFOs
	for (uint8_t tx_fifo_number = 0; tx_fifo_number < ENDPOINT_COUNT - 1; tx_fifo_number++)
	{
		MODIFY_REG(USB_OTG_FS->DIEPTXF[tx_fifo_number],
						USB_OTG_DIEPTXF_INEPTXSA,
						_VAL2LFD(USB_OTG_DIEPTXF_INEPTXSA,start_address)
				);


		start_address += _FLD2VAL(USB_OTG_DIEPTXF_INEPTXFD, USB_OTG_FS->DIEPTXF[tx_fifo_number]) * 4;
	}
}
static void configure_rxfifo_size(uint16_t size)
{
	size = 10+ (2*((size/4)+1)); //allocation 10 locations for setup packets as well


	MODIFY_REG(USB_OTG_FS->GRXFSIZ,
			USB_OTG_GRXFSIZ_RXFD,
			_VAL2FLD(USB_OTG_GRXFSIZ_RXFD, size)
			);

	reconfigure_fifo_start_addresses();
}
/* configures the TXFIFO of an IN endpoint
 * param endpoint number for the end point we would like to configure
 * param size of endpoint in terms of no_of packets
 * NOTE: Any change on any FIFO will update the registers of all TxFIFOs
 * to adapt to the start offsets
 */
static void configure_txfifo_size(uint8_t endpointnumber, uint16_t size)
{
	size = (size+3)/4; //get FIFO size in terms of 32 bit words

	//configure FIFO depth
	if(endpointnumber == 0)
	{

		MODIFY_REG(USB_OTG_FS->DIEPTXF0_HNPTXFSIZ,
				USB_OTG_TX0FD,
				_VAL2FLD(USB_OTG_TX0FD,size)
				);
	}

	else
	{

		MODIFY_REG(USB_OTG_FS->DIEPTXF[endpointnumber-1],
				USB_OTG_DIEPTXF_INEPTXFD,
				_VAL2LFD(USB_OTG_DIEPTXF_INEPTXFD,size)
		);
	}
	reconfigure_fifo_start_addresses();
}
/*
 * BRIEF: flushes the RXFIFO
 */
static void flush_rxfifo()
{

	SET_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH); //flush RXFIFO
}
/*
 * Brief flushes the FIFO for a particular endpoint
 */
static void flush_txfifo(uint8_t endPointNumber)
{
	//sets the number of the TXFIFO to be flushed and flush it
	MODIFY_REG(USB_OTG_FS->GRSTCTL,
			USB_OTG_GRSTCTL_TXFNUM ,
			_VAL2FLD(USB_OTG_DIEPCTL_TXFNUM, endPointNumber) | USB_OTG_GRSTCTL_TXFFLSH
			);
}

/*
 *Configure end-point 0
 */
static void configure_endpoint0(uint16_t endpointSize)
{
	//Unmask the Endpoint 0 interrupts for both IN and OUT
	SET_BIT(USB_OTG_FS_DEVICE->DAINTMSK, 1<< 0 | 1<<16);

	//configures maximum packet size, activates the endpoint and NAK the endpoint (to indicate that we are not yet ready to send data

	MODIFY_REG(IN_ENDPOINT(0)->DIEPCTL,
			USB_OTG_DIEPCTL_MPSIZ,
			_VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, endpointSize) | USB_OTG_DIEPCTL_USBAEP | USB_OTG_DIEPCTL_SNAK
		);

	//clears the NAK and enables endpoint Data transmission
	SET_BIT(OUT_ENDPOINT(0)->DOEPCTL,
			USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK
	);

	//64 Bytes is the maximum packet size for USB full-speed devices
	 configure_rxfifo_size(64);
	 configure_txfifo_size(0, endpointSize);
}

static void configure_in_endpoint(uint8_t endpointNumber, UsbEndpointType usbEndPointType, uint16_t endpointSize)
{
	//Unmask all interrupts for the targetted IN interrupt
	SET_BIT(USB_OTG_FS_DEVICE->DAINTMSK, 1 << endpointNumber);

	//Configures maximum packet size, activates the endpoint and NAK the endpoint (to inficate that we are not yet ready to send data)
	//SET Data0 PID
	//Assign a TxFIFO
	MODIFY_REG(IN_ENDPOINT(endpointNumber)->DIEPCTL,
			USB_OTG_DIEPCTL_MPSIZ | USB_OTG_DIEPCTL_EPTYP | USB_OTG_DIEPCTL_TXFNUM,
			_VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, endpointSize) | USB_OTG_DIEPCTL_USBAEP | _VAL2FLD(USB_OTG_DIEPCTL_TXFNUM,endpointNumber)
			| USB_OTG_DIEPCTL_SNAK | _VAL2FLD(USB_OTG_DIEPCTL_EPTYP, usbEndPointType) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM
	);

	//clears the NAK and enables endpoint Data transmission
	SET_BIT(OUT_ENDPOINT(endpointNumber)->DOEPCTL,
				USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK
		);

	configure_txfifo_size(endpointNumber,endpointSize);
}

static void deconfigure_endpoint(uint8_t endPointNumber)
{
	USB_OTG_INEndpointTypeDef *InEndpoint = IN_ENDPOINT(endPointNumber);
	USB_OTG_OUTEndpointTypeDef *OutEndpoint = OUT_ENDPOINT(endPointNumber);

	//clear all interrupts for the endpoint
	// Masks all interrupts of the targeted IN and OUT endpoints.
		CLEAR_BIT(USB_OTG_FS_DEVICE->DAINTMSK,
			(1 << endPointNumber) | (1 << 16 << endPointNumber)
		);

		// Clears all interrupts of the endpoint.
		SET_BIT(InEndpoint->DIEPINT, 0x28FB);
	    SET_BIT(OutEndpoint->DOEPINT, 0x313B);

		// Disables the endpoints if possible.
	    if (InEndpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
	    {
			// Disables endpoint transmission.
			SET_BIT(InEndpoint->DIEPCTL, USB_OTG_DIEPCTL_EPDIS);
	    }

		// Deactivates the endpoint.
		CLEAR_BIT(InEndpoint->DIEPCTL, USB_OTG_DIEPCTL_USBAEP);

	    if (endPointNumber != 0)
	    {
			if (OutEndpoint->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
			{
				// Disables endpoint transmission.
				SET_BIT(OutEndpoint->DOEPCTL, USB_OTG_DOEPCTL_EPDIS);
			}

			// Deactivates the endpoint.
			CLEAR_BIT(OutEndpoint->DOEPCTL, USB_OTG_DOEPCTL_USBAEP);

			 flush_txfifo(endPointNumber);
			 flush_rxfifo();
}

}



/*
 * USB RESET handle
 */
static void usbrst_handler()
{
	log_info("USB Reset signal detected!");
	for(uint8_t i=0; i<=ENDPOINT_COUNT; i++)
	{
			deconfigure_endpoint(i);
	}
}

static void enumdne_handler()
{
	log_info("USB Speed Enumeration Done!");
	configure_endpoint0(8);
}
static void rxflvl_handler()
{
	//pops status information from the RXFIFO
	uint32_t status = USB_OTG_FS_GLOBAL->GRXSTSP;

	//End-point Number that data was received
	uint8_t endpoint_number = _FLD2VAL(USB_OTG_GRXSTSP_EPNUM, status);
	//number of bytes of received data
	uint16_t byte_count = _FLD2VAL(USB_OTG_GRXSTSP_BCNT, status);
	//received packet status
	uint16_t packet_status = _FLD2VAL(USB_OTG_GRXSTSP_PKTSTS,status);

	switch(packet_status)
	{
		case 0x06:  //SETUP packet (includes data)
		//ToDo
			break;
		case 0x02: //OUT packet (includes data)
		//ToDo
			break;
		case 0x04: //SETUP stage has completed
		//Reenables data transimission on the endpoint
		SET_BIT(OUT_ENDPOINT(endpoint_number)->DOEPCTL, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);
			break;
		case 0x03: //OUT transfer stage has completed
		//Re-enables data transmission on the endpoint
		SET_BIT(OUT_ENDPOINT(endpoint_number)->DOEPCTL, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);
			break;
	}
}
/*
 *
 * handles the USB core interrupts
 */
static void gintsts_handler()
{
	volatile uint32_t gintsts = USB_OTG_FS_GLOBAL->GINTSTS; //volatile keyword since the value of this register will be changing

	if(gintsts & USB_OTG_GINTSTS_USBRST)
	{
		usbrst_handler();
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS,USB_OTG_GINTSTS_USBRST);//clear the interrupt once you are finished
	}
	else if(gintsts & USB_OTG_GINTSTS_ENUMDNE)
	{
		enumdne_handler();
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS,USB_OTG_GINTSTS_ENUMDNE); //clear the interrupt once you are finished
	}
	else if(gintsts & USB_OTG_GINTSTS_IEPINT)
	{
		//do your interrupt stuff here
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS,USB_OTG_GINTSTS_IEPINT); //clear the interrupt once you are finished
	}
	else if(gintsts & USB_OTG_GINTSTS_RXFLVL)
	{
		rxflvl_handler();
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS,USB_OTG_GINTSTS_RXFLVL); //clear the interrupt once you are finished
	}
	else if(gintsts & USB_OTG_GINTSTS_OEPINT)
	{
		//do your interrupt stuff here
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS,USB_OTG_GINTSTS_OEPINT); //clear the interrupt once you are finished
	}
}
 const USBDriver usb_driver = {
		 .initialize_core = &initialize_core,
		 .initialize_gpio_pins = &initialize_gpio_pins,
		 .connect = &connect,
		 .disconnect = &disconnect,
		 .flush_rxfifo = &flush_rxfifo,
		 .flush_txfifo = &flush_txfifo,
		 .configure_in_endpoint = &configure_in_endpoint,
		 .read_packet = &read_packet,
		 .write_packet = &write_packet,
		 .poll = &gintsts_handler
 };

