# USB-HID-Device

This is an ongoing projext to build a USB HID device based around the STM32F407VG-DISC1 Discovery board.

The motivation behind this project is to gain the ability to build Native USB applications i.e. without using something such as a USB to UART converter and also to 
obtain a deeper understanding of a complex communication protocol such as USB.

This project also gives good experience in developing modular and maintainable firmware

The firmeare architechure is as follows

          Driver Layer => Framework Layer => Application Layer
          
   The Driver Layer handles the low level configuration of the USB Core.
   
   The Framework Layer uses functions from the driver layer to implement protocol specific logic
   
   The application layer is the layer that simply calls functions from the framework layer to do things such as send data. 
   
   This is still a work in progress
