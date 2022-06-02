#include "stm32f4xx.h"
#include <stdint.h>
#include "logger.h"
#include "usbd_framework.h"
int main(void)
{
    /* Loop forever */

	log_info("Program Entry Point");
	usbd_initialize();
	for(;;);

}
