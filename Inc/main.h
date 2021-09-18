#ifndef MAIN_H
#define MAIN_H
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "usb_parser.h"
//#include "convert_data.h"
#include <string.h> 


typedef struct {
	uint8_t source_msg;
	uint32_t id;
	char buff_can_msg [27];
}Packet ;



#endif /*MAIN_H*/

