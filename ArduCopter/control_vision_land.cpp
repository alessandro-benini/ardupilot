/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */

void Copter::vision_land_setup_uart(const char *name)
{
    hal.uartE->begin(115200,256,256);
}

void Copter::vision_land_print_uart(const char *str)
{
    hal.uartE->printf("%s",str);
}

void Copter::vision_land_read_packet()
{
	char c = 0;
	char buf[250];
	unsigned int i = 0;

	vision_land_print_uart("Message @ 25 Hz\n");

//	// hal.uartE->available() > 0
//	while(c != '\n' && i < sizeof(buf))
//	{
//	      c = hal.uartE->read();
//	      buf[i++] = c;
//	}
//
//	vision_land_print_uart("\nReceived message:");
//	vision_land_print_uart(buf);
}

bool Copter::vision_land_init(bool ignore_checks)
{
	vision_land_setup_uart("VISION_DATA");
    return true;
}

void Copter::vision_land_run()
{

}
