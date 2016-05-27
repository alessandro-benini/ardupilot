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

	while(hal.uartE->available() > 0 && c != '\n' && i < sizeof(buf)-1)
	{
	      c = hal.uartE->read();
	      buf[i++] = c;
	}

	buf[sizeof(buf)-1] = '\0';

	cliSerial->printf("Received Message: %s\n",buf);

}

bool Copter::vision_land_init(bool ignore_checks)
{
	vision_land_setup_uart("VISION_DATA");
    return true;
}

void Copter::vision_land_run()
{

}
