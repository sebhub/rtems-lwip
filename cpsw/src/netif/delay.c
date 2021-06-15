#include <unistd.h>

void delay(unsigned int ms)
{
	usleep(ms*1000);
}