void delay(unsigned int ms)
{
	ms*=80000u;
	while (--ms)
	{
		__asm__("");
	}
}