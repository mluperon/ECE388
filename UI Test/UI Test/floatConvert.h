/*
 * floatConvert.h
 *
 * Created: 10/24/2019 1:08:22 PM
 *  Author: dylma
 */ 

#ifndef FLOATCONVERT_H_
#define FLOATCONVERT_H_
void ftoa(int n, char* res)
{
	char numbers[11] = "0123456789";
	int power = 0;

	if (n < 10)
	power = 0;
	else if (n < 100)
	power = 1;
	else
	power = 2;
	switch (power)
	{
		case 0: // tenth decimal place
		res[0] = '0';
		res[1] = '0';
		res[2] = '.';
		res[3] = numbers[n];
		res[4] = '\0';

		break;
		
		
		case 1:
		res[0] = '0';
		res[1] = numbers[n/10];
		res[2] = '.';
		res[3] = numbers[n % 10];
		res[4] = '\0';

		break;
		
		
		case 2:
		res[0] = numbers[n / 100];
		res[1] = numbers[(n / 10) % 10];
		res[2] = '.';
		res[3] = numbers[(n % 100) % 10];
		res[4] = '\0';
		break;
		
	}

}
#endif /* FLOATCONVERT_H_ */