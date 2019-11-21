/*
 * floatConvert.h
 *
 * Created: 10/24/2019 1:08:22 PM
 *  Author: dylma
 */ 

#ifndef FLOATCONVERT_H_
#define FLOATCONVERT_H_
void ftoa(double n, char* res)
{
	char numbers[11] = "0123456789";
	int power = 0;
	int numConvert = 0;

	if (n < 1)
	power = 0;
	else if (n < 10)
	power = 1;
	else
	power = 2;

	switch (power)
	{
		case 0:
		numConvert = n * 10;
		res[0] = '0';
		res[1] = '0';
		res[2] = '.';
		res[3] = numbers[numConvert];
		res[4] = '\0';
		break;
		case 1:
// 		if (n > 9.7 && n < 9.8) // weird case for 9.8
// 		{
// 			res[0] = '0';
// 			res[1] = '9';
// 			res[2] = '.';
// 			res[3] = '8';
// 			res[4] = '\0';
// 			break;
// 		}
// 		if (n > 9.8 && n < 9.9) // weird case for 9.9
// 		{
// 			res[0] = '0';
// 			res[1] = '9';
// 			res[2] = '.';
// 			res[3] = '9';
// 			res[4] = '\0';
// 			break;
// 		}
// 		if (n > 9.3 && n < 9.4) // weird case for 9.4
// 		{
// 			res[0] = '0';
// 			res[1] = '9';
// 			res[2] = '.';
// 			res[3] = '4';
// 			res[4] = '\0';
// 			break;
// 		}
		numConvert = floor(n);
		res[0] = '0';
		res[1] = numbers[numConvert];
		res[2] = '.';
		numConvert = (fmod(n, numConvert)) * 10;
		res[3] = numbers[numConvert];
		res[4] = '\0';
		break;

		case 2:
		numConvert = floor(n) / 10;
		res[0] = numbers[numConvert];
		numConvert = fmod(floor(n), 10);
		res[1] = numbers[numConvert];
		res[2] = '.';
		numConvert = fmod(n, floor(n)) * 10;
		res[3] = numbers[numConvert];
		res[4] = '\0';
		break;
	}
}
#endif /* FLOATCONVERT_H_ */