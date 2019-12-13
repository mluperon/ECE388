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
	
	res[0] = numbers[n / 10];
	res[1] = numbers[n % 10];
	res[2] = '\0';
// 	int power = 0;
// 
// 	if (n < 10)
// 	power = 0;
// 	else if (n < 100)
// 	power = 1;
// 	else
// 	power = 2;
// 	switch (power)
// 	{
// 		case 0: // tenth decimal place
// 		res[0] = '0';
// 		res[1] = '0';
// 		res[2] = '.';
// 		res[3] = numbers[n];
// 		res[4] = '\0';
// 
// 		break;
// 		
// 		
// 		case 1:
// 		res[0] = '0';
// 		res[1] = numbers[n/10];
// 		res[2] = '.';
// 		res[3] = numbers[n % 10];
// 		res[4] = '\0';
// 
// 		break;
// 		
// 		
// 		case 2:
// 		res[0] = numbers[n / 100];
// 		res[1] = numbers[(n / 10) % 10];
// 		res[2] = '.';
// 		res[3] = numbers[(n % 100) % 10];
// 		res[4] = '\0';
// 		break;
// 		
// 	}
}

// 65536 max
void itos(int value, char* result)
{
	char numbers[11] = "0123456789";
	result[0] = numbers[(value / 10000)];
	result[1] = numbers[(value % 10000) / 1000];
	result[2] = numbers[(value % 1000) / 100];
	result[3] = numbers[(value % 100) / 10];
	result[4] = numbers[(value %10)];
	result[5] = '\0';
}


#endif /* FLOATCONVERT_H_ */