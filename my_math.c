#include "my_math.h"

/*double my_cos(double input) {
    int i, j, toggle = 1;
    double angle, product, sum = 0.0;
 
    angle = input * MY_PI / 180;
    while (angle >= 2 * MY_PI)
        angle -= 2 * MY_PI;
    while (angle <= -2 * MY_PI)
        angle += 2 * MY_PI;
 
    for (i = 0; i < 10; i++) {
        product = 1.0;
        for (j = (2*i); j > 0 ; j--)
            product *= angle / j;
 
        sum +=  product * toggle;
        toggle = -toggle;
    }
 
    return sum;
}

double my_sin(double input) {
    return my_cos(input + 270);
}*/

double my_cos(double input) {
	int i, j, toggle = 1;
	double angle, product, sum = 0.0;

	angle = input;
	while(angle >= 2*MY_PI) {
		angle -= 2*MY_PI;
	}
	while(angle <= -2*MY_PI) {
		angle += 2*MY_PI;
	}

	for(i = 0; i < 50; i++) {
		product = 1.0;
		for(j = 2*i; j > 0; j--) {
			product *= angle / j;
		}

		sum += product * toggle;
		toggle = -toggle;
	}

	return sum;
}

double my_sin(double input) {
	while(input >= 2*MY_PI) {
		input -= 2*MY_PI;
	}
	while(input <= 2*MY_PI) {
		input += 2*MY_PI;
	}

	return -my_cos(input + MY_PI / 2);
}