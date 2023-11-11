
#include "main.h"
#include "arm_math.h"

void cSquareRoot(float num, float *result) {

	if (num == 0){
		*result = 0;
		return;
	}

	else if (num < 0){
		*result = NAN;
		return;
	}

	float x = num;

	while (1){

		float root = 0.5f * (x + num / x);
		*result =root;

		if (fabs(root - x) < 0.0001){ return; }

		x = root;
	}

	return;
}

