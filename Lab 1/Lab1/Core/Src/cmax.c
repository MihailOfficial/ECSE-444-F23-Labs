
#include "main.h"

#ifndef SRC_CMAX_C_
#define SRC_CMAX_C_


void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex) {
	(*max) = array[0];
	(*maxIndex) = 0;

	for (uint32_t i = 1; i < size; i++) {
		if (array[i] > (*max)) {
			(*max) = array[i];
			(*maxIndex) = i;
		} // if
	} // for


} // cMax

#endif /* SRC_CMAX_C_ */
