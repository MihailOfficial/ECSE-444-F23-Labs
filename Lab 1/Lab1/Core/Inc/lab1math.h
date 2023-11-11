/*
 * lab1math.h
 *
 *  Created on: Sep 7, 2023
 *      Author: mihailcalitoiu
 */

#ifndef LAB1MATH_H_
#define LAB1MATH_H_

void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

extern void asmSquareRoot(float *input, float *result);
extern void cSquareRoot(float num, float *result);

extern asmFunctionSolver(float *x, float *omega, float *phi, float *max_iterations);
extern void cFunctionSolver(float *x, float omega, float phi, int max_iterations);

#endif /* LAB1MATH_H_ */
