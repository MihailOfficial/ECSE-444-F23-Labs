/*
 * cmax.c
 *
 *  Created on: Sep 7, 2023
 *      Author: mihailcalitoiu
 */
#include "main.h"
#include "arm_math.h"

int cFunctionSolver(float *x, float omega, float phi, int max_iterations) {
    // Initial guess for x (can be tuned for different cases)
    float res = *x;
    float epsilon = 0.0001f;

    for (int i = 0; i < max_iterations; i++) {
        float cos_val = arm_cos_f32(omega * res + phi);
        float FX = res * res - cos_val;
        float FXd = 2 * res + omega * arm_sin_f32(omega * res + phi);

        // Check if the solution is within the desired precision
        if (fabs(FX/FXd) < epsilon) {
        	*x = res; // set x to found root
        	return;
        }

        res = res - FX / FXd;
    }

    *x = NAN; // X is NaN since no root is found within max_iterations
    return;
}

