
// unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label asmMax, which is expected by lab1math.h
.global asmFunctionSolver

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata

.extern arm_cos_f32
.extern arm_sin_f32

epsilon: .float 0.0001

@ We consulted with ChatGPT for general structure

asmFunctionSolver:
	PUSH {R4-R11, LR}
    @ Input parameters:
    @ r0 = Result pointer (output)
    @ r1 = Omega
    @ r2 = Phi
    @ r3 = Max iterations

	ldr r4, =epsilon
    vldr.f32 s5, [r4] 		@ Epsilon
    vmov.f32 s11, #1        @ Initialize iteration counter
    vldr.f32 s6, [r0]   	@ Initialize res
    vmov.f32 s7, #2 		@ Static 2
    vmov.f32 s8, #1 		@ Static 1
    vldr.f32 s10, [r3]		@ load value of max iterations to r6
    vldr.f32 s1, [r1]       @ Load omega
    vldr.f32 s9, [r2]   	@ Load phi


loop:
    @ Calculate f(x) = x^2 - cos(ωx + φ)
    vmul.f32 s3, s6, s1 @ Calculate ωx

    vadd.f32 s3, s3, s9 @ Calculate ωx + φ

    vmov.f32 s0, s3  	@ Set S0 to ωx + φ
    PUSH {LR}
    bl arm_cos_f32      @ Call CMSIS-DSP cosine function
    POP {LR}

	vmul.f32 s2, s6, s6 @ x^2

	vsub.f32 s2, s2, s0 @ FX

	vmov.f32 s0, s3  	@ Set S0 to ωx + φ
	PUSH {LR}
	bl arm_sin_f32      @ Call CMSIS-DSP cosine function
	POP {LR}

	//vldr.f32 s1, [r1]       @ Load omega
	vmul.f32 s3, s0, s1 @ omega * sin val
	vmul.f32 s4, s6, s7 @ 2 * x

	vadd.f32 s3, s3, s4 @ FXd

	vdiv.f32 s2, s2, s3  @ Calculate FX / FXd
	@ Check for convergence or maximum iterations
    vabs.f32 s0, s2		@ |FX / FXd|

    VCMP.F32 S0, s5		@ |FX / FXd| ? epsilon
	VMRS APSR_nzvc, FPSCR

    blt found_root

    vsub.f32 s6, s6, s2  @ x = x - FX / FXd

    @ Update iteration counter
    vadd.f32 s11, s11, s8
    vCMP.f32 s11, s10
    VMRS APSR_nzvc, FPSCR

    bgt max_iterations

    @ Continue looping
    b loop


found_root:
	vSTR.f32 s6, [R0]
	POP {R4-R11, LR}
	bx lr

max_iterations:
    VLDR.F32 s0, =0x7FC00000
	vSTR.f32 s0, [R0]
	POP {R4-R11, LR}
	bx lr



