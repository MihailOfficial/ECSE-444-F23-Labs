
// unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label asmMax, which is expected by lab1math.h
.global asmSquareRoot

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata

precision: .float 0.0001

@ We consulted with ChatGPT for general structure

asmSquareRoot:
  PUSH {R4}			// saving R4 according to calling convention
  ldr r4, =precision
  VLDR.f32 S0, [R0]		//loading n (input)
  VLDR.f32 S1, [R4]		//loading l (precision)
  VMOV.f32 S2, S0		//x = n
  VCMP.F32 S2, #0
  VMRS APSR_nzvc, FPSCR
  beq zero
  blt negative

loop_start:
    //S0 = n
    //S1 = l
    //S2 = x
    VDIV.F32 S4, S0, S2    /* s4 = n / x */
    VADD.F32 s5, s2, s4    /* s5 = x + (n / x) */
    VMOV.F32 s6, #2 	   /* store 2 */
    VDIV.F32 s5, s5, s6    /* s5 = 0.5 * (x + (n / x)) */
	VSUB.F32 s7, s5, s2    /* s7 = root - x */
	VABS.F32 s7, s7        /* abs (root - x) */
	VCMP.F32 S7, S1        //abs(root - x) < l
	VMRS APSR_nzvc, FPSCR
	BLT converged
	VMOV.F32 S2, s5         //  s5 is root
	B loop_start

converged:
    VSTR.f32 s5, [R1]
    POP {R4}
    bx lr

zero:
    VSTR.f32 s0, [R1]
    POP {R4}
    bx lr

negative:
    VLDR.F32 s0, =0x7FC00000
	vSTR.f32 s0, [R1]
    POP {R4}
    bx lr
