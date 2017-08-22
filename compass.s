@@ compass.s :
@@ A. Bordiuh, G. Yano. Assignment #3 code. Magnetometer

@@ <function block>
    .align  2                           @@ - 2^n alignment (n=2)
    .syntax unified
    .global clearAllLeds                @@ - Symbol name for function
    .code   16                          @@ - 16bit THUMB code (BOTH are required!)
    .thumb_func                         @@ /
    .type   clearAllLeds, %function     @@ - symbol type (not req)

    .equ GPIOE_BSRR, 0x48001018		@@This register is used to set/reset the GPIOE bits
    .equ GPIOE_ODR, 0x48001014		@@This register is used to read the states of the GPIOE

@@ Declaration : void clearAllLeds(void)
clearAllLeds:
    push {lr}
    push {r1}
    push {r0-r7}

    ldr r6, =GPIOE_BSRR			@@ store BSRR to r6
    mov  r0, #0					@@ using movt will cause the lsb of r0 to be loaded with
    movt r0, #0xFF00			@@ 2200, so r0 will be 0x00002200
    str r0,[r6]					@@ loading 0x00002200 to r0 will turn off led3 and led10F

    pop  {r0-r7}
    pop  {r1} 
    pop  {pc}
    .size   clearAllLeds, .-clearAllLeds    @@ - symbol size (not req)
