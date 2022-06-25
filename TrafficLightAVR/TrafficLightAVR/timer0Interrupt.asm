/*
 * timer0Interrupt.asm
 *
 *  Created: 24/06/2022 18:09:44
 *   Author: hgtll
 */


.cseg
jmp reset

.org 0x0020
jmp ISR_TOV0

ISR_TOV0:
	push r16
	in r16,SREG
	push r16
	call TOGGLEPIN
	pop r16
	out SREG,r16
	pop r16
	reti

reset:
	ldi r16,(1<<CS02);|(1<<CS00)
	out TCCR0B,r16 ; Timer clock = system clock / 256 ; 1024
	ldi r16,1<<TOV0
	out TIFR0,r16 ; Clear TOV0/ Clear pending interrupts
	ldi r16,1<<TOIE0
	sts TIMSK0,r16 ; Enable Timer/Counter0 Overflow Interrupt
	sei

loop:
	rjmp loop

TOGGLEPIN:
	sbic portb,PORTB5
	rjmp CLEARPIN
	nop
	sbi portb,PORTB5
	jmp RET1
CLEARPIN:
	cbi portb,PORTB5
	RET1:
	ret