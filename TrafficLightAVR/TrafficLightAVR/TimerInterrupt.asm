; Program 8.1 ? LED Blinker
; Illustrate the use of a Timer/Counter to blink an LED
; LEDBlinker_Timer.asm
;
; Created: 27/02/2017 11:56:54
; Author : Erick
;
;LED's on PORTB
;Clock speed 16 MHz

;Timer 1 é utilizado para definir um intervalo de 0,5 s
;A cada intervalo os LEDs piscam
;4 LEDs conectados a PORTB

.def temp = r16
.def leds = r17 ;current LED value
.cseg

jmp reset
.org OC1Aaddr
jmp OCI1A_Interrupt

OCI1A_Interrupt:
	push r16
	in r16, SREG
	push r16
	
	ldi temp, $FF
	eor leds, temp ;0b10101010 --> 0b01010101
	out PORTB, leds
	
	pop r16
	out SREG, r16
	pop r16
	reti


reset:
	;Stack initialization
	ldi temp, low(RAMEND)
	out SPL, temp
	ldi temp, high(RAMEND)
	out SPH, temp

	;leds display alternating pattern
	ldi temp, $FF
	out DDRB, temp
	ldi leds, $AA ;0b10101010
	out PORTB, leds ;alternating pattern

	#define CLOCK 16.0e6 ;clock speed
	#define DELAY 1.0 ;seconds
	.equ PRESCALE = 0b100 ;/256 prescale
	.equ PRESCALE_DIV = 256
	.equ WGM = 0b0100 ;Waveform generation mode: CTC
	;you must ensure this value is between 0 and 65535
	.equ TOP = int(0.5 + ((CLOCK/PRESCALE_DIV)*DELAY))
	.if TOP > 65535
	.error "TOP is out of range"
	.endif

	;On MEGA series, write high byte of 16-bit timer registers first
	ldi temp, high(TOP) ;initialize compare value (TOP)
	sts OCR1AH, temp
	ldi temp, low(TOP)
	sts OCR1AL, temp
	ldi temp, ((WGM&0b11) << WGM10) ;lower 2 bits of WGM
	; WGM&0b11 = 0b0100 & 0b0011 = 0b0000 
	sts TCCR1A, temp
	;upper 2 bits of WGM and clock select
	ldi temp, ((WGM>> 2) << WGM12)|(PRESCALE << CS10)
	; WGM >> 2 = 0b0100 >> 2 = 0b0001
	; (WGM >> 2) << WGM12 = (0b0001 << 3) = 0b0001000
	; (PRESCALE << CS10) = 0b100 << 0 = 0b100
	; 0b0001000 | 0b100 = 0b0001100
	sts TCCR1B, temp ;start counter

	lds r16, TIMSK1
	sbr r16, 1 <<OCIE1A
	sts TIMSK1, r16

	sei
	main_lp:
			rjmp main_lp