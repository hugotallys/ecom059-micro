/*

State	|         Binary		| Hexadecimal |  Time   |
-----	| ---------------------	| ----------- | ------- |
10000	| 001 100 10 100 100 10	|    32 92    |  20.13  |
20000	| 010 100 10 100 100 10	|    52 92    |  04.22  |
00001	| 100 100 01 100 100 01	|    91 91    |  12.99  |
00000	| 100 100 10 100 100 10	|    92 92    |  05.94  |
01100	| 100 001 10 001 100 10	|    86 32    |  20.25  |
01200	| 100 001 10 010 100 10	|    86 52    |  03.51  |
01010	| 100 001 10 100 001 10	|    86 86    |  53.42  |
02020	| 100 010 10 100 010 10	|    8A 8A    |  03.70  |

*/

.def ledsL = R16		; Define led low register
.def ledsH = R17		; Define led high register
.def timeInt = R18		; Define time interval register
.def currState = R19		; Define loop count register
.def temp = R20
.def stateTimer = R21

.equ clkPin = PINB2
.equ latchPin = PINB1
.equ dataPin = PINB0

.cseg
jmp reset

.org OC1Aaddr
jmp OCI1A_Interrupt

OCI1A_Interrupt:
	push r16
	in r16, SREG
	push r16
	
	inc stateTimer
	cp	stateTimer, timeInt
	brne skip
	rcall setState
	skip:
		pop r16
		out SREG, r16
		pop r16
		reti

reset:
	pArr:	.db	0x32, 0x92, 20, \
			0x52, 0x92, 5, \
			0x91, 0x91, 10, \
			0x92, 0x92, 05, \
			0x86, 0x32, 20, \
			0x86, 0x52, 5, \
			0x86, 0x86, 50, \
			0x8A, 0x8A, 5
	
	; Stack initialization
	ldi	temp, LOW(RAMEND)		; load low byte of RAMEND into r16
	out	SPL, temp			; store r16 in stack pointer low
	ldi	temp, HIGH(RAMEND)	; load high byte of RAMEND into r16
	out	SPH, temp			; store r16 in stack pointer high

	ldi temp, 0b11111111        
	out DDRB, temp            ;configura PORTB como saí­da
	
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

	rcall resetState
	rcall setState

	sei

	loop:
	  ;rcall setState							; Infinite loop
	  rjmp	loop

resetState:
  ldi		currState, 0x00
  ret

setState:
	ldi	ZL, LOW(2*pArr)			; Initialize Z pointer
	ldi ZH, HIGH(2*pArr)		; to pmem array address
	add ZL, currState
	lpm ledsL, Z+						; Load LED LOW value from flash memory
	lpm ledsH, Z+						; Load LED HIGH value from flash memory
	lpm timeInt, Z+						; Load time interval value from fash memory
	ldi stateTimer, 0x00
	subi currState, -3
  
	; For loop
	ldi temp, 1
	lowForLoop:
		cpi	temp, 0b10000000
		breq endLowForLoop
		push temp
		and temp, ledsL
		pop temp
		breq passLow
		sbi PORTB, dataPin
	passLow:
		sbi	PORTB, clkPin
		cbi	PORTB, clkPin
		cbi PORTB, dataPin
		lsl temp
		rjmp lowForLoop
	endLowForLoop:
		ldi temp, 1
	highForLoop:
		cpi	temp, 0b10000000
		breq endHighForLoop
		push temp
		and temp, ledsH
		pop temp
		breq passHigh
		sbi PORTB, dataPin
	passHigh:
		sbi	PORTB, clkPin
		cbi	PORTB, clkPin
		cbi PORTB, dataPin
		lsl temp
		rjmp highForLoop
	endHighForLoop:
	; end For loop

	cbi	PORTB, latchPin
	sbi	PORTB, latchPin
	cbi	PORTB, latchPin

	cpi		currState, 24
	breq	resetState
	ret