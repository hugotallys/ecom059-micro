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

.equ clkPin = PINB2
.equ latchPin = PINB1
.equ dataPin = PINB0

.cseg
.org 0

pArr:
	.db	0x32, 0x92, 0x20, \
		0x52, 0x92, 0x05, \
		0x91, 0x91, 0x10, \
		0x92, 0x92, 0x05, \
		0x86, 0x32, 0x20, \
		0x86, 0x52, 0x05, \
		0x86, 0x86, 0x50, \
		0x8A, 0x8A, 0x05

ldi temp, 0b11111111        
out DDRB, temp            ;configura PORTB como saí­da

ldi	R16, LOW(RAMEND)		; load low byte of RAMEND into r16
out	SPL, R16			; store r16 in stack pointer low
ldi	R16, HIGH(RAMEND)	; load high byte of RAMEND into r16
out	SPH, R16			; store r16 in stack pointer high

rcall resetState
rcall setState

loop:
  rcall setState							; Infinite loop
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