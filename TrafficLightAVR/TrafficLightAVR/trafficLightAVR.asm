/*
* Traffic Light - ATMEL AVR
*
* Main File:
*	trafficLightAVR.asm
*
* Authors:
*	Arquimedes
*	Hugo
*	Pamela
*	Sofia
* 
* Project Description :
*	TODO
*
* State |         Binary        | Hexadecimal |  Time   |
* ----- | --------------------- | ----------- | ------- |
* 10000	| 001 100 10 100 100 10	|    32 92    |  20.13  |
* 20000	| 010 100 10 100 100 10	|    52 92    |  04.22  |
* 00001	| 100 100 01 100 100 01	|    91 91    |  12.99  |
* 00000	| 100 100 10 100 100 10	|    92 92    |  05.94  |
* 01100	| 100 001 10 001 100 10	|    86 32    |  20.25  |
* 01200	| 100 001 10 010 100 10	|    86 52    |  03.51  |
* 01010	| 100 001 10 100 001 10	|    86 86    |  53.42  |
* 02020	| 100 010 10 100 010 10	|    8A 8A    |  03.70  |
*
*/

.def ledsL = R16		; Define led low register
.def ledsH = R17		; Define led high register

.def decDigit = R18		; Define time interval register
.def uniDigit = R19

.def decTimer = R20
.def uniTimer = R21

.def currState = R22		; Define loop count register
.def currMsg	= R23
.def temp = R24
.def currDisplay = R25

.equ clkPin = PINB2
.equ latchPin = PINB1
.equ dataPin = PINB0

.equ uniPin = PIND6
.equ decPin = PIND7


.equ input1Pin = PIND2
.equ input2Pin = PIND3
.equ input3Pin = PIND4
.equ input4Pin = PIND5

#define CLOCK 16.0e6 ;clock speed
#define DELAY 1 ;seconds
.equ PRESCALE = 0b100 ;/256 prescale
.equ PRESCALE_DIV = 256
.equ WGM = 0b0100 ;Waveform generation mode: CTC
;you must ensure this value is between 0 and 65535
.equ TOP = int(0.5 + ((CLOCK/PRESCALE_DIV)*DELAY))
.if TOP > 65535
.error "TOP is out of range"
.endif

.cseg
jmp reset

.org OC1Aaddr
jmp OCI1A_Interrupt
.org 0x0020
jmp ISR_TOV0

OCI1A_Interrupt:
	push R16
	in R16, SREG
	push R16
	
	inc uniTimer

	cpi uniTimer, 10
	brne skip1

	ldi uniTimer, 0
	inc decTimer

	skip1:
		cp	uniTimer, uniDigit
		brne skip2
		cp	decTimer, decDigit
		brne skip2
		rcall setState
	skip2:
		pop R16
		out SREG, R16
		pop R16
		reti

ISR_TOV0:
	push r16
	in r16,SREG
	push r16
	
	cpi currDisplay, 0
	brne display1

	sbi PORTD, decPin
	cbi PORTD, uniPin

	mov temp, decTimer
	rcall setDigit
	inc currDisplay
	rjmp endInte

	display1:

	sbi PORTD, uniPin
	cbi PORTD, decPin

	mov temp, uniTimer
	rcall setDigit
	dec currDisplay

	endInte:
	pop r16
	out SREG,r16
	pop r16
	reti

reset:
	; Stack initialization
	ldi	temp, LOW(RAMEND)		; load low byte of RAMEND into temp reg
	out	SPL, temp				; store temp reg in stack pointer low
	ldi	temp, HIGH(RAMEND)		; load high byte of RAMEND into temp reg
	out	SPH, temp				; store temp reg in stack pointer high

	ldi temp, 0b11111111        
	out DDRB, temp				; configura PORTB como saí­da
	out DDRD, temp				; configura PORTD como saída

	; On MEGA series, write high byte of 16-bit timer registers first
	ldi temp, high(TOP) ; initialize compare value (TOP)
	sts OCR1AH, temp
	ldi temp, low(TOP)
	sts OCR1AL, temp
	ldi temp, ((WGM&0b11) << WGM10) ;lower 2 bits of WGM
	; WGM&0b11 = 0b0100 & 0b0011 = 0b0000 
	sts TCCR1A, temp
	; upper 2 bits of WGM and clock select
	ldi temp, ((WGM>> 2) << WGM12)|(PRESCALE << CS10)
	sts TCCR1B, temp ; start counter

	lds R16, TIMSK1
	sbr r16, 1 << OCIE1A
	sts TIMSK1, R16
	
	;timer 0 setup

	ldi r16,(1<<CS02);|(1<<CS00)
	out TCCR0B,r16 ; Timer clock = system clock / 256 ; 1024
	ldi r16,1<<TOV0
	out TIFR0,r16 ; Clear TOV0/ Clear pending interrupts
	ldi r16,1<<TOIE0
	sts TIMSK0,r16 ; Enable Timer/Counter0 Overflow Interrupt

	;**************************************************************
	;* subroutine: initUART
	;*
	;* inputs: r17:r16 - baud rate prescale
	;*
	;* enables UART transmission with 8 data, 1 parity, no stop bit
	;* at input baudrate
	;*
	;* registers modified: r16
	;**************************************************************

	.equ	baud	= 9600			; baudrate
	.equ	bps	= (int(CLOCK)/16/baud) - 1	; baud prescale

	ldi	r16, LOW(bps)			; load baud prescale
	ldi	r17, HIGH(bps)			; into r17:r16

	sts	UBRR0L, r16			; load baud prescale
	sts	UBRR0H, r17			; to UBRR0

	ldi	r16, (1<<RXEN0) | (1<<TXEN0)	; enable transmitter
	sts	UCSR0B, r16			; and receiver

	rcall resetState
	rcall setState

	ldi currDisplay, 0
		
	sei	; Enable global interrupts

	loop:
		; do nothing
		rjmp	loop

;**************************************************************
;* subroutine: puts
;*
;* inputs: ZH:ZL - Program Memory address of string to transmit
;*
;* transmits null terminated string via UART
;*
;* registers modified: r16,r17,r30,r31
;**************************************************************

puts:
	push r16
	push r17

	putsLoop:
		lpm r16, Z+				; load character from pmem
		cpi	r16,0x00				; check if null
		breq	puts_end			; branch if null

		puts_wait:	
			lds	r17,UCSR0A			; load UCSR0A into r17
			sbrs	r17,UDRE0			; wait for empty transmit buffer
			rjmp	puts_wait			; repeat loop

		sts	UDR0,r16			; transmit character
		rjmp	putsLoop				; repeat loop

		puts_end:
			pop r17
			pop r16
			ret					; return from subroutine

setDigit:
	;begin switch
	cpi temp, 0
	breq case0
	cpi temp, 1
	breq case1
	cpi temp, 2
	breq case2
	cpi temp, 3
	breq case3
	cpi temp, 4
	breq case4
	cpi temp, 5
	breq case5
	cpi temp, 6
	breq case6
	cpi temp, 7
	breq case7
	cpi temp, 8
	breq case8
	cpi temp, 9
	breq case9
	rjmp casedefault
	;cases
	case0:
		cbi PORTD, input1Pin
		cbi PORTD, input2Pin
		cbi PORTD, input3Pin
		cbi PORTD, input4Pin
		rjmp end_switch
	case1:
		sbi PORTD, input1Pin
		cbi PORTD, input2Pin
		cbi PORTD, input3Pin
		cbi PORTD, input4Pin
		rjmp end_switch
	case2:
		cbi PORTD, input1Pin
		sbi PORTD, input2Pin
		cbi PORTD, input3Pin
		cbi PORTD, input4Pin
		rjmp end_switch
	case3:
		sbi PORTD, input1Pin
		sbi PORTD, input2Pin
		cbi PORTD, input3Pin
		cbi PORTD, input4Pin
		rjmp end_switch
	case4:
		cbi PORTD, input1Pin
		cbi PORTD, input2Pin
		sbi PORTD, input3Pin
		cbi PORTD, input4Pin
		rjmp end_switch
	case5:
		sbi PORTD, input1Pin
		cbi PORTD, input2Pin
		sbi PORTD, input3Pin
		cbi PORTD, input4Pin
		rjmp end_switch
	case6:
		cbi PORTD, input1Pin
		sbi PORTD, input2Pin
		sbi PORTD, input3Pin
		cbi PORTD, input4Pin
		rjmp end_switch
	case7:
		sbi PORTD, input1Pin
		sbi PORTD, input2Pin
		sbi PORTD, input3Pin
		cbi PORTD, input4Pin
		rjmp end_switch
	case8:
		cbi PORTD, input1Pin
		cbi PORTD, input2Pin
		cbi PORTD, input3Pin
		sbi PORTD, input4Pin
		rjmp end_switch
	case9:
		sbi PORTD, input1Pin
		cbi PORTD, input2Pin
		cbi PORTD, input3Pin
		sbi PORTD, input4Pin
		rjmp end_switch
	casedefault:
		ret
	end_switch:
		ret

resetState:
  ldi		currState, 0x00
  ldi		currMsg, 0x00
  ret

setState:
	ldi	ZL, LOW(2*pArr)			; Initialize Z pointer
	ldi ZH, HIGH(2*pArr)		; to pmem array address
	
	ldi temp, 0
	add ZL, currState
	adc ZH, temp
	
	lpm ledsL, Z+						; Load LED LOW value from flash memory
	lpm ledsH, Z+						; Load LED HIGH value from flash memory
	lpm decDigit, Z+						; Load time interval value from fash memory
	lpm uniDigit, Z+
	ldi decTimer, 0x00
	ldi uniTimer, 0x00
	subi currState, -4
  
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

	//cbi	PORTB, latchPin
	sbi	PORTB, latchPin
	cbi	PORTB, latchPin

	ldi	ZL,LOW(2*myMsg)			; load Z pointer with
	ldi	ZH,HIGH(2*myMsg)		; myStr address
	
	ldi temp, 0
	add ZL, currMsg
	adc ZH, temp

	subi currMsg, -28
	
	rcall puts				; transmit string

	cpi		currState, 32
	breq	resetState
	ret

pArr:	.db	0x32, 0x92, 2, 1, \
				0x52, 0x92, 0, 6, \
				0x91, 0x91, 1, 1, \
				0x92, 0x92, 0, 6, \
				0x86, 0x32, 2, 1, \
				0x86, 0x52, 0, 6, \
				0x86, 0x86, 5, 1, \
				0x8A, 0x8A, 0, 6

myMsg:	.db "| [Semaphore State] 10000 |", 0x00, \
			"| [Semaphore State] 20000 |", 0x00, \
			"| [Semaphore State] 00001 |", 0x00, \
			"| [Semaphore State] 00000 |", 0x00, \
			"| [Semaphore State] 01100 |", 0x00, \
			"| [Semaphore State] 01200 |", 0x00, \
			"| [Semaphore State] 01010 |", 0x00, \
			"| [Semaphore State] 02020 |", 0x00