;*******************************************************
;* Traffic Light AVR - ATmega328P
;*
;* Main File:
;*	trafficLightAVR.asm
;*
;* Authors:
;*	Arquimedes
;*	Hugo
;*	Pamela
;*	Sofia
;* 
;* Project Description :
;*	Firmware implementation of a system of 5 traffic lights.
;* 	Each state represents the current active color of the
;*	traffic light where [0 = Red | 1 = Green | 2 = Yellow].
;*
;* ----- | --------------------- | ----------- | ---- |
;* State |         Binary        | Hexadecimal | Time |
;* ----- | --------------------- | ----------- | ---- |
;* 10000 | 001 100 10 100 100 10 |    32 92    |  20  |
;* 20000 | 010 100 10 100 100 10 |    52 92    |  05  |
;* 00001 | 100 100 01 100 100 01 |    91 91    |  10  |
;* 00000 | 100 100 10 100 100 10 |    92 92    |  05  |
;* 01100 | 100 001 10 001 100 10 |    86 32    |  20  |
;* 01200 | 100 001 10 010 100 10 |    86 52    |  05  |
;* 01010 | 100 001 10 100 001 10 |    86 86    |  50  |
;* 02020 | 100 010 10 100 010 10 |    8A 8A    |  05  |
;*
;*******************************************************

.def	ledsL 		= 	R16		; LEDs low byte register
.def	ledsH 		= 	R17		; LEDs high byte register

.def 	decTimer 	= 	R18		; Timer decimal digit
.def 	uniTimer 	= 	R19		; Timer units digit

.def 	decDigit 	= 	R20		; Time interval decimal digit
.def 	uniDigit 	= 	R21		; Time interval units digit

.def 	currState 	= 	R22		; Current state pointer
.def 	currMsg		= 	R23		; Current message pointer
.def 	currDisplay	=	R24		; Seven segment display active flag
								; 0 = Dec 1 = Uni

.def 	temp 		= 	R25		; Temporary register

;*** Shift register control pins ***
.equ 	clkPin 		= 	PINB2
.equ 	latchPin 	= 	PINB1
.equ 	dataPin 	= 	PINB0

;*** 7Segment Display switch pins ***
.equ 	uniPin 		= 	PIND6
.equ 	decPin 		= 	PIND7

;*** BCD input pins ***
.equ 	input1Pin 	= 	PIND2
.equ 	input2Pin 	= 	PIND3
.equ 	input3Pin 	= 	PIND4
.equ 	input4Pin 	= 	PIND5

;*** Timer1 parameters ***
#define	CLOCK				16.0e6	; Clock speed
#define	DELAY				1		; Delay time in seconds
.equ 	PRESCALE		= 	0b100	; /256 prescale
.equ 	PRESCALE_DIV	=	256
.equ 	WGM				=	0b0100	; Waveform generation mode CTC

; Ensure that TOP value is between 0 and 65535
.equ 	TOP				=	int(0.5 + ((CLOCK/PRESCALE_DIV)*DELAY))
.if 	TOP 			> 	65535
.error	"TOP is out of range"
.endif

;*** USART parameters ***
.equ	baud		=	9600						; Baudrate
.equ	bps			=	(int(CLOCK)/16/baud) - 1	; Baud prescale

;*** Start of Code Segment ***
.cseg	
jmp	RESET
.org 0x0016
jmp ISR_OCI1A
.org 0x0020
jmp ISR_TOV0

;**************************************************************
;* Interrupt Service Routine: ISR_OCI1A
;*
;* Inputs:	R18 - Timer unit digit
;* 			R19 - Timer decimal digit
;*
;* Increments the unit and decimal timer counter and checks if
;* the timer expired in order to set the next state.
;*
;* Registers modified: R18, R19
;**************************************************************
ISR_OCI1A:
	push R16
	in R16, SREG
	push R16
	
	inc uniTimer

	cpi uniTimer, 10				; Check unit timer digit overflow
	brne skip1

	ldi uniTimer, 0
	inc decTimer

	skip1:
		cp	uniTimer, uniDigit		; Check if current timer is expired
		brne skip2					; by comparing both digits
		cp	decTimer, decDigit
		brne skip2
		rcall setState
	skip2:
		pop R16
		out SREG, R16
		pop R16
		reti

;**************************************************************
;* Interrupt Service Routine: ISR_TOV0
;*
;* Inputs:	R18 - Timer unit digit
;* 			R19 - Timer decimal digit
;* 			R24 - Current active display flag
;*
;* Switches the current active 7 segment display and sets its
;* digit accordingly.
;*
;* Registers modified: R16, R24, R25
;**************************************************************
ISR_TOV0:
	push R16
	in R16,	SREG
	push R16
	
	cpi currDisplay, 0
	brne display1

	sbi PORTD, decPin			; Turn on decimal digit display
	cbi PORTD, uniPin			; Turn off unit digit display

	mov temp, decTimer
	rcall setDigit				; Sets BCD input
	inc currDisplay				; In the next interruption display 1
	rjmp exitISR				; will be turned on

	display1:
		cbi PORTD, decPin		; Turn off decimal digit display
		sbi PORTD, uniPin		; Turn on unit digit display

		mov temp, uniTimer
		rcall setDigit			; In the next interruption display 0
		dec currDisplay			; will be turned on

	exitISR:
		pop R16
		out	SREG,	R16
		pop R16
		reti

RESET:
	;*** Stack initialization ***
	ldi	temp,	LOW(RAMEND)			; Load low byte of RAMEND into temp reg
	out	SPL,	temp				; Store temp reg in stack pointer low
	ldi	temp,	HIGH(RAMEND)		; Load high byte of RAMEND into temp reg
	out	SPH,	temp				; Store temp reg in stack pointer high

	;*** PORTB and PORTD pins as output ***
	ldi	temp,	0b11111111        
	out DDRB,	temp
	out DDRD,	temp

	;*** Timer1 setup ***
	; On MEGA series, write high byte of 16-bit timer registers first
	; Initialize compare value (TOP)
	ldi	temp,	high(TOP)
	sts OCR1AH,	temp
	ldi temp,	low(TOP)
	sts OCR1AL,	temp
	; Lower 2 bits of WGM
	ldi temp,	((WGM&0b11) << WGM10)
	sts TCCR1A,	temp
	; Upper 2 bits of WGM and clock select
	ldi temp,	((WGM>> 2) << WGM12)|(PRESCALE << CS10)
	sts TCCR1B,	temp ; Start counter

	lds R16,	TIMSK1
	sbr R16,	(1 << OCIE1A)
	sts TIMSK1,	R16	; Enable Timer/Counter1 Compare Interrupt
	
	;*** Timer0 setup ***
	ldi R16,	(1 << CS02)		; Selects /256 prescale
	out TCCR0B,	R16 			; Timer0 clock = system clock / 256
	ldi R16,	(1 << TOV0)
	out TIFR0,	R16 			; Clear TOV0/ Clear pending interrupts
	ldi R16,	(1 << TOIE0)
	sts TIMSK0,	R16				; Enable Timer/Counter0 Overflow Interrupt

	;*** Init UART ***
	;* Enables UART transmission with 8 data, 1 parity, no stop bit at input baudrate
	ldi	R16, 	LOW(bps)				; Load baud prescale
	ldi	R17, 	HIGH(bps)				; into r17:r16

	sts	UBRR0L,	R16						; Load baud prescale
	sts	UBRR0H, R17						; to UBRR0

	ldi	R16,	(1<<RXEN0) | (1<<TXEN0)	; Enable transmitter
	sts	UCSR0B,	R16						; and receiver

	;*** Sending initial message for logging ***
	ldi	ZL,	LOW(2*initMsg)			
	ldi	ZH,	HIGH(2*initMsg)	; Load bytes from program memory
	rcall puts

	;*** Initial state steup ***
	rcall resetState
	rcall setState

	sei	; Enable global interrupts

	mainLoop:
		rjmp mainLoop	; Do nothing

;**************************************************************
;* Subroutine: puts
;*
;* Inputs: ZH:ZL - Program Memory address of string to transmit.
;*
;* Transmits null terminated string via UART
;*
;* Registers modified: R16, R17, R30, R31
;**************************************************************
puts:
	push R16
	push R17

	putsLoop:
		lpm	R16,	Z+				; Load character from program memory
		cpi	R16,	0x00			; Check if null
		breq	puts_end			; and branch to subroutine end

		puts_wait:	
			lds		R17,	UCSR0A	; Load UCSR0A into R17 and
			sbrs	R17,	UDRE0	; wait for empty transmit buffer
			rjmp	puts_wait		; Repeat loop

		sts	UDR0,	R16				; Transmit character
		rjmp	putsLoop			; Repeat loop

	puts_end:
		pop R17
		pop R16
		ret							; Return from subroutine

;**************************************************************
;* Subroutine: setDigit
;*
;* Inputs: R25 - Digit to be displayed in the 7segment LEDs
;*
;* Sets the input pins (PORTD) of BCD accordingly.
;*
;* Registers modified:
;**************************************************************
setDigit:
	; Begin switch case on digit
	cpi	temp,	0
	breq	case0
	cpi	temp,	1
	breq 	case1
	cpi temp,	2
	breq 	case2
	cpi temp,	3
	breq 	case3
	cpi temp,	4
	breq 	case4
	cpi temp,	5
	breq 	case5
	cpi temp,	6
	breq 	case6
	cpi temp,	7
	breq 	case7
	cpi temp,	8
	breq 	case8
	cpi temp,	9
	breq 	case9

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
	end_switch:
		ret

;**************************************************************
;* Subroutine: resetState
;*
;* Inputs:
;*
;* Resets memory pointer registers to current state and message.
;*
;* Registers modified: R22, R23
;**************************************************************
resetState:
	ldi	currState,	0x00
	ldi	currMsg,	0x00
	ret

;**************************************************************
;* Subroutine: setState
;*
;* Inputs:	ZH:ZL - Program Memory address of current state data
;*			R22   - Current state pointer in program memory
;* 			R32	  - Current message pointer in program memory
;*
;* Loads current state data from program memory and activates
;* the shift registers to light the LEDs. Also sends logging
;* to serial monitor.
;*
;* Registers modified: R16 - R23, R25, R30, R31
;**************************************************************
setState:
	ldi	ZL,	LOW(2*pArr)			; Initialize Z pointer
	ldi ZH,	HIGH(2*pArr)		; to program memory array address
	
	ldi	temp,	0
	add	ZL,	currState
	adc	ZH,	temp
	
	lpm		ledsL,		Z+		; Loads LEDs LOW value from program memory
	lpm 	ledsH,		Z+		; Loads LEDs HIGH value from program memory
	lpm 	decDigit,	Z+		; Loads time interval decimal digit value from program memory
	lpm 	uniDigit,	Z+		; Loads time interval unit digit value from program memory
	ldi 	decTimer,	0x00	; Initializing time counter to [0][0]
	ldi 	uniTimer,	0x00
	subi 	currState,	-4		; Points to the next state data in program memory
  
	;*** Shift register for loop ***
	;* Reads ledsL and ledsH bit by bit from left to right
	;* and shifts bit inside the register
	ldi temp, 1
	highForLoop:
		; First loop throught the HIGH byte register
		cpi	temp,	0b00000000
		breq 	endHighForLoop
		push	temp
		and	temp,	ledsH
		pop	temp
		breq	highPass
		sbi	PORTB,	dataPin		; Bit is set
	highPass:
		sbi	PORTB,	clkPin		; Clock pin pulse
		cbi	PORTB,	clkPin		; to shift bit
		cbi PORTB,	dataPin		; Default is bit clear
		lsl 	temp			; Compare next bit
		rjmp	highForLoop
	endHighForLoop:
		ldi	temp,	1
	lowForLoop:
		; Second loop throught the LOW byte register
		cpi	temp,	0b00000000
		breq	endLowForLoop
		push	temp
		and	temp,	ledsL
		pop	temp
		breq	lowPass
		sbi	PORTB,	dataPin
	lowPass:
		sbi	PORTB,	clkPin
		cbi	PORTB,	clkPin
		cbi PORTB,	dataPin
		lsl	temp
		rjmp	lowForLoop
	endLowForLoop:

	sbi	PORTB,	latchPin	; Latch pin pulse
	cbi	PORTB,	latchPin	; to activate the LEDs

	;*** Sending current state data ***
	ldi	ZL,	LOW(2*myMsg)	; load Z pointer with
	ldi	ZH,	HIGH(2*myMsg)	; myMsg address
	
	ldi temp,	0
	add ZL,	currMsg
	adc ZH,	temp			; Points to the current message

	subi currMsg,	-27		; Pointing to the address of next message
	
	rcall	puts			; Transmit message

	cpi		currState, 32	; Check if last state
	breq	resetState		; has been reached and resets
	ret

;*** Storing constants in progam memory ***
pArr:	.db	0x32, 0x92, 2, 1, \
			0x52, 0x92, 0, 6, \
			0x91, 0x91, 1, 1, \
			0x92, 0x92, 0, 6, \
			0x86, 0x32, 2, 1, \
			0x86, 0x52, 0, 6, \
			0x86, 0x86, 5, 1, \
			0x8A, 0x8A, 0, 6

initMsg:	.db	"*** Traffic Lights AVR - ATmega328P - LOG ***", 0x0A, 0x0D, 0x00
myMsg:		.db "[State] 10000 [Time] 20s", 0x0A, 0x0D, 0x00, \
				"[State] 20000 [Time] 05s", 0x0A, 0x0D, 0x00, \
				"[State] 00001 [Time] 10s", 0x0A, 0x0D, 0x00, \
				"[State] 00000 [Time] 05s", 0x0A, 0x0D, 0x00, \
				"[State] 01100 [Time] 20s", 0x0A, 0x0D, 0x00, \
				"[State] 01200 [Time] 05s", 0x0A, 0x0D, 0x00, \
				"[State] 01010 [Time] 50s", 0x0A, 0x0D, 0x00, \
				"[State] 02020 [Time] 05s", 0x0A, 0x0D, 0x00