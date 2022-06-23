/*

State	|         Binary		    | Hexadecimal |  Time   |
-----	| ---------------------	| ----------- | ------- |
10000	| 001 100 10 100 100 10	|    32 92    |  20.13  |
20000	| 010 100 10 100 100 10	|    52 92    |  04.22  |
00001	| 100 100 01 100 100 01	|    91 91    |  12.99  |
00000	| 100 100 10 100 100 10	|    92 92    |  05.94  |
01100	| 100 001 10 001 100 10	|    86 32    |  20.25  |
01200	| 100 001 10 010 100 10	|    86 52    |  03.51  |
01010	| 100 001 10 100 001 10	|    86 86    |  53.42  |
02020	| 100 010 10 100 010 10	|    8A 8A    |  03.70  |

#define clk 10
#define latch 9
#define data 8

#define UNI 7
#define DEC 6

#define I1 2
#define I2 3
#define I3 4
#define I4 5

// Counter and compare values
const uint16_t t1_load = 0;
const uint16_t t1_comp = 62500;

int timer = 0;
int uni;
int dec;

int current = 0;
int states[]{0x3292, 0x5292, 0x9191, 0x9292, 0x8632, 0x8652, 0x8686, 0x8A8A};
int timers[]{20, 5, 10, 5, 20, 5, 50, 5};

void setup() {
  
  pinMode(clk, OUTPUT);
  pinMode(data, OUTPUT);
  pinMode(latch, OUTPUT);
  
  pinMode(UNI, OUTPUT);
  pinMode(DEC, OUTPUT);
  
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
  
  // Reset Timer1 Control Register A
  TCCR1A = 0;
  
  // Set CTC mode
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);
  
  // Set to prescaler of 256
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);
  
  // Reset Timer1 and set compare value
  TCNT1 = t1_load;
  OCR1A = t1_comp;
  
  // Enable Timer1 compare interrupt
  TIMSK1 = (1 << OCIE1A);
  
  // Enable global interrupts
  sei();
  
  setState(states[current]);
}

void number(int n, int dig){
  if(dig==0){
    digitalWrite(DEC, HIGH);
    digitalWrite(UNI, LOW);
  }
  
  if(dig==1){
    digitalWrite(DEC, LOW);
    digitalWrite(UNI, HIGH);
  }
  
  if(n==0){    
    digitalWrite(I1, LOW);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, LOW);
  }
  if(n==1){
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, LOW);
  }
  if(n==2){
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, LOW);
  }
  if(n==3){
    digitalWrite(I1, HIGH);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, LOW);  
  }
  if(n==4){
    digitalWrite(I1, LOW);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
  }
    if(n==5){
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
  }
    if(n==6){
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
  }
    if(n==7){
    digitalWrite(I1, HIGH);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
  }
    if(n==8){
    digitalWrite(I1, LOW);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
  }
    if(n==9){
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
  }
}

void setState(int state)
{
  int bit = 0;
  for (int i = 0; i < 16; i += 1)
  {
	bit = (state >> i) & 1;
    if (bit == 0)
    {
      digitalWrite(data, LOW);
    } else {
      digitalWrite(data, HIGH);
    }
    digitalWrite(clk, HIGH);
  	digitalWrite(clk, LOW);
  }
  digitalWrite(latch, LOW);
  digitalWrite(latch, HIGH);
  digitalWrite(latch, LOW);
}

void loop() {
  uni = timer % 10;
  dec = timer / 10;
  
  number(dec, 0);
  delay(50);
  number(uni, 1);
  delay(50);
}

ISR(TIMER1_COMPA_vect) {
  timer++;
  if (timer == timers[current] + 1)
  {
    timer = 1;
    current = (current + 1) % 8;
    setState(states[current]);
  }
}

*/

.def	ledsL	= R16		; Define led low register
.def	ledsH	= R17		; Define led high register
.def	timeInt	= R18		; Define time interval register
.def	currState	= R19		; Define loop count register
.def	temp	= R20

.equ	clkPin	= PINB2
.equ	latchPin = PINB1
.equ	dataPin	= PINB0

.cseg
.org 0

pArr:	.db		0x32, 0x92, 0x20, \
            0x52, 0x92, 0x05, \
            0x91, 0x91, 0x10, \
            0x92, 0x92, 0x05, \
            0x86, 0x32, 0x20, \
            0x86, 0x52, 0x05, \
            0x86, 0x86, 0x50, \
            0x8A, 0x8A, 0x05

ldi temp, 0b11111111        
out DDRB, temp            ;configura PORTB como saí­da

ldi	r16,LOW(RAMEND)		; load low byte of RAMEND into r16
out	SPL,r16			; store r16 in stack pointer low
ldi	r16,HIGH(RAMEND)	; load high byte of RAMEND into r16
out	SPH,r16			; store r16 in stack pointer high

rcall resetState
rcall	setState

loop:
  rcall	setState							; Infinite loop
  rjmp	loop

resetState:
  ldi		currState, 0x00 
  ret

/*void setState(int state)
{
  int bit = 0;
  for (int i = 0; i < 16; i += 1)
  {
	bit = (state >> i) & 1;
    if (bit == 0)
    {
      digitalWrite(data, LOW);
    } else {
      digitalWrite(data, HIGH);
    }
    digitalWrite(clk, HIGH);
  	digitalWrite(clk, LOW); 
  }
  digitalWrite(latch, LOW);
  digitalWrite(latch, HIGH);
  digitalWrite(latch, LOW);
}*/

setState:
	ldi		ZL, LOW(2*pArr)			; Initialize Z pointer
	ldi		ZH, HIGH(2*pArr)		; to pmem array address
	add		ZL, currState
	lpm		ledsL, Z+						; Load LED LOW value from flash memory
	lpm		ledsH, Z+						; Load LED HIGH value from flash memory
	lpm		timeInt, Z+						; Load time interval value from fash memory
	subi	currState, -3
  
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