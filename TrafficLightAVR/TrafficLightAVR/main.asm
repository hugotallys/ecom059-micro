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

; Define symbolic names for resources used

.def count = R16		; Reg 16 will hold counter value
.def temp = R17			; Reg 17 is used as a temporary register

ldi temp, 0xFF			; Configure PORTB as output
out DDRB, temp
ldi count, 0x00			; Initialize count at 0
        
lp:
	out PORTB, count	; Put counter value on PORT B
	inc count			; Increment counter
	rjmp lp				; Repeat (forever)