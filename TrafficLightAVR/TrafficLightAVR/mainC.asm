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