/* Copyright 2019 Rudra and Aditya LOL
*
* This code displays the rotation(in degrees) of the rotary encoder,
* number of revolutions and direction of rotaion.
*
*         Pinout    Description     Arduino
*         white --> phase A     --> digital pin number 3
*         green --> phase B     --> digital pin number 2
*         red   --> Vcc         --> Vcc
*         black --> Gnd         --> Gnd
*         
* This example code will:
* --> Display direction 1 for anti-clockwise and 0 for clockwise rotation
* --> positive rotation (in degrees) for clockwise and negative for anti-clockwise
*     number of revolutions and pulse count.
*
*/

#define PhaseAPin 3
#define PhaseBPin 2
//Pulses per revolution: 
#define Resolution 280
//Degrees per pulse:
#define DegPerPul 1.2857

void setup() {
  Serial.begin(9600);
  pinMode(PhaseAPin,INPUT_PULLUP);
  pinMode(PhaseBPin,INPUT_PULLUP);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PhaseAPin),phaseA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PhaseBPin),phaseB,CHANGE);
}

//checks direction of motor:
volatile int dir;

//initialize number of revolutions:
volatile long rev = -1;

//for rotaion in degrees:
float degree,error;

//MaxCount: counter that resets after 1600 counts(360 degrees):
uint16_t long nonVolatileCount,MaxCount;

//updates in the ISR:
volatile long long count;

int PWM=1;
int Kd = 0;
int Kp = 50;
int Ki = 0;

void loop() {
  nonVolatileCount = count;
  Serial.print(nonVolatileCount);
  Serial.print("                        ");
  MaxCount = nonVolatileCount%Resolution;
  degree  = (MaxCount*DegPerPul);
  Serial.print(rev);
  Serial.print("   x  ");
  Serial.print(degree);
  Serial.print("\t");
  Serial.println(dir);
  error = degree;
  if(error>-25){
    analogWrite(5,(Kp/2)*PWM);
    analogWrite(6,0);
  }
  else if(error>-50 && error<-25){
    analogWrite(5,Kp*PWM);
    analogWrite(6,0);
  }
  else if(error<25){
    analogWrite(5,0);
    analogWrite(6,(Kp/2)*PWM);
  }
  else if(error<50 && error>25){
    analogWrite(5,0);
    analogWrite(6,Kp*PWM);
  }
}

void phaseA(){
  if((digitalRead(PhaseBPin)==LOW && digitalRead(PhaseAPin)==HIGH)||(digitalRead(PhaseBPin)==HIGH&&digitalRead(PhaseAPin)==LOW))
  {
    dir = 0;
  }
  else if((digitalRead(PhaseBPin)==HIGH && digitalRead(PhaseAPin)==HIGH)||(digitalRead(PhaseBPin)==LOW && digitalRead(PhaseAPin)==LOW)){
    dir = 1;
  }
  
  if(dir==0){
    count++;
    if(count%Resolution==0){
      rev++;
    }
  }
  else if(dir==1)
  {
    count--;
    if(count%Resolution==0){
      rev--;
    }
  }
}

void phaseB(){
  if((digitalRead(PhaseBPin)==HIGH && digitalRead(PhaseAPin)==LOW)||(digitalRead(PhaseBPin)==LOW && digitalRead(PhaseAPin)==HIGH))
  {
    dir = 1;
  }
  else if((digitalRead(PhaseBPin)==HIGH && digitalRead(PhaseAPin)==HIGH)||(digitalRead(PhaseBPin)==LOW && digitalRead(PhaseAPin)==LOW)){
    dir = 0;
  }
  
  if(dir == 0){
    count++;
    if(count%Resolution==0){
      rev++;
    }
  }
  else if(dir == 1)
  {
    count--;
    if(count%Resolution==0){
      rev--;
    }
  }
}
