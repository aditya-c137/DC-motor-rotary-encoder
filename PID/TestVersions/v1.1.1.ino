/* Copyright 2019 Rudra and Aditya LOL
*
* This code displays the rotation(in degrees) of the rotary encoder,
* number of revolutions and direction of rotaion.
* Proportional part of the PID control loop.
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
* --> Proportional part of the PID control loop
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
float degree;

//MaxCount: counter that resets after 1600 counts(360 degrees):
uint16_t long MaxCount,error,requiredValue = 0;
long nonVolatileCount;

//updates in the ISR:
volatile long long count;

double Kd =200, Kp = 2, Ki = 0.002, errorIN = 0, previousError, derivative;
int output2;
int output1;
double PID;
unsigned long dt,lastStep = 0;

void loop() {
  nonVolatileCount = count;
  Serial.print(nonVolatileCount);
  Serial.print("\t");
  MaxCount = nonVolatileCount%Resolution;
  degree  = (MaxCount*DegPerPul);
  Serial.print("\t");
  Serial.print(rev);
  Serial.print("   x  ");
  Serial.print(degree);
  Serial.print("  ");
  //Serial.println(dir);
  Serial.print("  ");
  Serial.print(derivative);
  Serial.print("  ");
  Serial.print(output1);
  Serial.print("  ");
  Serial.print(output2);
  Serial.print("  ");
  Serial.println(PID);
  //requiredValue-Range(0,279)
  error = abs(requiredValue-nonVolatileCount);
  dt = millis()-lastStep;
  if(dt>=5){
    lastStep = millis();
    derivative = (error - previousError) / dt;
    errorIN +=(error + previousError) / 2 * dt / 1000.0;
    errorIN = constrain(errorIN, 0, 50);
    float u = (Kp * error) + (errorIN * Ki) + (derivative * Kd);
    if ((((error * u) > 0) && ((u > 100) || (u < 0)))) {
      float d_int = Ki * error;
      errorIN -= d_int;
      u -= d_int;
    }
    previousError = error;
  }
  //previousError = error;
  
  PID = (Kp*error)+(Ki*errorIN)+(Kd*derivative);
  if (requiredValue > nonVolatileCount)
  {
    output1 = constrain(PID, 0, 50);
    output2 = 0;
  }
  else if (requiredValue < nonVolatileCount)
  {
    output2 = constrain(PID, 0, 50);
    output1 = 0;
  }
  analogWrite(5,output1);
  analogWrite(6,output2);
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
