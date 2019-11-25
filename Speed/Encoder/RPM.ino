#define RPMcoeff 37500

volatile long _counter;
double current_counter = 0,prev_counter = 0;
long why_so_long = 0,why_so_much,why_so_early;
double delta = 0,RPM = 0;


void setup() {
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(3),eh,CHANGE);
  attachInterrupt(digitalPinToInterrupt(2),eh,CHANGE);
}

void loop() {
  why_so_early = micros();
  if((why_so_early-why_so_long) >100000){
    why_so_much = micros() - why_so_long;
    why_so_long = micros();
    delta = _counter - prev_counter;
    prev_counter = _counter;
    RPM = double(delta * RPMcoeff)/double(why_so_much);
    Serial.print(delta);
    Serial.print("\t");
    Serial.println(RPM);
  }
}

void eh(){
  _counter++;
}
