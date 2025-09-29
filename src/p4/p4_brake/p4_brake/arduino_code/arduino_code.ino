#include <Wire.h>
#include <Stepper.h>

Stepper stepper_1(200, 14, 15, 16, 17);
Stepper stepper_2(200, 7, 6, 5, 4);
Stepper stepper_3(200, 0, 1, 2, 3);
bool done = false;
int state = 0;

void apply_breaks() {
  int num_steps_to_break = 200 * 14;
  for(int i = 0; i < num_steps_to_break; i++){
    stepper_1.step(1);
    stepper_2.step(1);
    stepper_3.step(1);
    delay(2);
  }
  digitalWrite(0, LOW);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(14, LOW);
  digitalWrite(15, LOW);
  digitalWrite(16, LOW);
  digitalWrite(17, LOW);
}

void release_breaks() {
  while(digitalRead(9) || digitalRead(10) || digitalRead(11)){
    if(digitalRead(11))
      stepper_1.step(-1);
    if(digitalRead(9))
      stepper_2.step(-1);
    if(digitalRead(10))
      stepper_3.step(-1);
  }
  digitalWrite(0, LOW);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(14, LOW);
  digitalWrite(15, LOW);
  digitalWrite(16, LOW);
  digitalWrite(17, LOW);
}

void receive_event(){
  while(1 < Wire.available()){
    char c = Wire.read();
  }
  state = Wire.read();
}

void request_event(){
  //Wire.write(1);
  if (done)
    Wire.write(1);
  else
    Wire.write(0);
  done = false;
}

void setup() {
  Wire.begin(0x2a);
  Wire.onReceive(receive_event);
  Wire.onRequest(request_event);
  //Serial.begin(9600);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  stepper_1.setSpeed(50);
  stepper_2.setSpeed(50);
  stepper_3.setSpeed(50);
}

void loop() {
  if (state == 1) {
    done = false;
    release_breaks();
    state = -1;
    done = true;
  }
  else if (state == 2) {
    done = false;
    apply_breaks();
    state = -1;
    done = true;
  }
}
