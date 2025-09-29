#include <Wire.h>
#include <Stepper.h>

const int HOME_SWITCH = A5;   // Switch to register home position
const int GRIP_SWITCH = A4;   // Switch for gripping
bool done = TRUE;

Stepper stepper(200, A0, A1, A2, A3);

void home() {
  while (digitalRead(HOME_SWITCH)) {
    stepper.step(1);
    delay(10); // Keeps movement consistent and avoids bouncing issues
  }
}

void grip() {
  while (digitalRead(GRIP_SWITCH)) {
    stepper.step(-1);
    delay(10); // Maintains consistency for gripping
  }
}

void receive_event(){
  while(1 < Wire.available()){
    char c = Wire.read();
  }
  int x = Wire.read();
  done = FALSE;
  switch (x){
    case 0:
      home();
      break;
    case 1:
      grip();
      break;
    }
  done = TRUE;
}

void request_event(){
  if done{
    Wire.write(1);
    return;
  }
  Wire.write(0);
}

void setup() {
  Wire.begin(0x2b);
  Wire.onReceive(receive_event);
  Wire.onRequest(request_event);
  stepper.setSpeed(100); // Speed setting for the stepper motor
  pinMode(HOME_SWITCH, INPUT);  // Activate internal pull-ups for HOME_SWITCH
  pinMode(GRIP_SWITCH, INPUT);  // Activate internal pull-ups for GRIP_SWITCH
}

void loop() {
  delay(100);
}
