#include <Wire.h>
#include <Stepper.h>

const int HOME_SWITCH = 2;   // Switch to register home position
const int GRIP_SWITCH = 3;   // Switch for gripping
bool done = true;
int state = 0;

Stepper stepper(200, A0, A1, A2, A3);

void home() {
  Serial.print("homeing switch ");
  Serial.println(digitalRead(HOME_SWITCH));
  while (digitalRead(HOME_SWITCH)) {
    stepper.step(10);
  }
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
}

void grip() {
  Serial.print("gripping switch ");
  Serial.println(digitalRead(GRIP_SWITCH));
  while (digitalRead(GRIP_SWITCH)) {
    stepper.step(-10);
  }
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
}

void receive_event(){
  while(1 < Wire.available()){
    char c = Wire.read();
  }
  state = Wire.read();
}

void request_event(){
  if (done) 
    Wire.write(1);
  else
    Wire.write(0);
}

void setup() {
  Serial.begin(9600);
  Wire.begin(0x2b);
  Wire.onReceive(receive_event);
  Wire.onRequest(request_event);
  stepper.setSpeed(100); // Speed setting for the stepper motor
  pinMode(HOME_SWITCH, INPUT);  // Activate internal pull-ups for HOME_SWITCH
  pinMode(GRIP_SWITCH, INPUT);  // Activate internal pull-ups for GRIP_SWITCH
}

void loop() {
  if (state == 1){
    done = false;
    home();
    state = -1;
    done = true;
  }
  if (state == 2){
    done = false;
    grip();
    state = -1;
    done = true;
  }
}
