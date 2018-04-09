#include <SoftwareSerial.h>

#define LED 13

bool switch_led = false;
//bool going = false;

void setup() {
  Serial.begin(9600);
  while(!Serial);

  pinMode(LED, OUTPUT);
  Serial.println("yeah?");
}

void loop() {

  if(Serial.available()>0){
    /*char cmd = Serial.read();

    if(cmd){
      if(cmd == 's'){
        going != going;
      }
    }
    if(going) {*/
      Serial.println("I got it");
      digitalWrite(LED, switch_led);
      switch_led = !switch_led;
    //}
  }
    
  delay(30);
}

