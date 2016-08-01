#include <Arduino.h>

// Software Serial Sample for Packet Serial
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <PID_v1.h>


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=0, Kd=0.01;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


const int startbutton = 2;

SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.

void setup()
{
  pinMode(A3,INPUT);
  Serial.begin(115200);
  Serial.setTimeout(0);
  SWSerial.begin(38400);
  // ST.setBaudRate(38400);
  ST.autobaud();
  ST.setTimeout(500);
  ST.motor(1, 0);
  ST.motor(2, 0);
  pinMode(13,OUTPUT);
  pinMode(startbutton,INPUT_PULLUP);
  digitalWrite(13,LOW);
  int start_button_state = digitalRead(startbutton);
  int led_state = 0;
  /*
  Serial.print("Toggle the switch to run motor ");
  Serial.println(analogRead(A3));
  */
  while (digitalRead(startbutton) == start_button_state){
    delay(100);
    led_state ^= 1;
    digitalWrite(13,led_state);
  }
  digitalWrite(13,LOW);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  //myPID.SetOutputLimits(-40.0,40.0);
  myPID.SetOutputLimits(-40.0,40.0);
  Setpoint = 512;

}

int yellow_light_state = 0;
int yellow_light_interval = 200;
uint32_t last_light_toggle = 0;

void loop()
{

  if (millis() - last_light_toggle > yellow_light_interval){
    yellow_light_state ^= 1;
    ST.motor(2, yellow_light_state ? 0 : 127);
    last_light_toggle = millis();
  }

  static char buffer[5];
  if (readline(Serial.read(), buffer, 5) > 0) {
    int setpoint = atoi(buffer);
    setpoint = map(setpoint,-100,100,100,924);
    if (setpoint > 99 && setpoint < 925) {
      Setpoint = setpoint;
    }
  }

  Input = analogRead(A3);
  if (Input < 50 || Input > 970) {
    myPID.SetOutputLimits(-10,10);
    digitalWrite(13,HIGH);
  } else {
    myPID.SetOutputLimits(-40.0,40.0);
    digitalWrite(13,LOW);
  }

  if (myPID.Compute()){
    /*
    if (abs(Output) < 15){
      ST.motor(1, 0);
    } else {
      ST.motor(1, Output);
    }*/
    ST.motor(1, Output);
    /*
    Serial.print("set: ");
    Serial.print(Setpoint);
    Serial.print(" actual: ");
    Serial.print(Input);
    Serial.print(" out: ");
    Serial.println(Output);
    */
  }

}

int readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
      case '\n':
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len-1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}
