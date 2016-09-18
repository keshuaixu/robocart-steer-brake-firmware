#include <Arduino.h>

// Software Serial Sample for Packet Serial
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <PID_v1.h>
#include "FlexCAN.h"
// #include "can.h"
#include "robocart_can_def.h"


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=0, Kd=0.01;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


const int startbutton = 22;
const int steer_pot_pin = A0;

Sabertooth ST(128, Serial2); // Address 128, and use SWSerial as the serial port.
FlexCAN can(1000000);

void setup()
{
  pinMode(steer_pot_pin,INPUT);

  FLEXCAN_config_t can_config;

  // // IF you're using a 16mhz clock
  can_config.presdiv   = 2;  /*!< Prescale division factor. */
  can_config.propseg   = 0;  /*!< Prop Seg length. */
  can_config.rjw       = 1;  /*!< Sychronization Jump Width*/
  can_config.pseg_1    = 1;  /*!< Phase 1 length */
  can_config.pseg_2    = 1;  /*!< Phase 2 length */
  // IF you're using a 16mhz clock
  // can_config.presdiv   = 1;  /*!< Prescale division factor. */
  // can_config.propseg   = 2;  /*!< Prop Seg length. */
  // can_config.rjw       = 1;  /*!< Sychronization Jump Width*/
  // can_config.pseg_1    = 7;  /*!< Phase 1 length */
  // can_config.pseg_2    = 3;  /*!< Phase 2 length */
  // FLEXCAN_init(can_config);
  can.begin();
  // FLEXCAN_fifo_reg_callback(can_fifo_callback);

  FLEXCAN_frame_t cb_frame_a;
  cb_frame_a.id = 0x0030;
  cb_frame_a.srr = 0;
  cb_frame_a.ide = 0;
  cb_frame_a.rtr = 0;

  /* YEAH */
  // FLEXCAN_mb_write(FLEXCAN_RX_BASE_MB + 1, FLEXCAN_MB_CODE_RX_EMPTY, cb_frame_a);
  // FLEXCAN_mb_reg_callback(FLEXCAN_RX_BASE_MB + 1, flexcan_cb_030);

  // can.begin();


  Serial.begin(115200);
  Serial.setTimeout(0);
  Serial2.begin(38400);
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
  Serial.println(analogRead(steer_pot_pin));
  */
  // while (digitalRead(startbutton) == start_button_state){
  //   delay(100);
  //   led_state ^= 1;
  //   digitalWrite(13,led_state);
  // }
  digitalWrite(13,LOW);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  //myPID.SetOutputLimits(-40.0,40.0);
  myPID.SetOutputLimits(-40.0,40.0);
  Setpoint = 512;

}

int yellow_light_state = 0;
const int yellow_light_interval = 200;
uint32_t last_light_toggle = 0;

const int can_alive_interval = 1000;
uint32_t last_can_alive = 0;

const int can_sensor_report_interval = 100;
uint32_t last_can_sensor_report = 0;

void loop()
{
  uint32_t now = millis();
  if (now - last_can_alive > can_alive_interval){
    last_can_alive = now;
    CAN_message_t msg;
    msg.len = 0;
    msg.ext = 1;
    msg.id = CANID_STATESERVER_STEERBRAKE_ALIVE;
    can.write(msg);
  }
  read_and_process_can();
  if (now - last_light_toggle > yellow_light_interval){
    last_light_toggle = now;
    yellow_light_state ^= 1;
    ST.motor(2, yellow_light_state ? 0 : 127);

  }

  Input = analogRead(steer_pot_pin);
  if (Input < 50 || Input > 970) {
    myPID.SetOutputLimits(-40,40);
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
    Serial.print("set: ");
    Serial.print(Setpoint);
    Serial.print(" actual: ");
    Serial.print(Input);
    Serial.print(" out: ");
    Serial.println(Output);

  }

  if (now - last_can_sensor_report > can_sensor_report_interval){
    last_can_sensor_report = now;
    CAN_message_t msg;
    msg.len = 4;
    msg.ext = 1;
    msg.id = CANID_SENSOR_STEER_SENSOR_REPORT;
    msg.buf[0] = ((int16_t) Input) & 0xff;
    msg.buf[1] = ((int16_t) Input) >> 8 & 0xff;
    msg.buf[2] = ((int16_t) Output) & 0xff;
    msg.buf[3] = ((int16_t) Output) >> 8 & 0xff;
    can.write(msg);
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

void read_and_process_can(){
  while (can.available()) {
    CAN_message_t rxmsg;
    can.read(rxmsg);
    switch (rxmsg.id) {
      case CANID_EVERYONE_ESTOP:
        ST.stop();
      break;
      case CANID_EVERYONE_SYSTEM_STATE:
        switch (rxmsg.buf[0]) {

        }
      break;
      case CANID_STEERBRAKE_STEER_INPUT_HUMAN:
      case CANID_STEERBRAKE_STEER_INPUT_AUTONOMOUS:
        int8_t rx_setpoint = rxmsg.buf[0];
        int setpoint = map(rx_setpoint,-100,100,100,924);
        if (setpoint > 99 && setpoint < 925) {
          Setpoint = setpoint;
        }
        // Setpoint = (double)map(rx_setpoint,-100,100,100,924);
        Serial.println(Setpoint);
      break;
    }
  }
}
