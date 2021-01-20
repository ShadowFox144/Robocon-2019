/*
  Walk.cpp - Stores the Functions Declarations
             for walking of a bot

*/

#include "Arduino.h"
#include "PinChangeInterrupt.h"
#include "Walk.h"

#define max_speed 0.6
#define max_speed 0.6
#define follow_speed 0.6
#define DELAY 50

//DC_Motor:
  void Motor::DC_Motor(uint8_t M_Pin, uint8_t D_Pin){ // Intialize
    Motor_Pin = M_Pin;
    Dir_Pin = D_Pin;
    motor_init();
  }

  void Motor::motor_init(){
    pinMode(Motor_Pin, OUTPUT);
    pinMode(Dir_Pin, OUTPUT);
  }

  void Motor::set_motor_speed(int motor_speed) {
    if (abs(motor_speed) < 5)
      motor_speed = 0;
    if (motor_speed >= 0) {
      //Clockwise direction
      digitalWrite(Dir_Pin, HIGH);
      analogWrite(Motor_Pin, motor_speed);
    }
    else if (motor_speed < 0) {
      //Anti Clockwise direction
      digitalWrite(Dir_Pin, LOW);
      analogWrite(Motor_Pin, (-1 * motor_speed));
    }
  }

//Encoder :
  void Motor::Encoder(uint8_t A, uint8_t B){
    encoded = 0;
    last_encoded = 0;
    MSB = 0;
    LSB = 0;
    sum = 0;
    Pin_A = A;
    Pin_B = B;
    encoderValue =0;
    Encoder_init();

  }

  void Motor::Encoder_init(){
    pinMode(Pin_A, INPUT_PULLUP);
    pinMode(Pin_B, INPUT_PULLUP);
  }

  void Motor::encode_A(){
    MSB = digitalRead(Pin_A); //MSB = most significant bit
    LSB = digitalRead(Pin_B); //LSB = least significant bit

    encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
    sum  = (last_encoded << 2) | encoded; //adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
    last_encoded = encoded; //store this value for next time
  }

  void Motor::encode_B(){
    MSB = digitalRead(Pin_A); //MSB = most significant bit
    LSB = digitalRead(Pin_B); //LSB = least significant bit

    encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
    sum  = (last_encoded << 2) | encoded; //adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
    last_encoded = encoded; //store this value for next time
  }

  void Motor::print_position(){
      Serial.print(encoderValue);
      Serial.print(" ");
  }

//Motor
  void Motor::Motor_1(uint8_t A, uint8_t B, uint8_t M, uint8_t D){
      Encoder(A,B);
      DC_Motor(M,D);
      P = 0;
      I = 0;
      D = 0;
      error = 0;
      last_error =0;
      PID = 0;
    }

    void Motor::set_Kpid(float kp, float ki, float kd){
      Kp = kp;
      Kd = kd;
      Ki = ki;
    }

    int Motor::sign(int i) {
      if (i < 0)
        return -1;
      return 1;
    }

    void Motor::calc_PID(){
      error = set_position - encoderValue;
      P = error;
      I += error;
      D = error - last_error;
      PID = -1 * ((Kp * P) + (Ki * I) + (Kd * D));
      last_error = error;
      if (abs(PID) > max_speed)
        PID = sign(PID) * max_speed;
    }

    void Motor::motors(){
      set_motor_speed(PID);
    }

//Leg:
  Leg::Leg(uint8_t Mtr_Pin[2], uint8_t Dir_Pin[2], uint8_t PA[2], uint8_t PB[2]){
    PID_dt = 100;
    for (int i = 0; i < 2; i++)
      Mtr[i].Motor_1(PA[i],PB[i], Mtr_Pin[i], Dir_Pin[2]);

  }

  void Leg::set_points(int Q1[50], int Q2[50]){
    for (int i = 0; i < 50; i++) {
      theta_1_V[i] = Q1[i];
      theta_2_V[i] = Q2[i];
    }
  }

  void Leg::move_leg(){//To be modified
      for (int i = 1; i < 50; i++){
        last_ms = millis();
        Mtr[0].set_position = theta_1_V[i];
        Mtr[1].set_position = theta_2_V[i];
        while (millis() - last_ms < PID_dt)
        {
          print_position_encoders();
          cal_PID();
          setAllMotors();
        }
      }
  }

  void Leg::print_position_encoders(){
    for (int i = 0; i < 2; i++)
      Mtr[i].print_position();
  }

  void Leg::cal_PID(){
    for (int i = 0; i < 2; i++)
      Mtr[i].calc_PID();
  }

  void Leg::setAllMotors(){
    for(int i = 0; i < 2; i++)
      Mtr[i].motors();
    }
