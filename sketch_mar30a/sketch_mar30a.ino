#include <stdlib.h>

// Motor A (FL)
#define IN1_A 12
#define IN2_A 2
#define ENA 3

// Motor B (FR)
#define IN1_B 8
#define IN2_B 13
#define ENB 6

// Motor C (BR)
#define IN1_C 10
#define IN2_C 11
#define ENC 9

// Motor D (BL)
#define IN1_D 7
#define IN2_D 4
#define END 5



void setup() {
  // put your setup code here, to run once:
  
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  pinMode(IN1_C, OUTPUT);
  pinMode(IN2_C, OUTPUT);
  pinMode(ENC, OUTPUT);
  
  pinMode(IN1_D, OUTPUT);
  pinMode(IN2_D, OUTPUT);
  pinMode(END, OUTPUT);

  Serial.begin(9600);


}

void motorA(int speed, bool forward){
  digitalWrite(IN1_A, forward);
  digitalWrite(IN2_A, !forward);
  analogWrite(ENA, speed);
}

void motorB(int speed, bool forward){
  digitalWrite(IN1_B, !forward);
  digitalWrite(IN2_B, forward);
  analogWrite(ENB, speed);
}

void motorC(int speed, bool forward){
  digitalWrite(IN1_C, forward);
  digitalWrite(IN2_C, !forward);
;  analogWrite(ENC, speed);
}

void motorD(int speed, bool forward){
  digitalWrite(IN1_D, !forward);
  digitalWrite(IN2_D, forward);
  analogWrite(END, speed);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('x');  // Read until 'x' terminator
    int m1, m2, m3, m4;
    
    // Use c_str() to get C-style string for sscanf
    if (sscanf(data.c_str(), "%d,%d,%d,%d", &m1, &m2, &m3, &m4) == 4) {
      motorA(m1, true);
      motorB(m2, true);
      motorC(m3, true);
      motorD(m4, true);
      Serial.print(m1);
      Serial.print(", ");
      Serial.print(m2);
      Serial.print(", ");
      Serial.print(m3);
      Serial.print(", ");
      Serial.println(m4);
    } else {
      Serial.println("Error parsing motors");

    }
  }
}
