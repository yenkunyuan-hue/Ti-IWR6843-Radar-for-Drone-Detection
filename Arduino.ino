#include <string.h>
const int Light_Read = A1; 
const byte trig=9;
const byte echo=8;
int duration;
float distance;
float meter;
bool detection = false;
const byte motor_pin1 = 11;
const byte motor_pin2 = 10;
int speed = 200;
unsigned long time = 0;
unsigned long now = 0;
const byte LED_r = 3; 
const byte LED_b = 5; 
const byte LED_g = 6;
int count_rotate = 0;
bool rotate_dir = 0;

struct data {
  String exist;
  float dis;
  int LDR;
};
data comm_package[1];

void setup(){
 pinMode(Light_Read, INPUT_PULLUP); 
 Serial.begin(9600);
 pinMode(trig, OUTPUT);
 digitalWrite(trig, LOW);
 pinMode(echo, INPUT); 
 pinMode(motor_pin1, OUTPUT);
 pinMode(motor_pin2, OUTPUT);
}

void loop(){
  String package = "S";
  comm_package[0] = {"false",0.0,0};
  LED_show();
  sensorRead();
  Light();
  package += Assemble();
  Movement();
  Serial.println(package);
}

void sensorRead()
{
 digitalWrite(trig, HIGH);
 delayMicroseconds(10); 
 digitalWrite(trig, LOW);
 duration = pulseIn(echo, HIGH);
 distance = duration/58;
 meter=distance/100;
 comm_package[0].dis = meter;
 if (meter <0){
  detection = false;
  comm_package[0].exist = "false";
 }
 else{
  detection = true;
  comm_package[0].exist = "true";
 }
 }
 void Read_Jetson(){
  while (Serial.available() >0){
    String input = Serial.readStringUntil('\n');
  }
 }
 void Light(){
  int light = analogRead(Light_Read);
  comm_package[0].LDR = light;
 }
 String Assemble(){
  String send = "";
  send += String(comm_package[0].exist);
  send += ";";
  send += String(comm_package[0].dis);
  send += ";";
  send += String(comm_package[0].LDR);
  send += "S";
  return send;
 }
void Movement(){
  now = millis();
  if (now-time >= 15000000000000000 and rotate_dir == 0){
    time = now;
    analogWrite(motor_pin1,speed);
    analogWrite(motor_pin2,0);
    delay(150);
    analogWrite(motor_pin1,0);
    count_rotate += 1;
    if (count_rotate ==4) {
      rotate_dir = 1;
      count_rotate = 0; 
    }
  }
  if (now-time >= 15000000000000000 and rotate_dir == 1){
    time = now;
    analogWrite(motor_pin1,0);
    analogWrite(motor_pin2,speed);
    delay(150);
    analogWrite(motor_pin2,0);
    count_rotate += 1;
    if (count_rotate ==4) {
      rotate_dir = 0;
      count_rotate = 0; 
    }
  }
  else{
    analogWrite(motor_pin1,0);
    analogWrite(motor_pin2,0);
  }
}
void LED_show() {
  bool priori = 0;
  String read = "";
    if (Serial.available()){
      while (Serial.available()>0){
      read = Serial.readStringUntil('\n');
      if (read=="e"){
        break;
      }
      if (read == "r"){
        priori = 1;
      }
      }
    }
    if (priori == 1){
    analogWrite(LED_r,0);
    analogWrite(LED_b, 255);
    analogWrite(LED_g, 255);
    delay(5000);
    }
    if (read == "e"){
    analogWrite(LED_r,255);
    analogWrite(LED_b,0);
    analogWrite(LED_g,255);
    delay(5000);
    }
    else{
    analogWrite(LED_r,255);
    analogWrite(LED_b,255);
    analogWrite(LED_g,0);
    }
}
