#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define R_direction    4  //控制右邊馬達方向的Pin腳
#define RightWheel_Pin 5  //速度
#define LeftWheel_Pin  6  //速度
#define L_direction    7  //控制左邊馬達方向的Pin腳


char BTcmd;
SoftwareSerial BT(11,10); //RX and TX 

/*
int MotorRight1=4;

int MotorLeft1=5;
int MotorLeft2=6;
*/



void go()
{
    digitalWrite(R_direction,LOW);
    digitalWrite(L_direction,LOW);
    analogWrite(RightWheel_Pin,70); 
    analogWrite(LeftWheel_Pin,70);
    Serial.println("GET") ; 
    

//digitalWrite(MotorRight1,HIGH);

//digitalWrite(MotorLeft1,HIGH);


}

void left()
{
    digitalWrite(R_direction,LOW);  
    digitalWrite(L_direction,LOW);
    analogWrite(RightWheel_Pin,70);  //right_motor_speed
    analogWrite(LeftWheel_Pin,0);                   //left_motor_speed
    delay(150);

//digitalWrite(MotorRight1,HIGH);
//digitalWrite(MotorLeft1,LOW);


}
void right()
{
    digitalWrite(R_direction,LOW);  
    digitalWrite(L_direction,LOW);
    analogWrite(RightWheel_Pin,0);                  //right_motor_speed
    analogWrite(LeftWheel_Pin,70);    //left_motor_speed
    delay(150);
//digitalWrite(MotorRight1,LOW);
//digitalWrite(MotorLeft1,HIGH);


}

void back()
{
    digitalWrite(R_direction,LOW);  
    digitalWrite(L_direction,LOW);
    analogWrite(RightWheel_Pin,70);                  //right_motor_speed
    analogWrite(LeftWheel_Pin,70);    //left_motor_speed
    delay(150);

//digitalWrite(MotorRight1,LOW);
//digitalWrite(MotorLeft1,LOW);


}
void stop()
{

    digitalWrite(R_direction,LOW);  
    digitalWrite(L_direction,LOW);
    analogWrite(RightWheel_Pin,0);  //right_motor_speed
    analogWrite(LeftWheel_Pin,0);                   //left_motor_speed
//digitalWrite(MotorRight1,LOW);
//digitalWrite(MotorLeft1,LOW);


}


void setup()
{
Serial.begin(9600);
BT.begin(38400);

pinMode(R_direction,OUTPUT);
pinMode(RightWheel_Pin,OUTPUT);
pinMode(LeftWheel_Pin,OUTPUT);
pinMode(L_direction,OUTPUT);

}


void loop()
{
if (BT.available())
{
BTcmd=BT.read();
if ('F' == BTcmd) go();
else if ('L' ==BTcmd) left();
else if ('B' ==BTcmd) back();
else if ('R' == BTcmd) right();
else if('S'==BTcmd)stop();
}
else{
}
delay(200);
 


}