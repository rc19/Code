#include<Servo.h>
int temp,Speed; 
Servo m1,m2,m3,m4;
void setup()
{
  Serial.begin(9600); 
  m1.attach(5);
  m2.attach(6);
  m3.attach(9);
  
  m4.attach(10);
  
  Serial.println("Program begin...");
  Serial.println("This program will calibrate the ESC.");


  Serial.println("Now writing maximum output.");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  m1.write(2000);
  m2.write(2000);
  m3.write(2000);
  m4.write(2000);
  

  // Wait for input
  while (!Serial.available());
  Serial.read();

  // Send min output
  Serial.println("Sending minimum output");

  m1.write(1000);
  m2.write(1000);
  m3.write(1000);
  m4.write(1000);
  pinMode(A0,INPUT);
}
void loop()
{
     temp = analogRead(A0);
     Speed = map(temp,0,1023,1000,2000);
     m1.writeMicroseconds(Speed);
     m2.writeMicroseconds(Speed);
     m3.writeMicroseconds(Speed);
     m4.writeMicroseconds(Speed);
     Serial.println(Speed);  
}
