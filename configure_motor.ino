void configure_motors()
{

  m1.attach(5);
  m2.attach(6);
  m3.attach(9);
  
  m4.attach(10);
  
  Serial.println("Program begin...");
  Serial.println("This program will calibrate the ESC.");


  Serial.println("Now writing maximum output");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  m1.write(2000);
  m2.write(2000);
  m3.write(2000);
  m4.write(2000);
  


  m1.write(1000);
  m2.write(1000);
  m3.write(1000);
  m4.write(1000);

}
