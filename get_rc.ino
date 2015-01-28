
void get_rc()

{
//int rc_commandmean[4][4];
//Serial.print("inside get rc");
rcLastChange1 = micros();
rcLastChange2 = micros();
rcLastChange3 = micros();
rcLastChange4 = micros();
rcLastChange5 = micros();
rcLastChange6 = micros();
attachinterrupt();

}

void acquireLock(){
  interruptLock = false; 
}

void releaseLock(){
  interruptLock = false;
}

void rcInterrupt1(){
   if(!interruptLock) ch1 = micros() - rcLastChange1;
   rcLastChange1 = micros(); 
}

void rcInterrupt2(){
  if(!interruptLock) ch2 = micros() - rcLastChange2;
  rcLastChange2 = micros();
}

void rcInterrupt3(){
  if(!interruptLock) ch3 = micros() - rcLastChange3;
  rcLastChange3 = micros();
}

void rcInterrupt4(){
  if(!interruptLock) ch4 = micros() - rcLastChange4;
  rcLastChange4 = micros();
  //Serial.println(ch4);

}

void rcInterrupt5(){
  if(!interruptLock) ch5 = micros() - rcLastChange5;
  rcLastChange5 = micros();
}

void rcInterrupt6(){
  if(!interruptLock) ch6 = micros() - rcLastChange6;
  rcLastChange6 = micros();
}

