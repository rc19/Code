void mixTables()
{
  int16_t maxMotor;
  //uint8_t i;

  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

 
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));

  /****************                   main Mix Table                ******************/
 
//  #ifdef motor
//    QUADP[0] = PIDMIX( 0,+1,-1); //REAR
//    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
//    motor[2] = PIDMIX(+1, 0,+1); //LEFT
//    motor[3] = PIDMIX( 0,-1,-1); //FRONT
//  #endif
//  #ifdef QUADX
    
    motor[0] = PIDMIX(1,1,+1); //REAR_R
    motor[1] = PIDMIX(1,-1,-1); //FRONT_R
    motor[2] = PIDMIX(-1,-1,+1); //REAR_L
    motor[3] = PIDMIX(-1,1,-1); //FRONT_L
    
    write_motors();
}

void write_motors()
{

     m1.writeMicroseconds(motor[0]);
     m2.writeMicroseconds(motor[1]);
     m3.writeMicroseconds(motor[2]);
     m4.writeMicroseconds(motor[3]);

}






