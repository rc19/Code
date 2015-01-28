 /////////////////**************FIRST INCLUDE ALL LIBRARIES***********///////////////////////
  #include "I2Cdev.h"
  #include "MPU6050_6Axis_MotionApps20.h"
  // for rc_code//
  #include <Wire.h>
  #include <PinChangeInt.h>
  #include <PinChangeIntConfig.h>
  //for servos//
  #include<Servo.h>
  #include<def.h>
  /////////////////
  //**variables for motors**//////////////
  int temp,Speed; 
  Servo m1,m2,m3,m4;
 //////////////
  //variables for sensros////
#define YAW_DIRECTION 1
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
//  variables for rc//

#define RC_1 5
#define RC_2 6 
#define RC_3 11
#define RC_4 10
#define RC_5 9
#define RC_6 3

#define RC_HIGH_CH1 2000
#define RC_LOW_CH1 1000
#define RC_HIGH_CH2 2000
#define RC_LOW_CH2 1000
#define RC_HIGH_CH3 2000
#define RC_LOW_CH3 1000
#define RC_HIGH_CH4 2000
#define RC_LOW_CH4 1000
#define RC_HIGH_CH5 2000
#define RC_LOW_CH5 1000
#define RC_HIGH_CH6 2000
#define RC_LOW_CH6 1000

boolean interruptLock = false;
unsigned long rcLastChange1 = micros();
unsigned long rcLastChange2 = micros();
unsigned long rcLastChange3 = micros();
unsigned long rcLastChange4 = micros();
unsigned long rcLastChange5 = micros();
unsigned long rcLastChange6 = micros();

/////////////////////////////////////
//******************global variable for rc********************* //
static float ch1, ch2, ch3, ch4, ch5 ,ch6; // global variables
static int16_t rcData[8];          // interval [1000;2000]
int rcCommand[4]; // Roll->pitch->yaw->throttle

///***************************** global variables for sensors*************************//

#include "I2Cdev.h"     
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define calibration_sensors
#define calibration_time
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t gyroADC[3];


int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

#define OUTPUT_READABLE_YAWPITCHROLL
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
static float angle[3]    = {0,0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;// if you have initial values, initialise them here only.
/////////////////////////////////////////////////////////////////////////////



static int16_t motor[4];



//******************** for PID********************//
  #define GRAVITY 512
  #define GYRO_SCALE 0.00762 //4.
class confi
{ 
  public:
  float Pa[3];
  float Pg[3];
  float Ia[3];
  float Ig[3];
  float Di[3];
  
  float sdf[3];
  float angletrim[3];
  float kp ;
  float ki ;
}cofig;
  
  static float axisPID[3];
  uint8_t axis;
  int error,errorAngle;
  int16_t delta,deltaSum;
  float Pterm,Iterm,Dterm;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3],delta2[3];
  static int errorGyroI[3] = {0,0,0};
  static int errorAngleI[2] = {0,0};




void setup()
{
  // Intial variable declaration for motors, rc and sensors
  // Initial value assignment for the variables
Serial.begin(115200);
cofig.kp = 1;
cofig.ki = 1.0;

cofig.Pa[YAW]   = 2;
cofig.Pa[PITCH] = 2; 
cofig.Pa[ROLL]  = 2;

cofig.Pg[YAW]   = .95;
cofig.Pg[PITCH] = .95; 
cofig.Pg[ROLL]  = .95;


cofig.sdf[ROLL] = .32;
cofig.sdf[PITCH] = .32;
cofig.sdf[YAW] = .32;

cofig.Ia[ROLL] = 0;
cofig.Ia[PITCH] = 0;
cofig.Ia[YAW]  = 0;

cofig.Ig[ROLL] = 0;
cofig.Ig[PITCH] = 0;
cofig.Ig[YAW]  = 0;

cofig.Di[ROLL] =1.0f;
cofig.Di[PITCH] =1.0f;
cofig.Di[YAW] =1.0f;


  

    //configure_motors();
    acquireLock();
    get_rc();
    config_sensors();
    configure_escs(); // configure and initialise the variables
    



}
int cycle = 0;
void loop()
{
   
   //get_angles(true);// every 5th loop 
   // Angle_status will tell the function whether we want the angles or not which will be every fifth cycle
   // it updates gyro values and angle[with every fifth cycle]
  Serial.print(ch1);
  Serial.print(ch2);
  Serial.print(ch3);
  Serial.println(ch4);
  i++;
  if(i==6)
  {
  get_angles(1);
  Serial.println(angle[0]);
  Serial.println(angle[1]);
  Serial.println(angle[2]); 
  i=0;
  }
  int command;
  for(axis=0;axis<3;axis++) 

{  
    if( rcCommand[axis] < 2000)
    {

    command = (float)rcCommand[axis]-1500.0;   
    if (command > 7.0)
     command = command-7;
    else if (command <-7)
     command = command +7;
    else 
     command = 0;

    }
    
    
  //rcCommand[axis] = 1500;
  if  ( 1 && axis<2)
   { 
      
    errorAngle = (command)*cofig.sdf[axis] - angle[axis];
    Pterm = errorAngle*cofig.Pa[axis]; // And this Pa value should be around .04
    //constrain(Pterm ,-   
    errorAngleI[axis] = errorAngleI[axis] + errorAngle;
    errorAngleI[axis] = constrain(errorAngleI[axis], -600,600);
    Iterm = (errorAngleI[axis]*(cofig.Ia[axis]))/500.0 ;
    //if ( Iterm >2
//    if (errorAngleI[axis]>1050)
//     errorAngleI[axis] = 0 ;
     
   }
  
  else 
   {
 
    error = (command)*cofig.sdf[axis];
    error-= (int)gyroADC[axis]*(float)GYRO_SCALE;//dyna.P8 is to convert the raw gyro data to angle value
    
    
    Pterm = (command)*cofig.sdf[axis]*(cofig.Pa[YAW]); // here Pa is around .04
    //Pterm = Pterm*(cofig.sdf[axis])*(cofig.Pa[axis]);
    //Pterm = constrain (Pterm,-200.0,200.0);
    
    errorGyroI[axis] = constrain(errorGyroI[axis] + error,-1021.0 ,1021.0);
    
    if (abs (gyroADC[axis])>1000.0)             //if any big change happens than the integral term should become zero
     errorGyroI[axis]=0;                        //considering that the actual pterm will better take care of that.
    Iterm = (errorGyroI[axis]*cofig.Ig[axis])/32.0;
    

   }
   
   // constant is equal to 5*1
  Pterm -=  (float)gyroADC[axis]*(cofig.Pg[axis]); // Pg is around .2*  
  
  Pterm = Pterm*cofig.kp;
  Pterm = constrain(Pterm ,-150.0,150.0);
    if (Iterm >.5 ||Iterm < -.5)
     Iterm =0 ;   
    
                                             
  delta = (int)gyroADC[axis]-lastGyro[axis];                     //gyrodata to angle {after multiplying the cycletime}
  lastGyro[axis] = (int)gyroADC[axis];
  deltaSum       = delta1[axis]+delta2[axis]+delta;
  delta2[axis]   = delta1[axis];
  delta1[axis]   = delta;
  
  Dterm = (deltaSum*(float)GYRO_SCALE*.33*(cofig.Di[axis]));
  
  
  axisPID[axis] = -Pterm - Iterm;
  //if (axis==0)
  //Serial.println(axisPID[axis]);
//  if (axis == 2)
//  Serial.println(axisPID[axis]);
  
}
}
void get_angles(int angle_status)
{
	
    if (angle_status)
    {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) 
    {
      
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) 
   { 
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        /*
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif
        */
        /*
        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif
        */

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            */
            angle[0] = ypr[0] * 180/M_PI;
            angle[1] = ypr[1] * 180/M_PI;
            angle[2] = ypr[2] * 180/M_PI;

        #endif

        /*
        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif
        */
         /*
        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    */
      
      
    }
    }
    
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);    
        #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
            Serial.print("a/g:\t");
            Serial.print(ax); Serial.print("\t");
            Serial.print(ay); Serial.print("\t");
            Serial.print(az); Serial.print("\t");
            Serial.print(gx); Serial.print("\t");
            Serial.print(gy); Serial.print("\t");
            Serial.println(gz);
        #endif
}
 









