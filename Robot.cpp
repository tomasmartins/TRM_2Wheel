//Made by Tomás Roda Martins (t_martins)
//compile: git pull origin
//gcc -Wall -o robot Robot.cpp HMC5883L.cpp  MPU6050.cpp  I2Cdev.cpp -lwiringPi -lpthread
// sudo ./robot

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "MPU6050_6Axis_MotionApps20.h"

int	B1 = 5;
int	B2 = 6;
int	A1 = 3;
int	A2 = 4;
int LED = 2;
int16_t cx, cy, cz;
MPU6050 mpu;
HMC5883L magnetom;
int loops = 0 ;

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



//setup

 void setup (void)
{
    
    if (geteuid () != 0)
    {
        fprintf (stderr, "TM Needs to be root to run!") ;
        exit (0) ;
    }
    
    if (wiringPiSetup () == -1)
        exit (1) ;
    
    softPwmCreate (A1, 0 , 100) ;
    softPwmCreate (A2, 0 , 100) ;
    softPwmCreate (B1, 0 , 100) ;
    softPwmCreate (B2, 0 , 100) ;
    
    printf ("Setup ... ") ; fflush (stdout) ;
    pinMode (LED, OUTPUT) ;
    // initialize MPU6050 and HMC5883L.
    printf("Initializing I2C devices...\n");
    mpu.initialize();
 //   mpu.setI2CMasterModeEnabled(0);
 //   mpu.setI2CBypassEnabled(1);
    magnetom.initialize();
    
    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
    printf(magnetom.testConnection() ? "HMC5883L connection successful\n" : "HMC5883L connection failed\n");
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);
        
        mpuIntStatus = mpu.getIntStatus();
        
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;
        
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }
    
    printf ("OK\n") ;
}

void dmp_data (){
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        printf("FIFO overflow!\n");
        
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
/*
        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        printf("euler %7.2f %7.2f %7.2f    ", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);

        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        printf("areal %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);
*/
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);

        

        printf("\n");
    }


}


void read_MagneticField () {

    magnetom.getHeading(&cx, &cy, &cz);

}

//Sets the speed of the motor.
//SA1/2 is the speed of motor A SB1/2 is the speed os motor B

void motor_speed (int SA1,int SA2,int SB1,int SB2){

   softPwmWrite (A1,SA1) ;
   softPwmWrite (A2,SA2) ;
   softPwmWrite (B1,SB1) ;
   softPwmWrite (B2,SB2) ;
}

void ProsingSensorData (){


printf ("Done");

}

//Main:

int main (void)
{
    setup () ;
    motor_speed(50,10,50,10);
    while(loops < 100){
        loops++;
        delay(20);
        dmp_data ();

    }
    motor_speed(0,0,0,0);
    printf ("Done");
}