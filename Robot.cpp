
//Made by Tomás Roda Martins (t_martins)
//compile: git pull origin
//gcc -Wall -o robot Robot.cpp HMC5883L.cpp  MPU6050.cpp  I2Cdev.cpp -lwiringPi -lpthread
// sudo ./robot
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

int	B1 = 5;
int	B2 = 6;
int	A1 = 3;
int	A2 = 4;
int LED = 2;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t cx, cy, cz;
MPU6050 mpu;
HMC5883L magnetom;
int loops = 0 ;


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
    mpu.setI2CMasterModeEnabled(0);
    mpu.setI2CBypassEnabled(1);
    magnetom.initialize();
    
    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
    printf(magnetom.testConnection() ? "HMC5883L connection successful\n" : "HMC5883L connection failed\n");
    
    printf ("OK\n") ;
}
//
void read_accel () {
    
    mpu.getAcceleration(&ax, &ay, &az);
    
}

void read_gyro () {

    mpu.getRotation(&gx, &gy, &gz);

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
        read_accel ();
        read_gyro  ();
        read_MagneticField ();
        printf ("%5hd %5hd %5hd %5hd %5hd %5hd %5hd %5hd %5hd\n",ax,ay,az,gx,gy,gz,cx,cy,cz);
    }
    motor_speed(0,0,0,0);
    printf ("Done");
}
