#ifndef MOTORGROUP_H    // This stops the compiler trying to include the header multiple times.
#define MOTORGROUP_H

#include "Motor.h"
#include <BasicLinearAlgebra.h>
#include <Adafruit_PWMServoDriver.h>
 

using namespace BLA;
class Motorgroup
{
  public:

  //Variables
    Motor mList[6];
    BLA::Matrix<3,3> rMatrix;		//Rotation Matrix
    BLA::Matrix<3> tMatrix;			//Translational Position
    BLA::Matrix<3> Velocity1;
    BLA::Matrix<3> Velocity2;
    BLA::Matrix<3> Accel1;
    
    BLA::Matrix<3> Vi;
    BLA::Matrix<3> Pi;

    float servoLength;
    float rodLength;
    float M;
    float L;
    float N;

    //Functions    
    
    Motorgroup(Motor motor0, Motor motor1, Motor motor2, Motor motor3, Motor motor4, Motor motor5 );
    void resetPosition();
    void rotateAllMotor(bool reset);
    void rotateMotor(int motorIndex, float angle, bool showValues);
    void translate(float x, float y, float z);
    void incrementMotor(int motorIndex, int dir, int increment, bool showValues);
    void findMotorAngle(bool reset);
    float findDist(BLA::Matrix<3> A, BLA::Matrix<3> B);
    void displayValues();
	void movePlatform(float yawAngle, float pitchAngle, float rollAngle, float ax, float ay, float az,bool reset);

  private:
    float origin[3];
    int i;

};

Motorgroup::Motorgroup(Motor motor0, Motor motor1, Motor motor2, Motor motor3, Motor motor4, Motor motor5 ){
    servoLength = 30;
    rodLength = 163;
    tMatrix =   {0, 0, 155.07};
    Velocity1 = {0, 0, 0};
    Velocity2 = {0, 0, 0};
    Accel1 =    {0, 0, 0};
    mList[0] = motor0;   
    mList[1] = motor1;   
    mList[2] = motor2;  
    mList[3] = motor3;  
    mList[4] = motor4;  
    mList[5] = motor5;
    
}

void Motorgroup::displayValues(){
    for (i=0; i<6; i++){
      mList[i].displayValues();
    }
}
void Motorgroup::resetPosition(){
	 //tMatrix = {0, 0, 155.07};
    tMatrix = {0, 0, 155.05};

    Velocity1 = {0, 0, 0};
    Velocity2 = {0, 0, 0};
    Accel1 = {0, 0, 0};
	movePlatform(0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, true);
}
void Motorgroup::movePlatform(float yawAngle, float pitchAngle, float rollAngle, float ax, float ay, float az, bool reset=false){
    float Theta = float(pitchAngle);
    float Psi = float(yawAngle);
    float Phi = float(rollAngle);
   
//    
	Velocity2(0) = Velocity1(0) + Accel1(0) + 0.5*(ax-Accel1(0));
	tMatrix(0)   = tMatrix(0) + Velocity1(0) + 0.5*(Velocity2(0) - Velocity1(0));
	Velocity1(0) = Velocity2(0);
	Accel1(0) = ax;
    
	Velocity2(1) = Velocity1(1) + Accel1(1) + 0.5*(ay-Accel1(1));
	tMatrix(1)   = tMatrix(1) + Velocity1(1) + 0.5*(Velocity2(1) - Velocity1(1));
	Accel1(1) = ay;
	Velocity1(1) = Velocity2(1);

	Velocity2(2) = Velocity1(2) + Accel1(2) + 0.5*(az-Accel1(2));
	tMatrix(2)   = tMatrix(2) + Velocity1(2) + 0.5*(Velocity2(2) - Velocity1(2));
	Accel1(2) = az;
	Velocity1(2) = Velocity2(2);
	/*

    if(tMatrix(0) > 62.5){
      tMatrix(0) = 62.5;
    }else if(tMatrix(0) < -62.5){
      tMatrix(0) = -62.5;
    }else{
       Velocity2(0) = Velocity1(0) + Accel1(0) + 0.5*(ax-Accel1(0));
       tMatrix(0) = tMatrix(0) + Velocity1(0) + 0.5*(Velocity2(0) - Velocity1(0));
        Velocity1(0) = Velocity2(0);
        Accel1(0) = ax;
    }

    if(tMatrix(1) > 62.5){
      tMatrix(1) = 62.5;
    }else if(tMatrix(1) < -62.5){
      tMatrix(1) = -62.5;
    }else{
      Velocity2(1) = Velocity1(1) + Accel1(1) + 0.5*(ay-Accel1(1));
      tMatrix(1) = tMatrix(1) + Velocity1(1) + 0.5*(Velocity2(1) - Velocity1(1));
      Accel1(1) = ay;
      Velocity1(1) = Velocity2(1);

    }

    if(tMatrix(2) > 170){
      tMatrix(2) = 188;
    }else if(tMatrix(2) < 150){
      tMatrix(2) = 138;
    }else{
    Accel1(2) = az;
    Velocity1(2) = Velocity2(2);
    Velocity2(2) = Velocity1(2) + Accel1(2) + 0.5*(az-Accel1(2));
    tMatrix(2) = tMatrix(2) + Velocity1(2) + 0.5*(Velocity2(2) - Velocity1(2));
    }
    */
    tMatrix = {ax, ay, 155.05+ az};
    
    //tMatrix = tMatrix + posChange;
   

    rMatrix = { 
      cos(Psi)*cos(Theta),    -sin(Psi)*cos(Phi)+cos(Psi)*sin(Theta)*sin(Phi),    sin(Psi) *sin(Phi)+cos(Psi)*sin(Theta)*cos(Phi),
      sin(Psi)*cos(Theta),    cos(Psi) *cos(Phi)+sin(Psi)*sin(Theta)*sin(Phi),    -cos(Psi)*sin(Phi)+sin(Psi)*sin(Theta)*cos(Phi),
      -sin(Theta)        ,    cos(Theta)*sin(Phi)                            ,    cos(Theta)*cos(Phi)
    };
    

    findMotorAngle(reset);
    rotateAllMotor(reset);

}
void Motorgroup::findMotorAngle(bool reset=false){
    for(i=0;i<6;i++){
		//Finding Li coordinates
        mList[i].Platform = tMatrix + (rMatrix * mList[i].localPlatform);
        
        float MagL = findDist({0,0,0},mList[i].Platform);
        mList[i].mLength = mList[i].Platform - mList[i].motorDist;
        
		//Finding Magnitude of Li 
        mList[i].mLengthMag = findDist({0,0,0},mList[i].mLength); //Li

		//Finding other constants
        M = 2 * servoLength * (mList[i].Platform(2) - mList[i].motorDist(2));
        N = 2 * servoLength * (cos(mList[i].betaAngle) * (mList[i].Platform(0) - mList[i].motorDist(0)) + sin(mList[i].betaAngle) * (mList[i].Platform(1) - mList[i].motorDist(1)));
        //N = 2 * servoLength * ((mList[i].Platform(0) - mList[i].motorDist(0)));
        L = sq(mList[i].mLengthMag) - (sq(rodLength) - sq(servoLength));
        

        if(mList[i].mLengthMag > servoLength + rodLength){
          //Serial.println("Error length");
        }else if(L/sqrt(sq(M) + sq(N)) >= 1){
          //Serial.println("Error angle");
        }else{
          mList[i].alphaAngle = asin(L/sqrt(sq(M) + sq(N))) - atan2(N,M);
          //mList[i].alphaAngle = asin(L/sqrt(sq(M) + sq(N))) - atan(N/M);
        }
        
        if(reset){
            mList[i].homeAngle = mList[i].alphaAngle;
        }

        //Serial.println("Beta Angle: " +String(mList[i].betaAngle * 180/M_PI));
        //Serial.println("Length Mag: " +String(mList[i].mLengthMag) + " max L: " + String(servoLength + rodLength));
        //Serial.println("M: " +String(M)+ " N: " +String(N) + " L: " +String(L));
        //Serial.println("MagL: " +String(MagL));
        //Serial.println("Plat x: " +String(mList[i].Platform(0)) + " y: " +String(mList[i].Platform(1)) + " z: " +String(mList[i].Platform(2)) );
        //Serial.println("MDist x: " +String(mList[i].motorDist(0)) + " y: " +String(mList[i].motorDist(1)) + " z: " +String(mList[i].motorDist(2)) );
        //Serial.println("Length x: " +String(mList[i].mLength(0)) + " y: " +String(mList[i].mLength(1)) + " z: " +String(mList[i].mLength(2)) );
        //Add(mList[i].Platform,-mList[i].motorDist,mList[i].mLength);

        
        //Serial.println("Angle for " +mList[i].motorName +": " +String(mList[i].alphaAngle * 180/M_PI));
        
    }
}
float Motorgroup::findDist(BLA::Matrix<3> A, BLA::Matrix<3> B){
  float aLength = sq(A(0)) + sq(A(1)) + sq(A(2)) + sq(B(0)) + sq(B(1)) + sq(B(2)) - 2*(A(0)*B(0) + A(1)*B(1) + A(2)*B(2));
  return abs(sqrt(aLength));
}
void Motorgroup::rotateAllMotor(bool reset=false){
  
    for(i=0;i<6;i++){
      if(reset){
          //Serial.println("Reset rotation");
          mList[i].rotate(mList[i].homeAngle );
      }else{
          mList[i].rotate(mList[i].alphaAngle );
      }
        //Serial.println("Rotating " +mList[i].motorName +" at value: " +String(mList[i].motorValue));
    }
}
void Motorgroup::translate(float x, float y ,float z){
  BLA::Matrix<3> posChange = {x,y,z};
  tMatrix = tMatrix + posChange;
  movePlatform(0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0);
}
void Motorgroup::rotateMotor(int motorIndex, float angle, bool showValues=false){
    mList[motorIndex].rotate(angle);
}
void Motorgroup::incrementMotor(int motorIndex, int dir,  int increment=5, bool showValues=false){
    mList[motorIndex].increment(dir, increment);
}


#endif //MOTORGROUP_H
