#ifndef MOTOR_H    // This stops the compiler trying to include the header multiple times.
#define MOTOR_H
#include <BasicLinearAlgebra.h>

using namespace BLA;
class Motor{
  public:

    //Variables
    
	//Static
    BLA::Matrix<3>  motorDist;      // bi
    BLA::Matrix<3>  localPlatform;  // pi
	  float betaAngle;
    float homeAngle;
	  String motorName;
    int motorPin;
    int maxAbsValue;
    int minAbsValue;
    int maxValue;
    int minValue;
    int midValue;
    bool reversed;

	//Dynamic
    BLA::Matrix<3>  Platform;       // qi
    BLA::Matrix<3>  mLength;        // li

	  float mLengthMag;
    float alphaAngle;
    float motorValue;

    //Functions
    Motor();
    Motor(String mName, int mPin, int theAbsMax, int theAbsMin, int theMin, int theMax, BLA::Matrix<3> motorDistance, BLA::Matrix<3> plat, float bAngle, bool isReversed);
	  void rotate(float angle);
    void rotateMax();
    void rotateMin();

    void displayValues();
    void increment(int rotateDirection, int increment);
    
};
Motor::Motor(){
  motorName = "None";
}

Motor::Motor(String mName, int mPin, int theAbsMax, int theAbsMin, int theMin, int theMax, BLA::Matrix<3> motorDistance, BLA::Matrix<3> plat, float bAngle, bool isReversed){
  
  //Static
  homeAngle = 0;
  reversed = isReversed;
  motorDist = motorDistance;
  localPlatform = plat;
  //betaAngle = atan2(motorDistance(1),motorDistance(0));
  betaAngle = bAngle;
  motorName = mName;
  motorPin = mPin;
  maxAbsValue = theAbsMax;
  minAbsValue = theAbsMin;
  maxValue = theMax;
  minValue = theMin;
  midValue = (theMax + theMin)/2;

  //Dynamic
  Platform = {}; //Set in Motorgroup
  mLength = {}; //Set in Motorgroup
  mLengthMag = 0; //set in Motorgroup
  alphaAngle = 0; //set in Motorgroup
  if(reversed){
    motorValue = maxValue;
  }else{
    motorValue = minValue;    
  }

}

void Motor::displayValues(){
    Serial.println(String(motorName) + " properties" );
    Serial.println("Max:" + String(maxValue) + "  Min: " + String(minValue) + "   Motor value : " + String(motorValue));
	  //Serial.println("Platform Vector x: " + String(Platform(0)) + "  y: " + String(Platform(1)) + "   z : " + String(Platform(2)));
	  //Serial.println("Length Magnitude: " + String(mLengthMag)) ;

}
void Motor::rotate(float angle){
    float angleRatio;
    angleRatio = (angle-homeAngle)/(5*M_PI/9);
    if(angleRatio > 1){
      angleRatio = 1;
    }else if(angleRatio < -1){
      angleRatio = -1;
    }
    //Serial.println("Alpha: " +String(angle * 180/M_PI) + " Local ALpha: " + String(homeAngle * 180/M_PI));
    if(reversed){
        motorValue = midValue + angleRatio * (maxValue - minValue)/2;
    }else{
        motorValue = midValue - angleRatio * (maxValue - minValue)/2;
    }
}

void Motor::rotateMax(){
      if(reversed){

        motorValue = maxValue;
        }else{
          motorValue = minValue;
        }
}

void Motor::rotateMin(){
          if(reversed){

        motorValue = minValue;
        }else{
          motorValue = maxValue;
        }
}

void Motor::increment(int rotateDirection, int increment=5){
  if(rotateDirection == 0){
      motorValue -= increment;
  }else{
      motorValue += increment;
  }
  displayValues();
}

#endif //MOTOR_H
