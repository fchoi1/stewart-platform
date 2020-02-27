/* =================================================================================================== 
 *  This code has been provided as an example to help you get started on your project. The objective 
 *  is to provide user input to the Arduino board and have the servo motors actuate. Several lines of 
 *  code are accredited to the Adafruit PWM Servo Driver Library example code. To use the Adafruit 
 *  PWM Servo Shield, be sure to add the Adafruit library to your Arduino IDE. 
 *  (Adafruit example: File menu > Examples > Adafruit PWM Servo Drivers Library > servo)
 *  
 *  Add Adafruit Library: In the Arduino IDE application select: Sketch menu > Include Libraries > 
 *  Manage Libraries. In the Library Manager window, search and install the "Adafruit PWM Servo 
 *  Driver Library".
 *  
 *  NOTE: Depending on your servo motor, the pulse width min/max may be different. Adjust to match 
 *  your servo motor.
 =================================================================================================== */

#include "Motor.h"
#include "Motorgroup.h"
#include "imu.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>


int buttonPin1 = 10; //Start button 
int buttonPin2 = 11; //Stop button 

int ledPin = 8;
int buttonStatus1 = 0; 
int buttonStatus2 = 0; 

bool running = false;
String valInput;
int i;
int j;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*
Servo 1  ~ 175 degrees  (490, 130)
Servo 2  ~ 135 Degrees (620, 155)
Servo 3  ~ 175 Degrees (480, 130)
Servo 4  ~ 135 Degrees (630, 170)
Servo 5  ~ 175 Degrees (510, 130)
Servo 6  ~ 175 Degrees (480, 130)

Motor motor0("motor 0", 0, 510, 130, 235, 415,  {-82.87,  101.84,  0}, {-69.39,   71.82,  0},  false) ; //Motor 0  mid = 320
Motor motor1("motor 1", 1, 580, 160, 230, 450,  {82.87,   101.84,  0}, {69.39,    71.82,  0},  true); //Motor 1  mid = 320
Motor motor2("motor 2", 2, 490, 140, 245, 415,  {129.63,  20.85,   0}, {96.89,    24.18,  0},  false) ; //Motor 2  mid = 315
Motor motor3("motor 3", 3, 480, 130, 235, 405,  {46.76,   -122.69, 0}, {27.5,     -96,    0},  true); //Motor 3  mid = 305
Motor motor4("motor 4", 4, 630, 170, 240, 445,  {-46.76,  -122.69, 0}, {-27.5,    -96,    0},  false) ; //Motor 4  mid = 400 
Motor motor5("motor 5", 5, 490, 130, 235, 410,  {-129.63, 20.85,   0}, {-96.89,   24.18,  0},  true); //Motor 5  mid = 310
*/

Motor motor0("motor 0", 0, 510, 130, 235, 415,  {-82.87,  101.84,  0}, {-69.39,   71.82,  0}, 60.0  ,false) ; //Motor 0  mid = 320
Motor motor1("motor 1", 1, 580, 160, 230, 450,  {82.87,   101.84,  0}, {69.39,    71.82,  0}, 120.0 ,true); //Motor 1  mid = 320
Motor motor2("motor 2", 2, 490, 140, 245, 415,  {129.63,  20.85,   0}, {96.89,    24.18,  0}, 120.0 ,false) ; //Motor 2  mid = 315
Motor motor3("motor 3", 3, 480, 130, 235, 405,  {46.76,   -122.69, 0}, {27.5,     -96,    0}, 0.0   ,true); //Motor 3  mid = 305
Motor motor4("motor 4", 4, 630, 170, 240, 445,  {-46.76,  -122.69, 0}, {-27.5,    -96,    0}, 0.0   ,false) ; //Motor 4  mid = 400 
Motor motor5("motor 5", 5, 490, 130, 235, 410,  {-129.63, 20.85,   0}, {-96.89,   24.18,  0}, 60.0  ,true); //Motor 5  mid = 310

/*
Motor motor0("motor 0", 0, 510, 130, 205, 385,  {-82.87,  101.84,  0}, {-69.39,   71.82,  0},  false) ; //Motor 0  mid = 320 
Motor motor1("motor 1", 1, 580, 160, 260, 480,  {82.87,   101.84,  0}, {69.39,    71.82,  0},  true)  ; //Motor 1  mid = 320
Motor motor2("motor 2", 2, 490, 140, 215, 385,  {129.63,  20.85,   0}, {96.89,    24.18,  0},  false) ; //Motor 2  mid = 315
Motor motor3("motor 3", 3, 480, 130, 265, 435,  {46.76,   -122.69, 0}, {27.5,     -96,    0},  true)  ; //Motor 3  mid = 305
Motor motor4("motor 4", 4, 630, 170, 210, 415,  {-46.76,  -122.69, 0}, {-27.5,    -96,    0},  false) ; //Motor 4  mid = 400 
Motor motor5("motor 5", 5, 490, 130, 265, 440,  {-129.63, 20.85,   0}, {-96.89,   24.18,  0},  true)  ; //Motor 5  mid = 310
*/
//Initialize motor group
Motorgroup theMotorgroup(motor0, motor1, motor2, motor3, motor4, motor5);

//Initialize IMU
imu theIMU;
int calibrate = 64; // Used for initial positions

BLA::Matrix<3,3> AccelR;		//Rotation for acceleration Matrix
BLA::Matrix<3> rawA;			//Raw accel values
BLA::Matrix<3> realA;			// Linear Accel Values
BLA::Matrix<3> g = {0,0,9.81};

float intialYaw,intialPitch,intialRoll; 
float intialX, intialY, intialZ;
float Ax, Ay, Az;
float Axavg, Ayavg, Azavg;
float angleOne, angleTwo, angleThree;
int countx = 0, county = 0, countz = 0;
int Vthreshold = 8, Pthreshold = 1;

//Testing
float theAngle = M_PI/2;
int dir = 0; 
int d = 500;

unsigned long ElaspedTime;
unsigned long StartTime;

bool testLoop = false;

void setPwm(){
	for (i=0; i<6; i++) {
    //Serial.println("Values: " +String(theMotorgroup.mList[i].motorValue));
		pwm.setPWM(theMotorgroup.mList[i].motorPin, 0, theMotorgroup.mList[i].motorValue); 
	}
}

void setMaxPwm(){
  for (i=0; i<6; i++) {
    //Serial.println("Values: " +String(theMotorgroup.mList[i].motorValue));
    theMotorgroup.mList[i].rotateMax();
    pwm.setPWM(theMotorgroup.mList[i].motorPin, 0, theMotorgroup.mList[i].motorValue); 
  }
}

void setMinPwm(){
  for (i=0; i<6; i++) {
    //Serial.println("Values: " +String(theMotorgroup.mList[i].motorValue));
    theMotorgroup.mList[i].rotateMin();
    pwm.setPWM(theMotorgroup.mList[i].motorPin, 0, theMotorgroup.mList[i].motorValue); 
  }
}

void reset(){
    theMotorgroup.resetPosition();
    setPwm();
    delay(200);
    Serial.println("Reseting positions");
}

void setup() {

  //Initialize Button
  pinMode(buttonPin1, INPUT); 
  pinMode(buttonPin2, INPUT);  
 
  pinMode(ledPin, OUTPUT);  

  //Initialize Serial
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  
  Serial.println("Serial Working");
  Serial.println("Setting up IMU");

  //Initialize IMU
  theIMU.setupIMU();
  
  //Intialize PWM for servos and other stuff
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  //Intialize motor group and positions
  Serial.println("Reseting positions");
  reset();
  delay(100);
  
  StartTime = millis();
  ElaspedTime = millis();

  //theMotorgroup.displayValues();
  Serial.println(" ======================= STARTING MAIN LOOP ==================================");
      digitalWrite(ledPin, HIGH);
      delay(200); 
      digitalWrite(ledPin, LOW);
      delay(200);  
}

void loop() {

  buttonStatus1 = digitalRead(buttonPin1);
 
  //Check(==) if the first button(START) is HIGH, AND(&&) the second button (STOP) is LOW, if yes turn the LED on. 

  if (buttonStatus1 == HIGH){
    Serial.println("Starting");
    digitalWrite(ledPin, HIGH); 
    delay(200);
    Ax = 0; Ay = 0; Az = 0;
    Axavg = 0; Ayavg = 0; Azavg = 0;
    intialPitch = 0; intialRoll = 0;
    intialX =  0; intialY = 0; intialZ =  0;
    i = 0;
    do{
      theIMU.getAngles(false);
      intialPitch = intialPitch +  theIMU.Acc_angle_x;
      intialRoll = intialRoll +  theIMU.Acc_angle_y;
      intialX =  intialX + theIMU.Acc_rawX;
      intialY =  intialY + theIMU.Acc_rawY;
      intialZ =  intialZ + theIMU.Acc_rawZ;
      i = i + 1;
      }while(i <= calibrate);
      
    intialYaw =  0.0;
    intialPitch = intialPitch/calibrate;
    intialRoll =  intialRoll/calibrate;
    intialX =    intialX/calibrate;
    intialY =    intialY/calibrate;
    intialZ =    intialZ/calibrate;
    
    StartTime = millis();
    Serial.println("YPR: " +String(intialYaw,3) + " " +String(intialPitch,4) +" " +String(intialRoll,4) +"    XYZ: " +String(intialX,4) +" " +String(intialY,4) + " " +String(intialZ,4) );
    running = true;
  }

  buttonStatus1 = digitalRead(buttonPin1);
  
  if (buttonStatus2 == HIGH){
      Serial.println("Reset Stop");
      digitalWrite(ledPin, LOW); 
      reset();
      delay(500);      
  }
  buttonStatus2 = digitalRead(buttonPin2); 

  while(running){

    //Exit loop if button is pressed
      if (buttonStatus2 == HIGH){
          Serial.println("Stopping");
          digitalWrite(ledPin, LOW); 
          reset();
          delay(500);
          running = false;
          break;
      }

      if(testLoop){
              

	Serial.println("- 25 Y");	
	theMotorgroup.mList[0].rotate(5.52 + theMotorgroup.mList[0].homeAngle);
	theMotorgroup.mList[1].rotate(5.52 + theMotorgroup.mList[1].homeAngle);
	theMotorgroup.mList[2].rotate(-23.18 + theMotorgroup.mList[2].homeAngle);
	theMotorgroup.mList[3].rotate(-22.98 + theMotorgroup.mList[3].homeAngle);
	theMotorgroup.mList[4].rotate(-22.98 + theMotorgroup.mList[4].homeAngle);
	theMotorgroup.mList[5].rotate(-23.18 + theMotorgroup.mList[5].homeAngle);
	setPwm();

        delay(d);
        theMotorgroup.resetPosition();
        setPwm();
        delay(d);
      
/*
      for(j=-70; j<=70; j++){
       
      theMotorgroup.movePlatform(0 ,0, 0, 0, j, 0);
      setPwm();
      }
        Serial.println("- Y");

      for(j=70; j>=-70; j--){
       
      theMotorgroup.movePlatform(0 ,0, 0, 0, j, 0);
      setPwm();
      }

        Serial.println("+ XY");

      for(j=70; j>=-70; j--){
       
      theMotorgroup.movePlatform(0 ,0, 0, j/2,j/2, 0);
      setPwm();
      }
*/

      
//
//      delay(d);
//      theMotorgroup.resetPosition();
//      setPwm();
//      delay(d);
////
//      Serial.println("+ X");
//      theMotorgroup.movePlatform(0 ,0, 0, 70, 0, 0);
//      setPwm();
      



//      
//      Serial.println("+ Z");
//      theMotorgroup.movePlatform(0 ,0, 0, 0, 0, -20);
//      setPwm();
//      
//      delay(d);
//      theMotorgroup.resetPosition();
//      setPwm();
//      delay(d);
//      
//      Serial.println("- Z");
//      theMotorgroup.movePlatform(0 ,0, 0, 0, 0, 20);
//      setPwm();
//
//      delay(d);
//      theMotorgroup.resetPosition();
//      setPwm();
//      delay(d);
//      
//      Serial.println("Yaw");
//      theMotorgroup.movePlatform(M_PI/6 ,0, 0, 0, 0, 0);
//      setPwm();
//      delay(d);
//      
//      theMotorgroup.resetPosition();
//      setPwm();
//      delay(d);
//      
//      Serial.println("-Yaw");
//      theMotorgroup.movePlatform(-M_PI/6 ,0, 0, 0, 0, 0);
//      setPwm();
//      delay(d);
//      
//      theMotorgroup.resetPosition();
//      setPwm();
//      delay(d);
//      
//      Serial.println("pitch");
//      theMotorgroup.movePlatform(0 ,M_PI/12, 0, 0, 0, 0);
//      setPwm();
//
//      theMotorgroup.resetPosition();
//      setPwm();
//      delay(d);
//      
//      Serial.println("-pitch");
//      theMotorgroup.movePlatform(0 ,-M_PI/12, 0, 0, 0, 0);
//      setPwm();
//
//      theMotorgroup.resetPosition();
//      setPwm();
//      delay(d);
//      
//      Serial.println("roll");
//      theMotorgroup.movePlatform(0 ,0, M_PI/12, 0, 0, 0);
//      setPwm();
//
//      theMotorgroup.resetPosition();
//      setPwm();
//      delay(d);
//      
//      Serial.println("-roll");
//      theMotorgroup.movePlatform(0 ,0, -M_PI/12, 0, 0, 0);
//      setPwm();      
      
    }else if(not testLoop){
      
		theIMU.getAngles(false);
		ElaspedTime = millis();
		angleOne = 0;
		angleTwo = (theIMU.Acc_angle_x - intialPitch)* 1/4;
		angleThree = (theIMU.Acc_angle_y - intialRoll)* 1/4;

		rawA = {theIMU.Acc_rawX - intialX, theIMU.Acc_rawY - intialY, theIMU.Acc_rawZ - intialZ };

		AccelR = { 
		  cos(angleOne*4)*cos(angleTwo*4),    -sin(angleOne*4)*cos(angleThree*4)+cos(angleOne*4)*sin(angleTwo*4)*sin(angleThree*4),    sin(angleOne*4) *sin(angleThree*4)+cos(angleOne*4)*sin(angleTwo*4)*cos(angleThree*4),
		  sin(angleOne*4)*cos(angleTwo*4),    cos(angleOne*4) *cos(angleThree*4)+sin(angleOne*4)*sin(angleTwo*4)*sin(angleThree*4),    -cos(angleOne*4)*sin(angleThree*4)+sin(angleOne*4)*sin(angleTwo*4)*cos(angleThree*4),
		  -sin(angleTwo*4)				,     cos(angleTwo*4)*sin(angleThree*4)													  ,     cos(angleTwo*4)*cos(angleThree*4)
		};

		realA = AccelR*rawA + g;
		Ax = realA(0);
		Ay = realA(1);
		Az = realA(2);


		if(abs(Ax) < 0.2){
		Ax = 0;
		}
		if( abs(Ay) < 0.2){
		Ay = 0;
		}
		if( abs(Az) < 0.2){
		Az = 0;
		}
    
		if(Ax == 0){
			countx = countx + 1;
			}else{
			countx = 0;
			}
		if(Ay == 0){
			county = county + 1;
			}else{
			county = 0;
			}
		if(Az == 0){
			countz = countz + 1;
			}else{
			countz = 0;
			}
    
		if(countx > Vthreshold){
			theMotorgroup.Velocity2(0) = 0;
			theMotorgroup.Velocity1(0) = 0;
		}
		if(county > Vthreshold){
			theMotorgroup.Velocity2(1) = 0;
			theMotorgroup.Velocity1(1) = 0;
		}
		if(countz > Vthreshold){
			theMotorgroup.Velocity2(2) = 0;
			theMotorgroup.Velocity1(2) = 0;
		}

    
//    Serial.println(String((ElaspedTime-StartTime)) + " " +String(Ax,4) + " " + String(Ay,4) + " " + String(Az,4)
//	+" speed " +String(theMotorgroup.Velocity2(0),4) + " " + String(theMotorgroup.Velocity2(1),4) + " " + String(theMotorgroup.Velocity2(2),4) 
//	+" Position " +String(theMotorgroup.tMatrix(0),4) + " " +String(theMotorgroup.tMatrix(1),4) + " " +String(theMotorgroup.tMatrix(2),4));

    //Serial.println(String((ElaspedTime-StartTime)) + " " +String(Ax,4) + " " + String(Ay,4) + " " + String(Az,4) );

	//==================== Print Position ====================
    //Serial.println(String((ElaspedTime-StartTime)) + " " +String(theMotorgroup.tMatrix(0)) + " " +String(theMotorgroup.tMatrix(1)) + " " +String(theMotorgroup.tMatrix(2)));

	// ==================== Print Angle ====================
    //Serial.println(String((ElaspedTime-StartTime)) + " " +String(angleOne * 180/M_PI) + " " + String(angleTwo* 180/M_PI) + " " + String(angleThree* 180/M_PI));


	//==================== Both ====================
    //theMotorgroup.movePlatform( angleOne, angleTwo,  angleThree, Ax, Ay, Az);

	//==================== No Rotation ====================
    //theMotorgroup.movePlatform( 0, 0,  0, Ax, Ay, Az);

	//==================== No translation ====================
    theMotorgroup.movePlatform( angleOne, angleTwo,  angleThree, 0, 0, 0);

    setPwm();


    }
    buttonStatus2 = digitalRead(buttonPin2); 


    //Testing keyboard inputs
    
    if (Serial.available() > 0) {

      valInput = Serial.readString();
      Serial.print("I received: ");
      Serial.print(valInput);
      switch (valInput[0]) {
        // Input of "1" to "6" -> increase respective (1..6) values
        // Input of [q,w,e,r,t,y] -> decrease respective (1..6) values
        
        case 'q': 
          theAngle = 0;
          Serial.println("Angle 0");
          break;
        case 'w': 
          theAngle = M_PI/2;
          Serial.println("Angle 90");
        break;
        
        case 'e': 
          theAngle = M_PI;
          Serial.println("Angle 180");
        break;
          
        case 'r':
          dir = 0;
          Serial.println("direction up");
        break;
          
        case 't':
          dir = 1;
          Serial.println("direction down");
        break;
          
        case '1':
          theMotorgroup.rotateMotor(1,theAngle,true);
        break;
        
        case '2':
          theMotorgroup.rotateMotor(2,theAngle,true);
        break;
        
        case '3': 
          theMotorgroup.rotateMotor(3,theAngle,true);
        break;
 
        case '4':
          theMotorgroup.rotateMotor(4,theAngle,true);
        break;
        
        case '5':
          theMotorgroup.rotateMotor(5,theAngle,true);
        break;

        case '6':
          theMotorgroup.rotateMotor(0,theAngle,true);
        break;

        case 'a':
          theMotorgroup.incrementMotor(1,dir);
        break;
        
        case 's':
          theMotorgroup.incrementMotor(2,dir);
        break;
        
        case 'd':
          theMotorgroup.incrementMotor(3,dir);
        break;
        
        case 'f':
          theMotorgroup.incrementMotor(4,dir);
        break;
        
        case 'g':
          theMotorgroup.incrementMotor(5,dir);
        break;
        
        case 'h':
          theMotorgroup.incrementMotor(0,dir);
        break;
        
        default: Serial.print(" No action taken");
      } // end switch statement

      
      
    }
  }
}
