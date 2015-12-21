
#include<Wire.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 7); // RX, TX
float YPR[3];


unsigned char Re_buf[8], counter = 0;
unsigned char sign = 0;

int toreadGY25 = 1;
int powers = 0;
boolean battOK = true;
int onKiR = 0;
int onKiP = 0;
float pitchKit = 0;
float rollsKit = 0;


#define motor1  3
#define motor2  9
#define motor3  10
#define motor4  11

int countTemp = 0, ch3_powerTemp = 0;

int motor_A, motor_B, motor_C, motor_D;
float q_yaw, q_roll, q_pitch;

int countTempAngle = 0;
float rollTemp, pitchTemp;


unsigned long timer, sampling, timeDetect, timeLED13, samplePID, sampling2;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

// SET Variables to read data ESP
String inputString = "", b;        // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int ch1_Eleveltor = 126, ch2_roll = 126, ch3_power = 0, ch4_yaw = 126;
//char c[40];
//int countC;
//byte inByte;
//int ic[4];
boolean battStates = false;
int sensorValue = 0;        // value read from the pot
int ledState = LOW;
boolean caribate = false, tackFirstMPU = true;



//^^^^^^^^^^^^^^ Set var PID ^^^^^^^^^^^^^^^//
//Define Variables we'll be connecting to
double  SetpointP = 0, SetpointR = 0, SetpointY = 0;
double rollKp = 0, rollKi = 0, rollKd = 0;
int SampleTime  =  20;
float rollPitchYawGain = 0.4;

double  InputRoll, OutputRoll, OutputRollR; // right <---> left
//double InputRoll24, OutputRoll24;

double  InputPicht, OutputPicht, OutputPichtR; // forward <-----> back
//double InputPicht24, OutputPicht24;

double  InputYaw, OutputYaw, OutputYawR; // spin left <------> right  !!!!!!!!!!!   high 14 for spin left
//double InputYaw23, OutputYaw23; // !!!!!!!!!!!!!!   high  23 for spin right
float trimYaw = 0;



PID rollsR  (&InputRoll,  &OutputRollR,  &SetpointR, 0.8 , 0, 0.22,   DIRECT); // if roll < 0  13 Up(Throttle + OutputRoll)  and   if roll > 0  24 up(Throttle + OutputRoll)
PID pichtsR (&InputPicht, &OutputPichtR, &SetpointP, 0.8 , 0, 0.22,   DIRECT); // if Picht < 0 12 up(Throttle + OutputPicht)  and if picht > 0 34  up(Throttle + OutputPicht)
PID yawsR   (&InputYaw,   &OutputYawR,   &SetpointY, 0 , 0, 0, DIRECT); // spin right value degree +
//PID howers (&InputHower,   &OutputHower,   &SetpointHower, 1, 1, 1, DIRECT); // if az < setpoint  all motor up   and   az > setpoint all motor down

//revers PID
PID rolls  (&InputRoll,  &OutputRoll,  &SetpointR, 0.8 , 0, 0.24,   REVERSE); // if roll < 0  13 Up(Throttle + OutputRoll)  and   if roll > 0  24 up(Throttle + OutputRoll)
PID pichts (&InputPicht, &OutputPicht, &SetpointP, 0.8 , 0, 0.24,  REVERSE); // if Picht < 0 12 up(Throttle + OutputPicht)  and if picht > 0 34  up(Throttle + OutputPicht)
PID yaws   (&InputYaw,   &OutputYaw,   &SetpointY, 0 , 0, 0, REVERSE); // if yaw < 0 23 up(Throttle + OutYaw) and if yaw > 0 14 up(Throttle + OutYaw)



void setup() {
  // put your setup code here, to run once:
  // Wire.pins(0, 2);
  //Wire.begin();
  // Wire.beginTransmission(MPU);
  //Wire.write(0x6B);  // PWR_MGMT_1 register
  // Wire.write(0);     // set to zero (wakes up the MPU-6050)
  // Wire.endTransmission(true);
  Serial.begin(115200);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(115200);

  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------

  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  // TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  //---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------

  TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  //  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz

  analogWrite(motor1, 0);
  analogWrite(motor2, 0);
  analogWrite(motor3, 0);
  analogWrite(motor4, 0);


  // setup PID
  rolls.SetSampleTime(SampleTime);// SampleTime ms
  pichts.SetSampleTime(SampleTime);
  yaws.SetSampleTime(SampleTime);
  //howers.SetSampleTime(SampleTime);

  rollsR.SetSampleTime(SampleTime);// SampleTime ms
  pichtsR.SetSampleTime(SampleTime);
  yawsR.SetSampleTime(SampleTime);



  // end set up PID


}

void loop() {
  // put your main code here, to run repeatedly:
  if (battOK) {
    //if (true) {

    

    //
    readDataFromESP01();
    // if (toreadGY25 == 1) {

    //}
    //
    // if (timer - sampling >= 1000 ) {
    /// sampling2 = timer;
    // toreadGY25 = 1;
    // q_yaw -= 0.01;


    // }//end task read angle
  timer = millis();
    if (((timer - samplePID) >= 20)) {
      samplePID = timer;
      readGY25();
      //
      if (sign)
      {
        sign = 0;
        if (Re_buf[0] == 0xAA && Re_buf[7] == 0x55)   //检查帧头，帧尾
        {
          YPR[0] = (Re_buf[1] << 8 | Re_buf[2]) / 1; //合成数据，去掉小数点后2位
          YPR[1] = (Re_buf[3] << 8 | Re_buf[4]) / 1;
          YPR[2] = (Re_buf[5] << 8 | Re_buf[6]) / 1;
          /* Serial.print("YPR[0] : ");
            Serial.print(YPR[0]/100);
            Serial.print("  YPR[1] : ");
            Serial.print(YPR[1]/100);
            Serial.print("  YPR[2] : ");
            Serial.println(YPR[2]/100);
          */
          //protect when value fail
          q_yaw = YPR[0] / 100 ;
          q_pitch = YPR[2] / 100;
          q_roll = YPR[1] / 100;
          //stable check
          //+++++++++++++++++++++++++++++++++
          if (q_pitch < 2  && q_pitch > -2  && q_roll < 2 && q_roll > -2 ) {
            //caribate = true;
            //call led


            if (!caribate) {
              rolls.SetMode(AUTOMATIC);
              pichts.SetMode(AUTOMATIC);
              yaws.SetMode(AUTOMATIC);

              rollsR.SetMode(AUTOMATIC);
              pichtsR.SetMode(AUTOMATIC);
              yawsR.SetMode(AUTOMATIC);

              //SetpointY = q_yaw;
              caribate = true;
            }
          } else { // wait for MPU6050 set angle
            ledStatus(2000);
          }
          //-----------------------------------

          //

          InputRoll =  q_roll;
          InputPicht = q_pitch;
          InputYaw =  q_yaw; ; //outputYawSetpoint

          SetpointP = (ch1_Eleveltor - 126 ) * rollPitchYawGain;
          SetpointR = (ch2_roll - 126 ) * rollPitchYawGain;


          SetpointY = ((ch4_yaw  - 126 ) * 0.65 ) + q_yaw;

        }
      }
      // ch3_power = 180;// test

      //q_roll = 0;//test
      //q_pitch = 0;//test
      //q_yaw = 0;//test
      //pidControl();
      if (rolls.Compute() && pichts.Compute() && rollsR.Compute() && pichtsR.Compute() && yaws.Compute() && yawsR.Compute() && caribate ) {

        ledStatus(400);

        //sum PID

        //  if (headingDegrees > 1  || headingDegrees < 358.5) {// normall
        //if (q_pitch < 65  && q_pitch > -35  && q_roll < 35 && q_roll > -35 ) {
        /* motor_B = ((powers) + OutputRoll - OutputRollR  + OutputPichtR - OutputPicht - OutputYaw + OutputYawR) ;
          motor_A = (powers * 0.9) + OutputRollR - OutputRoll  + OutputPichtR - OutputPicht + OutputYaw - OutputYawR;
          motor_C = (powers * (0.79)) + OutputRoll - OutputRollR  + OutputPicht - OutputPichtR + OutputYaw - OutputYawR ;
          motor_D = (((powers * (0.79)) * 0.9)  + OutputRollR - OutputRoll + OutputPicht - OutputPichtR - OutputYaw + OutputYawR);
        */
        motor_B = (powers) + ((OutputRoll - OutputRollR  + OutputPichtR - OutputPicht - OutputYaw + OutputYawR) * 1) ;
        motor_A = (powers) + ((OutputRollR - OutputRoll  + OutputPichtR - OutputPicht + OutputYaw - OutputYawR) * 1);
        motor_C = (powers) + ((OutputRoll - OutputRollR  + OutputPicht - OutputPichtR + OutputYaw - OutputYawR ) * 1);
        motor_D = (powers)  + ((OutputRollR - OutputRoll + OutputPicht - OutputPichtR - OutputYaw + OutputYawR) * 1);
        // }
        // limmit output max, min
        if (motor_A < 1) {
          motor_A = 0;
        }
        if (motor_B < 1) {
          motor_B = 0;
        }
        if (motor_C < 1) {
          motor_C = 0;
        }
        if (motor_D < 1) {
          motor_D = 0;
        }

        if (motor_A > 255) {
          motor_A = 255;
        }
        if (motor_B > 255) {
          motor_B = 255;
        }
        if (motor_C > 255) {
          motor_C = 255;
        }
        if (motor_D > 255) {
          motor_D = 255;
        }

        //callPID();
        if (checkBattery()) {
          driveMotor(motor_B, motor_A, motor_C, motor_D);
        } else {
          driveMotor(0, 0, 0, 0);
          ledStatus(89);
        }
        /*Serial.print("m1  l ");
          Serial.print(motor_B);
          Serial.print("   m2  r ");
          Serial.print(motor_A);
          Serial.print("   m3  r ");
          Serial.print(motor_C);
          Serial.print("   m4  l ");
          Serial.print(motor_D);

          Serial.print("  Yaw  ");
          Serial.print(q_yaw);
          Serial.print("   roll  ");
          Serial.print(q_roll);
          Serial.print("   pitch  ");
          Serial.println(q_pitch);*/



      }//end call PID work
    } //end task 20ms
  } else {
    ledStatus(50);
    Serial.println("Low battery");
  }
}// end void loop


//Keep data from RX ESP01  serialEvent
/*
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

//end Keep data from RX ESP01  serialEvent

//  **************************  readDataFromESP01()
void readDataFromESP01() {
  // stringComplete = false;
  serialEvent(); //call the function
  // print the string when a newline arrives:
  // readSeial();
  if (stringComplete) {
    //Serial.println(inputString);

    if (inputString.length() > 0) {
      b = inputString;
      //lostConnect = false;
      //String dataSends =  "a"+Integer.toString(ch1_ele)+"b"+Integer.toString(ch2_roll)+"c"+Integer.toString(ch3_power)+"d"+Integer.toString(ch4_yaw)
      //+"p"+Integer.toString(kpSend)+"i"+Integer.toString(kiSend)+"k"+Integer.toString(kdSend)+"!";
      ch1_Eleveltor = b.substring((b.indexOf('a') + 1), b.indexOf('b')).toInt();
      ch1_Eleveltor = 126 - (ch1_Eleveltor - 126);
      ch2_roll = b.substring((b.indexOf('b') + 1), b.indexOf('c')).toInt();
      ch2_roll = 126 - (ch2_roll - 126);
      ch3_power = b.substring((b.indexOf('c') + 1), b.indexOf('d')).toInt();
      /*
            if (countTemp == 1) {
              ch3_powerTemp = ch3_power;
            }

            if (countTemp == 2 ) {
              if ((ch3_power - ch3_powerTemp) > 50) {
                ch3_power = ch3_powerTemp;
              }
            }


            countTemp++;
             if (countTemp > 2) {
              countTemp = 1;
            }
      */
      powers = ch3_power * 0.85;
      //powers = powers * 0.65;
      //powers = map(ch3_power, 0, 255, 0, 128);
      //ch3_power = ch3_power*0.4;
      ch4_yaw = b.substring((b.indexOf('d') + 1), b.indexOf('p')).toInt();
      //ch4_yaw = ch4_yaw - (rollKi*10);
      ch4_yaw = 126 - ( ch4_yaw - 126);
      //

      rollKp = b.substring((b.indexOf('p') + 1), b.indexOf('i')).toInt();
      rollKi = b.substring((b.indexOf('i') + 1), b.indexOf('k')).toInt();
      rollKd = b.substring((b.indexOf('k') + 1), b.indexOf('z')).toInt();
      //onKiR = b.substring((b.indexOf('z') + 1), b.indexOf('x')).toInt();
      //onKiP = b.substring((b.indexOf('x') + 1), b.indexOf('!')).toInt();

      rollKp = rollKp / 10;
      rollKi = rollKi / 10;
      rollKd = rollKd / 10;

      //
      // gainYaw14,gainYaw23

      yaws.SetTunings(rollKp , 0, 0);
      yawsR.SetTunings(rollKp, 0, 0);
      /*
         float pitchKit = 0;
        float rollsKit = 0;
      */
      //set ki to 0 when roll left or right

      //
      // rolls.SetTunings(rollKp , 0, rollKd);
      //rollsR.SetTunings(rollKp, 0, rollKd);


      // clear the string:
      inputString = "";
      //countC = 0;
      stringComplete = false;
      timeDetect = timer;

    }

    //digitalWrite(13, LOW);


  } else {
    if (timer - timeDetect > 500 ) {
      timeDetect = timer;
      //stop motor when no data
      ch3_power = 0;
      //lostConnect = true;

    }
  }


  //}
}



////  *********************   END  readDataFromESP01()

boolean checkBattery() {

  sensorValue = analogRead(A1);// get battery V
  if (sensorValue > 610) { // if battery V > 3 battStates = true
    battStates = true;
  } else { // if battery V < 3.3 battStates = false
    battStates = false;
    battOK = false;
  }

  return battStates;
}
// end funion for check batt low V cut off

void ledStatus(int timeBlink) {
  //timeLED13
  if ((millis() - timeLED13) >= timeBlink) {
    //blinkState = !blinkState;
    timeLED13 = millis();
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(13, ledState);
  }
}




// function drive Motor   %%%%%%%%%%%%%%%%%%%% MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

void driveMotor(int outputPID1, int outputPID2, int outputPID3, int outputPID4)
{
  // input ch4_yaw


  if (ch3_power > 5 ) {
    analogWrite(motor1, outputPID1);
    analogWrite(motor2, outputPID2);
    analogWrite(motor3, outputPID3);
    analogWrite(motor4, outputPID4);
  } else {
    analogWrite(motor1, 0);
    analogWrite(motor2, 0);
    analogWrite(motor3, 0);
    analogWrite(motor4, 0);
  }
}

// >>>>>>>>>>>>>>>>>>>>>>   End function drive Motor  <<<<<<<<<<<<<<<


// function read data from GY25
void readGY25() {
  mySerial.write(0XA5);
  mySerial.write(0X51);    //0xA5 + 0x51: Query mode, the direct return angle value,
  while (mySerial.available()) {
    Re_buf[counter] = (unsigned char)mySerial.read();
    if (counter == 0 && Re_buf[0] != 0xAA) return; // 检查帧头
    counter++;
    if (counter == 8)             //接收到数据
    {
      counter = 0;               //重新赋值，准备下一帧数据的接收
      sign = 1;
    }
  }

}


