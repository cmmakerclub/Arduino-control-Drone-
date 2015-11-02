#include<Wire.h>
#include <PID_v1.h>
#include <Arduino.h>
#include "config.h"
#include "mpu6050.h"
#include "ahrs_tin.h"




// SET Variables to read data ESP
String inputString = "", b;        // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int ch1_Eleveltor, ch2_roll, ch3_power, ch4_yaw;
//char c[40];
//int countC;
//byte inByte;
//int ic[4];


//^^^^^^^^^^^^^^ Set var PID ^^^^^^^^^^^^^^^//
//Define Variables we'll be connecting to
double  SetpointP = 0, SetpointR = 0, SetpointY = 0;
//double rollKp = 0, rollKi = 0, rollKd = 0;
int SampleTime  =  24;
//float rollPitchYawGain = 0.3;
#define RPYGIAN 0.3

double  InputRoll, OutputRoll, OutputRollR; // right <---> left
//double InputRoll24, OutputRoll24;

double  InputPicht, OutputPicht, OutputPichtR; // forward <-----> back
//double InputPicht24, OutputPicht24;

double  InputYaw, OutputYaw, OutputYawR; // spin left <------> right  !!!!!!!!!!!   high 14 for spin left
//double InputYaw23, OutputYaw23; // !!!!!!!!!!!!!!   high  23 for spin right

//double  InputHower, OutputHower, SetpointHower = 9.82;//
//boolean h = true;

// Variables will change :
int ledState = LOW;             // ledState used to set the LED

// ^^^^^^^^^^^^ End set var PID ^^^^^^^^^^^//




/*@@@@@@@@@@@@@@@@@@@@@@@@@@  Set var PWM   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
int motor1 = 3 , motor2 = 9 , motor3 = 10 , motor4 = 11;
/*#define MOTOR1 3
#define MOTOR2 9
#define MOTOR3 10
#define MOTOR4 11
*/
//int valueM1, valueM2, valueM3, valueM4;
//int throttle;
int m1, m2, m3, m4;
//float gainYaw14, gainYaw23,eY,iX;
/*
        m1                       m2
         o                      o
          o     forward       o
            o      /\       o
              o           o
              ------------
              |          |
              ]          ]
              [          ]
              o----------o
            o              o
          o                 o
         o                   o
        o                     o
      m3                       m4
*/
/*@@@@@@@@@@@@@@@@@@@@@@@@@ End set var PWM  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/


PID rolls  (&InputRoll,  &OutputRoll,  &SetpointR, 1.201 , 0, 0.19,   DIRECT); // if roll < 0  13 Up(Throttle + OutputRoll)  and   if roll > 0  24 up(Throttle + OutputRoll)
PID pichts (&InputPicht, &OutputPicht, &SetpointP, 1.201 , 0, 0.19,   DIRECT); // if Picht < 0 12 up(Throttle + OutputPicht)  and if picht > 0 34  up(Throttle + OutputPicht)
PID yaws   (&InputYaw,   &OutputYaw,   &SetpointY, 2.998, 0, 0, DIRECT); // spin right value degree +
//PID howers (&InputHower,   &OutputHower,   &SetpointHower, 1, 1, 1, DIRECT); // if az < setpoint  all motor up   and   az > setpoint all motor down

//revers PID
PID rollsR  (&InputRoll,  &OutputRollR,  &SetpointR, 1.201 , 0, 0.19,   REVERSE); // if roll < 0  13 Up(Throttle + OutputRoll)  and   if roll > 0  24 up(Throttle + OutputRoll)
PID pichtsR (&InputPicht, &OutputPichtR, &SetpointP, 1.201 , 0, 0.19,  REVERSE); // if Picht < 0 12 up(Throttle + OutputPicht)  and if picht > 0 34  up(Throttle + OutputPicht)
PID yawsR   (&InputYaw,   &OutputYawR,   &SetpointY, 2.998, 0, 0, REVERSE); // if yaw < 0 23 up(Throttle + OutYaw) and if yaw > 0 14 up(Throttle + OutYaw)

//int countReadMPU = 0;

//float p = 0, r = 0 , y = 0, z = 0;
boolean caribate = false;

#define LED_PIN 13


//set var multitask
unsigned long timeDetect, timeLED13, timePid; // timeLost, timePidRolls, timePidRollsR, timePidPitchs, timePidPitchsR, timeDriveMotor;
//end set var multitask
///unsigned long timeA, timeB, timeC;
//int countAB = 0;
boolean battStates = false;
int sensorValue = 0;        // value read from the pot

/*
 *
 *
 */


/*
 *
 */
//

float getAvg(float * buff, int size)
{
  float sum = 0.0;
  for (int i = 0; i < size; i++)
  {
    sum += buff[i];
  }
  return sum / size;
}


void setup() {
  // put your setup code here, to run once:


  Serial.begin(115200);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  //================== Setup PWM  ===============
  /* pinMode(motor1, OUTPUT);
   pinMode(motor1, OUTPUT);
   pinMode(motor1, OUTPUT);
   pinMode(motor1, OUTPUT);*/

  // set new pwm frq to 3.9kHz
  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------

  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  //---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------

  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
  //
  //end set new pwm frq
  analogWrite(motor1, 0);
  analogWrite(motor2, 0);
  analogWrite(motor3, 0);
  analogWrite(motor4, 0);

  //================= End setup PWM  ==================


  // setup PID
  rolls.SetSampleTime(SampleTime);// SampleTime ms
  pichts.SetSampleTime(SampleTime);
  yaws.SetSampleTime(SampleTime);
  //howers.SetSampleTime(SampleTime);

  rollsR.SetSampleTime(SampleTime);// SampleTime ms
  pichtsR.SetSampleTime(SampleTime);
  yawsR.SetSampleTime(SampleTime);

  SetpointP = 0;
  SetpointR = 0;
  //SetpointY = 0;


  // end set up PID
  /*
    rolls.SetMode(AUTOMATIC);
    pichts.SetMode(AUTOMATIC);
    yaws.SetMode(AUTOMATIC);

    rollsR.SetMode(AUTOMATIC);
    pichtsR.SetMode(AUTOMATIC);
    yawsR.SetMode(AUTOMATIC);
    //howers.SetMode(AUTOMATIC);

  */

  /*
   *
   * Set read angle from MPU
   */
  Wire.begin();
  delay(1);
  mpu6050_initialize();
  delay(1);
  MagHMC5883Int();
  delay(1);
  digitalWrite(13, HIGH);
  // baro.init(MS561101BA_ADDR_CSB_LOW);
  // UltrasonicInt();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  delay(1);
  for (uint8_t i = 0; i < 50; i++)
  {
    mpu6050_Gyro_Values();
    mpu6050_Accel_Values();
    Mag5883Read();
    // UltrasonicRead();
    // temperature = baro.getTemperature(MS561101BA_OSR_4096);
    // presser = baro.getPressure(MS561101BA_OSR_4096);
    // pushAvg(presser);
    delay(20);
  }
  //Altitude_Ground = Altitude_baro/10.0;
  //sea_press = presser + 0.11;//presser 1003.52
  //Serial.print("presser ");Serial.println(sea_press);
  digitalWrite(13, LOW);
  sensor_Calibrate();//sensor.h
  ahrs_initialize();//ahrs.h
  setupFourthOrder();
  //RC_Calibrate();//"multi_rxPPM2560.h"
  //Serial.print("TK_Quadrotor_Run_Roop_100Hz"); Serial.println("\t");
  sensorPreviousTime = micros();
  previousTime = micros();

  /*
   * End read angle from MPU6050
   */


}

void loop() {
  // put your main code here, to run repeatedly:

  /*__________^^________________*/  /*###################################################  Main  loop   $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/

  //times = millis();

  //$$$$$$$$$$$$  setup   &&&&&&&&&&&&&&&&&&&&&&&&
  // read MUP and caribate
  // pitch and roll must < 1'
  // set pitch and roll to Setpoint
  /*___________  task  doing___loop_____/
  1. read data MPU6050
  2. fitter data from MPU6050
  3. send to PID
  4. get Output from PID
  5. combination Output from PID and data from UDP. Throttle - PID Output!
  */
  //)))))))))))))))))))) caribate )))))))))))))))))))))))))))))))*/
  /*
   * read data from ESP01
   */
  readDataFromESP01();
  /*
   * End read data from ESP01
   */

  Dt_sensor = micros() - sensorPreviousTime;///////////Roop sensor/////////
  if (Dt_sensor <= 0)
  {
    Dt_sensor = 1001;
  }
  if (Dt_sensor >= 1000 && gyroSamples < 4) ////Collect 3 samples = 2760 us  && gyroSamples < 5  && gyroSamples < 5
  {
    sensorPreviousTime = micros();
    mpu6050_readGyroSum();
    mpu6050_readAccelSum();
  }
  Dt_roop = micros() - previousTime;// 100 Hz task loop (10 ms)  , 5000 = 0.02626 ms
  if (Dt_roop <= 0)
  {
    Dt_roop = 10001;
  }
  if (Dt_roop >= 20000)
  {
    previousTime = micros();
    //Serial.print("   Timestart  =  ");
    //Serial.print(micros());
    G_Dt = Dt_roop * 0.000001;
    frameCounter++;
    mpu6050_Get_accel();
    mpu6050_Get_gyro();
    ////////////////Moving Average Filters///////////////////////////
    GyroXf = (GyroX + GyroX2) / 2.0;
    GyroYf = (GyroY + GyroY2) / 2.0;
    GyroZf = (GyroZ + GyroZ2) / 2.0;
    GyroX2 = GyroX; GyroY2 = GyroY; GyroZ2 = GyroZ; //gyro Old1
    ////////////////Low pass filter/////////////////////////////////
    AccXf = AccX;
    AccYf = AccY;
    AccZf = AccZ;
    //AccXf = AccXf + (AccX - AccXf)*15.6*G_Dt;//29.6 15.4  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
    //AccYf = AccYf + (AccY - AccYf)*15.6*G_Dt;//15.4
    //AccZf = AccZf + (AccZ - AccZf)*15.6*G_Dt;//15.4
    ///////////////////Filter FourthOrder ///////////////////////////////////////
    Accel[XAXIS] = AccX;
    Accel[YAXIS] = AccY;
    Accel[ZAXIS] = AccZ;
    for (int axis = XAXIS; axis <= ZAXIS; axis++) {
      filteredAccel[axis] = computeFourthOrder(Accel[axis], &fourthOrder[axis]);//"ahrs_tin.h"
    }

    //AccXf = filteredAccel[XAXIS];
    //AccYf = filteredAccel[YAXIS];
    //AccZf = filteredAccel[ZAXIS];
    //////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////
    //ahrs_updateMARG(GyroXf, GyroYf, GyroZf, AccXf, AccYf, AccZf, c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);//quaternion ,direction cosine matrix ,Euler angles
    ahrs_updateMARG(GyroXf, GyroYf, GyroZf, filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);
    //x_angle = x_angle + (GyroXf*RAD_TO_DEG*G_Dt);
    //x_angle = kalmanCalculateX(ahrs_r*RAD_TO_DEG, GyroX*RAD_TO_DEG, G_Dt);
    //y_angle = kalmanCalculateY(ahrs_p*RAD_TO_DEG, GyroY*RAD_TO_DEG, G_Dt);
    /*Serial.print(" ahrs_y =  "); //Heading
    Serial.print(ahrs_y);
    Serial.print("   ahrs_p =  ");
    Serial.print(ahrs_p);
    Serial.print("   ahrs_r =  ");
    Serial.print(ahrs_r);
    Serial.print("   Heading  =  ");
    Serial.println(Heading);
    */
    //send value angle to PID
    InputRoll =  ahrs_p;
    InputPicht = ahrs_r;


    InputYaw =  Heading;

    // waite for read angle stable
    // blink LED to indicate activity
    if (ahrs_r < 3  && ahrs_r > -3  && ahrs_p < 3 && ahrs_p > -3 ) {
      //caribate = true;
      //call led


      if (!caribate) {
        rolls.SetMode(AUTOMATIC);
        pichts.SetMode(AUTOMATIC);
        yaws.SetMode(AUTOMATIC);

        rollsR.SetMode(AUTOMATIC);
        pichtsR.SetMode(AUTOMATIC);
        yawsR.SetMode(AUTOMATIC);

        SetpointY = Heading;
        //SetpointY = outputYawSetpoint;
        caribate = true;
      }
    } else { // wait for MPU6050 set angle
      ledStatus(1500);
    }
    //Serial.print("   TimestartPID  =  ");
    //Serial.println(micros());
    if (caribate) {// call PID
      callPID();
      //digitalWrite(13, HIGH);
      ledStatus(500);
    }

  }// end tack 10ms

  /*if(( micros() - timePid) >= 18000){// task 17ms
    timePid =  micros();
     if (caribate) {// call PID
      callPID();
      //digitalWrite(13, HIGH);
      ledStatus(500);
    }
  }*/


}//+++++++++++++++++++++++++++++++++++++++++++++++++  End void loop()  ++++++++++++++++++++<<<<<<<<<<<<<<



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
      ch2_roll = b.substring((b.indexOf('b') + 1), b.indexOf('c')).toInt();
      ch3_power = b.substring((b.indexOf('c') + 1), b.indexOf('d')).toInt();
      //ch3_power = ch3_power*0.4;
      ch4_yaw = b.substring((b.indexOf('d') + 1), b.indexOf('p')).toInt();
      //
      /*
      rollKp = b.substring((b.indexOf('p') + 1), b.indexOf('i')).toInt();
      rollKi = b.substring((b.indexOf('i') + 1), b.indexOf('k')).toInt();
      rollKd = b.substring((b.indexOf('k') + 1), b.indexOf('!')).toInt();

      rollKp = rollKp / 10;
      rollKi = rollKi / 10;
      rollKd = rollKd / 10;

      //
      // gainYaw14,gainYaw23

      yaws.SetTunings(rollKp , rollKi, rollKd);
      yawsR.SetTunings(rollKp, rollKi, rollKd);
      */

      // clear the string:
      inputString = "";
      //countC = 0;
      stringComplete = false;
      timeDetect = Dt_sensor;

    }

    //digitalWrite(13, LOW);


  } else {
    if (Dt_sensor - timeDetect > 500000 ) {
      timeDetect = Dt_sensor;
      //stop motor when no data
      ch3_power -= 10;
      // lostConnect = true;
      ///
    }
  }


  //}
}



////  *********************   END  readDataFromESP01()




// function drive Motor   %%%%%%%%%%%%%%%%%%%% MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

void driveMotor(int outputPID1, int outputPID2, int outputPID3, int outputPID4) {




  //input ch2_roll
  if (ch2_roll > 126) { //13 up for move right


    outputPID1 +=  (ch2_roll - 126) * RPYGIAN;
    outputPID3 +=  (ch2_roll - 126) * RPYGIAN;

    outputPID2 -=  (ch2_roll - 126) * RPYGIAN;
    outputPID4 -=  (ch2_roll - 126) * RPYGIAN;

  }
  if (ch2_roll < 126) { //24 up for move left


    outputPID2 +=  (126 - ch2_roll) * RPYGIAN;
    outputPID4 +=  (126 - ch2_roll) * RPYGIAN;

    outputPID1 -=  (126 - ch2_roll) * RPYGIAN;
    outputPID3 -=  (126 - ch2_roll) * RPYGIAN;


  }

  //end input ch2_roll

  //input ch1_Eleveltor
  if (ch1_Eleveltor > 126) { // 34up for move forward



    outputPID3 +=  (ch1_Eleveltor - 126) * RPYGIAN;
    outputPID4 +=  (ch1_Eleveltor - 126) * RPYGIAN;

    outputPID1 -=  (ch1_Eleveltor - 126) * RPYGIAN;
    outputPID2 -=  (ch1_Eleveltor - 126) * RPYGIAN;
  }
  if (ch1_Eleveltor < 126) { // 12up for move back
    outputPID1 +=  (126 - ch1_Eleveltor) * RPYGIAN;
    outputPID2 +=  (126 - ch1_Eleveltor) * RPYGIAN;

    outputPID3 -=  (126 - ch1_Eleveltor) * RPYGIAN;
    outputPID4 -=  (126 - ch1_Eleveltor) * RPYGIAN;

  }

  //end input ch1_Eleveltor

  // input ch4_yaw
  if (ch4_yaw > 126) { // 23up for spin right
    outputPID2 += (ch4_yaw - 126) * RPYGIAN;
    outputPID3 += (ch4_yaw - 126) * RPYGIAN;
    //
    outputPID1 -= (ch4_yaw - 126) * RPYGIAN;
    outputPID4 -= (ch4_yaw - 126) * RPYGIAN;
  }
  if (ch4_yaw < 126) { // 14up for spin left
    outputPID1 += (126 - ch4_yaw) * RPYGIAN;
    outputPID4 += (126 - ch4_yaw) * RPYGIAN;

    outputPID2 -= (126 - ch4_yaw) * RPYGIAN;
    outputPID3 -= (126 - ch4_yaw) * RPYGIAN;

  }


  //end input ch4_yaw
  //valueM3 = valueM3 * 0.9;
  if ( outputPID1 > 255) {
    outputPID1 = 255;
  }
  if ( outputPID1 < 0) {
    outputPID1 = 0;
  }

  if ( outputPID2 > 255) {
    outputPID2 = 255;
  }
  if ( outputPID2 < 0) {
    outputPID2 = 0;
  }

  if ( outputPID3 > 255) {
    outputPID3 = 255;
  }
  if ( outputPID3 < 0) {
    outputPID3 = 0;
  }

  if ( outputPID4 > 255) {
    outputPID4 = 255;
  }
  if ( outputPID4 < 0) {
    outputPID4 = 0;
  }
  // end fix pwm

  //Output pwm
  if (ch3_power < 10 ) {
    analogWrite(motor1, 0);
    analogWrite(motor2, 0);
    analogWrite(motor3, 0);
    analogWrite(motor4, 0);


  } else {

    analogWrite(motor1,  outputPID1);
    analogWrite(motor2,  outputPID2);
    analogWrite(motor3,  outputPID3);
    analogWrite(motor4,  outputPID4);
  }
}

// >>>>>>>>>>>>>>>>>>>>>>   End function drive Motor  <<<<<<<<<<<<<<<


/*
  InputRoll =  rollAngel;
    InputPicht = pitchAngel;


    InputYaw =  headingDegrees;//outputYawSetpoint
    //InputYaw = outputYawSetpoint;

    if (ch4_yaw > 129 || ch4_yaw < 123) {
      SetpointY = headingDegrees;
      //SetpointY = outputYawSetpoint;
    }
/*
    if (countAB == 1) {
     timeC =  micros();
    }
*/
// blink LED to indicate activity
/* if (pitchAngel < 4  && pitchAngel > -4  && rollAngel < 4 && rollAngel > -4 ) {
   //caribate = true;
   //call led


   if (!caribate) {
     rolls.SetMode(AUTOMATIC);
     pichts.SetMode(AUTOMATIC);
     yaws.SetMode(AUTOMATIC);

     rollsR.SetMode(AUTOMATIC);
     pichtsR.SetMode(AUTOMATIC);
     yawsR.SetMode(AUTOMATIC);

     SetpointY = headingDegrees;
     //SetpointY = outputYawSetpoint;
     caribate = true;
   }
 } else { // wait for MPU6050 set angle
   ledStatus(1500);
 }

*/


void callPID() {
  //if ((millis() - timePidRolls) >= 25 ) {//PID sampleing time = 25ms
  // timePidRolls = millis();

  /*InputRoll =  tempRoll / countReadMPU;
  InputPicht = tempPitch / countReadMPU;
  InputYaw = tempYaw / countReadMPU;
  countReadMPU = 0;
  tempRoll = 0;
  tempPitch = 0;
  tempYaw = 0;
  */
  //>>>>>>>>>>>>   call PID to work
  if (rolls.Compute() && pichts.Compute() && rollsR.Compute() && pichtsR.Compute() && yaws.Compute() && yawsR.Compute()) {



    //sum PID

    //if (headingDegrees > 1  || headingDegrees < 358.5) {// normall
    m1 = (ch3_power *  0.85) + OutputRoll - OutputRollR + OutputPichtR - OutputPicht - OutputYaw + OutputYawR;
    m2 = (ch3_power *  0.85) + OutputRollR - OutputRoll + OutputPichtR - OutputPicht + OutputYaw - OutputYawR;
    m3 = (ch3_power * 0.85) + OutputRoll - OutputRollR + OutputPicht - OutputPichtR + OutputYaw - OutputYawR;
    m4 = (ch3_power * 0.85)  + OutputRollR - OutputRoll + OutputPicht - OutputPichtR - OutputYaw + OutputYawR;


    // send  m1 m2 m3 m4 value to Motor drive for fix drones to stable
    if (checkBattery()) {
      driveMotor(m1, m2, m3, m4);
    } else {
      ch3_power -= 30;
      if (ch3_power < 1) {
        ch3_power = 0;
      }
      driveMotor(m1, m2, m3, m4);
      ledStatus(80);
    }



  }
  //}//************ End of //PID rolls sampleing time = 10ms  ***************
}

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

// funion for check batt low V cut off
boolean checkBattery() {

  sensorValue = analogRead(A1);// get battery V
  if (sensorValue > 676) { // if battery V > 3.3 battStates = true
    battStates = true;
  } else { // if battery V < 3.3 battStates = false
    battStates = false;
  }

  return battStates;
}
// end funion for check batt low V cut off





