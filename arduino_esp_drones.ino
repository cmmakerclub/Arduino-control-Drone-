#include <Arduino.h>
#include <Wire.h>
//#include "MS561101BA.h"
#include "config.h"
//#include "multi_rxPPM2560.h"
#include "mpu6050.h"
#include "ahrs_tin.h"
//#include "Control_PID.h"
//#include "motorX4.h"
//#include "GPS_multi.h"
//#include "Ultrasonic.h"




float getAvg(float * buff, int size)
{
  float sum = 0.0;
  for (int i = 0; i < size; i++)
  {
    sum += buff[i];
  }
  return sum / size;
}

//setup function
void checkBattery();
void pidControlRoll();
void pidControlPitch();
void pidControlYaw();
void ledStatus(int timeBlink);
void readDataFromESP01();

void setup()
{
  Serial.begin(115200);//38400
  pinMode(13, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);//(13=A=M),(31=B=STABLEPIN),(30=C,GPS FIG LEDPIN)
  digitalWrite(13, HIGH);
  //Serial1.begin(115200);//CRIUS Bluetooth Module pin code 0000
  //Serial3.begin(38400);//3DR Radio Telemetry Kit 433Mhz
  //configureReceiver();//find multi_rx.h
  // motor_initialize();//find motor.h
  // ESC_calibration();//find motor.h
  // GPS_multiInt();
  Wire.begin();
  delay(1);
  mpu6050_initialize();
  delay(1);
  //MagHMC5883Int();
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
    //Mag5883Read();
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
  //MagHMC5883Int();
  //Mag_Calibrate();
  //RC_Calibrate();//"multi_rxPPM2560.h"


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

  //))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))



  Serial.print("TK_Quadrotor_Run_Roop_100Hz"); Serial.println("\t");
  sensorPreviousTime = micros();
  previousTime = micros();
}
void loop() {

  //if (battStates) {
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
    if (Dt_roop >= 10000)
    {
      previousTime = micros();
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
      ahrs_p -= 3;// roll trim for test model
      ahrs_r -= 3.85; // pitch trim for test model

      //x_angle = x_angle + (GyroXf*RAD_TO_DEG*G_Dt);
      //x_angle = kalmanCalculateX(ahrs_r*RAD_TO_DEG, GyroX*RAD_TO_DEG, G_Dt);
      //y_angle = kalmanCalculateY(ahrs_p*RAD_TO_DEG, GyroY*RAD_TO_DEG, G_Dt);

      // make set point
      setHeading = ahrs_y;
      Heading *= 10;

      setPointRoll = (126 - ch2_roll) * rollPitchYawGain;
      setPointPitch = (126 - ch1_Eleveltor) * rollPitchYawGain;

      if (ch4_yaw > 125 && ch4_yaw < 127) {
        setPointYaw = 0.0;//yawGain
      } else {
        setPointYaw = ((ch4_yaw  - 126 ) * yawGain );//yawGain
      }



      //PID call
      pidControlRoll();
      pidControlPitch();
      pidControlYaw();

      //end PID call

      // drive Motor  powers
      motor_B = (powers - battcut) - OutputP + OutputR - OutputY;//front left(1)
      motor_A = (powers - battcut) - OutputP - OutputR + OutputY;//front right(2)
      motor_C = (powers - battcut) + OutputP + OutputR + OutputY;//back left(3)
      motor_D = (powers - battcut) + OutputP - OutputR - OutputY;//back right(4)
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


      driveMotor(motor_B, motor_A, motor_C, motor_D);
      ledStatus(500);


      //
      /*    Serial.print(" ahrs_y =  "); //Heading
          Serial.print(ahrs_y);
          Serial.print("   ahrs_p =  ");
          Serial.print(ahrs_p);//roll
          Serial.print("   ahrs_r =  ");
          Serial.print(ahrs_r);//pitch
          setHeading = ahrs_y;
          Serial.print("    Heading  =  ");
          Serial.println( Heading * 10, 2); //max spin 1.5//yaw
      */
      if (frameCounter >= 100) {// loop 1Hz
        checkBattery();// check battery V every 1sec
        if (!battStates){
          battcut += 20;
        }else{
          battcut = 0;
        }
        battcut = constrain(battcut , 0, 255);
        frameCounter = 0;
      }
    }// end loop task 10ms
    /*
       waiteing read data from ESP8266
    */
    readDataFromESP01();
    /*

    */

  //} else { // end check batt
   // ledStatus(50); //arlram batt low
  //}

}// end void loop
/*

   PIDtong
   Input=angle_y;
    error1 = setPoint - angle_y;
    errSum1 = errSum1+error1;
    dErr1 = (error1 - lastErr) / dt;
    Output = kp * error1 + ki * errSum1 + kd * dErr1;
    lastErr = error1;
*/
/*
   Function PID input : setpoint,angle   Output : PWM(int)
*/
void pidControlRoll() {

  //Input = angle_p;
  error1R = setPointRoll - ahrs_p -2;
  errSum1R = errSum1R + error1R;
  dErr1R = (error1R - lastErrR) / DTPID;
  OutputR = kpRoll * error1R + kiR * errSum1R + kdR * dErr1R;
  lastErrR = error1R;

}

void pidControlPitch() {

  //Input = angle_p;
  error1P = setPointPitch - ahrs_r - 2;
  errSum1P = errSum1P + error1P;
  dErr1P = (error1P - lastErrP) / DTPID;
  OutputP = kpPitch * error1P + kiP * errSum1P + kdP * dErr1P;
  lastErrP = error1P;

}


void pidControlYaw() {

  //Input = angle_p;
  error1Y = setPointYaw - Heading;
  errSum1Y = errSum1Y + error1Y;
  dErr1Y = (error1Y - lastErrY) / DTPID;
  OutputY = kpYaw * error1Y + kiY * errSum1Y + kdY * dErr1Y;
  lastErrY = error1Y;

}


/*
   end PID
*/


/*

*/
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
      powers = ch3_power * powerRate;
      
      //powers = powers * 0.65;
      //powers = map(ch3_power, 0, 255, 0, 128);
      //ch3_power = ch3_power*0.4;
      ch4_yaw = b.substring((b.indexOf('d') + 1), b.indexOf('p')).toInt();
      //ch4_yaw = ch4_yaw - (rollKi*10);
      //ch4_yaw = 126 - ( ch4_yaw - 126);
      //

      //      rollKp = b.substring((b.indexOf('p') + 1), b.indexOf('i')).toInt();
      //      rollKi = b.substring((b.indexOf('i') + 1), b.indexOf('k')).toInt();
      //      rollKd = b.substring((b.indexOf('k') + 1), b.indexOf('z')).toInt();
      //      //onKiR = b.substring((b.indexOf('z') + 1), b.indexOf('x')).toInt();
      //      //onKiP = b.substring((b.indexOf('x') + 1), b.indexOf('!')).toInt();
      //
      //      rollKp = rollKp / 100;
      //      rollKi = rollKi / 1000;
      //      rollKd = rollKd / 100;

      //
      // gainYaw14,gainYaw23

      //yaws.SetTunings(rollKp , 0, 0);
      //yawsR.SetTunings(rollKp, 0, 0);
      /*
         float pitchKit = 0;
        float rollsKit = 0;
      */
      //set ki to 0 when roll left or right

      //
      //rolls.SetTunings(rollKp , 0, rollKd + rollKi);
      // rollsR.SetTunings(rollKp, 0, rollKd + rollKi);


      // clear the string:
      inputString = "";
      //countC = 0;
      stringComplete = false;
      timeDetect = millis();

    }

    //digitalWrite(13, LOW);


  } else {
    if (millis() - timeDetect > 500 ) {
      timeDetect = millis();
      //stop motor when no data
      ch3_power = 0;
      //lostConnect = true;

    }
  }


  //}
}



////  *********************   END  readDataFromESP01()



void checkBattery() {

  sensorValue = analogRead(A1);// get battery V
  if (sensorValue > 610) { // if battery V > 3 battStates = true
    battStates = true;
  } else { // if battery V < 3.3 battStates = false
    battStates = false;
    //battOK = false;
  }

  //return battStates;
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




