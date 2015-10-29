#include<Wire.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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

float rollAngel, pitchAngel;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
/*
 *
 *             Set variable for HMC5883
 */
float heading;
float declinationAngle = 0.22;
float headingDegrees;
float outputYawSetpoint;

/*
 *            End Set variable for HMC5883
 *
 */

//--------MPU6050 _--------------
/*roll arround Y--->
 * pitch arround X--->
 * -roll = left
 * -pitch = back
 *           top view
 *        _________________
 *        |              o|INT      ^
 *        |               |         |
 *        |               |         |
 *        |               |        forward
 *        |               |
 *        |               |
 *        |               |
 *        |               |
 *        _______________o_VCC
 *
 *
 *yaw spin right value-
 */
//
// set MPU6050 Variables
/*#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536
//#define M_PI 3.14159265359
#define dt 0.004 // 5ms sample rate

const int MPU = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

float ax, ay, az, gx, gy, gz, angle_x, angle_y;
float pitch, roll, yaw, tempRoll, tempPitch, tempYaw;




//int counts = 0;
//unsigned long timer, sampling;
*/
// End set MPU6050 Variables

// _------------- SET time state Variables

//unsigned long times, time2, time3;
// _-------------END  SET time state Variables


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
double rollKp = 0, rollKi = 0, rollKd = 0;
int SampleTime  =  23;
float rollPitchYawGain = 0.3;

double  InputRoll, OutputRoll, OutputRollR; // right <---> left
//double InputRoll24, OutputRoll24;

double  InputPicht, OutputPicht, OutputPichtR; // forward <-----> back
//double InputPicht24, OutputPicht24;

double  InputYaw, OutputYaw, OutputYawR; // spin left <------> right  !!!!!!!!!!!   high 14 for spin left
//double InputYaw23, OutputYaw23; // !!!!!!!!!!!!!!   high  23 for spin right

double  InputHower, OutputHower, SetpointHower = 9.82;//
boolean h = true;

// Variables will change :
int ledState = LOW;             // ledState used to set the LED

// ^^^^^^^^^^^^ End set var PID ^^^^^^^^^^^//




/*@@@@@@@@@@@@@@@@@@@@@@@@@@  Set var PWM   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
int motor1 = 3 , motor2 = 9 , motor3 = 10 , motor4 = 11;
int valueM1, valueM2, valueM3, valueM4;
int throttle;
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
// kp 0.8,ki 0 ,kd 0.3//ch3_power * 0.6

PID rolls  (&InputRoll,  &OutputRoll,  &SetpointR, 0.9 , 0, 0.35,   DIRECT); // if roll < 0  13 Up(Throttle + OutputRoll)  and   if roll > 0  24 up(Throttle + OutputRoll)
PID pichts (&InputPicht, &OutputPicht, &SetpointP, 0.9, 0, 0.35,   DIRECT); // if Picht < 0 12 up(Throttle + OutputPicht)  and if picht > 0 34  up(Throttle + OutputPicht)
PID yaws   (&InputYaw,   &OutputYaw,   &SetpointY, 0.8, 0, 0.25, DIRECT); // spin right value degree +
//PID howers (&InputHower,   &OutputHower,   &SetpointHower, 1, 1, 1, DIRECT); // if az < setpoint  all motor up   and   az > setpoint all motor down

//revers PID
PID rollsR  (&InputRoll,  &OutputRollR,  &SetpointR, 0.9, 0, 0.35,   REVERSE); // if roll < 0  13 Up(Throttle + OutputRoll)  and   if roll > 0  24 up(Throttle + OutputRoll)
PID pichtsR (&InputPicht, &OutputPichtR, &SetpointP, 0.9, 0, 0.35,  REVERSE); // if Picht < 0 12 up(Throttle + OutputPicht)  and if picht > 0 34  up(Throttle + OutputPicht)
PID yawsR   (&InputYaw,   &OutputYawR,   &SetpointY, 0.8, 0, 0.25, REVERSE); // if yaw < 0 23 up(Throttle + OutYaw) and if yaw > 0 14 up(Throttle + OutYaw)

int countReadMPU = 0;

float p = 0, r = 0 , y = 0, z = 0;
boolean caribate = false, lostConnect = false;


//set var multitask
unsigned long times, timeDetect, timeReadMPU, timeLED13, timeLost, timePidRolls, timePidRollsR, timePidPitchs, timePidPitchsR, timeDriveMotor;
//end set var multitask
unsigned long timeA, timeB, timeC;
int countAB = 0;
boolean battStates = false;
int sensorValue = 0;        // value read from the pot

/*
 *
 *
 */
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

/*
 *
 */
//
void setup() {
  // put your setup code here, to run once:
  /*
   * set up HMC5883L
   */

  while (!mag.begin()) {
    //wait for HMC5883 ok
  }
  /*
   *
   */

  /*pinMode(13, OUTPUT);
  // set MPU6050
  // Wire.pins(0, 2);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);


  Serial.begin(9600);
  // end set MPU6050
  */

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  mpu.testConnection();
  /*
      // verify connection
      Serial.println(F("Testing device connections..."));
      Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

      // wait for ready
      Serial.println(F("\nSend any character to begin DMP programming and demo: "));
      while (Serial.available() && Serial.read()); // empty buffer
      while (!Serial.available());                 // wait for data
      while (Serial.available() && Serial.read()); // empty buffer again
  */
  // load and configure the DMP
  // Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setRate(9);
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }

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
  SetpointY = 0;


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


}

void loop() {
  // put your main code here, to run repeatedly:

  /*__________^^________________*/  /*###################################################  Main  loop   $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
  times = millis();


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
  //while (!caribate) {

  /* p = 0, r = 0 , y = 0, z = 0;
   for (int i = 0; i < 100; i++) {
     // if ((timesC1 - timesC2) >= 20 ) {
     // timesC2 = timesC1;

     //clearValue();
     readMPU();
     complementaryFilter();

     p += pitch;
     r += roll;
     y += yaw ;
     z += (az / 16384);
     digitalWrite(13, HIGH);
     delay(4);
     // }
   }//end for
  */

  // digitalWrite(13, HIGH);
  //mpu6050Dmp();

  /*if (SetpointP > 5 || SetpointP < -5 || SetpointR > 5 || SetpointR < -5 ) { // Drones not stable try to caribate again.
    caribate = false;

    rolls.SetMode(MANUAL);
    pichts.SetMode(MANUAL);
    yaws.SetMode(MANUAL);

    rollsR.SetMode(MANUAL);
    pichtsR.SetMode(MANUAL);
    yawsR.SetMode(MANUAL);

  } else {

    digitalWrite(13, LOW);
    caribate = true;// get out caribate
    if (ch3_power > 10) {
      rolls.SetMode(AUTOMATIC);
      pichts.SetMode(AUTOMATIC);
      yaws.SetMode(AUTOMATIC);

      rollsR.SetMode(AUTOMATIC);
      pichtsR.SetMode(AUTOMATIC);
      yawsR.SetMode(AUTOMATIC);
    }
  }*/
  /*
  // blink LED to indicate activity
  if (pitchAngel < 3  && pitchAngel > -3  && rollAngel < 3 && rollAngel > -3) { // Drones stable


    //if (ch3_power > 10) {
    rolls.SetMode(AUTOMATIC);
    pichts.SetMode(AUTOMATIC);
    yaws.SetMode(AUTOMATIC);

    rollsR.SetMode(AUTOMATIC);
    pichtsR.SetMode(AUTOMATIC);
    yawsR.SetMode(AUTOMATIC);

    SetpointP = 0;
    SetpointR = 0;
    SetpointY = 0;
    SetpointHower = 0;
    //digitalWrite(13, LOW);
    caribate = true;// get out caribate
    //}

  } else {
    rolls.SetMode(MANUAL);
    pichts.SetMode(MANUAL);
    yaws.SetMode(MANUAL);

    rollsR.SetMode(MANUAL);
    pichtsR.SetMode(MANUAL);
    yawsR.SetMode(MANUAL);

  }
  */
  // }// end while

  /*
  //(((((((((((((((((((((   end caribate ((((((((((((((((((((((((*/


  // read data from Phone

  //readDataFromESP01(); // read data from ESP
  mpu6050Dmp();
  //InputRoll =  rollAngel;
  //InputPicht = pitchAngel;
  // InputYaw = tempYaw / countReadMPU;

  // End read data from Phone

  // read MPU6050 timeReadMPU
  /*if ((times - timeReadMPU) >= 4) {//  ++++++++++++++++++++++++++++++++     read MPU data every 5ms
    timeReadMPU = times;
    countReadMPU++;

    //clearValue();

    readMPU();
    complementaryFilter();// read data MPU6050 and fitter data from MPU6050

    tempRoll += roll;
    tempPitch += pitch;
    tempYaw += yaw;
    //InputRoll = roll;
    //InputPicht = pitch;
    //InputYaw = yaw;

  }*///    End of read MPU data
  /*
   *
   *
   */

  /*
   *
   *
   *timePidRolls,timePidRollsR,timePidPitchs,timePidPitchsR;
   */

  // if ((times - timePidRolls) >= 15 ) {//PID sampleing time = 15ms
  //timePidRolls = times;

  /*InputRoll =  tempRoll / countReadMPU;
  InputPicht = tempPitch / countReadMPU;
  InputYaw = tempYaw / countReadMPU;
  countReadMPU = 0;
  tempRoll = 0;
  tempPitch = 0;
  tempYaw = 0;
  */
  //>>>>>>>>>>>>   call PID to work
  //if (rolls.Compute() && pichts.Compute() && rollsR.Compute() && pichtsR.Compute() /*&& yaws.Compute() && yawsR.Compute()*/) {



  //sum PID


  // m1 = (ch3_power * 0.4) + OutputRoll - OutputRollR;
  // m2 = (ch3_power * 0.4) + OutputRollR - OutputRoll ;
  // m3 = (ch3_power * 0.4) + OutputRoll - OutputRollR ;
  // m4 = (ch3_power * 0.4)  + OutputRollR - OutputRoll ;

  //Drones move left 13up use PID rolls
  // m1 +=  OutputRoll;
  //m3 +=  OutputRoll;

  /*
   *
  */

  //Drones move right 24up use PID rollsR
  //m2 +=  OutputRollR;
  // m4 +=  OutputRollR;

  /*
   *
   *
   */
  /*
     *
     */

  //Drones move forward use PID pitchsR 12 up
  // m1 +=  OutputPichtR;
  //m2 +=  OutputPichtR;


  /*
   *
   *
   */

  //// Drones move back use PID pitchs 34 up
  // m3 +=  OutputPicht;
  // m4 +=  OutputPicht;


  /*
   *
   *
   */
  // Drones spin right fix by PID yaws 14up for spin left

  //m1 += OutputYaw;
  //m4 += OutputYaw;
  /*
   *
   *
   */
  // Drones spin left fix by PID yawsR 23up for spin right
  //m2 += OutputYawR;
  //m3 += OutputYawR;

  /*
   *
   *
   */
  // send  m1 m2 m3 m4 value to Motor drive for fix drones to stable

  // driveMotor(m1, m2, m3, m4);

  /* rollKp = rollKp + (ch3_power * 0.05);
  // rollKi = rollKi + (ch3_power * 0.005);
   rollKd = rollKd + (ch3_power * 0.0025);
   rolls.SetTunings(rollKp, rollKi, rollKd);
   rollsR.SetTunings(rollKp, rollKi, rollKd);

   rollKp = 0.1;
   rollKi = 0;
   rollKd = 0;
   */
  /*if ((times - timeLED13) >= 1000) {
    timeLED13 = times;
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
  */
  /* Blink LED to show work in PID loop
  *
  *
  */


  /*
   *
   *
   */

  // }
  //}//************ End of //PID rolls sampleing time = 20ms  ***************


  /*
   *
   *
   */


  /*
   *
   *
   */





}//+++++++++++++++++++++++++++++++++++++++++++++++++  End void loop()  ++++++++++++++++++++<<<<<<<<<<<<<<

/*
// funtion for Filter data from MPU6050 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void complementaryFilter() {
  float pitchAcc, rollAcc;
  //Integeate the gyroscope data -> int(angularSpeed) = angle
  pitch += (gx / GYROSCOPE_SENSITIVITY) * dt; //Angle around the X-axis
  roll -= (gy / GYROSCOPE_SENSITIVITY) * dt; //Angle around the Y-axis
  //yaw += (gy / GYROSCOPE_SENSITIVITY) * dt;

  //Compensate for drift with accelerometer data if !bullshit
  //Sensitivity MPU = -2 to  2G = 32768 0.5G = 8192
  int forceMagnitudeApprox = abs(ax) + abs(ay) + abs(az);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768 ) {
    //Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2f(ay, az) * 180 / 3.14159265359;
    pitch = (pitch * 0.98) + pitchAcc  * 0.02;

    // Turning around the Y axis results in vector on the X-axis
    rollAcc = atan2f(ax, az) * 180 / M_PI;
    roll = (roll * 0.98) + rollAcc  * 0.02;


    yaw = gz / 131;

  }

}

//end funtion for Filter data from MPU6050



// funtion clear  value %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void clearValue() {
  //Clear
  ax = 0;
  ay = 0;
  az = 0;



  gx = 0;
  gy = 0;
  gz = 0;

  pitch = 0;
  roll = 0;
  yaw = 0;

  //
}

// end funtion clear  value



//  funtion read data from MPU6050 &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void readMPU() {
  //for (int i = 0; i < 10; i++) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  ax = AcX;
  ay = AcY;
  az = AcZ;

  gx = GyX;
  gy = GyY;
  gz = GyZ;

  ax = ax - 1360;
  //ay = ay + 20;
  //az = az + AcZ;

  gx = gx - (-403.31);
  gy = gy - (101.14);
  gz = gz - (-95.88);

}


// end funtion read data from MPU6050
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
      lostConnect = false;
      //String dataSends =  "a"+Integer.toString(ch1_ele)+"b"+Integer.toString(ch2_roll)+"c"+Integer.toString(ch3_power)+"d"+Integer.toString(ch4_yaw)
      //+"p"+Integer.toString(kpSend)+"i"+Integer.toString(kiSend)+"k"+Integer.toString(kdSend)+"!";
      ch1_Eleveltor = b.substring((b.indexOf('a') + 1), b.indexOf('b')).toInt();
      ch2_roll = b.substring((b.indexOf('b') + 1), b.indexOf('c')).toInt();
      ch3_power = b.substring((b.indexOf('c') + 1), b.indexOf('d')).toInt();
      //ch3_power = ch3_power*0.4;
      ch4_yaw = b.substring((b.indexOf('d') + 1), b.indexOf('p')).toInt();
      //

      /* rollKp = b.substring((b.indexOf('p') + 1), b.indexOf('i')).toInt();
       rollKi = b.substring((b.indexOf('i') + 1), b.indexOf('k')).toInt();
       rollKd = b.substring((b.indexOf('k') + 1), b.indexOf('!')).toInt();

       rollKp = rollKp / 10;
       rollKi = rollKi / 10;
       rollKd = rollKd / 10;
       */
      //
      // gainYaw14,gainYaw23

      //rolls.SetTunings(rollKp , rollKi, rollKd);
      //rollsR.SetTunings(rollKp, rollKi, rollKd);


      // clear the string:
      inputString = "";
      //countC = 0;
      stringComplete = false;
      timeDetect = times;

    }

    //digitalWrite(13, LOW);


  } else {
    if (times - timeDetect > 500 ) {
      timeDetect = times;
      //stop motor when no data
      ch3_power -= 10;
      lostConnect = true;

    }
  }


  //}
}



////  *********************   END  readDataFromESP01()




// function drive Motor   %%%%%%%%%%%%%%%%%%%% MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

void driveMotor(int outputPID1, int outputPID2, int outputPID3, int outputPID4) {



  //claer value
  valueM1 =  0;
  valueM2 =  0;
  valueM3 =  0;
  valueM4 =  0;





  //
  //throttle = ch3_power * 0.35 ; //
  // fix pwm output by PID

  valueM1 =  outputPID1;
  valueM2 =  outputPID2;
  valueM3 =  outputPID3;
  valueM4 =  outputPID4;
  //

  //input ch2_roll
  if (ch2_roll > 126) { //13 up for move right


    valueM1 +=  (ch2_roll - 126) * rollPitchYawGain;
    valueM3 +=  (ch2_roll - 126) * rollPitchYawGain;

    valueM2 -=  (ch2_roll - 126) * rollPitchYawGain;
    valueM4 -=  (ch2_roll - 126) * rollPitchYawGain;

  }
  if (ch2_roll < 126) { //24 up for move left


    valueM2 +=  (126 - ch2_roll) * rollPitchYawGain;
    valueM4 +=  (126 - ch2_roll) * rollPitchYawGain;

    valueM1 -=  (126 - ch2_roll) * rollPitchYawGain;
    valueM3 -=  (126 - ch2_roll) * rollPitchYawGain;


  }

  //end input ch2_roll

  //input ch1_Eleveltor
  if (ch1_Eleveltor > 126) { // 34up for move forward



    valueM3 +=  (ch1_Eleveltor - 126) * rollPitchYawGain;
    valueM4 +=  (ch1_Eleveltor - 126) * rollPitchYawGain;

    valueM1 -=  (ch1_Eleveltor - 126) * rollPitchYawGain;
    valueM2 -=  (ch1_Eleveltor - 126) * rollPitchYawGain;
  }
  if (ch1_Eleveltor < 126) { // 12up for move back
    valueM1 +=  (126 - ch1_Eleveltor) * rollPitchYawGain;
    valueM2 +=  (126 - ch1_Eleveltor) * rollPitchYawGain;

    valueM3 -=  (126 - ch1_Eleveltor) * rollPitchYawGain;
    valueM4 -=  (126 - ch1_Eleveltor) * rollPitchYawGain;

  }

  //end input ch1_Eleveltor

  // input ch4_yaw
  if (ch4_yaw > 126) { // 23up for spin right
    valueM2 += (ch4_yaw - 126) * rollPitchYawGain;
    valueM3 += (ch4_yaw - 126) * rollPitchYawGain;
    //
    valueM1 -= (ch4_yaw - 126) * rollPitchYawGain;
    valueM4 -= (ch4_yaw - 126) * rollPitchYawGain;
  }
  if (ch4_yaw < 126) { // 14up for spin left
    valueM1 += (126 - ch4_yaw) * rollPitchYawGain;
    valueM4 += (126 - ch4_yaw) * rollPitchYawGain;

    valueM2 -= (126 - ch4_yaw) * rollPitchYawGain;
    valueM3 -= (126 - ch4_yaw) * rollPitchYawGain;

  }


  //end input ch4_yaw
  //valueM3 = valueM3 * 0.9;
  if (valueM1 > 255) {
    valueM1 = 255;
  }
  if (valueM1 < 0) {
    valueM1 = 0;
  }

  if (valueM2 > 255) {
    valueM2 = 255;
  }
  if (valueM2 < 0) {
    valueM2 = 0;
  }

  if (valueM3 > 255) {
    valueM3 = 255;
  }
  if (valueM3 < 0) {
    valueM3 = 0;
  }

  if (valueM4 > 255) {
    valueM4 = 255;
  }
  if (valueM4 < 0) {
    valueM4 = 0;
  }
  // end fix pwm

  //Output pwm
  if (ch3_power < 10 || rollAngel >= 75 || pitchAngel >= 75) {
    analogWrite(motor1, 0);
    analogWrite(motor2, 0);
    analogWrite(motor3, 0);
    analogWrite(motor4, 0);


  } else {

    analogWrite(motor1, valueM1);
    analogWrite(motor2, valueM2);
    analogWrite(motor3, valueM3);
    analogWrite(motor4, valueM4);
  }
}

// >>>>>>>>>>>>>>>>>>>>>>   End function drive Motor  <<<<<<<<<<<<<<<

// MPU6050 DMP
void mpu6050Dmp() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    /*if (callPidWork) {
      callPID();
      ledStatus(100);
      callPidWork = false;
    }*/
    // read data from Phone
    readDataFromESP01(); // read data from ESP
    if (caribate) {

      //readDataFromESP01(); // read data from ESP
      callPID();
      ledStatus(400);
      // digitalWrite(LED_PIN, HIGH);

    }

    // .
    // .
  }
  /*
    countAB++;
    if (countAB == 1) {
      timeA = millis();
    }
    if (countAB == 2) {
      timeB = millis();
      countAB = 0;
      Serial.print("rate pid ms =  ");
      Serial.println(timeC - timeA);
    }
    */
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //rollAngel,pitchAngel;
    rollAngel = ypr[1] * 180 / M_PI;
    pitchAngel = ypr[2] * 180 / M_PI;

    /*
     * Get HMC5883 Data for Yaw
     */
    /* Get a new sensor event */
    sensors_event_t event;
    mag.getEvent(&event);
    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    heading = atan2(event.magnetic.y, event.magnetic.x);
    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.

    heading += declinationAngle;

    // Correct for when signs are reversed.
    if (heading < 0)
      heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * PI)
      heading -= 2 * PI;

    // Convert radians to degrees for readability.
    headingDegrees = heading * 180 / M_PI;  //<<<<<<<<<<<< Output  headingDegrees 0 - 359.99


    /*
     * End Get HMC5883 Data for Yaw
     */

    /* Serial.print("ypr\t  yaw ");
     Serial.print(ypr[0] * 180/M_PI);
     Serial.print("\t  roll   ");
     Serial.print(rollAngel);
     Serial.print("\t pitch  ");
     Serial.println(pitchAngel);
     //

    */
    /*
    // incress PID roll when Drone more angel
    if (rollAngel > 35) {

      rollsR.SetTunings(1.6 , 0, 0.6);
    } else {
      rollsR.SetTunings(0.8, 0, 0.3);
    }

    if (rollAngel < -35) {

      rolls.SetTunings(1.6 , 0, 0.6);
    } else {
      rolls.SetTunings(0.8, 0, 0.3);
    }
    // end incress PID roll

     // incress PID pitch when Drone more angel
    if (pitchAngel > 35) {

      pichtsR.SetTunings(1.6 , 0, 0.6);
    } else {
      pichtsR.SetTunings(0.8, 0, 0.3);
    }

    if (pitchAngel < -35) {

      pichts.SetTunings(1.6 , 0, 0.6);
    } else {
      pichts.SetTunings(0.8, 0, 0.3);
    }
    // end incress PID pitch
    */

    InputRoll =  rollAngel;
    InputPicht = pitchAngel;

    //convertHeadingDegree();
    /* if (headingDegrees < 1  || headingDegrees > 358.5) {
       //Serial.println("        revers PID yaw   ");
       yaws.SetMode(MANUAL);
       yawsR.SetMode(MANUAL);
     } else {
       yawsR.SetMode(AUTOMATIC);
       yaws.SetMode(AUTOMATIC);
     }
    */
    InputYaw =  headingDegrees;//outputYawSetpoint
    //InputYaw = outputYawSetpoint;

    if (ch4_yaw > 129 || ch4_yaw < 123) {
      SetpointY = headingDegrees;
      //SetpointY = outputYawSetpoint;
    }

    // SetpointY += (ch4_yaw - 126) * 0.1f * 0.021f;
    //
    /*
     * call PID
     */
    //callPidWork = true;
    /*
     *
     *
     */
    /*if (countAB == 1) {
     timeC =  millis();
    }
    */
    // blink LED to indicate activity
    if (pitchAngel < 4  && pitchAngel > -4  && rollAngel < 4 && rollAngel > -4 ) {
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

  }

  /*if (!mpuInterrupt) {
    rolls.SetMode(AUTOMATIC);
    pichts.SetMode(AUTOMATIC);
    yaws.SetMode(AUTOMATIC);

    rollsR.SetMode(AUTOMATIC);
    pichtsR.SetMode(AUTOMATIC);
    yawsR.SetMode(AUTOMATIC);
  }*/
}

//)))))))))))))))))))))))))))))
void callPID() {
  if ((times - timePidRolls) >= 23 ) {//PID sampleing time = 15ms
    timePidRolls = times;

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

      if (headingDegrees > 1  || headingDegrees < 358.5) {// normall
        m1 = (ch3_power *  1) + OutputRoll - OutputRollR + OutputPichtR - OutputPicht - OutputYaw + OutputYawR;
        m2 = (ch3_power *  1) + OutputRollR - OutputRoll + OutputPichtR - OutputPicht + OutputYaw - OutputYawR;
        m3 = (ch3_power * 1) + OutputRoll - OutputRollR + OutputPicht - OutputPichtR + OutputYaw - OutputYawR;
        m4 = (ch3_power * 1)  + OutputRollR - OutputRoll + OutputPicht - OutputPichtR - OutputYaw + OutputYawR;

      }

      if (headingDegrees < 1  || headingDegrees > 358.5) {// daed angel (North) 0' 359'  remove yaw
        m1 = (ch3_power *  1) + OutputRoll - OutputRollR + OutputPichtR - OutputPicht /*- OutputYaw + OutputYawR*/;
        m2 = (ch3_power *  1) + OutputRollR - OutputRoll + OutputPichtR - OutputPicht /*+ OutputYaw - OutputYawR*/;
        m3 = (ch3_power * 1) + OutputRoll - OutputRollR + OutputPicht - OutputPichtR /*+ OutputYaw - OutputYawR*/;
        m4 = (ch3_power * 1)  + OutputRollR - OutputRoll + OutputPicht - OutputPichtR /*- OutputYaw + OutputYawR*/;

        //>>>>>>>>>>>>>>>>>>>>>>>>
        SetpointY = headingDegrees;
      }

      //Drones move left 13up use PID rolls
      // m1 +=  OutputRoll;
      //m3 +=  OutputRoll;

      /*
       *
      */

      //Drones move right 24up use PID rollsR
      //m2 +=  OutputRollR;
      // m4 +=  OutputRollR;

      /*
       *
       *
       */
      /*
         *
         */

      //Drones move forward use PID pitchsR 12 up
      // m1 +=  OutputPichtR;
      //m2 +=  OutputPichtR;


      /*
       *
       *
       */

      //// Drones move back use PID pitchs 34 up
      // m3 +=  OutputPicht;
      // m4 +=  OutputPicht;


      /*
       *
       *
       */
      // Drones spin right fix by PID yaws 14up for spin left

      //m1 += OutputYaw;
      //m4 += OutputYaw;
      /*
       *
       *
       */
      // Drones spin left fix by PID yawsR 23up for spin right
      //m2 += OutputYawR;
      //m3 += OutputYawR;

      /*
       *
       *
       */
      // send  m1 m2 m3 m4 value to Motor drive for fix drones to stable
      if (checkBattery()) {
        driveMotor(m1, m2, m3, m4);
      } else {
        ch3_power -= 30;
        if (ch3_power < 1) {
          ch3_power = 0;
        }
        driveMotor(m1, m2, m3, m4);
        ledStatus(90);
      }


      /* rollKp = rollKp + (ch3_power * 0.05);
      // rollKi = rollKi + (ch3_power * 0.005);
       rollKd = rollKd + (ch3_power * 0.0025);
       rolls.SetTunings(rollKp, rollKi, rollKd);
       rollsR.SetTunings(rollKp, rollKi, rollKd);

       rollKp = 0.1;
       rollKi = 0;
       rollKd = 0;
       */
      /*if ((times - timeLED13) >= 1000) {
        timeLED13 = times;
        digitalWrite(13, HIGH);
      } else {
        digitalWrite(13, LOW);
      }
      */
      /* Blink LED to show work in PID loop
      *
      *
      */


      /*
       *
       *
       */

    }
  }//************ End of //PID rolls sampleing time = 20ms  ***************
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

// Function for convert 0' - 360'   to  0 --- 100 ----0 ----100 ---- 0 ---- nifinitry
void convertHeadingDegree() {
  //outputYawSetpoint
  outputYawSetpoint = (M_PI / 180) * (headingDegrees / 2);
  outputYawSetpoint = (sin( outputYawSetpoint)) * 180;//use outputYawSetpoint instead headingdegree

  if ((headingDegrees / 2) > 90) {
    //Serial.println("        revers PID yaw   ");
    yaws.SetControllerDirection(REVERSE);
    yawsR.SetControllerDirection(DIRECT);
  }
  if ((headingDegrees / 2) < 90) {
    // Serial.println("        normal PID  yaw ");
    yawsR.SetControllerDirection(REVERSE);
    yaws.SetControllerDirection(DIRECT);

  }
}
//  End  // Function for convert 0' - 360'   to  0 --- 100
