#include <Arduino.h>

//library MPU6050
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <arduinoFFT.h>
MPU6050 mpu;

//library RTC
#include <Wire.h>
#include "RTClib.h"

//library Json
#include <ArduinoJson.h>

//library NRF24L01
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include "printf.h"

//konfigurasi stack size
SET_LOOP_TASK_STACK_SIZE(32 * 1024); // 64KB

//konfigurasi RTC
RTC_DS3231 rtc;
char days[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//konfigurasi NRF24L01
RF24 radio(4, 5); //(pin CE, pin CSN)
RF24Network network(radio);      // Network uses that radio
RF24Mesh mesh(radio, network);
uint8_t dataBuffer[MAX_PAYLOAD_SIZE];  //MAX_PAYLOAD_SIZE is defined in RF24Network_config.h

//alamat node
#define this_node 3

//variabel DATA
int node_asal = 3; //ID node
String datakirim;

//variabel millis
unsigned long currentMillis = 0;

//konfigurasi MPU6050
const int MPU_ADDR = 0x69; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t AcXcal, AcYcal, AcZcal, GyXcal, GyYcal, GyZcal; //calibration variables
int16_t AcX, AcY, AcZ; // variables for accelerometer raw data
int16_t GyX, GyY, GyZ; // variables for gyro raw data
int16_t temperature; // variables for temperature data
float KalAcX, KalAcY, KalAcZ;
float pitch, roll;
float Xt, Xt_update, Xt_prev;
float Pt, Pt_update, Pt_prev;
float Kt, R, Q;
float Xt2, Xt2_update, Xt2_prev;
float Pt2, Pt2_update, Pt2_prev;
float Kt2, R2, Q2;
float Xt3, Xt3_update, Xt3_prev;
float Pt3, Pt3_update, Pt3_prev;
float Kt3, R3, Q3;
float kalPitch, kalRoll, f;
const uint16_t samples = 64;
double vReal[samples];
double vReal2[samples];
double vReal3[samples];
const double samplingFrequency = 15;
float offset_pitch, offset_roll, sum_pitch, sum_roll;
int a = 1;
int N = 1000;
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
#define SCL_FREQUENCY 0x02

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

//function to convert accelerometer values into pitch and roll
void getAngle(int Ax, int Ay, int Az)
{
  float x = Ax;
  float y = Ay;
  float z = Az;

  kalPitch = atan(x / sqrt((y * y) + (z * z))); //pitch calculation
  kalRoll = atan(y / sqrt((x * x) + (z * z))); //roll calculation

  //converting radians into degrees
  kalPitch = kalPitch * (180.0 / 3.14) - 1.3;
  kalRoll = kalRoll * (180.0 / 3.14) - 2.3 ;
}

#define DEBUG
#ifdef DEBUG
//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define  DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}//StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
#define DPRINTSTIMER(t)    if(false)
#define DPRINTSFN(...)     //blank line
#define DPRINTLN(...)      //blank line
#endif



#define LED_PIN 13 // 

// supply your own gyro offsets here, scaled for min sensitivity use MPU6050_calibration.ino
// -4232  -706  1729  173 -94 37
//                       XA      YA      ZA      XG      YG      ZG
//int MPUOffsets[6] = {  -1981,  4981,   331,    14,    -188,     58};
int MPUOffsets[6] = {  -1879,  4952,   3852,    20,    -185,     58};

// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
int FifoAlive = 0; // tests if the interrupt is triggering
int IsAlive = -20;     // counts interrupt start at -20 to get 20+ good values before assuming connected
// MPU control/status vars
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
byte StartUP = 100; // lets get 100 readings from the MPU before we start trusting them (Bot is not trying to balance at this point it is just starting up.)

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize(); // same
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();// same

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    MPU6050Connect(); // Lets try again
    return;
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu.getInterruptDrive=  "); Serial.println(mpu.getInterruptDrive());
  attachInterrupt(0, dmpDataReady, RISING); //pin 2 on the Uno
  mpuIntStatus = mpu.getIntStatus(); // Same
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt
}

// ================================================================
// ===                        MPU Math                          ===
// ================================================================
float Yaw, Pitch, Roll;
void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Yaw = (ypr[0] * 180.0 / M_PI);
  Pitch = (ypr[1] *  180.0 / M_PI);
  Roll = (ypr[2] *  180.0 / M_PI);
  DPRINTSTIMER(100) {
    DPRINTSFN(15, "\tYaw:", Yaw, 6, 1);
    DPRINTSFN(15, "\tPitch:", Pitch, 6, 1);
    DPRINTSFN(15, "\tRoll:", Roll, 6, 1);
    DPRINTLN();
  }
}

// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void GetDMP() { // Best version I have made so far
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());
  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) { // we have failed Reset and wait till next time!
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    LastGoodPacketTime = millis();
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}

void PrintVector(double * vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
        break;
    }
  }
}

//Fungsi untuk 2 loop
TaskHandle_t Task1;

//program loop 2
void bacasensor( void * parameter) {
 for (;;) {
   for (uint16_t i = 0; i < samples; i++) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

      // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
      AcX = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      AcY = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
      AcZ = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
      //temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
      GyX = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
      GyY = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
      GyZ = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
      R = 100; Q = 1;
      Pt_prev = 1;
      R2 = 100; Q2 = 1;
      Pt2_prev = 1;
      R3 = 100; Q3 = 1;
      Pt3_prev = 1;

      Xt_update = Xt_prev;
      Pt_update = Pt_prev + Q;
      Kt = Pt_update / (Pt_update + R);
      Xt = Xt_update + (Kt * (AcX - Xt_update));
      Pt = (1 - Kt) * Pt_update;

      Xt_prev = Xt;
      Pt_prev = Pt;

      KalAcX = Xt;

      Xt2_update = Xt2_prev;
      Pt2_update = Pt2_prev + Q2;
      Kt2 = Pt2_update / (Pt2_update + R2);
      Xt2 = Xt2_update + (Kt2 * (AcY - Xt2_update));
      Pt2 = (1 - Kt2) * Pt2_update;

      Xt2_prev = Xt2;
      Pt2_prev = Pt2;

      KalAcY = Xt2;

      Xt3_update = Xt3_prev;
      Pt3_update = Pt3_prev + Q3;
      Kt3 = Pt3_update / (Pt3_update + R3);
      Xt3 = Xt3_update + (Kt3 * (AcZ - Xt3_update));
      Pt3 = (1 - Kt3) * Pt3_update;

      Xt3_prev = Xt3;
      Pt3_prev = Pt3;

      KalAcZ = Xt3;

      vReal2[i] = KalAcY;
    }

    PrintVector(vReal2, (samples >> 1), SCL_FREQUENCY);
    f = FFT.MajorPeak(vReal2, samples, samplingFrequency);

    //  get pitch/roll
    getAngle(KalAcX, KalAcY, KalAcZ);
    kalPitch = kalPitch - 0.8;
    kalRoll = kalRoll + 2.4;
    f = abs(f - 0.12);
    Serial.println("Running on Core : "+String(xPortGetCoreID())+", Pitch : "+String(kalPitch)+", Roll : "+String(kalRoll)+", Frekuensi : "+String(f));
 }
}

void setup() {
  Serial.begin(115200);

  //MPU6050
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.println("Setup complete");
  pinMode(LED_PIN, OUTPUT);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  if (! rtc.begin()) {
    Serial.println("Tidak dapat menemukan RTC! Periksa sirkuit.");
    while (1);
  }

  while (!Serial) {
    // some boards need this because of native USB capability
  }
  mesh.setNodeID(this_node); //Set the Node ID
  Serial.println(F("Menghubungkan ke jaringan..."));

  if (!mesh.begin()) {
    if (radio.isChipConnected()) {
      do {
        // mesh.renewAddress() will return MESH_DEFAULT_ADDRESS on failure to connect
        Serial.println(F("Gagal terhubung ke jaringan.\nMenghubungkan ke jaringan..."));
      } while (mesh.renewAddress() == MESH_DEFAULT_ADDRESS);
    } else {
      Serial.println(F("NRF24L01 tidak merespon."));
      while (1) {
        // hold in an infinite loop
      }
    }
  }
  printf_begin();
  radio.printDetails();  // print detail konfigurasi NRF24L01

  //Fungsi untuk 2 loop
   xTaskCreatePinnedToCore(
     bacasensor,    /* Task function. */
     "baca_sensor", /* name of task. */
     32768,         /* Stack size of task */
     NULL,          /* parameter of the task */
     1,             /* priority of the task */
     &Task1,        /* Task handle to keep track of created task */
     0);            /* pin task to core 0 */

  // print memori stack keseluruhan
  Serial.printf("Arduino Stack was set to %d bytes", getArduinoLoopTaskStackSize());
  // print sisa memori stack pada void setup
  Serial.printf("\nSetup() - Free Stack Space: %d", uxTaskGetStackHighWaterMark(NULL));
}

void loop() {
  // print sisa memori stack pada void loop
  Serial.printf("\nLoop() - Free Stack Space: %d", uxTaskGetStackHighWaterMark(NULL));

  mesh.update();
  DateTime now = rtc.now();
  StaticJsonDocument<128> doc;

  // Mengirim data ke master
  if (millis() - currentMillis >= 250) {
    currentMillis = millis();
    doc["NodeID"] = String(node_asal);
    doc["Pitch"] = String(kalPitch);
    doc["Roll"] = String(kalRoll);
    doc["Frekuensi"] = String(f);
    doc["Unixtime"] = String(now.unixtime());
    datakirim = "";
    serializeJson(doc, datakirim);
    char kirim_loop[datakirim.length() + 1];
    datakirim.toCharArray(kirim_loop, sizeof(kirim_loop));

    if (!mesh.write(&kirim_loop, '3', sizeof(kirim_loop))) {
      if (!mesh.checkConnection()) {
        Serial.println("Memperbaharui Alamat");
        if (mesh.renewAddress() == MESH_DEFAULT_ADDRESS) {
          mesh.begin();
        }
      } else {
        Serial.println("Gagal Mengirim ke Master, Tes jaringan OK");
      }
    } else {
      Serial.print("Berhasil Mengirim ke Master : ");
      Serial.println(datakirim);
    }
  }
}
