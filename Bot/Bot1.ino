#include <SPI.h>
#include <Wire.h>   

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_XOUT_H  0x04
#define AK8963_YOUT_L  0x05
#define AK8963_YOUT_H  0x06
#define AK8963_ZOUT_L  0x07
#define AK8963_ZOUT_H  0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value
#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F
#define SELF_TEST_A      0x10
#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F
#define MOT_DUR          0x20
#define ZMOT_THR         0x21
#define ZRMOT_DUR        0x22
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A
#define PWR_MGMT_1       0x6B
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D
#define DMP_RW_PNT       0x6E
#define DMP_REG          0x6F
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E
#define ADO 1
#if ADO
#define MPU9250_ADDRESS 0x68
#else
#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C  
#endif  
#define AHRS true         
#define SerialDebug true   

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0,
  MFS_16BITS      
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; 
uint8_t Mmode = 0x02;        
float aRes, gRes, mRes;      
  
// Pin definitions
int intPin = 12;
int myLed = 13;

int16_t accelCount[3];  
int16_t gyroCount[3];   
int16_t magCount[3];    
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      
int16_t tempCount;     
float   temperature;    
float   SelfTest[6];  

float GyroMeasError = PI * (40.0f / 180.0f);  
float GyroMeasDrift = PI * (0.0f  / 180.0f);  
float beta = sqrt(3.0f / 4.0f) * GyroMeasError; 
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; 
#define Kp 2.0f * 5.0f 
#define Ki 0.0f

uint32_t delt_t = 0;
uint32_t count = 0, sumCount = 0
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;       
uint32_t lastUpdate = 0, firstUpdate = 0; 
uint32_t Now = 0;        

float ax, ay, az, gx, gy, gz, mx, my, mz; 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    
float eInt[3] = {0.0f, 0.0f, 0.0f};       

float magBias[3],magScale[3];

const float declinationAngle = -0.018617;
float distance = 0.0;
float angle = 0.0;
float error = 0.0;
float pastError = 0.0;
float pastErrors = 0.0;
const float KP = 5.0;
const float KI = 0.0;
const float KD = 10.0;

//int pwm1 = 13; int forward=12; int reverse=11;
//int encoderValue=0; void count(void);
int pwm1 = 10;
int encoderValue = 0;
void counting(void);

WiFiClient client;
WiFiServer server(80);
const char* ssid = "hotspot";
const char* password = "password";
char angleid = "x";
int ang = 0;

void setup()
{
  server.begin();
  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);
  //pinMode(9,INPUT);
  pinMode(3,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), count, RISING);
  //attachInterrupt(2,count,FALLING);
  encoderValue=0;

  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  Serial.println("MPU9250");
  Serial.println("9-DOF 16-bit"); 
  Serial.println("motion sensor"); 
  Serial.println("60 ug LSB");
  delay(800);
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); 
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  delay(800); 

  if (c == 0x71) 
  {  
    Serial.println("MPU9250 is online...");
    
    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
 
    calibrateMPU9250(gyroBias, accelBias); 

    Serial.println("MPU9250 bias");
    Serial.println(" x   y   z  ");
    Serial.print((int)(1000*accelBias[0])); 
    Serial.print((int)(1000*accelBias[1])); 
    Serial.print((int)(1000*accelBias[2])); 
    Serial.print("mg");
    Serial.print(gyroBias[0], 1); 
    Serial.print(gyroBias[1], 1); 
    Serial.print(gyroBias[2], 1); 
    Serial.println("o/s"); 
    delay(1000); 
  
    initMPU9250(); 
    Serial.println("MPU9250 initialized for active data mode...."); 
  
    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

    delay(1000); 
  
    // Get magnetometer calibration from AK8963 ROM
    initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode....");
    getMres();
    magcalMPU9250(magBias,magScale); 

  
  if(SerialDebug) {
    //  Serial.println("Calibration values: ");
    Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
  }
  
    Serial.println("AK8963");
    Serial.println("ASAX ");
    Serial.println(magCalibration[0], 2);
    Serial.println("ASAY "); 
    Serial.println(magCalibration[1], 2);
    Serial.println("ASAZ ");
    Serial.println(magCalibration[2], 2);
    delay(1000);  
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; 
  }
}

void loop()
{  
  client = server.available();
  if (!client) return; 
  data = checkClient ();
  command = String(data);
  angleid = command[0];
  if (angleid == "f") angle = ang;
  else if (angleid == "b") angle = ang + 180;
  else if (angleid == "l") angle = ang + 90;
  else if (angleid == "r") angle = ang + 270;
  if (angle > 360) angle = angle - 360;
  //angle = 90;
  distance = command.substring(1,3);  
  
  //distance = 90;
  //angle = 90;
  while(1){
 
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) { 
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();
    
    ax = (float)accelCount[0]*aRes; // - accelBias[0];
    ay = (float)accelCount[1]*aRes; // - accelBias[1];   
    az = (float)accelCount[2]*aRes; // - accelBias[2];  
   
    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
 
    gx = (float)gyroCount[0]*gRes; 
    gy = (float)gyroCount[1]*gRes;  
    gz = (float)gyroCount[2]*gRes;   
  
    readMagData(magCount);  // Read the x/y/z adc values
    getMres();
//    magbias[0] = +470.;  
//    magbias[1] = +120.; 
//    magbias[2] = +125.;
    
    // Calculate the magnetometer values in milliGauss
    mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0]; 
    my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
    mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];   
  }
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); 
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
//MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

    if (!AHRS) {
    delt_t = millis() - count;
    if(delt_t > 500) {
  
      if(SerialDebug) {
      }
         
      count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
      }
    }
    else {
      
    delt_t = millis() - count;
    if (delt_t > 50) { 

      if(SerialDebug) {
      }               
    
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI; 
      if (yaw < 0){
        yaw = 360.0f + yaw;
      }
      yaw   += declinationAngle;
      roll  *= 180.0f / PI;
  
      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(yaw, 2);
      Serial.print(", ");
      Serial.print(pitch, 2);
      Serial.print(", ");
      Serial.println(roll, 2);
  
        error = angle - yaw;
        if (angle > yaw){
          if ((abs(angle - yaw)) < abs(-360 + angle - yaw)){
            error = angle - yaw;
          }
          else{
            error = -360 + angle - yaw; 
          }
        }
        else{
          if ((abs(angle - yaw)) < abs(360+angle-yaw)){
            error = angle - yaw;
          }
          else{
            error = 360+angle-yaw;
          }
        }

        if (distance < encoderValue){
          
        if (abs(error) >= 5.0){
          int pwm = (int)((KP*error) + (KI*(error+pastErrors)) + (KD*(error-pastError)));
          if (abs(pwm) < 70){
            if (pwm > 0) pwm = 70;
            else pwm = -70;
          }
          if (abs(pwm) > 255){
            if (pwm > 0) pwm = 255;
            else pwm = -255;
          }
          Serial.print(F("error: ")); Serial.print(error); Serial.print(F(" pwm: ")); Serial.println(pwm);
          moveBot(pwm, -pwm, 0);
          pastErrors += error;   
          pastError = error;      
        }
        else{
          pastErrors = 0;
          pastError = 0;
          moveBot(200, 200, 0);
        }
        else{
          moveBot(0, 0, 0)
          delay(1000);
          break;
          }
        }
      count = millis(); 
      sumCount = 0;
      sum = 0;    
      }
    }
  }
  //send data to PC
  Serial.print("DONE");
}

void counting(){
  encoderValue++;
}

void moveBot(int left, int right, int del){
    if (encoderValue < distance){
      if (left < 0){
        digitalWrite(in2, HIGH);
      }
      else{
        digitalWrite(in2, LOW);
      }
      if (right < 0){
        digitalWrite(in3, LOW);
      }
      else{
        digitalWrite(in3, HIGH);
      }
      int sL = abs(left);
      int sR = abs(right);
      analogWrite(enAL, sL);
      analogWrite(enBR, sR);
      
      if (del) delay(del);
    }
}

//functions to access acceleration. gyroscope, magnetometer, and temperature data

void getMres() {
  switch (Mscale)
  {
    case MFS_14BITS:
          mRes = 10.*4912./8190.; 
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0;
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6]; 
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ; 
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); 
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ; 
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { 
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { 
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ; 
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
   }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]); 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  
}
       
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3]; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); 
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F);
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]); 
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); 
  delay(10);
}


void initMPU9250()
{ 
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); 
  delay(100); // Wait for all registers to reset 
  
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  delay(200); 

  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  

  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04); 
     
 
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x02); 
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); 
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3); 
  
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); 

  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F);   
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03);

   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01); 
   delay(100);
}


void calibrateMPU9250(float * dest1, float * dest2)
{  
  uint8_t data[12];
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
  delay(100);
   
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);                                    

  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);  
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);  
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); 
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    
  delay(15);
  
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);     
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); 
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);    
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); 
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; 
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
  
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  int32_t accel_bias_reg[3] = {0, 0, 0}; 
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); 
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; 
  uint8_t mask_bit[3] = {0, 0, 0}; 
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; 
  }
  
  accel_bias_reg[0] -= (accel_bias[0]/8); 
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; 
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2];
  
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
   

}
   
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
   float factoryTrim[6];
   uint8_t FS = 0;
   
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS);  
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS); 

  for( int ii = 0; ii < 200; ii++) { 
  
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);       
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; 
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);      
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); 
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); 
   delay(25); 

  for( int ii = 0; ii < 200; ii++) {  
  
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); 
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; 
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) { 
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); 
   selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL);
   selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); 
   selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  
   selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  
   selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); 
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); 
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); 
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); 
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) ));
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); 
 
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];  
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; 
   }
   
}

        
        // Wire.h read and write protocols
        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  
  Wire.write(subAddress);          
  Wire.write(data);                
  Wire.endTransmission();           
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         
  Wire.write(subAddress);                 
  Wire.endTransmission(false);             
  Wire.requestFrom(address, (uint8_t) 1);  
  data = Wire.read();                      
  return data;                            
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);  
  Wire.write(subAddress);            
  Wire.endTransmission(false);      
  uint8_t i = 0;
        Wire.requestFrom(address, count); 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }       
}
