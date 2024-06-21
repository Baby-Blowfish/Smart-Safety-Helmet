#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define Photo_R A0
#define Left_LED 5
#define Right_LED 6
int Day_or_Night = 0;


#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void angle_data_read(void)
{
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


  /* Print Data */
#if 0 // Set to 1 to activate
  // Serial.print(accX); Serial.print("\t");
  // Serial.print(accY); Serial.print("\t");
  // Serial.print(accZ); Serial.print("\t");

  // Serial.print(gyroX); Serial.print("\t");
  // Serial.print(gyroY); Serial.print("\t");
  // Serial.print(gyroZ); Serial.print("\t");

  // Serial.print("\t");
#endif

  //Serial.print(roll); Serial.print("\t");
  //Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print(compAngleX); Serial.print("\t");
  // Serial.print(" X angle : ");
  // Serial.print(kalAngleX); Serial.print("\t");

  // Serial.print("\t");

  //Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
  //Serial.print(compAngleY); Serial.print("\t");
  // Serial.print(" Y angle : ");
  // Serial.print(kalAngleY); Serial.print("\t");

#if 0 // Set to 1 to print the temperature
  // Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  // Serial.print(temperature); Serial.print("\t");
#endif

  // Serial.print("\r\n");
  delay(2);


}

void Lamp_OFF(void)
{
  digitalWrite(Left_LED, LOW);
  digitalWrite(Right_LED, LOW);
  
}

void Lamp(int ledPin) {
  for (int i = 0; i < 5; i++)
  {
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
      delay(500);
    
  }
}




#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include "memorysaver.h"
//This demo can only work on OV2640_MINI_2MP or OV5642_MINI_5MP or OV5642_MINI_5MP_BIT_ROTATION_FIXED platform.
#if !( defined OV2640_MINI_2MP_PLUS)
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
#define BMPIMAGEOFFSET 66
const char bmp_header[BMPIMAGEOFFSET] PROGMEM =
{
  0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
  0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
  0x00, 0x00
};
// set pin 7 as the slave select for the digital pot:
const int CS = 7;
bool is_header = false;
int mode = 0;
uint8_t start_capture = 0;
#if defined (OV2640_MINI_2MP_PLUS)
ArduCAM myCAM( OV2640, CS );
#endif
uint8_t read_fifo_burst(ArduCAM myCAM);







void setup() {

  // Lamp setup
  pinMode(Left_LED, OUTPUT);
  pinMode(Right_LED, OUTPUT);

  digitalWrite(Left_LED, LOW);
  digitalWrite(Right_LED, LOW);

  // Photoresistor
  pinMode(Photo_R, INPUT);


  // put your setup code here, to run once:
  uint8_t vid, pid;
  uint8_t temp;
#if defined(__SAM3X8E__)
  Wire1.begin();
  Serial.begin(115200);
#else
  Wire.begin();
  Serial.begin(921600);
#endif
  Serial.println(F("ACK CMD ArduCAM Start! END"));
  // set the CS as an output:
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  // initialize SPI:
  SPI.begin();
    //Reset the CPLD
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);
  
  while (1) {
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55) {
      Serial.println(F("ACK CMD SPI interface Error!END"));
      delay(1000); continue;
    } else {
      Serial.println(F("ACK CMD SPI interface OK.END")); break;
    }
  }

#if defined (OV2640_MINI_2MP_PLUS)
  while (1) {
    //Check if the camera module type is OV2640
    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))) {
      Serial.println(F("ACK CMD Can't find OV2640 module!"));
      delay(1000); continue;
    }
    else {
      Serial.println(F("ACK CMD OV2640 detected.END")); break;
    }
  }
#endif
  //Change to JPEG capture mode and initialize the OV5642 module
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
#if defined (OV2640_MINI_2MP_PLUS)
  myCAM.OV2640_set_JPEG_size(OV2640_320x240);
#endif
  delay(1000);
  myCAM.clear_fifo_flag();






    // mpu6050 setting
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    // Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();



}







void loop() {

  // put your main code here, to run repeatedly:
  uint8_t temp = 0xff, temp_last = 0;
  bool is_header = false;
  if (Serial.available())
  {
    temp = Serial.read();
    switch (temp)
    {
      case 0:
#if defined (OV2640_MINI_2MP_PLUS)
        myCAM.OV2640_set_JPEG_size(OV2640_160x120); delay(1000);
        Serial.println(F("ACK CMD switch to OV2640_160x120END"));
#elif defined (OV3640_MINI_3MP)
        myCAM.OV3640_set_JPEG_size(OV3640_176x144); delay(1000);
        Serial.println(F("ACK CMD switch to OV2640_160x120END"));
#else
        myCAM.OV5642_set_JPEG_size(OV5642_320x240); delay(1000);
        Serial.println(F("ACK CMD switch to OV5642_320x240END"));
#endif
        temp = 0xff;
        break;
      case 1:
#if defined (OV2640_MINI_2MP_PLUS)
        myCAM.OV2640_set_JPEG_size(OV2640_176x144); delay(1000);
        Serial.println(F("ACK CMD switch to OV2640_176x144END"));
#elif defined (OV3640_MINI_3MP)
        myCAM.OV3640_set_JPEG_size(OV3640_320x240); delay(1000);
        Serial.println(F("ACK CMD switch to OV3640_320x240END"));
#else
        myCAM.OV5642_set_JPEG_size(OV5642_640x480); delay(1000);
        Serial.println(F("ACK CMD switch to OV5642_640x480END"));
#endif
        temp = 0xff;
        break;
      case 2:
#if defined (OV2640_MINI_2MP_PLUS)
        myCAM.OV2640_set_JPEG_size(OV2640_320x240); delay(1000);
        Serial.println(F("ACK CMD switch to OV2640_320x240END"));
#elif defined (OV3640_MINI_3MP)
        myCAM.OV3640_set_JPEG_size(OV3640_352x288); delay(1000);
        Serial.println(F("ACK CMD switch to OV3640_352x288END"));
#else
        myCAM.OV5642_set_JPEG_size(OV5642_1024x768); delay(1000);
        Serial.println(F("ACK CMD switch to OV5642_1024x768END"));
#endif
        temp = 0xff;
        break;
      case 3:
        temp = 0xff;
#if defined (OV2640_MINI_2MP_PLUS)
        myCAM.OV2640_set_JPEG_size(OV2640_352x288); delay(1000);
        Serial.println(F("ACK CMD switch to OV2640_352x288END"));
#elif defined (OV3640_MINI_3MP)
        myCAM.OV3640_set_JPEG_size(OV3640_640x480); delay(1000);
        Serial.println(F("ACK CMD switch to OV3640_640x480END"));
#else
        myCAM.OV5642_set_JPEG_size(OV5642_1280x960); delay(1000);
        Serial.println(F("ACK CMD switch to OV5642_1280x960END"));
#endif
        break;
      case 4:
        temp = 0xff;
#if defined (OV2640_MINI_2MP)
        myCAM.OV2640_set_JPEG_size(OV2640_640x480); delay(1000);
        Serial.println(F("ACK CMD switch to OV2640_640x480END"));
#elif defined (OV3640_MINI_3MP)
        myCAM.OV3640_set_JPEG_size(OV3640_800x600); delay(1000);
        Serial.println(F("ACK CMD switch to OV3640_800x600END"));
#else
        myCAM.OV5642_set_JPEG_size(OV5642_1600x1200); delay(1000);
        Serial.println(F("ACK CMD switch to OV5642_1600x1200END"));
#endif
        break;
      case 5:
        temp = 0xff;
#if defined (OV2640_MINI_2MP_PLUS)
        myCAM.OV2640_set_JPEG_size(OV2640_800x600); delay(1000);
        Serial.println(F("ACK CMD switch to OV2640_800x600END"));
#elif defined (OV3640_MINI_3MP)
        myCAM.OV3640_set_JPEG_size(OV3640_1024x768); delay(1000);
        Serial.println(F("ACK CMD switch to OV3640_1024x768END"));
#else
        myCAM.OV5642_set_JPEG_size(OV5642_2048x1536); delay(1000);
        Serial.println(F("ACK CMD switch to OV5642_2048x1536END"));
#endif
        break;
      case 6:
        temp = 0xff;
#if defined (OV2640_MINI_2MP_PLUS)
        myCAM.OV2640_set_JPEG_size(OV2640_1024x768); delay(1000);
        Serial.println(F("ACK CMD switch to OV2640_1024x768END"));
#elif defined (OV3640_MINI_3MP)
        myCAM.OV3640_set_JPEG_size(OV3640_1280x960); delay(1000);
        Serial.println(F("ACK CMD switch to OV3640_1280x960END"));
#else
        myCAM.OV5642_set_JPEG_size(OV5642_2592x1944); delay(1000);
        Serial.println(F("ACK CMD switch to OV5642_2592x1944END"));
#endif
        break;
      case 7:
        temp = 0xff;
#if defined (OV2640_MINI_2MP_PLUS)
        myCAM.OV2640_set_JPEG_size(OV2640_1280x1024); delay(1000);
        Serial.println(F("ACK CMD switch to OV2640_1280x1024END"));
#else
        myCAM.OV3640_set_JPEG_size(OV3640_1600x1200); delay(1000);
        Serial.println(F("ACK CMD switch to OV3640_1600x1200END"));
#endif
        break;
      case 8:
        temp = 0xff;
#if defined (OV2640_MINI_2MP_PLUS)
        myCAM.OV2640_set_JPEG_size(OV2640_1600x1200); delay(1000);
        Serial.println(F("ACK CMD switch to OV2640_1600x1200END"));
#else
        myCAM.OV3640_set_JPEG_size(OV3640_2048x1536); delay(1000);
        Serial.println(F("ACK CMD switch to OV3640_2048x1536END"));
#endif
        break;
      case 0x10:
        mode = 1;
        temp = 0xff;
        start_capture = 1;
        Serial.println(F("ACK CMD CAM start single shoot.END"));
        break;
      case 0x11:
        temp = 0xff;
        Serial.println(F("ACK CMD Change OK.END"));
        myCAM.set_format(JPEG);
        myCAM.InitCAM();
        myCAM.OV2640_set_JPEG_size(OV2640_320x240); 
        break;
      case 0x20:
        mode = 2;
        temp = 0xff;
        start_capture = 2;
        Serial.println(F("ACK CMD CAM start video streaming.END"));
        break;
      case 0x30:
        mode = 3;
        temp = 0xff;
        start_capture = 3;
        Serial.println(F("ACK CMD CAM start single shoot.END"));
        break;
      case 0x31:
        temp = 0xff;
        myCAM.set_format(BMP);
        myCAM.InitCAM();
        break;
      default:
        break;
    }
  }
  if (mode == 1)
  {
    if (start_capture == 1)
    {
      myCAM.flush_fifo();
      myCAM.clear_fifo_flag();
      //Start capture
      myCAM.start_capture();
      start_capture = 0;
    }
    if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
    {
      Serial.println(F("ACK CMD CAM Capture Done.END"));
      delay(50);
      read_fifo_burst(myCAM);
      //Clear the capture done flag
      myCAM.clear_fifo_flag();
    }
  }
  else if (mode == 2)
  {
    while (1)
    {
      angle_data_read(); // angle 값 읽기
      Day_or_Night = analogRead(Photo_R); // Photo resistor 

      if(Day_or_Night<590)
      {
        if(kalAngleX<-30) Lamp(Left_LED);
        else if(30<kalAngleX) Lamp(Right_LED);
        else Lamp_OFF();
      }
      else
      {
        digitalWrite(Left_LED, HIGH);
        digitalWrite(Right_LED, HIGH);
        delay(100);
        digitalWrite(Left_LED, LOW);
        digitalWrite(Right_LED, LOW);
        delay(100);
        if(kalAngleX<-60) Lamp(Left_LED);
        else if(60<kalAngleX) Lamp(Right_LED);
        else ;
      }




      temp = Serial.read();
      if (temp == 0x21)
      {
        start_capture = 0;
        mode = 0;
        Serial.println(F("ACK CMD CAM stop video streaming.END"));
        break;
      }
      if (start_capture == 2)
      {
        myCAM.flush_fifo();
        myCAM.clear_fifo_flag();
        //Start capture
        myCAM.start_capture();
        start_capture = 0;
      }
      if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
      {
        uint32_t length = 0;
        length = myCAM.read_fifo_length();
        if ((length >= MAX_FIFO_SIZE) | (length == 0))
        {
          myCAM.clear_fifo_flag();
          start_capture = 2;
          continue;
        }
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();//Set fifo burst mode
        temp =  SPI.transfer(0x00);
        length --;
        while ( length-- )
        {
          temp_last = temp;
          temp =  SPI.transfer(0x00);
          if (is_header == true)
          {
            Serial.write(temp);
          }
          else if ((temp == 0xD8) & (temp_last == 0xFF))
          {
            is_header = true;
            Serial.println(F("ACK CMD IMG END"));
            Serial.write(temp_last);
            Serial.write(temp);
          }
          if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
            break;
          delayMicroseconds(15);
        }
        myCAM.CS_HIGH();
        myCAM.clear_fifo_flag();
        start_capture = 2;
        is_header = false;
      }
    }
  }
  else if (mode == 3)
  {
    if (start_capture == 3)
    {
      //Flush the FIFO
      myCAM.flush_fifo();
      myCAM.clear_fifo_flag();
      //Start capture
      myCAM.start_capture();
      start_capture = 0;
    }
    if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
    {
      Serial.println(F("ACK CMD CAM Capture Done.END"));
      delay(50);
      uint8_t temp, temp_last;
      uint32_t length = 0;
      length = myCAM.read_fifo_length();
      if (length >= MAX_FIFO_SIZE )
      {
        Serial.println(F("ACK CMD Over size.END"));
        myCAM.clear_fifo_flag();
        return;
      }
      if (length == 0 ) //0 kb
      {
        Serial.println(F("ACK CMD Size is 0.END"));
        myCAM.clear_fifo_flag();
        return;
      }
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();//Set fifo burst mode

      Serial.write(0xFF);
      Serial.write(0xAA);
      for (temp = 0; temp < BMPIMAGEOFFSET; temp++)
      {
        Serial.write(pgm_read_byte(&bmp_header[temp]));
      }
      char VH, VL;
      int i = 0, j = 0;
      for (i = 0; i < 240; i++)
      {
        for (j = 0; j < 320; j++)
        {
          VH = SPI.transfer(0x00);;
          VL = SPI.transfer(0x00);;
          Serial.write(VL);
          delayMicroseconds(12);
          Serial.write(VH);
          delayMicroseconds(12);
        }
      }
      Serial.write(0xBB);
      Serial.write(0xCC);
      myCAM.CS_HIGH();
      //Clear the capture done flag
      myCAM.clear_fifo_flag();
    }
  }
}
uint8_t read_fifo_burst(ArduCAM myCAM)
{
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  length = myCAM.read_fifo_length();
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //512 kb
  {
    Serial.println(F("ACK CMD Over size.END"));
    return 0;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println(F("ACK CMD Size is 0.END"));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();//Set fifo burst mode
  temp =  SPI.transfer(0x00);
  length --;
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    if (is_header == true)
    {
      Serial.write(temp);
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      Serial.println(F("ACK CMD IMG END"));
      Serial.write(temp_last);
      Serial.write(temp);
    }
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
      break;
    delayMicroseconds(15);
  }
  myCAM.CS_HIGH();
  is_header = false;
  return 1;
}
