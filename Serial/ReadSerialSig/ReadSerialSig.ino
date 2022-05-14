/*
Read IMU Acc and Gyro data with sample time.
在此錄檔案給Matlab繪圖，未來希望能做成python realtime
單位: 
1. 加速度: g
2. 陀螺儀: rad/s
*/

#include <Arduino_LSM9DS1.h>
boolean viewInSerialPlotter=true;      //true optimises for serial plotter, false for serial monitor

unsigned long t0;
float sampling_rate = 10*1000; // 10 ms
static long prevMicros = 0;

void setup() 
{  Serial.begin(115200);
   while (!Serial);
 
   if (!IMU.begin())
   { Serial.println("Failed to initialize IMU!");
     while (1);
   }
/*******************    For an improved accuracy run the DIY_Calibration_Accelerometer sketch first.     ****************
********************         Copy/Replace the lines below by the code output of the program              ****************/
   IMU.setAccelFS(3);           
   IMU.setAccelODR(5);           // 
   IMU.setAccelOffset(0, 0, 0);  //   uncalibrated
   IMU.setAccelSlope (1, 1, 1);  //   uncalibrated
   IMU.setGyroFS(3);    
   IMU.setGyroODR(3);
   IMU.setGyroOffset (0, 0, 0);  // = uncalibrated
   IMU.setGyroSlope  (1, 1, 1);  // = uncalibrated
/***********************************************************************************************************************************
*******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
*******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******
************************************************************************************************************************************/
   IMU.accelUnit=  GRAVITY;    // GRAVITY or  METERPERSECOND2    
   IMU.gyroUnit= RADIANSPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND  
   
//   if (!viewInSerialPlotter)
//   {  Serial.println("Gyroscope in degrees/second \n");
//      Serial.print("Accelerometer Full Scale = ±");
//      Serial.print(IMU.getAccelFS());
//      Serial.println ("g");
//      Serial.print("Accelerometer sample rate = ");
//      Serial.print(IMU.getAccelODR());        // alias  AccelerationSampleRate());
//      Serial.println(" Hz \n");
//      delay(4000);
//   }
//   Serial.println(" X \t Y \t Z ");
}

void loop() {
  float x, y, z;
  float gx, gy, gz;

  if (IMU.accelAvailable())                   // alias IMU.accelerationAvailable in library version 1.01
  {  
    t0=micros();
    if (micros() - prevMicros >= sampling_rate)
    {
    Serial.print(t0);
    Serial.print('\t');
    prevMicros += sampling_rate;
    
    IMU.readAccel(x, y, z);                  // alias IMU.readAcceleration  in library version 1.01
     Serial.print(-x);
     Serial.print('\t');
     Serial.print(y);
     Serial.print('\t');
     Serial.print(z);
     Serial.print('\t');
     
     IMU.readGyro(gx, gy, gz);   // alias IMU.readGyroscope
     Serial.print(-gx);
     Serial.print('\t');
     Serial.print(gy);
     Serial.print('\t');
     Serial.println(gz);
    }
  }
}
























/*
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
 Serial.print("I am Malenia, blade of Miquella");
 Serial.print("  ,");
 Serial.println("And I have never known defeat.");
}
*/
