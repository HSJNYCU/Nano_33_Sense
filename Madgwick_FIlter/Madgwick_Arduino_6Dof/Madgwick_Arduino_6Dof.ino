/*
測試 Madgwick filter，使用Arduino nana 33 sense的LSM9DS1蒐集數據並轉換成尤拉角輸出
需求加速度單位: m/s^2
需求角速度單位: rad/s

紀錄:
---------------------------------
2022/05/14 初版完成
*/


#include <Arduino_LSM9DS1.h>
boolean viewInSerialPlotter=true;      //true optimises for serial plotter, false for serial monitor

unsigned long t0;
float sampling_rate = 10*1000; // 10 ms
static long prevMicros = 0;

// System constants
#define deltat 0.01f                                     // sampling period in seconds (shown as 10 ms)
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError            // compute beta

// Global system variables
float a_x, a_y, a_z; // accelerometer measurements
float w_x, w_y, w_z; // gyroscope measurements in rad/s
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  if (!IMU.begin())
  { Serial.println("Failed to initialize IMU!");
   while (1);
  }
  
  IMU.setAccelFS(3);           
  IMU.setAccelODR(5);           // 
  IMU.setAccelOffset(0, 0, 0);  //   uncalibrated
  IMU.setAccelSlope (1, 1, 1);  //   uncalibrated
  IMU.setGyroFS(3);    
  IMU.setGyroODR(3);
  IMU.setGyroOffset (0, 0, 0);  // = uncalibrated
  IMU.setGyroSlope  (1, 1, 1);  // = uncalibrated

  IMU.accelUnit=  METERPERSECOND2;    // or  METERPERSECOND2    
  IMU.gyroUnit= RADIANSPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND  
  
//  if (!viewInSerialPlotter)
//  {  Serial.println("Gyroscope in degrees/second \n");
//    Serial.print("Accelerometer Full Scale = ±");
//    Serial.print(IMU.getAccelFS());
//    Serial.println ("g");
//    Serial.print("Accelerometer sample rate = ");
//    Serial.print(IMU.getAccelODR());        // alias  AccelerationSampleRate());
//    Serial.println(" Hz \n");
//    delay(4000);
//  }
//  Serial.println(" X \t Y \t Z ");
}

void loop() {
  if (IMU.accelAvailable())                   // alias IMU.accelerationAvailable in library version 1.01
  {  
    t0=micros();
    if (micros() - prevMicros >= sampling_rate)
    {
      IMU.readAccel(a_x, a_y, a_z);  // alias IMU.readAcceleration  in library version 1.01
      IMU.readGyro(w_x, w_y, w_z);   // alias IMU.readGyroscope
      prevMicros += sampling_rate;

//      Serial.print("t");
//      Serial.print(t0);
//      Serial.print('\t');
     
      // Local system variables
      float norm; // vector norm
      float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
      float f_1, f_2, f_3; // objective function elements
      float J_11, J_12, J_13, J_14, J_32, J_33; // objective function Jacobian elements
      float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
      
      // Axulirary variables to avoid reapeated calcualtions
      float halfSEq_1 = 0.5f * SEq_1;
      float halfSEq_2 = 0.5f * SEq_2;
      float halfSEq_3 = 0.5f * SEq_3;
      float halfSEq_4 = 0.5f * SEq_4;
      float twoSEq_1 = 2.0f * SEq_1;
      float twoSEq_2 = 2.0f * SEq_2;
      float twoSEq_3 = 2.0f * SEq_3;
    
      // Normalise the accelerometer measurement
      norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
      a_x /= -norm;
      a_y /= norm;
      a_z /= norm;
      
      // Compute the objective function and Jacobian
      f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
      f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
      f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
      J_11 = twoSEq_3; // J_11 negated in matrix multiplication
      J_12 = 2.0f * SEq_4;
      J_13 = twoSEq_1; // J_12 negated in matrix multiplication
      J_14 = twoSEq_2;
      J_32 = 2.0f * J_14; // negated in matrix multiplication
      J_33 = 2.0f * J_11; // negated in matrix multiplication
      
      // Compute the gradient (matrix multiplication)
      SEqHatDot_1 = J_14 * f_2 - J_11 * f_1;
      SEqHatDot_2 = J_12 * f_1 + J_13 * f_2 - J_32 * f_3;
      SEqHatDot_3 = J_12 * f_2 - J_33 * f_3 - J_13 * f_1;
      SEqHatDot_4 = J_14 * f_1 + J_11 * f_2;
      
      // Normalise the gradient
      norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
      SEqHatDot_1 /= norm;
      SEqHatDot_2 /= norm;
      SEqHatDot_3 /= norm;
      SEqHatDot_4 /= norm;
      
      // Compute the quaternion derrivative measured by gyroscopes
      SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
      SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
      SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
      SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
      
      // Compute then integrate the estimated quaternion derrivative
      SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
      SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
      SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
      SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
      
      // Normalise quaternion
      norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
      SEq_1 /= norm; 
      SEq_2 /= norm;
      SEq_3 /= norm;
      SEq_4 /= norm; // equivalent to the expression SEq_4 = SEq_4 / norm;

      // Check Euler angle
      float Psi, Theta, Phi;
      Psi   = quaternion2Psi(SEq_1, SEq_2, SEq_3, SEq_4);
      Theta = quaternion2Theta(SEq_1, SEq_2, SEq_3, SEq_4);
      Phi   = quaternion2Phi(SEq_1, SEq_2, SEq_3, SEq_4);

      Serial.print("Psi");
      Serial.print(Psi*57.3);
      Serial.print(",");
      Serial.print("Theta");
      Serial.print(Theta*57.3);
      Serial.print(",");
      Serial.print("Phi");
      Serial.println(Phi*57.3);
    }
  }
}








// Quaternion to Euler angle Phi
float quaternion2Phi(float q0, float q1, float q2, float q3){
  float phi;
  phi = atan2( 2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2) );
  return phi;
}

// Quaternion to Euler angle Theta
float quaternion2Theta(float q0, float q1, float q2, float q3){
  float theta;
  theta = asin( 2*(q0*q2-q3*q1) );
  return theta;
}

// Quaternion to Euler angle Psi
float quaternion2Psi(float q0, float q1, float q2, float q3){
  float psi;
  psi = atan2( 2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3) );
  return psi;
}


// Quaternion to Euler angle
float quaternion2Euler(float q0, float q1, float q2, float q3){
  float psi;
  psi = atan2( 2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3) );
  return psi;
}
