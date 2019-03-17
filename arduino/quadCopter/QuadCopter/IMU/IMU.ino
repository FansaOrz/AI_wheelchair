#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <ADXL345.h> 
#include <HMC5883L.h> 
#include <ITG3200.h>
/*LOW PASS FILTER*/
/*https://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/*/
ADXL345 acc; //variable adxl is an instance of the ADXL345 library
HMC5883L compass;
ITG3200 gyro = ITG3200();

int ax,ay,az;  
int rawX, rawY, rawZ;
float X, Y, Z;
int error = 0; 
float aoffsetX, aoffsetY, aoffsetZ;
float rollX,pitchY,yawZ = 0.0;
sensor_msgs::Imu Imu_msg;
ros::Publisher chatter("imu_data", &Imu_msg);

void setup(){
  Serial.begin(9600);
  acc.powerOn();
  compass = HMC5883L();
  error = compass.SetScale(1.3); // Set the scale to +/- 1.3 Ga of the compass
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  
 // Serial.println("Setting measurement mode to continous");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  for (int i = 0; i <= 200; i++) {
    acc.readAccel(&ax, &ay, &az);
    if (i == 0) {
      aoffsetX = ax;
      aoffsetY = ay;
      aoffsetZ = az;
    }
    if (i > 1) {
      aoffsetX = (ax + aoffsetX) / 2;
      aoffsetY = (ay + aoffsetY) / 2;
      aoffsetZ = (az + aoffsetZ) / 2;
    }
  }
  Serial.println("Start...");
}

void loop(){
   // Accelerometer angles (roll and pitch)
  acc.readAccel(&ax, &ay, &az); //read the accelerometer values and store them in variables  x,y,z
  
  rawX = ax - aoffsetX;
  rawY = ay - aoffsetY;
  rawZ = az  - (255 - aoffsetZ);
  
  X = rawX/256.00; // used for angle calculations
  Y = rawY/256.00; // used for angle calculations
  Z = rawZ/256.00; // used for angle calculations
  
  rollX = 180*(atan(Y/sqrt(X*X+Z*Z)))/PI; // calculated angle in degrees
  pitchY = 180*(atan(X/sqrt(Y*Y+Z*Z)))/PI; // calculated angle in degrees

  // Code fragment for Magnetometer heading (yaw)
  MagnetometerRaw raw = compass.ReadRawAxis();
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  float declinationAngle = 0.0457;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  yawZ = heading * 180/M_PI;
  Serial.print(yawZ);
  Serial.print(",");
  Serial.print(pitchY);
  Serial.print(",");
  Serial.println(rollX);
}
