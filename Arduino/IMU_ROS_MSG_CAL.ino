#include "MPU9250.h"
#include "SimpleFOC.h"
MPU9250 my_MPU;

// ------ Set Serial Communication properties here ------- //


float pitch, roll, yaw, x_accel, ang_z, filtered_acc; 

float accel_x_bias = -64.98;
float accel_y_bias = -14.76;
float accel_z_bias = -31.83; // All in g

float gyro_x_bias = -1.5;
float gyro_y_bias = 0.61;
float gyro_z_bias = -0.15;

float mag_x_bias = -253.53;
float mag_y_bias = 423.93;
float mag_z_bias = -865.1; // mg
LowPassFilter filter = LowPassFilter(0.005);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  /* Custom settings for the IMU */
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ; // 5ms sampling rate
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
  /* ------------------------------------------ */

  if(!my_MPU.setup(0x68, setting)) {
    while(1){
      Serial.println("MPU-Connection has failed");
      delay(1000);
    }
  }
  
  my_MPU.setAccBias(accel_x_bias, accel_y_bias, accel_z_bias); 
  my_MPU.setGyroBias(gyro_x_bias, gyro_y_bias, gyro_z_bias);
  my_MPU.setMagBias(mag_x_bias, mag_y_bias, mag_z_bias); 

}

void loop() {
  if (my_MPU.update()) {
    getAllAngles(); 
    
  }

}

void getAllAngles(){
  pitch = my_MPU.getPitch();
  roll = my_MPU.getRoll();
  yaw = my_MPU.getYaw(); 
  x_accel = my_MPU.getLinearAccX();
  ang_z = my_MPU.getGyroZ() * 3.1415/180;
  int header = 300;

  byte headerArray[2] = {
  ((uint8_t*)&header)[0],
  ((uint8_t*)&header)[1]
  };

  byte xAcceleration[4] = {
    ((uint8_t*)&x_accel)[0],
    ((uint8_t*)&x_accel)[1],
    ((uint8_t*)&x_accel)[2],
    ((uint8_t*)&x_accel)[3]
  };

  byte zAngular[4] = {
    ((uint8_t*)&ang_z)[0],
    ((uint8_t*)&ang_z)[1],
    ((uint8_t*)&ang_z)[2],
    ((uint8_t*)&ang_z)[3]
  }; 

  byte yawArray[4] = {
    ((uint8_t*)&yaw)[0],
    ((uint8_t*)&yaw)[1],
    ((uint8_t*)&yaw)[2],
    ((uint8_t*)&yaw)[3]   
  };

  Serial.write(headerArray, 2);
  Serial.write(xAcceleration, 4);
  Serial.write(zAngular, 4); 
  Serial.write(yawArray, 4); 

  // Serial.print("Acceleration: ");
  // Serial.print(" ");
  // Serial.print(x_accel, 3);
  // Serial.print(" ");
  // Serial.print("Angular: ");
  // Serial.print(" ");
  // Serial.print(ang_z, 3);
  // Serial.print(" ");
  // Serial.print("Yaw");
  // Serial.println(yaw, 3);
  delay(10);
}
