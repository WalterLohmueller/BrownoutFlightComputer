//#include <MadgwickAHRS.h>
//Madgwick filter;

float get_offsets() {
  if (Time / 1000 / offset_counter > 5 && systemstate < 0) {
    offset_counter++;
    velocity_offset = velocity_new;
    altitude_offset = accel_altitude;
    angle_offsetx = anglex;
    angle_offsety= angley;
    angle_offsetz = anglez;
  }
  ///////GYROSCOPE OFFSETS////////////
  if (systemstate == -1 && Time/1000 < 5) {
    gyrox_offset = gyrox;
    gyroy_offset = gyroy;
    gyroz_offset = gyroz;

    // delay(50);
    gyrox_offset1 = gyrox;
    gyroy_offset1 = gyroy;
    gyroz_offset1 = gyroz;
    // delay(50);
    gyrox_offset2 = gyrox;
    gyroy_offset2 = gyroy;
    gyroz_offset2 = gyroz;

    gyrox_offset = (gyrox_offset + gyrox_offset1 + gyrox_offset2) / 3;
    gyroy_offset = (gyroy_offset + gyroy_offset1 + gyroy_offset2) / 3;
    gyroz_offset = (gyroz_offset + gyroz_offset1 + gyroz_offset2) / 3;



    //DETERMINE STARTING ANGLE OF THE ROCKET
    starting_pitch = -atan2(accely / 9.8, accelx / 9.8) / 2 / 3.141592654 * 360;
    starting_roll = atan2(accelz / 9.8, accelx / 9.8) / 2 / 3.141592654 * 360;
  }
}





float Estimate_state() {


/*
  if (Time_step > 0.05) {
    Time_step = 0.03;
  }
*/
  velocity_new = velocity_new + ax * (Time_step / -1000);
  accel_altitude = accel_altitude + (velocity_new - (velocity_offset)) * (Time_step / -1000);

}
float caculate_angles() {




  /*
// Define the initial quaternion representing the sensor's orientation in the global frame
Quaternion initialOrientation = {1.0, 0.0, 0.0, 0.0};  // Set the initial orientation to identity


 // Convert gyro rates to radians per second
  float gyroX = (gyrox - gyrox_offset) / 2;  // Convert from degrees/s to radians/s
  float gyroY = (gyroy - gyroy_offset) / 2;
  float gyroZ = (gyroz - gyroz_offset) / 2;
/*
    // Update filter with gyro and accelerometer data
  filter.updateIMU(gyroX, gyroY, gyroZ, ax, ay, az);

float roll,pitch,yaw;

pitch = getPitch();
yaw = getYaw();
roll = getRoll();



  // Print the angles in degrees
  Serial.print("Roll: ");
  Serial.print(roll );
  Serial.print(" Pitch: ");
  Serial.print(pitch );
  Serial.print(" Yaw: ");
  Serial.println(yaw );
  */

  anglex = ((gyrox - gyrox_offset) * Time_step/1000) + anglex;
  angley = ((gyroy - gyroy_offset) * Time_step/1000) + angley;
  anglez = ((gyroz - gyroz_offset) * Time_step/1000) + anglez;
}