/*
float get_quat() {

      td = micros() - lastMicros;
      lastMicros = micros();
      dt = (float) td / (float) 1000000.0;  // this is the time interval in microseconds

      //now convert the gyro readings into degrees per second, based on the lsb per degree from the datasheet
      Gx = (gyrox - gyrox_offset);  // 4000=140, 2000=70, 1000=35, 500=17.5, 250=8.75, 125 = 4.375 (datasheet pg 25)
      Gy = (gyroy - gyroy_offset);
      Gz = (gyroz - gyroz_offset);

    
      // now factor for fraction of a second
      Gx = Gx * dt;
      Gy = Gy * dt;
      Gz = Gz * dt;

      // done getting what we need from the Gyro. Got Gx, Gy, Gz represented in degrees moved since last sampled

      // now convert hardware orientation to our quaternion orientation. Quaternion functions expects x = roll, y = pitch, z = yaw
      float qX, qY, qZ;
      qX = Gy;
      qY = Gz;
      qZ = Gx;

      gyroRotate(qX, qY, qZ); // does the four quaternion functions and updates angX, angY, angZ
    
    float globalx = 0;
    float globaly = 0;
    float globalz= 0;
    globalx = angX + angY * sin(globalz) * tan(globaly) + angZ * cos(globalz) * tan(globaly);
  globaly = angY * cos(globalz) - angZ * sin(globalz);
  globalz = angY * sin(globalz) / cos(globaly) + angZ * cos(globalz) / cos(globaly);
  
   //output to the monitor every 200ms so we can see it working
   Serial.print("Roll = ");Serial.print(globalx,1);
   Serial.print("   Pitch = ");Serial.print(globaly,1);
   Serial.print("   Yaw = ");Serial.println(globalz,1);
}




void gyroPrelaunch() {
 
  // set the baseline quaternion tilt values using accel data before launch 
  masterQuaternion = { .r = 1, .x = 0, .y = 0, .z = 0 };


  Ax = ax + AxOffset;
  Ay = ay + AyOffset;
  Az = az + AzOffset;
                  
  double a1 = 0.00, a2 = 0.00;       //Roll & Pitch are the angles which rotate by the axis X and y   
  double x_Buff = float(Az);
  double y_Buff = float(Ax);
  double z_Buff = float(Ay);
  //set angles for pitch and yaw
  a1 = atan2(y_Buff , z_Buff) * 57.3; //pitch
  a2 = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3; //yaw
    
  //now integrate the angles into our freshly minted master quaternion (ignoring roll)
  gyroRotate(0,a1,a2);

}


void gyroRotate(float gRoll, float gPitch, float gYaw) {

  // R, P, Y inputs should be degrees factored for time interval

  //first convert the gyro tiny rotation into a half euler and store it in a temporary quaternion
  quaternionInitHalfEuler(gRoll, gPitch, gYaw);

  //now combine it with the masterQuaternion to get an integrated rotation
  quaternionMultiply();

  //now normalize the masterQuaternion in case it went out of bounds
  quaternionNormalize();

  //now we need to translate the masterQuaternion back to angles that humans can read
  quaternionAxisAngles();   
}


void quaternionAxisAngles() {
 
  quaternion p = masterQuaternion;

  // Compute the X (roll) angle in radians
  if((1-2*(p.x*p.x+p.y*p.y)) != 0) {
     angX = atan(2*(p.r*p.x+p.y*p.z)/(1-2*(p.x*p.x+p.y*p.y)));
   } else {
    if((p.r*p.x+p.y*p.z) > 0) {
      angX = M_PI / 2.0f;
    } else {
      if((p.r*p.x+p.y*p.z) < 0) {
        angX = -1.0f * M_PI / 2.0f;
      } else {
        angX = 9999999;
        // SIGNAL ERROR DIVIDE BY ZERO!!!
      }
    }
  }
  // Convert x (roll) from radian to degrees
  angX = angX * 180.0f / M_PI;


  //Compute the Y (pitch) angle in radians
  if((2*(p.x*p.y-p.z*p.x)) <= -1) {
    angY = -1.0f * M_PI / 2.0f;
  } else {
    if((2*(p.r*p.y-p.z*p.x)) >= 1) {
      angY = M_PI / 2.0f;
    } else {
      angY = asin(2*(p.r*p.y-p.z*p.x));
    }
  }
  // Convert y (pitch) from radian to degrees
  angY = angY * 180.0f / M_PI; 


  // Compute the Z (Yaw) angle in radians
  if((1-2*(p.x*p.x+p.y*p.y)) != 0) {
     angZ = atan(2*(p.r*p.z+p.x*p.y)/(1-2*(p.y*p.y+p.z*p.z)));
   } else {
    if((p.r*p.z+p.x*p.y) > 0) {
      angZ = M_PI / 2.0f;
    } else {
      if((p.r*p.z+p.x*p.y) < 0) {
        angZ = -1.0f * M_PI / 2.0f;
      } else {
        angZ = 9999999;
        // SIGNAL ERROR DIVIDE BY ZERO!!!
      }
    }
  }
  // Convert z (Yaw) from radian to degrees
  angZ = angZ * 180.0f / M_PI;
 
}

void quaternionNormalize() {

  quaternion p = masterQuaternion;
  float testP = 0.0;

  testP = (p.r * p.r) + (p.x * p.x) + (p.y * p.y) + (p.z * p.z);
  if(testP > 1.0) {
     p.r = p.r * (float)(1.0f / sqrtf(testP));
     p.x = p.x * (float)(1.0f / sqrtf(testP));
     p.y = p.y * (float)(1.0f / sqrtf(testP));
     p.z = p.z * (float)(1.0f / sqrtf(testP));
  }
  masterQuaternion = p; 
}

void quaternionMultiply() {
 
  // combine t with the masterQuaternion to get an integrated rotation
  quaternion p = masterQuaternion;
  quaternion t = tempQuaternion;

  masterQuaternion.r = (p.r * t.r) + (-p.x * t.x) + (-p.y * t.y) + (-p.z * t.z);
  masterQuaternion.x = (p.r * t.x) + (p.x * t.r) + (p.y * t.z) + (-p.z * t.y);
  masterQuaternion.y = (p.r * t.y) + (-p.x * t.z) + (p.y * t.r) + (p.z * t.x);
  masterQuaternion.z = (p.r * t.z) + (p.x * t.y) + (-p.y * t.x) + (p.z * t.r);

}


void quaternionInitHalfEuler(float gRoll, float gPitch, float gYaw) {

  // store the tiny rotation from the gyro in a new temporary quaternion
  // roll, pitch, yaw input is in degrees

  float s_x, c_x;
  float s_y, c_y;
  float s_z, c_z;

  float x = (gRoll / 2.0f) * M_PI/180.0f;  // half the degree and convert it to radians
  float y = (gPitch / 2.0f) * M_PI/180.0f;
  float z = (gYaw / 2.0f) * M_PI/180.0f;

  s_x = sin(x); c_x = cos(x);
  s_y = sin(y); c_y = cos(y);
  s_z = sin(z); c_z = cos(z);

  // This is our quaternion that represents the rotation difference
  tempQuaternion.r = c_x * c_y * c_z + s_x * s_y * s_z;
  tempQuaternion.x = s_x * c_y * c_z - c_x * s_y * s_z;
  tempQuaternion.y = c_x * s_y * c_z + s_x * c_y * s_z;
  tempQuaternion.z = c_x * c_y * s_z - s_x * s_y * c_z;

}


*/


