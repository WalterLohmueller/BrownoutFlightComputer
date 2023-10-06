/////////////INITALIZE SENSORS//////////////////
void start_IMU(){

  ///////IMU////////////
 if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    errorboi = 1;
  }
}




//////////////////READING SENSORS///////////////////////////
float read_IMU() {
  // DEFAULT IMU 

    IMU.readGyroscope(gyrox, gyroy, gyroz);

    IMU.readAcceleration(accelx, accely, accelz);

accelx = accelx * 9.81;
accely = accely * 9.81;
accelz = accelz * 9.81;



 ///////ACCELEROMETER OFFSETS////////////

  /////////////HIGH G ACCELEROMETER////////////////
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  high_accelx = event.acceleration.x;


  accelx = accelx - 9.81;
  high_accelx = (high_accelx * -1) - 8.81;

  ///////SWITCH BETWEEN HIGH AND LOW GRAVITY ACCELEROMETERS//////////
  if(abs(accelx) < 20) {
  ax = accelx;
ay = accely;
az = accelz;
  }

  else{
  ax = high_accelx;
  }
}




  


///////////SOFTWARE IN THE LOOP//////////////////////////
float SITLread_IMU() {

high_accelx = SITL_accelx[a];
accelx = SITL_accelx[a];
//accely = SITL_accely[a];
//accelz = SITL_accelz[a];


 ///////ACCELEROMETER OFFSETS////////////



  accelx = accelx - 9.45;
  high_accelx = high_accelx - 9.81;

    ///////SWITCH BETWEEN HIGH AND LOW GRAVITY ACCELEROMETERS//////////
  if(abs(accelx) < 20) {
  ax = accelx;
ay = accely;
az = accelz;
  }

  else{
  ax = high_accelx;
  }
  
a++;
}