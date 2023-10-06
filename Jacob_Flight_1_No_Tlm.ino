/* Written by Walter Lohmueller
#lmao at this point there is a lot of "clutter" building up in the code. 
#things also aren't commented/organized very well
*/



////////////////CUSTOM VARIABLES///////////////////////////////
float main_deploy_alt = 500;   //when the main parachute charge fires
float apogee_time = 10;        //the flight computer automatically fires the apogee charge after this time
float main_time = 30;          //the flight computer automatically fires the main charge after this time
float apogee_wait_time = 1.5;  //how long the computer waits before firing the backup
//apogee charge
float main_wait_time = 1.5;  //how long the computer waits before firing the backup
//main charge
float apogee_counter = 0;
float pyro_safe_alt = 100;  // pyros will not fire when the rocket is under this altitude





/////////INTEGERS TO BE TRANSMITTED TO THE GROUNDSTATION////////////
int apogee_deploy_1 = 0;
int apogee_deploy_2 = 0;


int main_deploy_1 = 0;
int main_deploy_2 = 0;

int errorboi = 0;  //set this variable high if errors are detected
int systemstate = -1;




///////////PYRO CHANNEL PINS///////////////
int apogee_pyro_1 = 0;
int apogee_pyro_2 = 0;
int main_pyro_1 = 0;
int main_pyro_2 = 0;









//////SENSOR LIBRARIES///////////

#include <Wire.h>
#include <Adafruit_Sensor.h>



////////////////////HIGH G ACCELEROMETER///////////////////////////
#include <Adafruit_ADXL375.h>

#define ADXL375_SCK 13
#define ADXL375_MISO 12
#define ADXL375_MOSI 11
#define ADXL375_CS 10

/* Assign a unique ID to this sensor at the same time */
/* Uncomment following line for default Wire bus      */
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);









///////////////////BAROMETER////////////////////////////
#include <MS5607.h>

MS5607 P_Sens;
float P_val, T_val, H_val;
float H_val_past;
float H_val_offset;





////////////////////IMU///////////////////////
#include <Arduino_LSM6DS3.h>

#include <math.h>






////////STATE ESTIMATE VARIABLES/////////






//IMU//
float gyroz = 0;
float gyrox = 0;
float gyroy = 0;

float gyrox_offset;
float gyroy_offset;
float gyroz_offset;

float gyrox_offset1;
float gyroy_offset1;
float gyroz_offset1;

float gyrox_offset2;
float gyroy_offset2;
float gyroz_offset2;


float angle_offsetx = 0;
float angle_offsety = 0;
float angle_offsetz = 0;

float anglex;
float angley;
float anglez;

float accel_pitch;
float accel_roll;


float velocity_offset = 0;
float altitude_offset = 0;
float offset_counter = 1;
float velocity_new = 0;
float accel_altitude = 0;

//HIGH G ACCELEROMETER
float high_accelx = 0;  //only the vertical acceleration is nessescary
float high_accelxoffset = 2.66;




/////TIMEKEEPER//////////
float Time;           // set up time at start of loop
float time_actual;    //set up the difference in time, also the actual time
float time_offset;    // set up the time offset
float Time_step = 0;  // time it takes to complete each loop
float time_old = 0;
float time_new = 0;




///////////////////GUIDENCE NAVIGATION CONTROL///////////////////////

float starting_pitch = 0;
float starting_roll = 0;



///////////QUATERNIONS///////////////////
// thanks to Mike (Alldigital) for providing this quaternion cod
float Ax, Ay, Az;
float Gx, Gy, Gz;

float AxOffset = 0;
float AyOffset = 0;
float AzOffset = 0;

float GxOffset = 0;
float GyOffset = 0;
float GzOffset = 0;

unsigned long td;
unsigned long lastMicros;
float dt = 0.0;


// QUATERNION STUFF
#include "math.h"
struct quaternion {
  float r;       /* real bit */
  float x, y, z; /* imaginary bits */
};
quaternion masterQuaternion;
quaternion tempQuaternion;
float angX, angY, angZ;


float ax, ay, az;
float accelx, accely, accelz;





int a = 0;






//////////SOFTWARE IN THE LOOP/////////////////
float SITL_accelx[] = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.11, 0.16, 52.33, 34.4, 79.67, 59.34, 69.75, 74.93, 91.58, 80.56, 87.25, 85.6, 70.33, 78.55, 75.5, 48.33, 88.23, 80.1, 63.35, 65.4, 58.97, 53.11, 51.13, 39.38, 47.78, 37.85, 41.73, 38.45, 39.44, 35.5, 52, 36, 31.65, 32.46, 18.82, 18.15, 25.98, 28.56, 18.38, 11.92, 7.22, 0.79, 3.84, 8.85, 0.19, 2.79, -10.8, -8.37, -15.43, -16.52, -14.98, -21.39, -22.47, -19.8, -12.91, -14.54, -22.79, -15.77, -21.12, -19.55, -19.19, -20.48, -20, -16.59, -20.03, -13.98, -19.27, -16.1, -19.3, -20.38, -18.4, -17.45, -17.92, -13.75, -17.28, -17.07, -17.87, -15.9, -14.53, -18.04, -16.4, -16.83, -13.62, -15.78, -15.56, -12.45, -14.02, -11.74, -13.73, -14.82, -14.62, -14.26, -18.81, -12.71, -14.23, -13.41, -13.09, -14.24, -10.98, -13.08, -12.8, -13.3, -13.02, -14.92, -10.92, -9.8, -11.89, -11.39, -11.65, -13.45, -12.25, -11.42, -11.38, -13.14, -12.52, -12.33, -12.69, -13.73, -12.23, -11, -11.56, -12.47, -11.59, -12.52, -11.71, -11.97, -10.67, -10.95, -11.1, -11.97, -13.02, -11.3, -11.21, -11.83, -9.74, -10.55, -11.23, -11.07, -10.89, -9.89, -11.67, -9.52, -10.41, -10.6, -10.84, -10.96, -9.59, -11.44, -11.74, -10.7, -11.35, -10.62, -10.21, -9.82, -9.7, -10.38, -10.43, -9.8, -10.19, -9.99, -10.17, -10.09, -9.85, -10.09, -9.9, -9.96, -9.67, -9.75, -9.77, -10.09, -9.77, -10.18, -9.81, -9.83, -9.89, -9.85, -9.75, -9.49, -9.85, -9.55, -9.54, -9.7, -9.56, -9.89, -8.27, -9.95, -9.59, 32.86, -9.44 };




float SITL_baro_data[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 00, 0, 0, 0, 16.2446198225764, 13.0700367517103, 27.8903670908913, 27.8903670908913, 45.8906667579809, 64.561321886819, 92.2100870347501, 92.2100870347501, 74.2429105772029, 74.2429105772029, 111.467557726949, 227.418234026439, 634.230912249908, 634.230912249908, 464.564767999068, 464.564767999068, 422.44520343757, 349.812690654027, 349.812690654027, 588.218750419461, -687.889923158227, -687.889923158227, 230.136109721811, 262.639400749559, 262.639400749559, 341.421454374665, 405.784935682123, 405.784935682123, 406.825631636929, 456.28572819125, 456.28572819125, 490.214665869557, 487.030064468064, 487.030064468064, 496.23855329878, 587.540916297924, 587.540916297924, 517.669371219891, 520.737838480047, 520.737838480047, 583.243394669529, 221.081057059127, 221.081057059127, -119.746313167218, 3904.87761339862, 3904.87761339862, 959.83721281591, 705.064214225145, 705.064214225145, 799.915500261499, 879.466113914762, 879.466113914762, 773.406690274779, 888.518124429359, 888.518124429359, 775.456488118904, 803.19601834021, 803.19601834021, 872.112104456458, 816.066631430773, 816.066631430773, 1000.68288421319, 942.721939803497, 942.721939803497, 1095.0739738541, 792.633024722328, 792.633024722328, 752.39169086958, 570.338010326934, 570.338010326934, 543.029249723099, 251.308222480711, 196.720631530896, 196.720631530896, 327.068034458756, 327.068034458756, 496.142511210807, 310.380298627281, 361.678666573653, 361.678666573653, 369.675010600107, 377.317232224651, 377.317232224651, 336.63672529066, 335.740977438174, 335.740977438174, 329.39197268411, 366.404116689829, 366.404116689829, 333.847747595504, 352.002202445067, 352.002202445067, 294.697315999732, 374.320646579622, 374.320646579622, 335.349364001414, 337.207400997001, 337.207400997001, 306.411982745501, 381.935539767111, 381.935539767111, 304.894844016279, 342.621607038371, 342.621607038371, 375.700184962569, 415.052393680321, 415.052393680321, 312.068519591518, 353.679689776439, 296.22725847417, 296.22725847417, 334.088994171581, 334.088994171581, 368.593367364716, 268.825079245999, 268.825079245999, 304.817226113918, 298.973812314532, 323.304777606627, 323.304777606627, 243.880638349012, 243.880638349012, 286.786699551351, 349.19574290494, 349.19574290494, 378.272279251785, 311.125952061681, 311.125952061681, 283.319161155027, 267.943952755078, 267.943952755078, 296.681310430334, 317.437156125737, 317.437156125737, 301.13349122042, 285.223955932266, 285.223955932266, 271.861796872074, 306.939826954143, 306.939826954143, 277.480690994406, 320.245199175632, 320.245199175632, 272.135235074237, 280.31149169007, 280.31149169007, 221.8186273322, 270.518956034053, 270.518956034053, 379.638912615632, 299.95788055662, 299.95788055662, 254.743043016654, 249.631597158461, 249.631597158461, 273.907541158483, 243.672208680042, 243.672208680042, 253.352453948169, 322.668495404505, 322.668495404505, 259.731005625102, 302.574731966698, 302.574731966698, 214.954163623, 283.631670492238, 283.631670492238, 242.742531851675, 245.201173773951, 245.201173773951, 242.915644780642, 244.780224245209, 244.780224245209, 228.053350233424, 181.767940687412, 181.767940687412, 275.622315110918, 277.079312288259, 277.079312288259, 239.225248108786, 295.35932486482, 295.35932486482, 250.86202365203, 194.458932569134, 194.458932569134, 275.559743361546, 188.148905774818, 188.148905774818, 261.22121890541, 212.582898100511, 212.582898100511, 236.219749036683, 216.21125823614, 216.21125823614, 231.425715058775, 191.871986068718, 191.871986068718, 253.697459624314, 253.697459624314, 224.322805964937, 229.418839006146, 193.147600955118, 193.147600955118, 228.286649636506, 228.286649636506, 171.577500894132, 218.65454884242, 218.65454884242, 199.337332618113, 193.601905512354, 193.601905512354, 270.35077202116, 212.973743533119, 212.973743533119, 203.82725131463, 224.141004454528, 224.141004454528, 185.31835188469, 189.636980143958, 189.636980143958, 150.99328233013, 193.74641743059, 193.74641743059, 205.992183576679, 140.558277963344, 140.558277963344, 192.222193840136, 198.384511003259, 198.384511003259, 194.895576806037, 180.657269822674, 180.657269822674, 180.951922625112, 184.706414528597, 184.706414528597, 180.446116023954, 186.94345568638, 186.94345568638, 175.104824099887, 151.196413570403, 151.196413570403, 161.856812632936, 160.665307021427, 160.665307021427, 158.346834877004, 139.07659899962, 139.07659899962, 201.240052810642, 146.474433933206, 146.474433933206, 197.624207310449, 135.871761452364, 135.871761452364, 173.25846934119, 144.176378029029, 144.176378029029, 169.259992034695, 169.259992034695, 110.889675975397, 142.094502799183, 142.094502799183, 197.262245708428, 164.814470610282, 164.814470610282, 133.895252426905, 112.509044513373, 112.509044513373, 166.247160187044, 172.19594147884, 172.19594147884, 144.486046773342, 121.686243922013, 121.686243922013, 168.126654142396, 145.214598582308, 145.214598582308, 114.26200818134, 136.404807586471, 136.404807586471, 102.017677890306, 142.378299347281, 142.378299347281, 117.000727818088, 108.892035505477, 108.892035505477, 120.584349489068, 115.814744951662, 115.814744951662, 151.376801540088, 105.927789567015, 105.927789567015, 142.144015988279, 147.977410487816, 147.977410487816, 60.7521442298988, 120.277117534914, 120.277117534914, 119.23802025875, 137.396074479952, 137.396074479952, 128.357661188316, 107.283659540389, 107.283659540389, 75.563714908426, 103.138918399999, 103.138918399999, 84.5577928966498, 95.5185012119455 };
int i = 1;
//int b = 1;





/////////DATALOGGING///////////////////
#include <SD.h>
File dataFile;  //set data fi\le for micro sd card
float chipSelect = 10;
float SD_close_counter = 0;
float SD_pad_close = 0;


void setup() {
  Serial.begin(115200);  // start serial monitor
  SD.begin(chipSelect);  // start the SD card
  Wire.setClock(3400000);
  Wire.begin(9);


  ///////////PYRO CHANNELS////////////////

  pinMode(main_pyro_1, OUTPUT);
  pinMode(main_pyro_2, OUTPUT);
  pinMode(apogee_pyro_1, OUTPUT);
  pinMode(apogee_pyro_1, OUTPUT);


  pinMode(2, OUTPUT);

  ////////////////////START SENSORS//////////////////////////////////

  start_IMU();
  //start_accel();
  //gps_start();
  // baro_start();
  read_IMU();

  //Quaternion stuff
  // set up the master quaternion using the accelerometer ground values before launching
  //gyroPrelaunch();

  lastMicros = micros();


  delay(500);

  Serial.println(" Sensor startup complete, starting main loop ");
  dataFile.println(" Sensor startup complete, starting main loop ");







  ////////////////HIGH G ACCELEROMETER INITILIZATION////////////////
  /* Initialise the sensor */
  if (!accel.begin()) {
    errorboi = 1;
  }

  // Range is fixed at +-200g

  /* Display some basic information on this sensor */
  accel.printSensorDetails();





  ///////BAROMETER INITILIZATION/////////////
  if (!P_Sens.begin()) {
    Serial.println("Error in Communicating with sensor, check your connections!");
    errorboi = 1;
  } else {
    Serial.println("MS5607 initialization successful!");
  }

  delay(5000);
}

void loop() {



  ///////////////////////////STATE MACHINE////////////////////////////////////
  if (systemstate == -1) {
    //  Serial.print(" systemstate: Pad Idle ");
    dataFile.print(" systemstate: Ready for launch ");
   read_IMU();  //reads both the IMU and high g accelerometer
   //  SITLread_IMU();
    Estimate_state();
    get_offsets();
    //get_quat();
    DATALOG();
    get_baro();
    //SITL_baro();
    caculate_angles();
  }

  if (systemstate == 0) {
    //Serial.print(" systemstate: Powered Flight ");
    dataFile.print(" systemstate: Powered Flight ");
    Estimate_state();
   // SITLread_IMU();
    //    get_quat();
    read_IMU();
    DATALOG();
    caculate_angles();
    //cal_accel();
    apogee_check();
    get_baro();
    //SITL_baro();
  }

  if (systemstate == 1) {
    //  Serial.print(" systemstate: apogee ");
    dataFile.print(" systemstate: apogee ");
    Estimate_state();
    chute_deploy();
    // get_quat();
    main_check();
    DATALOG();
    read_IMU();
   // SITLread_IMU();
    caculate_angles();
    get_baro();
    //SITL_baro();
    //cal_accel();
  }

  if (systemstate == 2) {
    //  Serial.print(" systemstate: drouge descent ");
    dataFile.print(" systemstate: drouge descent ");
    Estimate_state();
    DATALOG();
    //    get_quat();
    read_IMU();
  //  SITLread_IMU();
    get_baro();
    // SITL_baro();
    chute_deploy();
    main_check();
    caculate_angles();
  }


  if (systemstate == 3) {
    //   Serial.print(" systemstate: main descent ");
    dataFile.print(" systemstate: main descent ");
    Estimate_state();
    DATALOG();
    read_IMU();
    //SITLread_IMU();
    main_deploy();
    get_baro();
    //  get_quat();
    //SITL_baro();
    caculate_angles();
  }






  ///////////////////////STATE ADVANCE/EXIT CONDITIONS/////////////////////////////////
  //delay(50);



  // check the accelerometer to see if we have launched
  if (ax > 12 && systemstate == -1) {
    advance_state();
  }


  //if(systemstate == 0){
  //launch_check();
  // }
  /////////////////////IF STATEMENTS//////////////////////////////////////////


  //Print out what data values are
  if (Time / 1000 < 0.3) {
    pad_print();
  }

  if (errorboi == 1) {
    digitalWrite(2, HIGH);
  }
  //////////////////////////TIME KEEPER//////////////////////////////////////////

  if (systemstate < 0) {
    //delay(100);
    time_offset = millis();
  }

  time_old = Time;

  //  delay(12);

  Time_step = time_old - time_new;
  Time = millis();

  time_new = Time;

  time_actual = Time / 1000 - time_offset / 1000;
  Time_step = time_old - time_new;
}


float advance_state() {
  systemstate = systemstate + 1;
}

