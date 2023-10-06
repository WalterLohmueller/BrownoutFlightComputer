
float get_baro() {
  H_val_past = H_val;

  P_Sens.readDigitalValue();
 T_val = P_Sens.getTemperature();
    P_val = P_Sens.getPressure();
    H_val = P_Sens.getAltitude();

  //OFFSETS/////

if(millis()/1000 < 15) {
H_val_offset = H_val;
}


    H_val = H_val - H_val_offset;


/*
/////////PRESSURE TRANSIENT FILTERING////////////////////
if(abs((H_val - H_val_past) / dt) > 1000) {
H_val = H_val_past;
}
*/

}


float SITL_baro() {

H_val = SITL_baro_data[a];
a++;
//delay(100);

/*
/////////PRESSURE TRANSIENT FILTERING////////////////////
if(abs((H_val - H_val_past)) > 600) {
H_val = H_val_past;
}
H_val_past = H_val;
*/

}