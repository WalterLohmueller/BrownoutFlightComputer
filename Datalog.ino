float pad_print() {


  dataFile = SD.open("dat.txt", FILE_WRITE);  //open the sd card file

  /////////////////PRINT DATA////////////////////////////////////
  /*
dataFile.print(" lat ");
dataFile.print(" lon ");
dataFile.print(" gpsvel ");
dataFile.print("  gpsalt ");
dataFile.print(" fix ");
dataFile.print(" timediff ");
dataFile.print(" state ");
dataFile.print(" error ");
dataFile.print(" veloff ");
dataFile.print(" altoff ");
dataFile.print(" vel ");
dataFile.print(" alt ");
dataFile.print(" timeL ");
dataFile.print(" timeS ");
dataFile.print(" Lax ");
dataFile.print(" Lay ");
dataFile.print(" Laz ");
dataFile.print(" ax ");
dataFile.print(" ay ");
dataFile.print(" az ");
dataFile.print(" Hax ");
dataFile.print(" HGoffset ");
dataFile.print(" gx ");
dataFile.print(" gy ");
dataFile.print(" gz ");
dataFile.print(" OriX ");
dataFile.print(" OriY ");
dataFile.print(" OriZ ");
dataFile.print(" temp ");
dataFile.print(" AP1 ");
dataFile.print(" AP2 ");
dataFile.print(" MP1 ");
dataFile.print(" MP2 ");
dataFile.print(" LP ");
  dataFile.close();
  */
}

float DATALOG() {

  if (Time / 1000 / SD_pad_close > 0.07) {
 SD_pad_close++;

   dataFile = SD.open("dat.txt", FILE_WRITE);  //open the sd card file
 // approximately every 2 seconds or so, print out the current stats


     Serial.print("  ");
    Serial.print(H_val * 3.22);
     Serial.print("    ");
    Serial.print(Time_step);
    Serial.print("    ");
    Serial.print(systemstate);
    Serial.print("   ");
    Serial.print(velocity_offset * 2.22);
    Serial.print("   ");
    Serial.print(altitude_offset * 3.28);
    Serial.print("    ");
    Serial.print(velocity_new * 2.23 - velocity_offset * 2.22);
    Serial.print("   ");
    Serial.print(accel_altitude * 3.28 - altitude_offset * 3.28);
    Serial.print("    ");
   Serial.print(Time / 1000);
    Serial.print("    ");
    Serial.print(time_actual);
    
    Serial.print("  ");
    Serial.print(accelx);

    

    Serial.print("    ");
    Serial.print(ax);


    Serial.print("    ");
  
    Serial.print(high_accelx);
    Serial.print("  ");
  
    Serial.print(gyrox - gyrox_offset);
    Serial.print("  ");
    Serial.print(gyroy - gyroy_offset);
    Serial.print("  ");
    Serial.print(gyroz - gyroz_offset);
    Serial.print("  ");
    Serial.print(anglex - angle_offsetx);
    Serial.print("  ");
    Serial.print(angley - angle_offsety);
    Serial.print(" ");
    Serial.print(anglez - angle_offsetz);
    Serial.print("  ");
    Serial.print(apogee_deploy_1);
    Serial.print("  ");
    Serial.print(apogee_deploy_2);
    Serial.print("  ");
    Serial.print(main_deploy_1);
    Serial.print("  ");
    Serial.print(main_deploy_2);
    Serial.println("  ");


    dataFile.print("    ");
    dataFile.print(H_val * 3.22);
    dataFile.print("    ");
    dataFile.print(Time_step);
    dataFile.print("    ");
    dataFile.print(systemstate);
    dataFile.print("   ");
    dataFile.print(velocity_offset * 2.22);
    dataFile.print("   ");
    dataFile.print(altitude_offset * 3.28);
    dataFile.print("    ");
    dataFile.print(velocity_new * 2.23 - velocity_offset * 2.22);
    dataFile.print("   ");
    dataFile.print(accel_altitude * 3.28 - altitude_offset * 3.28);
    dataFile.print("    ");
    dataFile.print(Time / 1000);
    dataFile.print("    ");
    dataFile.print(time_actual);
    dataFile.print("  ");
    dataFile.print(accelx);
    dataFile.print("  ");
    dataFile.print(ax);
    dataFile.print("    ");
    dataFile.print(high_accelx);
    dataFile.print("  ");
    dataFile.print(gyrox - gyrox_offset);
    dataFile.print("  ");
    dataFile.print(gyroy - gyroy_offset);
    dataFile.print("  ");
    dataFile.print(gyroz - gyroz_offset);
    dataFile.print("  ");
    dataFile.print(anglex - angle_offsetx);
    dataFile.print("  ");
    dataFile.print(angley - angle_offsety);
    dataFile.print(" ");
    dataFile.print(anglez- angle_offsetz);
    dataFile.print("  ");
    dataFile.print(apogee_deploy_1);
    dataFile.print("  ");
    dataFile.print(apogee_deploy_2);
    dataFile.print("  ");
    dataFile.print(main_deploy_1);
    dataFile.print("  ");
    dataFile.println(main_deploy_2);
    dataFile.print("  ");

    dataFile.close();

    
    
}

}