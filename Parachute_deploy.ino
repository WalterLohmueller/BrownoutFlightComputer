float chute_deploy() {
 // digitalWrite(mos, HIGH);
 apogee_deploy_1 = 1;

if(Time/1000 - apogee_counter > apogee_wait_time){
// digitalWrite(mos, HIGH);
apogee_deploy_2 = 1;
}

if(apogee_deploy_1 ==1 && apogee_deploy_2==1 && systemstate == 1){
advance_state();
}
}






float apogee_check() {
if(time_actual > 2 && (velocity_new -velocity_offset) < 0 && systemstate == 0) {
advance_state();
apogee_counter = Time/1000;
}

if (time_actual > apogee_time && systemstate == 0) {
apogee_counter = Time / 1000;
advance_state();
}
}




float main_check() {
if (time_actual > main_time && systemstate == 2) {
main_time = Time / 1000;
advance_state();
}

if(H_val * 3.22 < main_deploy_alt && systemstate == 2) {
main_time = Time / 1000;
advance_state();
}


}

float main_deploy() {
 // digitalWrite(mos, HIGH);
 main_deploy_1 = 1;

 if(Time / 1000 - main_time > main_wait_time){
// digitalWrite(mos, HIGH);
 main_deploy_2 = 1;
}
}
