/**
  Anupam N Godse        angodse@ncsu.edu
  Brayden W McDonald    bwmcdon2@ncsu.edu
  */

/**
 * This program was created by Anupam Godse and Brayden McDonald for CSC 714
 * It is based on the linetrace code that was provided, but does not use the same algorithm to follow the line
 */

#include "ev3api.h"
#include "app.h"
#include <stdlib.h>
//#include "../../kernel/mutex.h"
#include "../../kernel/kernel_impl.h"

const T_CMTX mutex_sonar, mutex_light_ground, mutex_light_ambiance;
ID mutex_sonar_id, mutex_lg_id, mutex_la_id;

#define abs(x) (x<0?-x:x) 

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * Define the connection ports of the sensors and motors.
 * By default, this application uses the following ports:
 */
const int color_sensor_ambiance = EV3_PORT_1, color_sensor_ground = EV3_PORT_2, ultrasonic_sensor=EV3_PORT_3, touch_sensor=EV3_PORT_4,
      drive_motor = EV3_PORT_D, turn_motor = EV3_PORT_A;//defines the ports used to access the sensors and motors

//global variables required
int ground_value, ambiance_value, previous_turn=LEFT, line_threshold=11, white=15, black=7, obstacle_distance=100, power=60, detection_range=8, reverse_count=0, steer_by=0, reverse_by, turn_by, phase = MOVE_FORWARD, turn_motor_counts=0, ambiance_threshold=11, escape_count, extra_miles, target_miles=800, steer_by_max=23, evasive_direction=LEFT, collisions=0, is_pressed=0;

//bluetooth file ptr
static FILE *bt = NULL;

//for calculating execution times
SYSTIM start_od, end_od, start_gv, end_gv, start_av, end_av, start_um, end_um, start_access, end_access, max_od=0, max_av=0, max_gv=0, max_um=0, max_access=0;

// a cyclic handler to activate a task
void task_activator(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    assert(ercd == E_OK);
}

/*
 * This function accesses the touch sensor.
 * It updates the value of the global is_pressed variable; 1 if the sensor is currently being pressed, 0 if it is not
 * It also increases the value of collisions whenever is_pressed goes from 1 to 0 
 */
//touch sensor count recorder 
void touch_sensor_task() {
    bool_t val = ev3_touch_sensor_is_pressed(touch_sensor);
    if(val && is_pressed==0) {//if the button is pressed, set the is_pressed variable
        is_pressed = 1;
    }
    else if(!val && is_pressed==1) {//when the button is released, increment collisions
        collisions++;
        is_pressed = 0;
    }
}

//required bluetooth non real time task to print collisions recorded by touch sensor
void bluetooth() {
    fprintf(bt, "collisions=%d\n", collisions);    
    //fprintf(bt, "max_od=%lu max_gv=%lu max_av=%lu max_um=%lu max_access=%lu\n", max_od, max_gv, max_av, max_um, max_access);    
}



/*
 * This function accesses the ultrasonic sensor, and returns the distance to the nearest object.
 * It also updates the value of the global obstacle_distance variable, which is used by the motor function
 */
void sonar_sensor(intptr_t unused) {
    //get_utm(&start_od);
    loc_mtx(mutex_sonar_id);
    obstacle_distance = ev3_ultrasonic_sensor_get_distance(ultrasonic_sensor);
    unl_mtx(mutex_sonar_id);
    //get_utm(&end_od);
    //max_od = max_od<(end_od-start_od)?end_od-start_od:max_od;
} 


/*
 * This function accesses the light sensor reading which points towards ground and can be used to detect
 * a line for reference
 * It also updates the value of the global ground_value variable, which is used by the motor function
 */
void light_sensor_ground(intptr_t unused) {
    //get_utm(&start_gv);
    loc_mtx(mutex_lg_id);
    ground_value = ev3_color_sensor_get_reflect(color_sensor_ground);
    unl_mtx(mutex_lg_id);
    //get_utm(&end_gv);
    //max_gv = max_gv<(end_gv-start_gv)?end_gv-start_gv:max_gv;
}

/*
 * This function accesses the light sensor which points upwards to detect the ambiance 
 * It also updates the value of the global ambiance_value variable, which is used by the motor function
 */
void light_sensor_ambiance(intptr_t unused) {
    //get_utm(&start_av);
    loc_mtx(mutex_la_id);
    ambiance_value = ev3_color_sensor_get_ambient(color_sensor_ambiance);
    unl_mtx(mutex_la_id);
    //get_utm(&end_av);
    //max_av = max_av<(end_av-start_av)?end_av-start_av:max_av;
}

/**
 * This is the motor function. It contains most of the logic for the robot's navigation
 */
void update_motor(intptr_t unused){

    //get_utm(&start_um);

    //variables to store values after accessed from global variables
    int temp_obstacle_distance, temp_ground_value, temp_ambiance_value;

    //for recording critical section execution time

    //get_utm(&start_access);
    loc_mtx(mutex_sonar_id);
    temp_obstacle_distance = obstacle_distance;
    unl_mtx(mutex_sonar_id);
    loc_mtx(mutex_lg_id);
    temp_ground_value = ground_value;
    unl_mtx(mutex_lg_id);
    loc_mtx(mutex_la_id);
    temp_ambiance_value = ambiance_value;
    unl_mtx(mutex_la_id);
    //get_utm(&end_access);

    //max_access = max_access<(end_access-start_access)?end_access-start_access:max_access;

    //each time we update the phase according to the condition
    switch(phase) {
        case MOVE_FORWARD://normal execution 
            if(temp_obstacle_distance >=0 && temp_obstacle_distance <= detection_range) { //if obstacle detected then change phase to evasion
                phase = REVERSE_EVADE;
                reverse_by = 800;
                ev3_motor_reset_counts(drive_motor);
            }
            else if(temp_ground_value < line_threshold) { //detect line towards either side and change phase to reverse
                phase = REVERSE;
                ev3_motor_reset_counts(drive_motor);
                reverse_by = 600;
            }
            else { //keep going
                ev3_motor_set_power(drive_motor, power);
            }
            break;
        case REVERSE: //reverses when line detected by rover so that it stays on path
            if(abs(ev3_motor_get_counts(drive_motor)) > reverse_by) { //stop reversing and change phase to turn to align on path
                //phase = STEER;
                reverse_by = 0;
                steer_by = (steer_by_max) * previous_turn;

                //changed code
                phase = TURN;
                turn_by = 600;
                ev3_motor_reset_counts(drive_motor);

            }
            else { //reverse
                ev3_motor_set_power(drive_motor, -power);
            }
            break;
        case STEER: //this is not used 
            turn_motor_counts = ev3_motor_get_counts(turn_motor);
            if(turn_motor_counts == steer_by) { //if done steer then TURN
                phase = TURN;
                turn_by = 300; 
                ev3_motor_reset_counts(drive_motor);
            }
            else {
                ev3_motor_set_power(drive_motor, 0);
            }
            break;
        case TURN: //turn to get aligned on path
            if(abs(ev3_motor_get_counts(drive_motor)) > turn_by) { //stop turning and set to normal execution
                //phase = STEER_BACK;
                //steer_by = 0;
                phase = MOVE_FORWARD;
                steer_by = 0;
            }
            else { //keep turning
                ev3_motor_set_power(drive_motor, power);
                if(temp_ground_value < line_threshold) {//if line detected while turning means we are moving outside the path, make correction
                    previous_turn *= -1;
                    phase = REVERSE;
                    reverse_by = 400;
                }
            }
            break;
        case STEER_BACK: //not used
            turn_motor_counts = ev3_motor_get_counts(turn_motor);
            if(turn_motor_counts == steer_by) {
                phase = MOVE_FORWARD;
            }
            else {
                ev3_motor_set_power(drive_motor, 0);
            }
            break;
        case REVERSE_EVADE: //reverse after seeing obstacle
            if(abs(ev3_motor_get_counts(drive_motor)) > reverse_by) { //stop reversing and invoke steering away from obstacle
                phase = STEER_EVADE;
                reverse_by = 0;
                steer_by = (steer_by_max) * evasive_direction;
            }
            else { //keep reversing
                ev3_motor_set_power(drive_motor, -power);
            }
            break;
        case STEER_EVADE: //after reversing we need to steer away from obstacle
            turn_motor_counts = ev3_motor_get_counts(turn_motor);
            if(turn_motor_counts == steer_by) { //stop steering
                phase = TURN_EVADE;
                //turn_by = 300; 
                ev3_motor_reset_counts(drive_motor);
            }
            else { //stop rear wheels while steering
                ev3_motor_set_power(drive_motor, 0);
            }
            break;
        case TURN_EVADE: //go around obstacle
            if(temp_ground_value < line_threshold) { //if we find line then go beyond line
                phase = GET_AWAY;
                extra_miles = abs(ev3_motor_get_counts(drive_motor));
                //steer_by = -steer_by;
            }
            else { //keep turning
                ev3_motor_set_power(drive_motor, power);
            }
            break;
        case GET_AWAY: //outside the path to go around obstacle
            //if outside the line then now turn to go around
            if(temp_ground_value >= line_threshold && abs(ev3_motor_get_counts(drive_motor))  >= target_miles) {
                phase = TURN_BACK_EVADE;
                escape_count = abs(ev3_motor_get_counts(drive_motor));
                steer_by = -steer_by;
            }
            else { //keep turning
                ev3_motor_set_power(drive_motor, power);
            }
            break;
        case TURN_BACK_EVADE: //once outside try to find line again while going around obstacle
            if(temp_ground_value < line_threshold) { //line found then get back on path
                phase = GET_IN;
                //steer_by = -steer_by;
            }
            //if we get in dark or see an obstacke then we should reverse and get around obstacle in other direction
            else if(temp_ambiance_value < ambiance_threshold || temp_obstacle_distance < detection_range) {
                phase = REVERSE_GET_LINE;     
                steer_by = 0;
            }
            else {//keep turning
                ev3_motor_set_power(drive_motor, power);
            }
            break;
        case GET_IN: //get inside the path -> successfully got around obstacle
            if(temp_ground_value >= line_threshold) { //see for inside of path and set phase to realign
                phase = REALIGN;
                ev3_motor_reset_counts(drive_motor);
                steer_by = -steer_by;
            }
            else { //keep going 
                ev3_motor_set_power(drive_motor, power);
            }
            break;
        case REALIGN: //relign to get back on path
            if(abs(ev3_motor_get_counts(drive_motor)) >= escape_count) { //if back on path then set to normal, evasion success
                phase = MOVE_FORWARD;
                steer_by = 0;
            }
            else { //keep going
                ev3_motor_set_power(drive_motor, power);
            }
            break;
        case REVERSE_GET_LINE: //if we see obstacle or darkness while tackling obstacle then we should tackle it in another direction
            if(temp_ground_value < line_threshold) { //line found then set phase to return back on track
                phase = RETURN;
                ev3_motor_reset_counts(drive_motor);
                steer_by = steer_by_max*evasive_direction;
            }
            else {//go back
                ev3_motor_set_power(drive_motor, -power);
            }
            break;
        case RETURN: //get back on track
            if(abs(ev3_motor_get_counts(drive_motor)) > escape_count) { //once back on track then make wheels straight
                phase = RETURN_STEER_BACK;
                steer_by = 0;
            }
            else {
                ev3_motor_set_power(drive_motor, -power);
            }
            break;
        case RETURN_STEER_BACK: //straighten the wheels
            turn_motor_counts = ev3_motor_get_counts(turn_motor);
            if(turn_motor_counts == steer_by) { //once wheels are straightned then change evasion direction and trigger normal execution
                evasive_direction = -evasive_direction;
                phase = MOVE_FORWARD;
            }
            else {
                ev3_motor_set_power(drive_motor, 0);
            }
            break;
    } 

    turn_motor_counts = ev3_motor_get_counts(turn_motor);
    ev3_motor_set_power(turn_motor, steer_by-turn_motor_counts);

    //get_utm(&end_um);
    //max_um = max_um<(end_um-start_um)?end_um-start_um:max_um;

}
static void button_clicked_handler(intptr_t button) {
    switch(button) {
        case BACK_BUTTON:
            syslog(LOG_NOTICE, "Back button clicked.");
            break;
        case LEFT_BUTTON:
            syslog(LOG_NOTICE, "Left button clicked.");
    }
}

void main_task(intptr_t unused) {
    // Register button handlers
    ev3_button_set_on_clicked(BACK_BUTTON, button_clicked_handler, BACK_BUTTON);

    // Configure motors
    ev3_motor_config(drive_motor, LARGE_MOTOR);
    ev3_motor_config(turn_motor, LARGE_MOTOR);

    // Configure sensors
    ev3_sensor_config(color_sensor_ground, COLOR_SENSOR);
    ev3_sensor_config(color_sensor_ambiance, COLOR_SENSOR);
    ev3_sensor_config(ultrasonic_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);


    //reset motor counts
    ev3_motor_reset_counts(drive_motor);
    ev3_motor_reset_counts(turn_motor);


    // Open Bluetooth file
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);
    setvbuf(bt, NULL, _IONBF, 0);

    //sleep while bluetooth is not connected
    while (!ev3_bluetooth_is_connected()) tslp_tsk(100);

    /**
     * Play tones to indicate program is running
     */
    ev3_speaker_play_tone(NOTE_C4, 100);
    tslp_tsk(5000);
    ev3_color_sensor_get_ambient(color_sensor_ambiance);
    ev3_color_sensor_get_reflect(color_sensor_ground);
    ev3_ultrasonic_sensor_get_distance(ultrasonic_sensor);

    //mutex_sonar.ceilpri = PRIORITY_SONAR_SENSOR_TASK;
    //mutex_light_ground.ceilpri = PRIORITY_LIGHT_GROUND_TASK;
    //mutex_light_ambiance.ceilpri = PRIORITY_LIGHT_AMBIANCE_TASK;

    acre_mtx(&mutex_sonar);
    acre_mtx(&mutex_light_ground);
    acre_mtx(&mutex_light_ambiance);

    tslp_tsk(1000);




    //start all cycles
    ev3_sta_cyc(CYC_LIGHT_SENSOR_GROUND_TASK);
    ev3_sta_cyc(CYC_LIGHT_SENSOR_AMBIANCE_TASK);
    ev3_sta_cyc(CYC_SONAR_SENSOR_TASK);
    ev3_sta_cyc(CYC_MOTOR_TASK);

    while(1) {
        touch_sensor_task();
        bluetooth();
    }
    //never comes here
    ev3_stp_cyc(CYC_LIGHT_SENSOR_GROUND_TASK);
    ev3_stp_cyc(CYC_LIGHT_SENSOR_AMBIANCE_TASK);
    ev3_stp_cyc(CYC_SONAR_SENSOR_TASK);
    ev3_stp_cyc(CYC_MOTOR_TASK);
    return;

}
