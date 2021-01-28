//----------------------------------------------------------------------
// https://github.com/clearwater/SwitecX25
//
// Using the SwitchX25 library.
// It zero's the motor, sets the position to mid-range
// and waits for serial input to indicate new motor positions.
//
// Open the serial monitor and try entering values
// between 0 and 944.
//
// Note that the maximum speed of the motor will be determined
// by how frequently you call update().  If you put a big slow
// serial.println() call in the loop below, the motor will move
// very slowly!
//----------------------------------------------------------------------
#include <SwitecX25.h>

// #define LOGS            // Comment out this line if want to remove serial code!


/**
   DEFINES
*/
// Motor pis
#define MOTOR_PIN1      4
#define MOTOR_PIN2      5
#define MOTOR_PIN3      6
#define MOTOR_PIN4      7

#define SENSOR_PIN      2

#define STEPS          (315*3)    // Standard X25.168 range 315 degrees at 1/3 degree steps

#define MOTOR_ZERO      20
#define MOTOR_MAX       255

#define UPDATE_SLEEP_MS 1000
#define CONST_CONV_CM_PER_MS_TO_KM_PER_H 36

// 6 impulses -> full turn (360) so we have 39 cm per impulse
#define CM_FOR_IMPULSE  39.0f  // [cm/impulse]

/**
   GLOBALS
*/
// For motors connected to digital pins 4,5,6,7
SwitecX25 motor(STEPS, MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN3, MOTOR_PIN4);

volatile unsigned int impulse_counter = 0;
unsigned long last_check_ms = 0;

/**
   SETUP
*/
void setup(void) {
#ifdef LOGS
    Serial.begin(9600);
#endif

    // Run the motor against the stops
    motor.zero();
    // Set the motor to the defined zero
    motor.setPosition(MOTOR_ZERO);

    last_check_ms = millis();

    // Setup interrupt
    pinMode(SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), interrupt_handler, FALLING);

#ifdef LOGS
    Serial.println("Setup done...");
#endif
}


void loop() {

    const unsigned long now = millis();
    const unsigned long elapsed_time_ms = now - last_check_ms;

    if (elapsed_time_ms > UPDATE_SLEEP_MS) {

        last_check_ms = now;

        const unsigned int current_impulses = impulse_counter;
        impulse_counter = 0;

        const float distance_from_last_check = current_impulses * CM_FOR_IMPULSE;


        const float speed_cm_per_ms = distance_from_last_check / (float)elapsed_time_ms;
        const unsigned long km_per_sec = (unsigned long)(speed_cm_per_ms * CONST_CONV_CM_PER_MS_TO_KM_PER_H);

        const unsigned long motor_pos = calc_motor_position(km_per_sec);

        // Check if the motor position is in the range
        if (motor_pos <= MOTOR_MAX && motor_pos >= MOTOR_ZERO) {
            // Set new position
            motor.setPosition(motor_pos);
        }

#ifdef LOGS
        Serial.print("imp: ");
        Serial.print(current_impulses);
        Serial.print(", dist cm: ");
        Serial.print(distance_from_last_check);
        Serial.print(", elapsed ms: ");
        Serial.print(elapsed_time_ms);
        Serial.print(", speed cm/ms: ");
        Serial.print(speed_cm_per_ms);
        Serial.print(", Km/h: ");
        Serial.print(km_per_sec);
        Serial.print(", motor pos: ");
        Serial.print(motor_pos);
        Serial.print("\n");
#endif
    }

    // We can not use the sleep because the update needs to be called as often as possible
    motor.update();
}


void interrupt_handler() {
    ++impulse_counter;
}


unsigned long calc_motor_position(const unsigned long km_h) {
    int motor_position = 0;

    if (km_h < 10) {
        motor_position = map(km_h, 0, 9, MOTOR_ZERO, 65);
    } else if (km_h < 20) {
        motor_position = map(km_h, 10, 19, 0, 65);
    } else if (km_h < 30) {
        motor_position = map(km_h, 20, 29, 0, 65);
    } else if (km_h < 40) {
        motor_position = map(km_h, 30, 39, 0, 65);
    } else if (km_h < 50) {
        motor_position = map(km_h, 40, 49, 0, 65);
    } else if (km_h < 60) {
        motor_position = map(km_h, 50, 59, 0, 65);
    } else if (km_h < 70) {
        motor_position = map(km_h, 60, 69, 0, 65);
    } else if (km_h < 80) {
        motor_position = map(km_h, 70, 79, 0, 65);
    } else if (km_h < 90) {
        motor_position = map(km_h, 80, 89, 0, 65);
    } else if (km_h < 100) {
        motor_position = map(km_h, 90, 99, 0, 65);
    } else if (km_h < 110) {
        motor_position = map(km_h, 100, 109, 0, 65);
    } else if (km_h < 120) {
        motor_position = map(km_h, 110, 119, 0, 65);
    } else if (km_h < 130) {
        motor_position = map(km_h, 120, 129, 0, 65);
    } else if (km_h < 140) {
        motor_position = map(km_h, 130, 139, 0, 65);
    } else if (km_h < 150) {
        motor_position = map(km_h, 140, 149, 0, 65);
    } else {
        motor_position = map(km_h, 150, 500, 0, 65);
    }

    return motor_position;
}
