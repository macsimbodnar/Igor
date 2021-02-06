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

#define MOTOR_ZERO      17
#define MOTOR_MAX       650

#define UPDATE_SLEEP_MS 250

/**
   GLOBALS
*/
// For motors connected to digital pins 4,5,6,7
SwitecX25 motor(STEPS, MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN3, MOTOR_PIN4);

volatile unsigned int impulse_counter = 0;
unsigned long last_check_ms = 0;

/**
 * The motor on idle (900 RPM) to 30 impulses at second
 * So we have 15 rounds per second -> 2 impulses per round -> 0,d rounds per impulse
 */
#define ROUNDS_PER_IMPULSE 0.5f


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

        const float rounds_from_last_check = current_impulses * ROUNDS_PER_IMPULSE;
        const unsigned long rounds_per_min = (const unsigned long)((rounds_from_last_check * 60000) / (float) elapsed_time_ms);

        const unsigned long motor_pos = calc_motor_position(rounds_per_min);

        // Check if the motor position is in the range
        if (motor_pos <= MOTOR_MAX && motor_pos >= MOTOR_ZERO) {
            // Set new position
            motor.setPosition(motor_pos);
        }

#ifdef LOGS
        Serial.print("imp: ");
        Serial.print(current_impulses);
        Serial.print(", el ms: ");
        Serial.print(elapsed_time_ms);
        Serial.print(", R/check: ");
        Serial.print(rounds_from_last_check);
        Serial.print(", RPM: ");
        Serial.print(rounds_per_min);
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
    unsigned long motor_position = 0;

    if (km_h < 1000) {
        motor_position = map(km_h, 0, 999, MOTOR_ZERO, 100);
    } else if (km_h < 1500) {
        motor_position = map(km_h, 1000, 1499, 100, 166);
    } else if (km_h < 2000) {
        motor_position = map(km_h, 1500, 1999, 166, 231);
    } else if (km_h < 2500) {
        motor_position = map(km_h, 2000, 2499, 231, 297);
    } else if (km_h < 3000) {
        motor_position = map(km_h, 2500, 2999, 297, 360);
    } else if (km_h < 3500) {
        motor_position = map(km_h, 3000, 3499, 360, 422);
    } else if (km_h < 4000) {
        motor_position = map(km_h, 3500, 3999, 422, 485);
    } else if (km_h < 4500) {
        motor_position = map(km_h, 4000, 4499, 485, 547);
    } else if (km_h < 5001) {
        motor_position = map(km_h, 4500, 5000, 547, 610);
    } else {
        motor_position = MOTOR_MAX;
    }

    return motor_position;
}
