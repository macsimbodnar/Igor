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
#include <Smoothed.h> // https://github.com/MattFryer/Smoothed


// #define LOGS            // Comment out this line if want to remove serial code!


/**
 * DEFINES
 */
// Motor pis
#define MOTOR_PIN1      4
#define MOTOR_PIN2      5
#define MOTOR_PIN3      6
#define MOTOR_PIN4      7

#define SENSOR_PIN      A0          // датчик ДУТ
#define AVERAGE_FACTOR  0           // коэффициент сглаживания показаний (0 = не сглаживать)

#define POLLING_RATE    1000        // Milliseconds 

#define STEPS          (315*3)      // Standard X25.168 range 315 degrees at 1/3 degree steps

#define MOTOR_ZERO      35          //положение стрелки мин.
#define MOTOR_MAX       285         //положение стрелки макс.

#define SENSOR_ZERO     0           //цифра ДУТ мин.
#define SENSOR_MAX      400         //цифра ДУТ макс.


/**
 * GLOBALS
 */
// For motors connected to digital pins 4,5,6,7
SwitecX25 motor(STEPS, MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN3, MOTOR_PIN4);
unsigned long last_check  = 0;
int motor_position        = 0;
int last_sensor_val       = 0;


/**
 * FILTER
 */
#define FILTER_ACCURACY 10
#define FILTER_METHOD SMOOTHED_EXPONENTIAL // SMOOTHED_AVERAGE

Smoothed <float> filter;

int exponential_filter(const int sensor_val) {
    // Read a sensor value from analogue pin 0
    filter.add(sensor_val);
    // Output the smoothed sensor value to the serial
    return filter.get();
}


/**
 * SETUP
 */
void setup(void) {
#ifdef LOGS
    Serial.begin(9600);
#endif

    // Run the motor against the stops
    motor.zero();
    // Set the motor to the defined zero
    motor.setPosition(MOTOR_ZERO);

    // Init last_check
    last_check = millis();

#ifdef LOGS
    Serial.println("GO!");
#endif

    filter.begin(FILTER_METHOD, FILTER_ACCURACY);
}


/**
 * MAIN LOOP
 */
void loop(void) {
    // Read past time
    unsigned long now = millis();

    // Check sensors and update the position only after polling rate expires
    if (now - last_check > POLLING_RATE) {

        // Read sensor value
        const int raw_val = analogRead(SENSOR_PIN);
        const int val = exponential_filter(raw_val);

        if (val >= SENSOR_ZERO && val <= SENSOR_MAX) {

            // Invert the value
            // val = SENSOR_MAX - val;

            // Update only if the values differs
            if (abs(val - last_sensor_val) > AVERAGE_FACTOR) {
                last_sensor_val = val;

                motor_position = calc_motor_position(last_sensor_val);

                // Check if the motor position is in the range
                if (motor_position <= MOTOR_MAX && motor_position >= MOTOR_ZERO) {
                    // Set new position
                    motor.setPosition(motor_position);
                }

#ifdef LOGS
                else {
                    Serial.println("Motor out of range!");
                }

#endif
            }
        }

#ifdef LOGS
        else {
            Serial.println("Sensor out of range!");
        }

#endif

#ifdef LOGS
        char buff[50];
        sprintf(buff, "SENSOR: %04d(raw) %04d(filtered) %04d  ->  MOTOR: %04d", raw_val, val,
                last_sensor_val, motor_position);
        Serial.println(buff);
#endif

        last_check = now;
    }

    // The motor only moves when you call update
    motor.update();
}


int calc_motor_position(const int sens_val) {
    int motor_position = 0;

    // Using different mapping
    // if (sens_val < 50) {
    //   motor_position = map(sens_val, SENSOR_ZERO, 50, 260, 330);
    // } else if (sens_val >= 50 && sens_val < 100) {
    //   motor_position = map(sens_val, 50, 100, 205, 260);
    // } else if (sens_val >= 100 && sens_val < 150) {
    //   motor_position = map(sens_val, 100, 150, 163, 205);
    // } else if (sens_val >= 150 && sens_val < 200) {
    //   motor_position = map(sens_val, 150, 200, 130, 163);
    // } else { //if (sens_val > 200) {
    //   motor_position = map(sens_val, 200, SENSOR_MAX, 163, MOTOR_MAX);
    // }
    if (sens_val < 130) {
        motor_position = map(sens_val, SENSOR_ZERO, 129, MOTOR_MAX, 200);
    } else if (sens_val < 163) {
        motor_position = map(sens_val, 130, 163, 200, 150);
    } else if (sens_val < 205) {
        motor_position = map(sens_val, 163, 205, 150, 100);
    } else if (sens_val < 240) {
        motor_position = map(sens_val, 205, 240, 50, 100);
    } else {
        motor_position = map(sens_val, 240, SENSOR_MAX, MOTOR_ZERO, 50);
    }

    return motor_position;
}


// int averaging_filter(const int sens_val[], const int size) {
//     int av = 0;

//     for (int i = 0; i < size; ++i) {
//         av += MeasureTemperature();
//         delay(1);
//     }

//     av /= size;
//     return av;
// }
