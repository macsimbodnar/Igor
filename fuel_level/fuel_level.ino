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
   DEFINES
*/
// Motor pis
#define MOTOR_PIN1      4
#define MOTOR_PIN2      5
#define MOTOR_PIN3      6
#define MOTOR_PIN4      7

#define RESERV_LED_PIN  8

#define SENSOR_PIN      A0          // датчик ДУТ
#define AVERAGE_FACTOR  0           // коэффициент сглаживания показаний (0 = не сглаживать)

#define POLLING_RATE    30 * 1000   // Milliseconds 

#define STEPS           (315*3)     // Standard X25.168 range 315 degrees at 1/3 degree steps

#define MOTOR_ZERO      35          //положение стрелки мин.
#define MOTOR_MAX       285         //положение стрелки макс.
#define SENSOR_ZERO     14          //цифра ДУТ мин.
#define SENSOR_MAX      454         //цифра ДУТ макс.

// MOTOR 50% THRASHOLD
#define MOTOR_HALF_PERCENT_TH   165 //положение стрелки  *1/2* бака
#define SENSOR_HALF             ((SENSOR_MAX - SENSOR_ZERO) / 2)

#define IN_RESERVE_TH   35          // лампа сигнализации резерва


/**
   GLOBALS
*/
// For motors connected to digital pins 4,5,6,7
SwitecX25 motor(STEPS, MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN3, MOTOR_PIN4);
unsigned long last_check  = 0;
int motor_position        = 0;
int last_sensor_val       = 0;
int led_status            = LOW;


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
   SETUP
*/
void setup(void) {
#ifdef LOGS
    Serial.begin(9600);
#endif
    digitalWrite(RESERV_LED_PIN, led_status);
    pinMode(RESERV_LED_PIN, OUTPUT);

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
   MAIN LOOP
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
            const int inverted_val = SENSOR_MAX - val;

            // Update only if the values differs
            if (abs(inverted_val - last_sensor_val) > AVERAGE_FACTOR) {
                last_sensor_val = inverted_val;

                check_if_reserve(last_sensor_val);

                // Using different mapping if the sensor is under or above the 50%
                if (last_sensor_val > SENSOR_HALF) {
                    // We are over the 50%
                    motor_position = map(last_sensor_val, SENSOR_HALF, SENSOR_MAX, MOTOR_HALF_PERCENT_TH + 1,
                                         MOTOR_MAX);
                } else {
                    // We are under 50%
                    motor_position = map(last_sensor_val, SENSOR_ZERO, SENSOR_HALF, MOTOR_ZERO, MOTOR_HALF_PERCENT_TH);
                }

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
        sprintf(buff, "SENSOR: %04d(raw) %04d(filtered) %04d  ->  MOTOR: %04d",
                raw_val, val, last_sensor_val, motor_position);
        Serial.println(buff);
#endif

        last_check = now;
    }

    // the motor only moves when you call update
    motor.update();
}


void check_if_reserve(int sensor_val) {
    int new_led_status;

    if (sensor_val > IN_RESERVE_TH) {
        // OK
        new_led_status = LOW;
    } else {
        // In reserve, turn on LED
        new_led_status = HIGH;
    }

    // Change the led status only if differrent from the previous check
    if (new_led_status != led_status) {
        led_status = new_led_status;
        digitalWrite(RESERV_LED_PIN, led_status);
    }
}
