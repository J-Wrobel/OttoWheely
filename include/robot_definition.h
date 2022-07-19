/* ========== Robot debug ========== */

#define SERIAL_SPEED        9600

/* ========== Robot electronics ========== */

//Color sensor (for documentation purposes)
#define I2C_SCL             A5
#define I2C_SDA             A4

//SPI normally uses pins 10-13 for CS,MOSI,MISO,CLK
//LED definitions (Otto matrix uses digital pin writes insted of SPI???)
#define LED_CLK             2 
#define LED_CS              12
#define LED_MOSI            13
#define LED_MISO            
#define LED_Orientation     1  // 8x8 LED Matrix orientation  Top  = 1, Bottom = 2, Left = 3, Right = 4

//Steppers
#define STPR_L_IN1          7
#define STPR_L_IN2          6
#define STPR_L_IN3          5
#define STPR_L_IN4          4

#define STPR_R_IN1          11
#define STPR_R_IN2          10
#define STPR_R_IN3          9
#define STPR_R_IN4          8

// Ultrasonic sensor, NewPing easily allows to use one pin for both
#define DISTANCE_TRIGG_ECHO 3
#define DISTANCE_TRIGG      2 
#define DISTANCE_ECHO       3


//Reflective sensors, could be iterchangeable with sound sensors
#define REFLECTIVE_M_PIN    A2
#define REFLECTIVE_R_PIN    A1
#define REFLECTIVE_L_PIN    A0

//Sound sensors
#define SOUND_L_PIN         A7
#define SOUND_R_PIN         A6

//Buzzer and color light WARNING! Serial uses pins 0 and 1
#ifndef SERIAL_SPEED
#   define BUZZER_PIN        1
#   define COLOR_LIGHT_PIN   0
#endif

/* ========== Robot sensor properties ========== */

#define MAX_DISTANCE_CM     30

/* ========== Robot mechanics ========== */

#define STPR_STEPS          4098U
#define MAX_WHEEL_SPEED_RPM 30U

#define WHEEL_DIAMETER_MM   70U
// wheel distance from one another
#define WHEEL_DISTANCE_MM   100U
// disables stepper acceleration if set to 0
#define SKIP_STEPPER_ACCEL  1

