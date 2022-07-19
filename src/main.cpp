
// -------------- includes --------------
#include <robot_definition.h>
#include <Arduino.h>

#include <Otto.h>

#include <PlayRtttl.hpp>

#include <Wire.h>

#include "Adafruit_TCS34725.h"

#include "ColorConverterLib.h"

#include <AccelStepper.h>

#include <NewPing.h>

#include <Ticker.h>

/* TODOs:
   - performance of Otto mouth is pretty terrible, find way to make it faster or only on demand

*/

// -------------- defines --------------
#define SEC_TO_USEC 1000000UL

const unsigned long MAX_STEPS_PER_SEC      = (unsigned long)MAX_WHEEL_SPEED_RPM * STPR_STEPS / 60U;
const unsigned long STEPPER_UPDATE_PERIOD  = (unsigned long)SEC_TO_USEC/ (MAX_STEPS_PER_SEC * 2U);
const unsigned long DEFAULT_SPEED          = MAX_STEPS_PER_SEC / 4U;

const int SOUND_SENSORS_NUM = 2;
const int REFLECTIVE_SENSORS_NUM = 3;

// -------------- typedefs --------------

typedef enum{
  UNDEFINED = 0,
  RED,
  ORANGE,
  YELLOW,
  GREEN,
  CYAN,
  BLUE,
  VIOLET,
  COLORS_LAST
} colors_e;

typedef struct sensor_color
{
  float r,g,b;
  double h,s,v;
  uint16_t raw_r, raw_b, raw_g, raw_c;
} sensor_color_t;

typedef enum sensor_position
{
  SENSOR_L,
  SENSOR_R,
  SENSOR_M
} sensor_position_e;

typedef struct robot_state
{
  sensor_color_t sensor_color;
  colors_e last_seen_color;
  
  //sensor readings
  unsigned long distance_raw;
  int sound_raw[SOUND_SENSORS_NUM]; // see sensor_position_e
  int reflection_raw[REFLECTIVE_SENSORS_NUM]; // see sensor_position_e

  //melodies
  bool playing_melody;

  //performance
  unsigned int mainloop_counter;

  //steppers
  float motor_speed;
  bool ml_position_reached;
  bool mr_position_reached;

} robot_state_t;

void reset_mainloop_counter( robot_state_t * robot )
{
  #ifdef SERIAL_SPEED
  Serial.print("main:");
  Serial.println(robot->mainloop_counter, DEC);
  #endif
  robot->mainloop_counter = 0;
}

void tick_mainloop_counter( robot_state_t * robot )
{
  robot->mainloop_counter++;
}


// -------------- prototypes --------------

void task_50ms();
void task_100ms();
void task_500ms();
void task_1000ms();
void color_sensor_capturecolor();


// -------------- globals --------------

robot_state_t robot_state;
Otto Otto;
// const char data[] = "VARIABLE#";
// unsigned long int matrix;

Adafruit_TCS34725 tcs34725 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS,TCS34725_GAIN_4X );

#ifdef DISTANCE_TRIGG_ECHO
  NewPing sonar = NewPing(DISTANCE_TRIGG_ECHO, DISTANCE_TRIGG_ECHO, MAX_DISTANCE_CM);  
#else
  NewPing sonar = NewPing(DISTANCE_TRIGG, DISTANCE_ECHO, MAX_DISTANCE_CM);
#endif


AccelStepper stepper_l = AccelStepper(AccelStepper::FULL4WIRE,STPR_L_IN1, STPR_L_IN3, STPR_L_IN2, STPR_L_IN4);
AccelStepper stepper_r = AccelStepper(AccelStepper::FULL4WIRE,STPR_R_IN1, STPR_R_IN3, STPR_R_IN2, STPR_R_IN4);

/** periodic tasks */
Ticker task_50(task_50ms, 100, 0, MILLIS); 
Ticker task_100(task_100ms, 100, 0, MILLIS); 
Ticker task_500(task_500ms, 500, 0, MILLIS); 
Ticker task_1000(task_1000ms, 1000, 0, MILLIS); 

/** one shot tasks */
Ticker color_capture(color_sensor_capturecolor, 50, 1, MILLIS); //one shot on trigger

// -------------- local functions --------------

static inline robot_state_t * get_robot_state_obj(void)
{
  return &robot_state;
}

void log_robot_state( robot_state_t * state )
{
#ifdef SERIAL_SPEED
  Serial.print("spd:");
  Serial.print(state->motor_speed, DEC);
  Serial.print(" dst:");
  Serial.print(state->distance_raw, DEC);

  Serial.print(" sL:");
  Serial.print(state->sound_raw[SENSOR_L], DEC);
  Serial.print(" sR:");
  Serial.print(state->sound_raw[SENSOR_R], DEC);
  
  Serial.print(" rL:");
  Serial.print(state->reflection_raw[SENSOR_L], DEC);
  Serial.print(" rR:");
  Serial.print(state->reflection_raw[SENSOR_R], DEC);
  Serial.print(" rM:");
  Serial.print(state->reflection_raw[SENSOR_M], DEC);

  Serial.print(" clr:");
  Serial.print(state->last_seen_color, DEC);
  Serial.print("\n");
#endif
}

void set_color_light(bool on)
{
#ifdef COLOR_LIGHT_PIN
  digitalWrite(COLOR_LIGHT_PIN, on);
#endif
}

inline void color_sensor_convert_hsv(sensor_color_t *color)
{
	RGBConverter::RgbToHsv( static_cast<uint8_t>(color->r), 
                          static_cast<uint8_t>(color->g),
                          static_cast<uint8_t>(color->b), 
                          color->h, color->s,color->v);
	color->h *= 360;
	color->s *= 100;
	color->v *= 100;
}

inline bool color_sensor_iscolor(sensor_color_t *color, colors_e _color)
{
  if(color->h > 330 || color->h < 20){ if(_color==RED) return true; }
	else if(color->h < 45){ if(_color==ORANGE) return true; }
	else if(color->h < 90){ if(_color==YELLOW) return true; }
	else if(color->h < 150){ if(_color==GREEN) return true; }
	else if(color->h < 210){ if(_color==CYAN) return true; }
	else if(color->h < 270){ if(_color==BLUE) return true; }
 	else if(color->h < 330){ if(_color==VIOLET) return true; }
	return false;
}

inline void color_sensor_get_color(sensor_color_t *color, colors_e *_color)
{
  if(color->h > 330 || color->h < 20) { *_color=RED; }
	else if(color->h < 45)              { *_color=ORANGE; }
	else if(color->h < 90)              { *_color=YELLOW; }
	else if(color->h < 150)             { *_color=GREEN; }
	else if(color->h < 210)             { *_color=CYAN; }
	else if(color->h < 270)             { *_color=BLUE; }
 	else if(color->h < 330)             { *_color=VIOLET; }
	else  *_color=UNDEFINED;
}

inline static void color_sensor_trigger()
{
    set_color_light(1);
    color_capture.start();
}

void color_sensor_capturecolor()
{
  set_color_light(0);
  sensor_color_t *color = &(get_robot_state_obj()->sensor_color);
  tcs34725.getRGB(&color->r, &color->g, &color->b);
  color_sensor_convert_hsv( color );
  color_sensor_get_color( color, &get_robot_state_obj()->last_seen_color );
}

void read_ultrasound_distance(robot_state_t * robot)
{
  robot->distance_raw = sonar.ping();
}

void setup_LED() {
  Otto.initMATRIX( LED_MOSI, LED_CS, LED_CLK, LED_Orientation);
  Otto.putMouth(happyOpen);
  Otto.matrixIntensity(8);
}

void update_LED()
{  
  // test: cycle through mouths
  static int mouth = 0;
  Otto.putMouth(mouth);  
  mouth++;
  if(mouth>NUMBER_OF_ELEMENTS-1) mouth = 0;
}

void set_speed(float speed_l, float speed_r) {
  stepper_l.setSpeed(speed_l);
  stepper_r.setSpeed(speed_r);
}

void drive_motors() 
{
  
  if(SKIP_STEPPER_ACCEL)
  {
    get_robot_state_obj()->ml_position_reached = !stepper_l.runSpeedToPosition();
    get_robot_state_obj()->mr_position_reached = !stepper_r.runSpeedToPosition();
  }
  else
  {
    stepper_l.run();
    stepper_r.run();
  }
}

void setup_buzzer()
{
#ifdef BUZZER_PIN
  pinMode(BUZZER_PIN, OUTPUT);
  setTonePinIsInverted(1);
#endif
}
void setup_distance()
{
#ifdef DISTANCE_TRIGG_ECHO
  // lib will set it own
#else
  pinMode(DISTANCE_TRIGG, OUTPUT);
  pinMode(DISTANCE_ECHO, INPUT);
#endif
  
}

void setup_sound_sensors()
{
  // for analog pins mode is not needed unless theyy were se to digital
  //pinMode(SOUND_L_PIN, INPUT);
  //pinMode(SOUND_R_PIN, INPUT);
}

void setup_reflective_sensors()
{
  // for analog pins mode is not needed unless theyy were se to digital
  //pinMode(REFLECTIVE_L_PIN, INPUT);
  //pinMode(REFLECTIVE_R_PIN, INPUT);
  //pinMode(REFLECTIVE_M_PIN, INPUT);
}

void read_sound_sensors(robot_state_t *robot)
{
  robot->sound_raw[SENSOR_L] = analogRead(SOUND_L_PIN);
  robot->sound_raw[SENSOR_R] = analogRead(SOUND_R_PIN);
  // float l = analogRead(SOUND_L_PIN)*5.0/1024;
  // float r = analogRead(SOUND_R_PIN)*5.0/1024;  
}

void read_reflective_sensors(robot_state_t *robot)
{
  // TODO consider filtering i.e. weighted filter
  robot->reflection_raw[SENSOR_L] = analogRead(REFLECTIVE_L_PIN);
  robot->reflection_raw[SENSOR_R] = analogRead(REFLECTIVE_R_PIN);
  robot->reflection_raw[SENSOR_M] = analogRead(REFLECTIVE_M_PIN);
  // float l = analogRead(SOUND_L_PIN)*5.0/1024;
  // float r = analogRead(SOUND_R_PIN)*5.0/1024;  
}

void setup_color_sensor()
{
  #ifdef SERIAL_SPEED
  if( !tcs34725.begin() ) Serial.println("color sensor setup failed");
  #endif
  tcs34725.setInterrupt(false);
#ifdef COLOR_LIGHT_PIN
  pinMode(COLOR_LIGHT_PIN, OUTPUT);
  digitalWrite(COLOR_LIGHT_PIN, 0);
#endif
}

void on_melody_complete()
{
  #ifdef BUZZER_PIN
  get_robot_state_obj()->playing_melody = false;
  #endif
}

void start_random_melody( robot_state_t *robot )
{
  #ifdef BUZZER_PIN
  if( !robot->playing_melody )
  {
    startPlayRtttlPGM(BUZZER_PIN, StarWars, on_melody_complete);
    robot->playing_melody = true;
  }
  #endif
}

void drive_melodies_nb( robot_state_t *robot )
{
  #ifdef BUZZER_PIN
  if( robot->playing_melody ) updatePlayRtttl();
  #endif
}

inline void beep( int note)
{
  #ifdef BUZZER_PIN
  if( !get_robot_state_obj()->playing_melody )
  {
    NewTone(BUZZER_PIN, note, 25);
  }
  #endif
}

//heading is in degrees:0 is straigth forward -90 is left
void set_new_heading(uint16_t heading)
{

}

//heading is in degrees:0 is straigth forward
//check radians
//add enum if both or left, or right wheel
void set_motor_distance_mm( long dist_l, long dist_r )
{
  Serial.print("Set distance to go ");
  Serial.print(dist_l);
  Serial.print(",");
  Serial.print(dist_r);
  Serial.print("->");
  Serial.print(dist_l*STPR_STEPS/WHEEL_DIAMETER_MM);
  Serial.print(",");
  Serial.print(dist_r*STPR_STEPS/WHEEL_DIAMETER_MM);

  stepper_l.move(dist_l*STPR_STEPS/WHEEL_DIAMETER_MM);

  stepper_r.move(dist_r*STPR_STEPS/WHEEL_DIAMETER_MM);
}


void turn_in_place(int16_t heading)
{
  //circle around robot is 
  float whole_circle_distance = WHEEL_DISTANCE_MM * PI;

  //in place we use both wheels to turn
  float one_wheel_dist = whole_circle_distance*(heading/360)/2;

  set_motor_distance_mm(one_wheel_dist/2, one_wheel_dist/2);
}


void setup_motors() {

  stepper_l.setMaxSpeed(MAX_STEPS_PER_SEC);
  stepper_l.setAcceleration(800.0);
  stepper_l.moveTo(10000);

  stepper_r.setMaxSpeed(MAX_STEPS_PER_SEC);
  stepper_r.setAcceleration(800.0);
  stepper_r.moveTo(-10000);

  set_speed(DEFAULT_SPEED, DEFAULT_SPEED);

  // if failed loop forever
  //if(!setup_interrupts(STEPPER_UPDATE_PERIOD)) while(1) {};
  sonar.timer_us(STEPPER_UPDATE_PERIOD, drive_motors); // setup timer interupt to drive steppers
}

void setup() {
  // put your setup code here, to run once:

  //clear robot struct
  memset( get_robot_state_obj(), 0, sizeof(robot_state_t) );

#ifdef SERIAL_SPEED
  Serial.begin(SERIAL_SPEED);
  Serial.println("setup started");

  Serial.print("MAX_STEPS_PER_SEC: ");
  Serial.println(MAX_STEPS_PER_SEC, DEC);
  Serial.print("STEPPER_UPDATE_PERIOD: ");
  Serial.println(STEPPER_UPDATE_PERIOD, DEC);
#endif
  
  // setup delay just in case
  delay(2*1000);

  setup_color_sensor();
  setup_buzzer();
  setup_distance();
  setup_motors();
  setup_LED();
  setup_sound_sensors();
  setup_reflective_sensors();

  task_50.start();
  task_100.start();
  task_500.start();
  task_1000.start();

  start_random_melody( get_robot_state_obj() );

  #ifdef SERIAL_SPEED
  Serial.println("setup done");
  #endif
  set_motor_distance_mm(70,-70);
  set_motor_distance_mm(140,-140);

  set_motor_distance_mm(10,-10);
}


inline void task_500ms()
{

}

inline void task_1000ms()
{
  robot_state_t * robot = get_robot_state_obj();
  color_sensor_trigger();    

  // update_LED();
  log_robot_state( robot );
  
  update_LED();

  reset_mainloop_counter( robot );

  beep(NOTE_E1);
}



inline void task_50ms()
{
  read_sound_sensors(get_robot_state_obj());
  read_reflective_sensors(get_robot_state_obj());
}

inline void task_100ms()
{
  read_ultrasound_distance(get_robot_state_obj());
}

//avoid blocking tasks here, gotta go fast!
inline void task_0ms()
{
  drive_melodies_nb(get_robot_state_obj());
  tick_mainloop_counter(get_robot_state_obj());
}

void loop() 
{
  // put your main code here, to run repeatedly:
  //static unsigned long current_millis;

  //current_millis = millis();
  
  task_0ms();

  color_capture.update();

  //50ms task
  //if(current_millis % 50U == 0) task_50ms();
  task_50.update();

  //100ms task
  //if(current_millis % 100U == 0) task_100ms();
  task_100.update();

  //500ms task
  //if(current_millis % 500U == 0) task_500ms();
  task_500.update();

  //1000ms task
  //if(current_millis % 1000U == 0) task_1000ms();
  task_1000.update();
}


#if 0 
/*****************************************************************************
** SetUpInterrupts
** ===============
Set up interrupt routine to service stepper motor run() function.
*/
bool setup_interrupts(const int usecs)
{
  // initialize Timer1
  cli(); // disable global interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B

  // set compare match register to desired timer count (1ms):
  // ATmega328 with a 16MHz clock, clk/8
  // (# timer counts + 1) = (target time) / (timer resolution)
  // = .0001s / 6.25e-8 s * 8
  // = 200
  const float targetSecs = ((float) usecs) / 1e6;
  const float timerSteps = 6.25e-8; // 1/16MHz
  int count = 0;
  int prescale = 1; // valid values: 1, 8, 64, 256, 1024
  do {
  count = targetSecs / (timerSteps * prescale);
  if(count < 65535) // Timer 1 is 16-bits wide
  break;
  prescale *= 8;
  } while (prescale <= 1024);
  if(prescale > 1024) // time too long
  return false;
  if(prescale == 1 && count < 100) // time too short
  return false;

  OCR1A = count; // Eg, 200 = 0.1ms - I found 1ms gives rough acceleration
  // turn on CTC mode (Clear Timer on Compare Match):
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  // CS12 CS11 CS10
  // 0 0 0 no clock source, Timer/counter stopped
  // 0 0 1 clk/1 no prescaling
  // 0 1 0 clk/8
  // 0 1 1 clk/64
  // 1 0 0 clk/256
  // 1 0 1 clk/1024
  // 1 1 0 external clock on T1 pin, falling edge
  // 1 1 1 external clock on T1 pin, rising edge
  switch(prescale) {
  case 1:
  TCCR1B |= (1 << CS10); // 0 0 1
  break;
  case 8:
  TCCR1B |= (1 << CS11); // 0 1 0
  break;
  case 64:
  TCCR1B |= (1 << CS11) & (1 << CS10); // 0 1 1
  break;
  case 256:
  TCCR1B |= (1 << CS12); // 1 0 0
  break;
  case 1024:
  TCCR1B |= (1 << CS12) & (1 << CS10); // 1 0 1
  break;
  }
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  // enable global interrupts:
  sei();

  return true;
}
/****************************************************************************/


/*****************************************************************************
** ISR
** ===
Interrupt service routine for when Timer 1 matches compare value
*/
// ISR(TIMER1_COMPA_vect)
// {
//   drive_motors();
// }
/****************************************************************************/
#endif

