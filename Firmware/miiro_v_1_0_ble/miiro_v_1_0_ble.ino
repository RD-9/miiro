/*  Firmware for MiiRo Robot
 *  This version is tailored to be compatible with the LoFi Blocks Application
 *  Version 1.2
 *  Date: May 2021
 *  Author: Tyrone van Balla
 *  Company: RD9 Solutions
 */
 
// included libraries
#include <SoftwareSerial.h>
#include <Servo.h>

// pin assignments

// bluetooth module communication
#define rx_pin 7
#define tx_pin 8

// continuous rotation servo motors
#define servo_right_pin 5
#define servo_left_pin 6

// RBG LED
#define led_r_pin 19 
#define led_g_pin 18 
#define led_b_pin 17 

// buzzer
#define buzzer_pin A0

// push button
#define push_button_pin 3

// grove connector
#define grove_data_1 15
#define grove_data_2 16

// commands
#define i_uss
#define i_button

#define o_buzzer 201
#define o_motor_1 202
#define o_motor_2 203
#define o_servo 208 // servo, output_1
#define o_led_r 204 // output_1
#define o_led_g 205 // output_2
#define o_led_b 206 // output_3

// constants

#define baud_rate 57600

int current_byte = 0;
int prev_byte = 0;
int input_button = 0;
int cntr = 0;
int motor_direction = 0;
int motor_a_speed = 0;
int motor_b_speed = 0;

// variables for creating delay in sending data
unsigned long previous_timestamp = 0;
unsigned long current_timestamp;
const long time_delay = 1250; // milliseconds

// for ultrasonic sensor
int dist = 0;
int uss_trigger_pin = grove_data_1;
int uss_echo_pin = grove_data_2;

// for push button
int button_val;

// create objects
SoftwareSerial mySerial (rx_pin, tx_pin); // RX, TX from Arduino's Perspective

Servo right_servo;
Servo left_servo;

// function prototypes
void configure_pins(void);
void initialize_pins(void);
void initialization_ok(void);
void receive_data_from_miicode(void);
void outputs_set(void);
void send_data_to_miicode(void);
void initialize_servos(void);
long read_distance_uss(void);
void receive_data_from_lofi(void);
void send_data_to_lofi(void);
void stop_all(void);

void setup()
{

  // configure all pin modes
  configure_pins();

  // initialize all pins
  initialize_pins();

  // initialize servos
  initialize_servos();

  // board initialization - ok!
  initialization_ok();

  // initialize serial communications at specified baud rate
  mySerial.begin(baud_rate);

  // setup complete
}

void loop()
{
    // receive data from the app
    //receive_data_from_miicode(); // for miicode
    receive_data_from_lofi(); // for lofi

    // check time passed
    current_timestamp = millis();
    if (current_timestamp - previous_timestamp >= time_delay)
    {
      // reset the previous timestamp
      previous_timestamp = current_timestamp;
      
      // send data
      //send_data_to_miicode(); // for miicode
      send_data_to_lofi(); // for lofi
    }
}

// board initialization - ok!: led red flash, then green, then green, then off
void initialization_ok()
{
  analogWrite(led_r_pin, 0);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);
  delay(500);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);
  delay(500);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 0);
  analogWrite(led_b_pin, 255);
  delay(500);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);
  delay(500);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 0);
  analogWrite(led_b_pin, 255);
  delay(500);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);
}


// receive data from Lofi Application via Bluetooth
void receive_data_from_lofi(void)
{
  if (mySerial.available() > 0)
  {
      current_byte = mySerial.read();
      outputs_set();
      prev_byte = current_byte;
  } 
}

// receives data from the miicode application via Bluetooth
void receive_data_from_miicode()
{
  if (mySerial.available() > 0)
  {
    // catch any motor related commands as these commands consist of three parts
    if ((prev_byte == 202 or prev_byte == 203 or prev_byte == 235) and cntr == 0)
    {      
      // current byte will be the direction
      motor_direction = mySerial.read();
      // keep prev_byte
      cntr = 1;
    }
    else
    {
      current_byte = mySerial.read();
      outputs_set();
      prev_byte = current_byte;
    }
 
  } 
}

// function to perform output based on data received from MiiCode
void outputs_set()
{   
  // buzzer on/off set tone
  if (prev_byte == o_buzzer)
  {
    // turn the buzzer on
    if (current_byte == 1)
    {
      tone(buzzer_pin, 2400);
    }

    // turn the buzzer off
    if (current_byte == 0)
    {
      noTone(buzzer_pin);
    }
  }

  // The two motors have to be driven in opposite directions to account for their
  // orientation on the robot.
  // Motor A - right motor when viewing from behind, left motor when viewing from the front
  if (prev_byte == o_motor_1)  
  {
    // determine what value to write for the motors
    if (current_byte == 0)
    {
      // stop
      motor_a_speed = 90;
    }
    
    else if (current_byte <= 100)
    {
      motor_a_speed = (90 - int(current_byte*0.9));
    }
    else
    {
      // reverse direction
      motor_a_speed = (90 + int(current_byte*0.9));
    }
    right_servo.attach(servo_right_pin);
    right_servo.write(motor_a_speed);
    cntr = 0;

    if (motor_a_speed == 90)
    {
      right_servo.detach();
    }
    
  }

  // Motor B - left motor when viewing from behind, right motor when viewing from the front
  if (prev_byte == o_motor_2)  
  {
    // determine what value to write for the motors
    if (current_byte == 0)
    {
      // stop
      motor_b_speed = 90;
    }

    else if (current_byte <= 100)
    {
      motor_b_speed = (90 + int(current_byte*0.9));
    }
    else
    {
      // reverse direction
      motor_b_speed = (90 - int(current_byte*0.9));
    }
    left_servo.attach(servo_left_pin);
    left_servo.write(motor_b_speed);
    cntr = 0;

    if (motor_b_speed == 90){
      left_servo.detach();
    }
    
  }

  // RGB Output
  if (prev_byte == o_led_r)
  {
      analogWrite(led_r_pin, 255-(current_byte*2.55));
  }

  if (prev_byte == o_led_g)
  {
      analogWrite(led_g_pin, 255-(current_byte*2.55));
  }

  if (prev_byte == o_led_b)
  {
      analogWrite(led_b_pin, 255-(current_byte*2.55));
  }

  if (prev_byte == 213 && current_byte == 99)
  {
    stop_all();
  }
}

// sends data to the LoFi application via Bluetooth
void send_data_to_lofi(void)
{

  // push button input
  button_val = digitalRead(push_button_pin);
  
  if (button_val == LOW){
    input_button = 1;
  }
  else{
    input_button = 0;
  }
  
//[224, 115, 2, 225, 102, 4, 226, 107, 5, 227, 63, 6]
  mySerial.write(224);
  mySerial.write(input_button);
  mySerial.write(225);
  mySerial.write(byte(0));
  mySerial.write(226);
  mySerial.write(byte(0));
  mySerial.write(227);
  mySerial.write(byte(0));

  read_distance_uss();

  mySerial.write(240);
  mySerial.write(byte(dist));
  // last byte "i" character as a delimiter for BT2.0 on Android
  mySerial.write(105);
}

// sends data to the miicode application via Bluetooth
void send_data_to_miicode()
{
  // push button input
  button_val = digitalRead(push_button_pin);
  
  mySerial.write('abc');
  
  if (button_val == LOW){
    input_button = 1;
  }
  else{
    input_button = 0;
  }
  
  mySerial.write(input_button);
  mySerial.write('abc');
  
  // ultra sonic sensor output
  read_distance_uss();
  mySerial.write('xyz');
  mySerial.write(dist);
  mySerial.write('xyz');
}

// function to configure pins and pin modes
void configure_pins()
{
  // RGB LED pins
  pinMode(led_r_pin, OUTPUT);
  pinMode(led_g_pin, OUTPUT);
  pinMode(led_b_pin, OUTPUT);

  // buzzer
  pinMode(buzzer_pin, OUTPUT);

  // button
  pinMode(push_button_pin, INPUT);

  // ultrasonic sensor pins
  pinMode(uss_trigger_pin, OUTPUT);
  pinMode(uss_echo_pin, INPUT);
  
}

// function to set the default pin states
void initialize_pins()
{
  // turn all LEDs off
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);

  // turn buzzer off
  noTone(buzzer_pin);
}

void initialize_servos()
{
  // create objects
  right_servo.attach(servo_right_pin);
  left_servo.attach(servo_left_pin);

  // set initial speed - brake
  right_servo.write(90);
  left_servo.write(90);

  // detach servo motor pins after initialization
  right_servo.detach();
  left_servo.detach();
  
}

// ultrasonic sensor function
long read_distance_uss()
{
  
  long duration, distance;
  digitalWrite(uss_trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(uss_trigger_pin, HIGH);

  delayMicroseconds(5);
  digitalWrite(uss_trigger_pin, LOW);
  duration = pulseIn(uss_echo_pin, HIGH, 5000);
  distance = (duration/2) / 29.1;
  
  dist = distance;
}

void stop_all(void) 
{
  initialize_pins();
  initialize_servos();
}

//end
