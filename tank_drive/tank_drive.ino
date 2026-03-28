#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <Servo.h>
#include <AccelStepper.h>

// ========== DIFFERENTIAL DRIVE MOTOR PINS ==========
#define LEFT_MOTOR_PIN 2
#define RIGHT_MOTOR_PIN 3

// ========== SCIENCE MODULE PINS ==========
#define AUGER_MOTOR_PIN 8
#define CAROUSEL_STEP_PIN 9
#define CAROUSEL_DIR_PIN 10
#define PLUNGER_STEP_PIN 11
#define PLUNGER_DIR_PIN 12
#define ACTUATOR_MOTOR_PIN 7


//============ Wheel, Battery, and Motor Specs ================

const double WHEEL_RADIUS = 0.14605/2; // in meters (11.5 inches diameter)
const double Kv = 473;  //Motor velocity constant in RPM/V
const double VOLTAGE_MAX = 14.5; // max voltage of the battery pack
const double GEAR_RATIO = 25; // motor gear ratio

const double MAX_RPM = Kv * VOLTAGE_MAX / GEAR_RATIO; // Max RPM at max voltage
const double MAX_VELOCITY = MAX_RPM * WHEEL_RADIUS * 2 * PI / 60; // (2.09 m/s) Max linear velocity in m/s of rover (2.09 m/s)

const double MAX_DESIRED_WHEEL_SPEED = 0.5;  // Speed limit that you put on the rover

// ========== DIFFERENTIAL DRIVE OBJECTS ==========
Servo left_motor;
Servo right_motor;

const int MIN_PWM = 1000;
const int NEUTRAL_PWM = 1500;
const int MAX_PWM = 2000;

const double WHEEL_BASE = 0.92;
const double MAX_LINEAR_VEL = 1.34;    //this is in m/s
const double MAX_ANGULAR_VEL = 2.0;   //this is in rad/s
const float DEADZONE = 0.05;

const unsigned long CMD_TIMEOUT_MS = 300;
unsigned long last_cmd_time = 0;

// ========== SCIENCE MODULE OBJECTS ==========
Servo auger_motor;
Servo actuator_motor;

AccelStepper carousel_stepper(AccelStepper::DRIVER, CAROUSEL_STEP_PIN, CAROUSEL_DIR_PIN);
AccelStepper plunger_stepper(AccelStepper::DRIVER, PLUNGER_STEP_PIN, PLUNGER_DIR_PIN);

const int STEPPER_MAX_SPEED = 2000;
const int STEPPER_ACCELERATION = 5000;

// ========== MICRO-ROS OBJECTS ==========
rcl_subscription_t twist_subscriber;
rcl_subscription_t science_subscriber;
geometry_msgs__msg__Twist twist_msg;

// Static buffer instead of malloc - much more reliable on embedded systems
#define SCIENCE_DATA_LEN 4
std_msgs__msg__Float32MultiArray science_msg;
float science_data_buffer[SCIENCE_DATA_LEN];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ========== ERROR HANDLING ==========
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// ========== DIFFERENTIAL DRIVE FUNCTIONS ==========
void setNeutralPWM() {
  left_motor.writeMicroseconds(NEUTRAL_PWM);
  right_motor.writeMicroseconds(NEUTRAL_PWM);
}


//=====================================================================================================================================
void setMotorPWM(double left_vel, double right_vel) {
  // Use the global constants defined at the top (1000, 1500, 2000)
  // We calculate the scaled output based on your desired speed limits
  
  // Map the velocities (-1.0 to 1.0) to the PWM range defined by your limits
  // left_vel and right_vel come from (linear +/- angular)
  
  float left_out = 1500 + (left_vel * 500 * (MAX_DESIRED_WHEEL_SPEED / MAX_VELOCITY));
  float right_out = 1500 + (right_vel * 500 * (MAX_DESIRED_WHEEL_SPEED / MAX_VELOCITY));

  left_motor.writeMicroseconds(constrain((int)left_out, MIN_PWM, MAX_PWM));
  right_motor.writeMicroseconds(constrain((int)right_out, MIN_PWM, MAX_PWM));
}

//====================================================================================================================================

void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_time = millis();
  double linear = msg->linear.x;
  double angular = msg->angular.z;

  if (abs(linear) < DEADZONE) linear = 0.0; //dont think i really need this
  if (abs(angular) < DEADZONE) angular = 0.0;

  //linear = constrain(linear, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);      // I guess you could constrain this to 1... also dont think i really need this.... planning on getting 1 as max angular and max linear speeds from python file
  //angular = constrain(angular, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

  double left_vel = linear - angular;        // Okay here is where i then re-create the left and right wheel velocities
  double right_vel = linear + angular;

  setMotorPWM(left_vel, right_vel);
}




void science_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg =
    (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size < 4) return;

  // Index 0: Auger motor
  int auger_pwm = constrain((int)msg->data.data[0], MIN_PWM, MAX_PWM);
  auger_motor.writeMicroseconds(auger_pwm);

  // Index 1: Carousel stepper
  float carousel_cmd = msg->data.data[1];
  if (abs(carousel_cmd) > 0.5) {
    carousel_stepper.move((long)carousel_cmd);
    carousel_stepper.runToPosition();
  }

  // Index 2: Plunger stepper
  float plunger_cmd = msg->data.data[2];
  if (abs(plunger_cmd) > 0.5) {
    plunger_stepper.move((long)plunger_cmd);
    plunger_stepper.runToPosition();
  }

  // Index 3: Actuator motor
  int actuator_pwm = constrain((int)msg->data.data[3], MIN_PWM, MAX_PWM);
  actuator_motor.writeMicroseconds(actuator_pwm);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
      setNeutralPWM();
    }
    carousel_stepper.run();
    plunger_stepper.run();
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("DEBUG: Starting setup...");

  // ===== Setup Motors =====
  left_motor.attach(LEFT_MOTOR_PIN);
  right_motor.attach(RIGHT_MOTOR_PIN);
  setNeutralPWM();
  
  auger_motor.attach(AUGER_MOTOR_PIN);
  auger_motor.writeMicroseconds(NEUTRAL_PWM);
  actuator_motor.attach(ACTUATOR_MOTOR_PIN);
  actuator_motor.writeMicroseconds(NEUTRAL_PWM);
  
  carousel_stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  carousel_stepper.setAcceleration(STEPPER_ACCELERATION);
  plunger_stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  plunger_stepper.setAcceleration(STEPPER_ACCELERATION);

  // Replace your fixed delay(2000) before micro-ROS setup with this:
  set_microros_transports();
  allocator = rcl_get_default_allocator();

  // Wait until agent is available
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 10)) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
  digitalWrite(LED_BUILTIN, HIGH);

  // FIX: Set size to 0 initially. The capacity is what matters for the buffer.
  science_msg.data.data = science_data_buffer;
  science_msg.data.capacity = SCIENCE_DATA_LEN;
  science_msg.data.size = 0; 
  memset(science_data_buffer, 0, sizeof(science_data_buffer));

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_driver_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
    &science_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/new_science_module/commands"));

  RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(100), timer_callback, true));

  // FIX: Increased handle count to 4 to prevent resource exhaustion errors
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &science_subscriber, &science_msg, &science_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("DEBUG: Setup complete!");

  pinMode(39, OUTPUT);
  digitalWrite(39, HIGH);

}

// ========== LOOP ==========
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  carousel_stepper.run();
  plunger_stepper.run();
  delay(10);
}