#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/pose_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

// micro-ROS messages
rcl_publisher_t publisher;
rcl_publisher_t publisherPoseArray;
rcl_subscription_t subscriber_odom;
rcl_subscription_t subscriber_scan;
std_msgs__msg__Int32 msg;

// Message types
nav_msgs__msg__Odometry odometryMsg;
sensor_msgs__msg__LaserScan laserScanMsg;
geometry_msgs__msg__PoseArray msgPoseArray;

// Executors
rclc_executor_t executor;
rclc_executor_t executor_scan_sub;
rclc_executor_t executor_odom_sub;
rclc_executor_t executor_particle_cloud_pub;


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

  // ==================================================================================================================================================
  // =                                                                                                                                                =
  // =                                                         Callbacks                                                                              =
  // =                                                                                                                                                =
  // ==================================================================================================================================================

// Odometry message cb
void odom_subscription_callback(const void *msgin) {

  // Turn on LED
  // digitalWrite(LED_BUILTIN, HIGH);

//   RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//   msg.data++;
//   // const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
//   // if velocity in x direction is 0 turn off LED, if 1 turn on LED
//   // digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
}

// Odometry message cb
void scan_subscription_callback(const void *msgin) {

  // Turn on LED
  digitalWrite(LED_BUILTIN, HIGH);

}

void setup() {

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // LED setup and start up indicator
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "monte_carlo_localizer_teensy_node", "", &support));

  // ==================================================================================================================================================
  // =                                                                                                                                                =
  // =                                                         Subscribers declare                                                                    =
  // =                                                                                                                                                =
  // ==================================================================================================================================================

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber_odom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber_scan,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "scan"));

  // ==================================================================================================================================================
  // =                                                                                                                                                =
  // =                                                         Publisher   declare                                                                    =
  // =                                                                                                                                                =
  // ==================================================================================================================================================

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisherPoseArray,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseArray),
    "particlecloud_teensy"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // ==================================================================================================================================================
  // =                                                                                                                                                =
  // =                                                         Publishers init                                                                        =
  // =                                                                                                                                                =
  // ==================================================================================================================================================

  // publisher
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_executor_init(&executor_particle_cloud_pub, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // ==================================================================================================================================================
  // =                                                                                                                                                =
  // =                                                         Subscribers init                                                                       =
  // =                                                                                                                                                =
  // ==================================================================================================================================================

  // odom subscriber
  RCCHECK(rclc_executor_init(&executor_odom_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_odom_sub, &subscriber_odom, &odometryMsg, &odom_subscription_callback, ON_NEW_DATA));  

  // scan subscriber
  RCCHECK(rclc_executor_init(&executor_scan_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_scan_sub, &subscriber_scan, &laserScanMsg, &scan_subscription_callback, ON_NEW_DATA));  

  msg.data = 0;
}

void loop() {
  delay(100);
  // Spin ros executors
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_odom_sub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_scan_sub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_particle_cloud_pub, RCL_MS_TO_NS(100)));
}

