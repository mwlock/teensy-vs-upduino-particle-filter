#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/type_utilities.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/pose_array.h>
#include <geometry_msgs/msg/pose.h>
#include <std_msgs/msg/string.h>

#include "particle.hpp"
#include "particleFilter.hpp"

// Config headers
#include "../config/motion_model.h"
#include "../config/sensor_model.h"

// Random numbers
#include "FastRNG.hpp"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

// micro-ROS messages
rcl_publisher_t publisher;
rcl_publisher_t publisherPoseArray;
rcl_publisher_t publisherDebug;

rcl_subscription_t subscriber_odom;
rcl_subscription_t subscriber_scan;
std_msgs__msg__Int32 msg;

// Message types
nav_msgs__msg__Odometry odometryMsg;
sensor_msgs__msg__LaserScan laserScanMsg;
geometry_msgs__msg__PoseArray poseArray;
std_msgs__msg__String stringMsg;

// Executors
rclc_executor_t executor;
rclc_executor_t executor_scan_sub;
rclc_executor_t executor_odom_sub;
rclc_executor_t executor_particle_cloud_pub;
rclc_executor_t executor_string_pub;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#include <Entropy.h>

// Particle filter
ParticleFilter particleFilter;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Global variables
bool ledStatus = false;
nav_msgs__msg__Odometry lastOdom;

//consts
const int timeout_ms = 1000;

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

// Publish debug message and take string as a parameter
void publishDebugMessage(const char* message) {
  stringMsg.data.data = (char*)message;
  stringMsg.data.size = strlen(message);
  RCSOFTCHECK(rcl_publish(&publisherDebug, &stringMsg, NULL));
}

// ==================================================================================================================================================
// =                                                                                                                                                =
// =                                                         Timer callback                                                                         =
// =                                                                                                                                                =
// ==================================================================================================================================================

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  publishDebugMessage("Timer callback");

  // Send data
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;

    // Alive alert, alert alive
    ledStatus = !ledStatus;
    digitalWrite(LED_BUILTIN, ledStatus);

    // Check if odom is initalised (jump out function if not)
    if (!particleFilter.isInitialised()){
      return;
    }

    // Update particle set
    particleFilter.updateParticles();
    // publishDebugMessage("updated particles");

    // Calculate pose array
    geometry_msgs__msg__PoseArray particleCloud = particleFilter.getPoseArray();
    // publishDebugMessage("got pose array");

    // Publish pose array
    RCSOFTCHECK(rcl_publish(&publisherPoseArray, &particleCloud, NULL));
    // publishDebugMessage("published pose array");

    // Calculate estimated pose
    // geometry_msgs__msg__Pose estimatedPose = particleFilter.etimatePose();

    // Update the latest odom used
    particleFilter.updatePreviousOdom();
    // publishDebugMessage("updated previous odom");


  }
}

  // ==================================================================================================================================================
  // =                                                                                                                                                =
  // =                                                         Callbacks                                                                              =
  // =                                                                                                                                                =
  // ==================================================================================================================================================

// Odometry message cb
void odom_subscription_callback(const void *msgin) {

    // Check if particle filter is updating
  if (particleFilter.isUpdating()){
    return;
  }

  const nav_msgs__msg__Odometry * msg = (const nav_msgs__msg__Odometry *)msgin;

  // Copy msg to lastOdom field by field
  lastOdom.header = msg->header;
  lastOdom.child_frame_id = msg->child_frame_id;
  lastOdom.pose.pose.position.x = msg->pose.pose.position.x;
  lastOdom.pose.pose.position.y = msg->pose.pose.position.y;
  lastOdom.pose.pose.position.z = msg->pose.pose.position.z;
  lastOdom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  lastOdom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  lastOdom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  lastOdom.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  lastOdom.twist.twist.angular.x = msg->twist.twist.angular.x;
  lastOdom.twist.twist.angular.y = msg->twist.twist.angular.y;
  lastOdom.twist.twist.angular.z = msg->twist.twist.angular.z;
  lastOdom.twist.twist.linear.x = msg->twist.twist.linear.x;
  lastOdom.twist.twist.linear.y = msg->twist.twist.linear.y;
  lastOdom.twist.twist.linear.z = msg->twist.twist.linear.z; 

  // Print pose of latest odom
  char buffer[100];
  sprintf(buffer, "Odom poose: %f, %f, %f", lastOdom.pose.pose.position.x, lastOdom.pose.pose.position.y, lastOdom.pose.pose.position.z);
  publishDebugMessage(buffer);

  particleFilter.updateLatestOdom(lastOdom);
  publishDebugMessage("updated latest odom");

}

// Odometry message cb
void scan_subscription_callback(const void *msgin) {

  // Check if particle filter is updating
  if (particleFilter.isUpdating()){
    return;
  }

  // Get the message
  const sensor_msgs__msg__LaserScan * msg = (const sensor_msgs__msg__LaserScan *)msgin;
  particleFilter.updateLatestLaserScan(*msg);

}

void setup() {

  // Initialise Entropy
  Entropy.Initialize();

  // Allocate memory for LaserScan
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_string_capacity = 50;
  conf.max_ros2_type_sequence_capacity = 21;
  conf.max_basic_type_sequence_capacity = 21;
  bool success = micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    &laserScanMsg,
    conf
  );

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // LED setup and start up indicator
  ledStatus = false;
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledStatus);

  int on_off_times = 3;
  for (int i = 0; i < on_off_times*2; i++) {
    ledStatus = !ledStatus;
    delay(200); 
    digitalWrite(LED_BUILTIN, ledStatus);
  }

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
    "particlecloud"));

  // debug publisher
  RCCHECK(rclc_publisher_init_default(
    &publisherDebug,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "debug"));
  // initDebugPublisher(&node);

  // Make debug publisher
  // DebugPublisher & debug_publisher = DebugPublisher::getInstance();
  // debug_publisher.initPublisher(&node);

  // create timer,
  const unsigned int timer_timeout = 200; // in ms
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

  RCCHECK(rclc_executor_init(&executor_particle_cloud_pub, &support.context, 1, &allocator)); // particle cloud publisher
  RCCHECK(rclc_executor_init(&executor_string_pub, &support.context, 1, &allocator)); // string publisher

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

  // ==================================================================================================================================================
  // =                                                                                                                                                =
  // =                                                          Create particle filter                                                                =
  // =                                                                                                                                                =
  // ==================================================================================================================================================

  particleFilter = ParticleFilter(publishDebugMessage);    // Create particle filter

  msg.data = 0;
}

void loop() {
  delay(100);

  // Syncronize time
  RCCHECK(rmw_uros_sync_session(timeout_ms));

  // Spin ros executors
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_odom_sub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_scan_sub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_particle_cloud_pub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_string_pub, RCL_MS_TO_NS(100)));

  if (!particleFilter.isParticlesInitialised()){

    delay(100);      
    particleFilter.initParticleFilter();
    publishDebugMessage("Particles initialised");

    // ==================================================================================================================================================
    // =                                                                                                                                                =
    // =                                                     Test the gaussian random numers                                                            =
    // =                                                                                                                                                =
    // ==================================================================================================================================================

    // delay(1000);
    // const int32_t N = 10000;
    // int32_t samples[N];
    // // FastRNG gen;
    // double mean = 0.0, var = 0.0; 
    // elapsedMillis timer = 0;
    // for (int32_t i = 0; i < N; i++)
    // {
    //   samples[i] = normalDistribution(17.0, 2.0); 
    //   mean += samples[i];
    // }
    // mean /= (N);
    // for (int32_t i = 0; i < N; i++)
    // {
    //   var += (samples[i] - mean)*(samples[i] - mean); 
    // }
    // const uint32_t t = timer;
    
    // // Publish mean and standard deviation to debug topic
    // char buffer[100];
    // sprintf(buffer, "Mean: %f, Std: %f, Time: %d", mean, sqrt(var/(N-1)), t);
    // publishDebugMessage(buffer);

  }

}

