#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/type_utilities.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <string>

#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/pose_array.h>
#include <geometry_msgs/msg/pose.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/string.h>

#include "particle.hpp"
#include "particleFilter.hpp"

// Config headers
#include "../config/motion_model.h"
#include "../config/sensor_model.h"

// Random numbers
#include "FastRNG.hpp"

// Utils
#include "memoryUtil.hpp"

// Particle filter parameters
#define RESAMPLING_FREQUENCY 30 // Hz
#define RESAMPLING_TIME_MILLISECONDS int((1.0/RESAMPLING_FREQUENCY)*1000)  // ms

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

// micro-ROS messages
rcl_publisher_t publisher;
rcl_publisher_t publisherPoseArray;
rcl_publisher_t publisherEstimatedPose;
rcl_publisher_t publisherDebug;
rcl_publisher_t publisherTime;
rcl_publisher_t publisherConfigString;

rcl_subscription_t subscriber_odom;
rcl_subscription_t subscriber_scan;
std_msgs__msg__Int32 msg;

// Message types
nav_msgs__msg__Odometry odometryMsg;
sensor_msgs__msg__LaserScan laserScanMsg;
geometry_msgs__msg__PoseArray poseArray;
geometry_msgs__msg__Pose estimatedPoseMsg;
std_msgs__msg__String stringMsg;
std_msgs__msg__Float64 time_msg;
std_msgs__msg__String configStringMsg;

// Executors
rclc_executor_t executor;
rclc_executor_t executor_scan_sub;
rclc_executor_t executor_odom_sub;
rclc_executor_t executor_estimated_pose_pub;
rclc_executor_t executor_particle_cloud_pub;
rclc_executor_t executor_string_pub;
rclc_executor_t executor_time_pub;
rclc_executor_t executor_config_string_pub;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Particle filter
ParticleFilter particleFilter;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Global variables
bool ledStatus = false;
bool sent_config = false;
char configString[1000];
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

double getStartTime(){
  struct timespec ts;
  clock_gettime(0, &ts);
  return ts.tv_sec + ts.tv_nsec / 1e9;
}

double getTimeInterval(double start_time){
  struct timespec ts;
  clock_gettime(0, &ts);
  double end_time = ts.tv_sec + ts.tv_nsec / 1e9;
  return (end_time - start_time) * 1000.0;
}

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

    // Check if config has been sent
    if (!sent_config){
      rcl_publish(&publisherConfigString, &configStringMsg, NULL);
      sent_config = true;
    }

    // Update particle set
    geometry_msgs__msg__Pose estimatedPose;
    geometry_msgs__msg__PoseArray particleCloud;
    bool particleUpdated;
    double start_time = getStartTime();
    std::tie(particleCloud,estimatedPose,particleUpdated) = particleFilter.updateParticles();
    double time_interval = getTimeInterval(start_time);
    // PRINT TIME TAKEN
    char buffer[100];
    sprintf(buffer, "Time taken to update particles: %f ms", time_interval);
    publishDebugMessage(buffer);

    // If the particle were updated, publish the new update time
    if (particleUpdated){
      time_msg.data = time_interval;
      RCSOFTCHECK(rcl_publish(&publisherTime, &time_msg, NULL));
    }

    // Update the latest odom used
    particleFilter.updatePreviousOdom();

    RCSOFTCHECK(rcl_publish(&publisherPoseArray, &particleCloud, NULL));
    RCSOFTCHECK(rcl_publish(&publisherEstimatedPose, &estimatedPose, NULL));

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
  particleFilter.updateLatestOdom(lastOdom);

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
  // Serial.begin(115200);
  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  delay(2000);

  // Init Serial2 if hardware acceleration is enabled
  #ifdef HARDWARE_ACCELERATION
    Serial2.begin(3000000);
  #endif

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
    "/particle_filter/particle_pose_array"));

  // Create estimated pose publisher
  RCCHECK(rclc_publisher_init_default(
    &publisherEstimatedPose,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
    "particle_filter/esimated_pose"));

  // debug publisher
  RCCHECK(rclc_publisher_init_default(
    &publisherDebug,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "debug"));

  // ConfigString publisher
  RCCHECK(rclc_publisher_init_default(
    &publisherConfigString,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "particle_filter/config_string"));

  // Time publisher
  RCCHECK(rclc_publisher_init_default(
    &publisherTime,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "particle_filter/update_time"));

  // create timer,
  const unsigned int timer_timeout = RESAMPLING_TIME_MILLISECONDS; // in ms
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

  RCCHECK(rclc_executor_init(&executor_particle_cloud_pub, &support.context, 1, &allocator));         // particle cloud publisher
  RCCHECK(rclc_executor_init(&executor_string_pub, &support.context, 1, &allocator));                 // string publisher
  RCCHECK(rclc_executor_init(&executor_estimated_pose_pub, &support.context, 1, &allocator));         // estimated pose publisher
  RCCHECK(rclc_executor_init(&executor_time_pub, &support.context, 1, &allocator));                   // estimated pose publisher
  RCCHECK(rclc_executor_init(&executor_config_string_pub, &support.context, 1, &allocator));          // config string publisher

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
  // delay(100);

  // Syncronize time
  RCCHECK(rmw_uros_sync_session(timeout_ms));

  // Spin ros executors
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_odom_sub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_scan_sub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_particle_cloud_pub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_string_pub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_estimated_pose_pub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_time_pub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_config_string_pub, RCL_MS_TO_NS(100)));

  if (!particleFilter.isParticlesInitialised()){

    delay(100);      
    particleFilter.initParticleFilter();
    publishDebugMessage("Particles initialised");

    // CHECK MEMORY USAGE
    int stack;
    int heap;
    int psram;
    std::tie(stack, heap, psram) = MemoryUtil::memInfo();
    char buffer[100];
    sprintf(buffer, "Memory: stack (kb): %d, heap (kb): %d, psram (kb): %d", stack, heap, psram);
    publishDebugMessage(buffer);
    // MemoryUtil::getFreeITCM();

    // Compose config string with number of particles using sprintf
    int char_used = sprintf(configString, "==============Config string==============:\n" \
                                          "num_particles: %d\n" \
                                          "memory used (kb): %d\n"\
                                          "=========================================\n" \
                                          "Sensor model: \n" \
                                          "LIKELIHOOD_STD_DEV = %f\n" \
                                          "=========================================\n" \
                                          "Motion model: \n" \
                                          "ALPHA1 = %f\n" \
                                          "ALPHA2 = %f\n" \
                                          "ALPHA3 = %f\n" \
                                          "ALPHA4 = %f\n" \
                                          "MULTIPLIER = %f\n" \
                                          "=========================================\n" \
                                          "Resampling: \n" \
                                          "RESAMPLING_FREQUENCY = %d\n" \
                                          "RESAMPLING_TIME_MILLISECONDS = %d\n" \
                                          "=========================================\n" \

    , (int)NUM_OF_PARTICLES, stack+heap+psram,LIKELIHOOD_STD_DEV, ALPHA1, ALPHA2, ALPHA3, ALPHA4, NOISE_MULTIPLIER, RESAMPLING_FREQUENCY, RESAMPLING_TIME_MILLISECONDS);
    configStringMsg.data.data = configString;
    configStringMsg.data.size = char_used;

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

