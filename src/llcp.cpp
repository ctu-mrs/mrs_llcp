#include <ros/package.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <string>
#include <serial_port.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace llcp
{

/* class Llcp //{ */

class Llcp : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  /* enum serial_receiver_state */
  /* { */
  /*   WAITING_FOR_MESSSAGE, */
  /*   EXPECTING_SIZE, */
  /*   EXPECTING_PAYLOAD, */
  /*   EXPECTING_CHECKSUM */
  /* }; */


  /* ros::Timer serial_timer_; */
  /* ros::Timer fake_timer_; */
  /* ros::Timer maintainer_timer_; */

  /* ros::ServiceServer ser_send_int; */
  /* ros::ServiceServer ser_send_int_raw; */

  /* void interpretSerialData(uint8_t data); */
  /* void callbackSerialTimer(const ros::TimerEvent &event); */
  /* void callbackFakeTimer(const ros::TimerEvent &event); */
  /* void callbackMaintainerTimer(const ros::TimerEvent &event); */

  /* bool callbackNetgunSafe(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res); */
  /* bool callbackNetgunArm(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res); */
  /* bool callbackNetgunFire(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res); */
  /* void callbackSendMessage(const mrs_msgs::LlcpConstPtr &msg); */
  /* void callbackSendRawMessage(const mrs_msgs::SerialRawConstPtr &msg); */
  /* void callbackMagnet(const std_msgs::EmptyConstPtr &msg); */

  /* bool callbackSendInt([[maybe_unused]] mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res); */
  /* bool callbackSendIntRaw([[maybe_unused]] mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res); */


  /* uint8_t connectToSensor(void); */
  /* void    processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct); */


  ros::NodeHandle nh_;

  /* ros::Publisher range_publisher_A_; */
  /* ros::Publisher range_publisher_B_; */
  /* ros::Publisher llcp_publisher_; */

  /* ros::Subscriber raw_message_subscriber; */
  /* ros::Subscriber llcp_subscriber; */
  /* ros::Subscriber magnet_subscriber; */

  /* serial_port::SerialPort serial_port_; */

  /* boost::function<void(uint8_t)> serial_data_callback_function_; */

  /* ros::Time interval_      = ros::Time::now(); */
  /* ros::Time last_received_ = ros::Time::now(); */
};

//}

/* onInit() //{ */

void Llcp::onInit() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  ROS_INFO("[Llcp]: node initialized");
  ROS_INFO("[Llcp]: pes steka panove");

  /* nh_.param("uav_name", uav_name_, std::string("uav")); */
  /* nh_.param("portname", portname_, std::string("/dev/ttyUSB0")); */
  /* nh_.param("baudrate", baudrate_, 115200); */
  /* nh_.param("publish_bad_checksum", publish_bad_checksum, false); */
  /* nh_.param("simulate_fake_garmin", simulate_fake_garmin, false); */
  /* nh_.param("use_timeout", use_timeout, true); */
  /* nh_.param("swap_garmins", swap_garmins, false); */
  /* nh_.param("serial_rate", serial_rate_, 5000); */
  /* nh_.param("serial_buffer_size", serial_buffer_size_, 1024); */

  /* ser_send_int     = nh_.advertiseService("send_int", &Llcp::callbackSendInt, this); */
  /* ser_send_int_raw = nh_.advertiseService("send_int_raw", &Llcp::callbackSendIntRaw, this); */

  /* // Publishers */
  /* std::string postfix_A = swap_garmins ? "_up" : ""; */
  /* std::string postfix_B = swap_garmins ? "" : "_up"; */
  /* range_publisher_A_    = nh_.advertise<sensor_msgs::Range>("range" + postfix_A, 1); */
  /* range_publisher_B_    = nh_.advertise<sensor_msgs::Range>("range" + postfix_B, 1); */
  /* garmin_A_frame_       = uav_name_ + "/garmin" + postfix_A; */
  /* garmin_B_frame_       = uav_name_ + "/garmin" + postfix_B; */

  /* llcp_publisher_ = nh_.advertise<mrs_msgs::Llcp>("baca_protocol_out", 1); */

  /* llcp_subscriber = nh_.subscribe("baca_protocol_in", 10, &Llcp::callbackSendMessage, this, ros::TransportHints().tcpNoDelay()); */

  /* raw_message_subscriber = nh_.subscribe("raw_in", 10, &Llcp::callbackSendRawMessage, this, ros::TransportHints().tcpNoDelay()); */

  /* // Output loaded parameters to console for double checking */
  /* ROS_INFO_THROTTLE(1.0, "[%s] is up and running with the following parameters:", ros::this_node::getName().c_str()); */
  /* ROS_INFO_THROTTLE(1.0, "[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str()); */
  /* ROS_INFO_THROTTLE(1.0, "[%s] baudrate: %i", ros::this_node::getName().c_str(), baudrate_); */
  /* ROS_INFO_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName().c_str() << "] publishing messages with wrong checksum: " << publish_bad_checksum); */

  /* connectToSensor(); */

  /* serial_timer_     = nh_.createTimer(ros::Rate(serial_rate_), &Llcp::callbackSerialTimer, this); */
  /* fake_timer_       = nh_.createTimer(ros::Rate(fake_garmin_rate_), &Llcp::callbackFakeTimer, this); */
  /* maintainer_timer_ = nh_.createTimer(ros::Rate(1), &Llcp::callbackMaintainerTimer, this); */

  /* is_initialized_ = true; */
}
//}

} 

PLUGINLIB_EXPORT_CLASS(llcp::Llcp, nodelet::Nodelet);
