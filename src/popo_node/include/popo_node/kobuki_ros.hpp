#ifndef KOBUKI_ROS_HPP_
#define KOBUKI_ROS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
// same headers in both yoyo and kobuki
#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <angles/angles.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ecl/sigslots.hpp>

// overwrite headers
#include <driver_msgs/ButtonEvent.h>          // #include <kobuki_msgs/ButtonEvent.h>
#include <driver_msgs/BumperEvent.h>          // #include <kobuki_msgs/BumperEvent.h>
#include <driver_msgs/CliffEvent.h>           // #include <kobuki_msgs/CliffEvent.h>
                                              // #include <kobuki_msgs/ControllerInfo.h>
#include <driver_msgs/DigitalOutput.h>        // #include <kobuki_msgs/DigitalOutput.h>
#include <driver_msgs/DigitalInputEvent.h>    // #include <kobuki_msgs/DigitalInputEvent.h>
#include <driver_msgs/ExternalPower.h>        // #include <kobuki_msgs/ExternalPower.h>
#include <driver_msgs/DockInfraRed.h>         // #include <kobuki_msgs/DockInfraRed.h>
#include <driver_msgs/Led.h>                  // #include <kobuki_msgs/Led.h>
#include <driver_msgs/MotorPower.h>           // #include <kobuki_msgs/MotorPower.h>
#include <driver_msgs/PowerSystemEvent.h>     // #include <kobuki_msgs/PowerSystemEvent.h>
#include <driver_msgs/RobotStateEvent.h>      // #include <kobuki_msgs/RobotStateEvent.h>
#include <driver_msgs/SensorState.h>          // #include <kobuki_msgs/SensorState.h>
                                              // #include <kobuki_msgs/Sound.h>
#include <driver_msgs/VersionInfo.h>          // #include <kobuki_msgs/VersionInfo.h>
#include <kobuki_driver/kobuki.hpp>           // #include <kobuki_driver/kobuki.hpp>
#include "odometry.hpp"                       // #include "odometry.hpp"
                                              // #include <kobuki_msgs/WheelDropEvent.h>
// New headers by yoyo
#include <driver_msgs/BatteryInfo.h>
#include <driver_msgs/CurrentInfo.h>
#include <driver_msgs/WheelStatus.h>
#include <driver_msgs/RangeSensorEvent.h>
#include <std_msgs/Int32.h>


/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{
class KobukiRos
{
public:
  KobukiRos(std::string& node_name);
  ~KobukiRos();
  bool init(ros::NodeHandle& nh, ros::NodeHandle& nh_pub);
  bool update();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  /*********************
   ** Variables
   **********************/
  std::string name; // name of the ROS node
  Kobuki kobuki;
  sensor_msgs::JointState joint_states;
  Odometry odometry;
  bool cmd_vel_timed_out_; // stops warning spam when cmd_vel flags as timed out more than once in a row
  bool serial_timed_out_; // stops warning spam when serial connection timed out more than once in a row
  ros::Timer auth_timer;   // timer က မူလ kobuki မှာမပါဘူး။

  /*********************
   ** Ros Comms
   **********************/
  // ဒီကောင်တွေက kobuki နဲ့ yoyo မှာရှိတယ်။
  ros::Publisher version_info_publisher, controller_info_publisher;
  ros::Publisher sensor_state_publisher, joint_state_publisher, dock_ir_publisher;
  ros::Publisher button_event_publisher, input_event_publisher, robot_event_publisher;
  ros::Publisher bumper_event_publisher, cliff_event_publisher, power_event_publisher; 
  ros::Publisher raw_data_command_publisher, raw_data_stream_publisher, raw_control_command_publisher;

  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
  // ဒီ publisher တွေက kobuki မှာ မရှိတော့ တိုက်ရိုက်ယူလို့မရနိုင်ဘူး။ မှီငြမ်းရေးဖို့လိုပါတယ်။ 
  ros::Publisher send_app_publisher, battery_event_publisher;
  ros::Publisher upgrade_result_publisher;
  ros::Publisher power_off_publisher;
  ros::Publisher battery_info_publisher;
  ros::Publisher current_info_publisher;
  ros::Publisher auth_info_publisher;
  ros::Publisher wheel_status_publisher;
  ros::Publisher range_sensor_publisher;

  // ဒီကောင်တွေက kobuki နဲ့ yoyo မှာရှိတယ်။
  ros::Subscriber velocity_command_subscriber, digital_output_command_subscriber, external_power_command_subscriber;
  ros::Subscriber controller_info_command_subscriber;
  ros::Subscriber led1_command_subscriber, led2_command_subscriber;
  ros::Subscriber motor_power_subscriber, reset_odometry_subscriber;

  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
  ros::Subscriber controller_iap_subscriber;
  ros::Subscriber force_stop_subscriber;
  ros::Subscriber set_docking_subscriber;
  ros::Subscriber battery_info_subscriber;
  ros::Subscriber current_info_subscriber;
  ros::Subscriber cancel_iap_subscriber;
  ros::Subscriber send_to_base_subscriber;

  // ဒီကောင်တွေက kobuki နဲ့ yoyo မှာရှိတယ်။
  void advertiseTopics(ros::NodeHandle& nh);
  void subscribeTopics(ros::NodeHandle& nh);
  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
  void authTimer(const ros::TimerEvent& e);
  std::vector<std::string> splitString(std::string srcStr, std::string delimStr,bool repeatedCharIgnored);

  /*********************
  ** Ros Callbacks
  **********************/
  // ဒီကောင်တွေက kobuki နဲ့ yoyo မှာရှိတယ်။
  void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr);
  void subscribeLed1Command(const driver_msgs::LedConstPtr);
  void subscribeLed2Command(const driver_msgs::LedConstPtr);
  void subscribeDigitalOutputCommand(const driver_msgs::DigitalOutputConstPtr);
  void subscribeExternalPowerCommand(const driver_msgs::ExternalPowerConstPtr);
  void subscribeResetOdometry(const std_msgs::EmptyConstPtr);
  void subscribeMotorPower(const driver_msgs::MotorPowerConstPtr msg);

  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
  void subscribeControllerIapCommand(const std_msgs::String::ConstPtr& msg);
  void subscribeForceStop(const std_msgs::Int32::ConstPtr& msg);
  void subscribeSetDocking(const std_msgs::Int32::ConstPtr& msg);
  void subscribeBatteryInfo(const std_msgs::Empty::ConstPtr& msg);
  void subscribeCurrentInfo(const std_msgs::Int32::ConstPtr& msg);
  void subscribeCancelIap(const std_msgs::Empty::ConstPtr& msg);
  void subscribeSendToBase(const std_msgs::String::ConstPtr& msg);

  /*********************
   ** SigSlots
   **********************/
  // ဒီကောင်တွေက kobuki နဲ့ yoyo မှာရှိတယ်။
  ecl::Slot<const VersionInfo&> slot_version_info;
  ecl::Slot<> slot_stream_data;
  ecl::Slot<const ButtonEvent&> slot_button_event;
  ecl::Slot<const BumperEvent&> slot_bumper_event;
  ecl::Slot<const CliffEvent&>  slot_cliff_event;
  ecl::Slot<const PowerEvent&>  slot_power_event;
  ecl::Slot<const InputEvent&>  slot_input_event;
  ecl::Slot<const RobotEvent&>  slot_robot_event;
  ecl::Slot<const std::string&> slot_debug, slot_info, slot_warn, slot_error;
  ecl::Slot<const std::vector<std::string>&> slot_named;
  ecl::Slot<Command::Buffer&> slot_raw_data_command;
  ecl::Slot<PacketFinder::BufferType&> slot_raw_data_stream;
  ecl::Slot<const std::vector<short>&> slot_raw_control_command;
  ecl::Slot<const WheelStatus&> slot_wheel_event; // msg type အနည်းငယ် ပြောင်းထားတယ်။

  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
  ecl::Slot<const PowerEvent&> slot_battery_event;
  ecl::Slot<const UpgradeEvent&>  slot_upgrade_event;
  ecl::Slot<const PowerOffEvent&> slot_power_off_event;
  ecl::Slot<const RangeSensorEvent&> slot_range_sensor_event;
  ecl::Slot<const BatteryInfoEvent&> slot_battery_info_event;
  ecl::Slot<const CurrentInfoEvent&> slot_current_info_event;
  ecl::Slot<PacketFinder::BufferType&> slot_send_app_steam;

  /*********************
   ** Slot Callbacks
   **********************/
  // ဒီကောင်တွေက kobuki နဲ့ yoyo မှာရှိတယ်။
  void publishVersionInfo(const VersionInfo &version_info);
  void processStreamData();
  void publishWheelState();
  void publishSensorState();
  void publishDockIRData();
  void publishControllerInfo();
  void publishButtonEvent(const ButtonEvent &event);
  void publishBumperEvent(const BumperEvent &event);
  void publishCliffEvent(const CliffEvent &event);
  void publishPowerEvent(const PowerEvent &event);
  void publishInputEvent(const InputEvent &event);
  void publishRobotEvent(const RobotEvent &event);
  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
  void publishWheelEvent(const WheelStatusEvent &event);
  void publishBatteryEvent(const PowerEvent &event);
  void publishUpgradeEvent(const UpgradeEvent &event);
  void publishPowerOffEvent(const PowerOffEvent &event);
  void publishRangeSensorEvent(const RangeSensorEvent &event);
  void publishBatteryInfoEvent(const BatteryInfoEvent &event);
  void publishCurrentInfoEvent(const CurrentInfoEvent &event);
  // မတွေ့
  void publishSendAppStream(); 
    
  // debugging
  // ဒီကောင်တွေက kobuki နဲ့ yoyo မှာရှိတယ်။
  void rosDebug(const std::string &msg) { ROS_DEBUG_STREAM("PoPo : " << msg); }
  void rosInfo(const std::string &msg) { ROS_INFO_STREAM("PoPo : " << msg); }
  void rosWarn(const std::string &msg) { ROS_WARN_STREAM("PoPo : " << msg); }
  void rosError(const std::string &msg) { ROS_ERROR_STREAM("PoPo : " << msg); }
  void rosNamed(const std::vector<std::string> &msgs) {
    if (msgs.size()==0) return;
    if (msgs.size()==1) { ROS_INFO_STREAM("PoPo : " << msgs[0]); }
    if (msgs.size()==2) {
      if      (msgs[0] == "debug") { ROS_DEBUG_STREAM("PoPo : " << msgs[1]); }
      else if (msgs[0] == "info" ) { ROS_INFO_STREAM ("PoPo : " << msgs[1]); }
      else if (msgs[0] == "warn" ) { ROS_WARN_STREAM ("PoPo : " << msgs[1]); }
      else if (msgs[0] == "error") { ROS_ERROR_STREAM("PoPo : " << msgs[1]); }
      else if (msgs[0] == "fatal") { ROS_FATAL_STREAM("PoPo : " << msgs[1]); }
    }
    if (msgs.size()==3) {
      if      (msgs[0] == "debug") { ROS_DEBUG_STREAM_NAMED(msgs[1], "PoPo : " << msgs[2]); }
      else if (msgs[0] == "info" ) { ROS_INFO_STREAM_NAMED (msgs[1], "PoPo : " << msgs[2]); }
      else if (msgs[0] == "warn" ) { ROS_WARN_STREAM_NAMED (msgs[1], "PoPo : " << msgs[2]); }
      else if (msgs[0] == "error") { ROS_ERROR_STREAM_NAMED(msgs[1], "PoPo : " << msgs[2]); }
      else if (msgs[0] == "fatal") { ROS_FATAL_STREAM_NAMED(msgs[1], "PoPo : " << msgs[2]); }
    }
  }

  // ဒီကောင်တွေက kobuki နဲ့ yoyo မှာရှိတယ်။
  void publishRawDataCommand(Command::Buffer &buffer);
  void publishRawDataStream(PacketFinder::BufferType &buffer);
  void publishSendApp(PacketFinder::BufferType &buffer);
  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
  void publishRawControlCommand(const std::vector<short> &velocity_commands);

  bool check_file(std::string file_path);
};

} // namespace kobuki

#endif /* KOBUKI_ROS_HPP_ */
