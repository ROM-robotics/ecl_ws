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
#include <driver_msgs/CliffEvent.h>           // #include <kobuki_msgs/CliffEvent.h> အတိအကျမတူ
                                              // #include <kobuki_msgs/ControllerInfo.h>
#include <driver_msgs/DigitalOutput.h>        // #include <kobuki_msgs/DigitalOutput.h>
#include <driver_msgs/DigitalInputEvent.h>    // #include <kobuki_msgs/DigitalInputEvent.h>
#include <driver_msgs/ExternalPower.h>        // #include <kobuki_msgs/ExternalPower.h>
#include <driver_msgs/DockInfraRed.h>         // #include <kobuki_msgs/DockInfraRed.h>
#include <driver_msgs/Led.h>                  // #include <kobuki_msgs/Led.h>
#include <driver_msgs/MotorPower.h>           // #include <kobuki_msgs/MotorPower.h>
#include <driver_msgs/PowerSystemEvent.h>     // #include <kobuki_msgs/PowerSystemEvent.h> သိပ်မတူ
#include <driver_msgs/RobotStateEvent.h>      // #include <kobuki_msgs/RobotStateEvent.h>
#include <driver_msgs/SensorState.h>          // #include <kobuki_msgs/SensorState.h> အကြမ်းဖျဥ်းတူ
                                              // #include <kobuki_msgs/Sound.h>
#include <driver_msgs/VersionInfo.h>          // #include <kobuki_msgs/VersionInfo.h> အကြမ်းဖျဥ်းတူ
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
  ros::Publisher version_info_publisher;
  ros::Publisher button_event_publisher; 
  ros::Publisher bumper_event_publisher;
  ros::Publisher cliff_event_publisher; 
  ros::Publisher power_event_publisher;
  ros::Publisher input_event_publisher;
  ros::Publisher robot_event_publisher;
  ros::Publisher joint_state_publisher;
  ros::Publisher sensor_state_publisher;
  ros::Publisher dock_ir_publisher;
  
  ros::Publisher raw_data_command_publisher;
  ros::Publisher raw_data_stream_publisher;
  ros::Publisher raw_control_command_publisher;
  ros::Publisher wheel_status_publisher;

  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
  // ဒီ publisher တွေက kobuki မှာ မရှိတော့ တိုက်ရိုက်ယူလို့မရနိုင်ဘူး။ မှီငြမ်းရေးဖို့လိုပါတယ်။ 
  ros::Publisher battery_event_publisher;
  ros::Publisher upgrade_result_publisher;
  ros::Publisher power_off_publisher;
  ros::Publisher range_sensor_publisher;
  ros::Publisher battery_info_publisher;
  ros::Publisher current_info_publisher;
  ros::Publisher send_app_publisher;

  
  ros::Publisher auth_info_publisher;
  ros::Publisher controller_info_publisher;
  //joint_state pub သာရှိ slot,slotcb() မရှိပါ။
  //Publisher စုစုပေါင်း ၂၃ ခု 

  // ဒီကောင်တွေက kobuki နဲ့ yoyo မှာရှိတယ်။
  ros::Subscriber digital_output_command_subscriber; 
  ros::Subscriber external_power_command_subscriber;
  ros::Subscriber led1_command_subscriber; 
  ros::Subscriber led2_command_subscriber;
  ros::Subscriber motor_power_subscriber; 
  ros::Subscriber reset_odometry_subscriber;
  ros::Subscriber velocity_command_subscriber; 
  // ဒါက yoyo မှာပါပေမဲ့ မသုံးဘူးထင်တယ်။
  ros::Subscriber controller_info_command_subscriber;
  
  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
  ros::Subscriber battery_info_subscriber;
  ros::Subscriber cancel_iap_subscriber;
  ros::Subscriber current_info_subscriber;
  ros::Subscriber force_stop_subscriber;
  ros::Subscriber send_to_base_subscriber;
  ros::Subscriber set_docking_subscriber;
  ros::Subscriber controller_iap_subscriber;

//Subscriber စုစုပေါင်း ၁၅ ခု

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

  //Subscribercallback စုစုပေါင်း ၁၄ ခု
  //ros::Subscriber controller_info_command_subscriber; သူ့တွက် subscriebercallbak() မရှိ
  

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
  ecl::Slot<const std::vector<std::string>&> slot_named; // slot,callback() ရှိ pub မရှိ
  ecl::Slot<Command::Buffer&> slot_raw_data_command;
  ecl::Slot<PacketFinder::BufferType&> slot_raw_data_stream;
  ecl::Slot<const std::vector<short>&> slot_raw_control_command;
  ecl::Slot<const WheelStatus&> slot_wheel_event; // msg type အနည်းငယ် ပြောင်းထားတယ်။
  //ecl::Slot<const WheelStatusEvent&> slot_wheel_status_event; 
  // msg မှာ WheelStatus ဘဲရှိတယ်။

  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
  ecl::Slot<const PowerEvent&> slot_battery_event;
  ecl::Slot<const UpgradeEvent&>  slot_upgrade_event;
  ecl::Slot<const PowerOffEvent&> slot_power_off_event;
  ecl::Slot<const RangeSensorEvent&> slot_range_sensor_event;
  ecl::Slot<const BatteryInfoEvent&> slot_battery_info_event;
  ecl::Slot<const CurrentInfoEvent&> slot_current_info_event;
  ecl::Slot<PacketFinder::BufferType&> slot_send_app_steam;

  //slot စုစုပေါင်း ၁၉+၅ = ၂၄ ခု
  // Publisher ၂၃ ခုရှိသော်လည်း အောက်ပါ pub ၄ ခုအတွက် Sigslots မရှိပါ။
  // ros::Publisher joint_state_publisher;
  // ros::Publisher dock_ir_publisher;
  // ros::Publisher auth_info_publisher;
  // ros::Publisher controller_info_publisher;

  /*********************
   ** Slot Callbacks
   **********************/
  // ဒီကောင်တွေက kobuki နဲ့ yoyo မှာရှိတယ်။
  zvoid publishVersionInfo(const VersionInfo &version_info);
  zvoid processStreamData();
  zvoid publishWheelState();
  void publishSensorState(); //slot မရှိ
  //joint_state က slot လဲမရှိ slotcallback လဲမရှိ 
  void publishDockIRData(); //slot မရှိ
  void publishControllerInfo(); //slot မရှိ
  zvoid publishButtonEvent(const ButtonEvent &event);
  zvoid publishBumperEvent(const BumperEvent &event);
  zvoid publishCliffEvent(const CliffEvent &event);
  zvoid publishPowerEvent(const PowerEvent &event);
  zvoid publishInputEvent(const InputEvent &event);
  zvoid publishRobotEvent(const RobotEvent &event);
  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။ slot_callbacks.cpp မှာ input parameters များပြင်ပြီး။
  void publishWheelStatusEvent(const WheelStatus &event);
  zvoid publishBatteryEvent(const PowerEvent &event);
  zvoid publishUpgradeEvent(const UpgradeEvent &event);
  zvoid publishPowerOffEvent(const PowerOffEvent &event);
  zvoid publishRangeSensorEvent(const RangeSensorEvent &event);
  zvoid publishBatteryInfoEvent(const BatteryInfoEvent &event);
  zvoid publishCurrentInfoEvent(const CurrentInfoEvent &event);

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
  zvoid publishRawDataCommand(Command::Buffer &buffer);
  zvoid publishRawDataStream(PacketFinder::BufferType &buffer);
  zvoid publishSendApp(PacketFinder::BufferType &buffer);
  // ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
  zvoid publishRawControlCommand(const std::vector<short> &velocity_commands);

  //slotcallback ၁၉+၃+၁+၅ = ၂၈ ခု
  //slot ၁၉ ခု (Pub,slot,slotcb)+ 
  //slot မရှိ CB() ရှိ ၃ ခု (Pub,NULL,slotcb)+ 
  //slot မရှိ CB() မရှိ ၁ခု <publishWheelStatusEvent> (NULL,NULL,slotcb)+ 
  //debug ၅ ခု (NULL,slot,slotcb)

 
 
  bool check_file(std::string file_path);

};

} // namespace kobuki

#endif /* KOBUKI_ROS_HPP_ */
