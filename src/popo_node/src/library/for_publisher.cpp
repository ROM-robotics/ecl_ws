







  // yoyo နဲ့ kobuki မှာပါတယ်။
  version_info_publisher        = nh.advertise < driverr_msgs::VersionInfo > ("version_info",  100, true); // latched publisher
  button_event_publisher        = nh.advertise < driverr_msgs::ButtonEvent > ("events/button", 100);
  bumper_event_publisher        = nh.advertise < driverr_msgs::BumperEvent > ("events/bumper", 100);
  cliff_event_publisher         = nh.advertise < driverr_msgs::CliffEvent >  ("events/cliff",  100);
  power_event_publisher         = nh.advertise < std_msgs::Int32 > ("events/power_XXX", 100);
  input_event_publisher         = nh.advertise < driverr_msgs::DigitalInputEvent > ("events/digital_input", 100);
  robot_event_publisher         = nh.advertise < driverr_msgs::RobotStateEvent > ("events/online", 100, true); // also latched
  // debug
  // info
  // warn
  // error
  // named
  raw_data_command_publisher    = nh.advertise < std_msgs::String > ("debug/raw_data_command", 100);
  raw_data_stream_publisher     = nh.advertise < std_msgs::String > ("debug/raw_data_stream", 100);
  raw_control_command_publisher = nh.advertise < std_msgs::Int16MultiArray > ("debug/raw_control_command", 100);
  wheel_status_publisher        = nh.advertise < driverr_msgs::WheelStatus > ("events/wheel_status", 100);// topic ရှိ, အမည်ပြောင်းပေးထားတယ်။ [driver_msgs/WheelStatus]

  // yoyo မှာရှိပြီး kobuki မှာမရရှိ။ တည်ဆောက်ပေးရန်လိုတယ်။
  battery_event_publisher       = nh.advertise < driver_msgs::PowerSystemEvent > ("events/battery", 100);
  upgrade_result_publisher      = nh.advertise < std_msgs::String > ("upgrade_result", 100);    
  power_off_publisher           = nh.advertise < std_msgs::Int32 > ("events/power_off", 100);
  range_sensor_publisher        = nh.advertise < driver_msgs::RangeSensorEvent  > ("events/range_sensor", 100);
  battery_info_publisher        = nh.advertise < driver_msgs::BatteryInfo > ("events/battery_info", 100);
  current_info_publisher        = nh.advertise < driver_msgs::CurrentInfo > ("events/current_info", 100);
  send_app_publisher            = nh.advertise < std_msgs::String > ("debug/send_data_app", 100);

  // kobuki မှာရှိပြီး yoyo မှာမပါပါ။ ဖျက်ရန်
  //sensor_state_publisher        = nh.advertise < driverr_msgs::SensorState > ("sensors/core", 100);
  //dock_ir_publisher             = nh.advertise < driverr_msgs::DockInfraRed > ("sensors/dock_ir", 100);
  //imu_data_publisher            = nh.advertise < sensor_msgs::Imu > ("sensors/imu_data", 100);
  //raw_imu_data_publisher        = nh.advertise < sensor_msgs::Imu > ("sensors/imu_data_raw", 100);


    slot_version_info(&KobukiRos::publishVersionInfo, *this),
    slot_button_event(&KobukiRos::publishButtonEvent, *this),
    slot_bumper_event(&KobukiRos::publishBumperEvent, *this),
    slot_cliff_event(&KobukiRos::publishCliffEvent, *this),
    slot_power_event(&KobukiRos::publishPowerEvent, *this),
    slot_input_event(&KobukiRos::publishInputEvent, *this),
    slot_robot_event(&KobukiRos::publishRobotEvent, *this),
    slot_debug(&KobukiRos::rosDebug, *this),
    slot_info(&KobukiRos::rosInfo, *this),
    slot_warn(&KobukiRos::rosWarn, *this),
    slot_error(&KobukiRos::rosError, *this),
    slot_named(&KobukiRos::rosNamed, *this),
    slot_raw_data_command(&KobukiRos::publishRawDataCommand, *this),
    slot_raw_data_stream(&KobukiRos::publishRawDataStream, *this),
    slot_raw_control_command(&KobukiRos::publishRawControlCommand, *this),
    slot_wheel_event(&KobukiRos::publishWheelEvent, *this),
    slot_battery_event(&KobukiRos::publishBatteryEvent, *this),
    slot_upgrade_event(&KobukiRos::publishUpgradeEvent, *this),
    slot_power_off_event(&KobukiRos::publishPowerOffEvent, *this),
    slot_range_sensor_event(&KobukiRos::publishRangeSensorEvent, *this),
    slot_battery_info_event(&KobukiRos::publishBatteryInfoEvent, *this),
    slot_current_info_event(&KobukiRos::publishCurrentInfoEvent, *this),
    slot_send_app_steam(&KobukiRos::publishSendAppStream, *this)