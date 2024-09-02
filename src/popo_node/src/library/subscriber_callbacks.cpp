/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "../../include/popo_node/kobuki_ros.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void KobukiRos::subscribeDigitalOutputCommand(const driver_msgs::DigitalOutputConstPtr msg)
{
  // DigitalOutput digital_output;
  // for ( unsigned int i = 0; i < 4; ++i ) {
  //   digital_output.values[i] = msg->values[i];
  //   digital_output.mask[i] = msg->mask[i];
  // }
  // kobuki.setDigitalOutput(digital_output);
  // return;
}

void KobukiRos::subscribeExternalPowerCommand(const driver_msgs::ExternalPowerConstPtr msg)
{
  // Validate message
  // if (!((msg->source == driver_msgs::ExternalPower::PWR_3_3V1A) ||
  //       (msg->source == driver_msgs::ExternalPower::PWR_5V1A) ||
  //       (msg->source == driver_msgs::ExternalPower::PWR_12V5A) ||
  //       (msg->source == driver_msgs::ExternalPower::PWR_12V1_5A)))
  // {
  //   ROS_ERROR_STREAM("Kobuki : Power source " << (unsigned int)msg->source << " does not exist! [" << name << "].");
  //   return;
  // }
  // if (!((msg->state == driver_msgs::ExternalPower::OFF) ||
  //     (msg->state == driver_msgs::ExternalPower::ON)))
  // {
  //   ROS_ERROR_STREAM("Kobuki : Power source state "
  //       << (unsigned int)msg->state << " does not exist! [" << name << "].");
  //   return;
  // }

  // DigitalOutput digital_output;
  // for ( unsigned int i = 0; i < 4; ++i )
  // {
  //   if (i == msg->source)
  //   {
  //     if (msg->state)
  //     {
  //       digital_output.values[i] = true; // turn source on
  //       ROS_INFO_STREAM("Kobuki : Turning on external power source "
  //           << (unsigned int)msg->source << ". [" << name << "].");
  //     }
  //     else
  //     {
  //       digital_output.values[i] = false; // turn source off
  //       ROS_INFO_STREAM("Kobuki : Turning off external power source "
  //           << (unsigned int)msg->source << ". [" << name << "].");
  //     }
  //     digital_output.mask[i] = true; // change source state
  //   }
  //   else
  //   {
  //     digital_output.values[i] = false; // values doesn't matter here, since mask is set false, what means ignoring
  //     digital_output.mask[i] = false;
  //   }
  // }
  // kobuki.setExternalPower(digital_output);
  // return;
}

void KobukiRos::subscribeLed1Command(const driver_msgs::LedConstPtr msg)
{
  // switch( msg->value ) {
  // case driver_msgs::Led::GREEN:  kobuki.setLed(Led1, Green ); break;
  // case driver_msgs::Led::ORANGE: kobuki.setLed(Led1, Orange ); break; 
  // case driver_msgs::Led::RED:    kobuki.setLed(Led1, Red ); break;
  // case driver_msgs::Led::BLACK:  kobuki.setLed(Led1, Black ); break;
  // default: ROS_WARN_STREAM("Kobuki : led 1 command value invalid."); break;
  // }
  // return;
}

void KobukiRos::subscribeLed2Command(const driver_msgs::LedConstPtr msg)
{
  // switch( msg->value ) {
  // case driver_msgs::Led::GREEN:  kobuki.setLed(Led2, Green ); break;
  // case driver_msgs::Led::ORANGE: kobuki.setLed(Led2, Orange ); break;
  // case driver_msgs::Led::RED:    kobuki.setLed(Led2, Red ); break;
  // case driver_msgs::Led::BLACK:  kobuki.setLed(Led2, Black ); break;
  // default: ROS_WARN_STREAM("Kobuki : led 2 command value invalid."); break;
  // }
  // return;
}

void KobukiRos::subscribeMotorPower(const driver_msgs::MotorPowerConstPtr msg)
{
  // if (msg->state == driver_msgs::MotorPower::ON)
  // {
  //   ROS_INFO_STREAM("Kobuki : Firing up the motors. [" << name << "]");
  //   kobuki.enable();
  //   odometry.resetTimeout();
  // }
  // else if (msg->state == driver_msgs::MotorPower::OFF)
  // {
  //   kobuki.disable();
  //   ROS_INFO_STREAM("Kobuki : Shutting down the motors. [" << name << "]");
  //   odometry.resetTimeout();
  // }
  // else
  // {
  //   ROS_ERROR_STREAM("Kobuki : Motor power command specifies unknown state '" << (unsigned int)msg->state
  //                    << "'. [" << name << "]");
  // }
}

void KobukiRos::subscribeResetOdometry(const std_msgs::EmptyConstPtr /* msg */)
{
  // ROS_INFO_STREAM("Kobuki : Resetting the odometry. [" << name << "].");
  // joint_states.position[0] = 0.0; // wheel_left
  // joint_states.velocity[0] = 0.0;
  // joint_states.position[1] = 0.0; // wheel_right
  // joint_states.velocity[1] = 0.0;
  // odometry.resetOdometry();
  // kobuki.resetOdometry();
  // return;
}

void KobukiRos::subscribeVelocityCommand(const geometry_msgs::TwistPtr msg)
{ /mobile_base_nodelet_manager
  // if (kobuki.isEnabled())
  // {
  //   // For now assuming this is in the robot frame, but probably this
  //   // should be global frame and require a transform
  //   //double vx = msg->linear.x;        // in (m/s)
  //   //double wz = msg->angular.z;       // in (rad/s)
  //   ROS_DEBUG_STREAM("Kobuki : velocity command received [" << msg->linear.x << "],[" << msg->angular.z << "]");
  //   kobuki.setBaseControl(msg->linear.x, msg->angular.z);
  //   odometry.resetTimeout();
  // }
  // return;
}

// ဒီကောင်တွေက yoyo မှာရှိတယ်။ kobuki မှာ မရှိဘူး။
void KobukiRos::subscribeBatteryInfo(const std_msgs::EmptyConstPtr msg)
{ // pub by yoyo_nativeapp

}

void KobukiRos::subscribeCancelIap(const std_msgs::EmptyConstPtr msg)
{ // pub by yoyo_nativeapp

}

void KobukiRos::subscribeCurrentInfo(const std_msgs::Int32ConstPtr msg)
{ // pub by yoyo_nativeapp

}

void KobukiRos::subscribeForceStop(const std_msgs::Int32ConstPtr msg)
{ // pub by yoyo_nativeapp

}

void KobukiRos::subscribeSendToBase(const std_msgs::StringConstPtr msg)
{ // pub by yoyo_nativeapp

}

void KobukiRos::subscribeSetDocking(const std_msgs::Int32ConstPtr msg)
{ // publisher မရှိ။ ခလုတ်နှိပ်တဲ့အချိန်, low battery ဖြစ်ချိန်ဖြစ်နိုင်တယ်။

}

void KobukiRos::subscribeControllerIap(const std_msgs::StringConstPtr msg)
{ // pub by yoyo_nativeapp

}


} // namespace kobuki
