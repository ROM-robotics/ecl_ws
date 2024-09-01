/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>
#include "bobo_node/kobuki_ros.hpp"


namespace kobuki
{

class boboNodelet : public nodelet::Nodelet
{
public:
  boboNodelet() : shutdown_requested_(false) {};
  ~boboNodelet()  // ojbect ဖျက်တဲ့ အခါ thread ကို ပြီးအောင်စောင့်သွားမယ်။
  { 
    NODELET_DEBUG_STREAM("Kobuki : waiting for update thread to finish.");
    shutdown_requested_ = true;
    update_thread_.join();
  }
  virtual void onInit()
  {
    NODELET_DEBUG_STREAM("Kobuki : initialising nodelet...");
    std::string nodelet_name = this->getName();
    kobuki_.reset(new KobukiRos(nodelet_name));
    
    // if there are latency issues with callbacks, we might want to move to process callbacks 
    // in multiple threads (use MTPrivateNodeHandle)
    if (kobuki_->init(this->getPrivateNodeHandle(), this->getNodeHandle()))
    {
      update_thread_.start(&boboNodelet::update, *this);
      NODELET_INFO_STREAM("Kobuki : initialised.");
    }
    else
    {
      NODELET_ERROR_STREAM("Kobuki : could not initialise! Please restart.");
    }
  }
private:
  // shutdown_request true , ros::ok() , kobukiRos ရဲ့ update လဲ true ဖြစ်ရင် 
  // 10 Hz နှုန်းနဲ့ update, sleep လုပ်နေမှာဖြစ်တယ်။
  void update()  
  {
    ros::Rate spin_rate(10);
    while (!shutdown_requested_ && ros::ok() && kobuki_->update())
    {
      spin_rate.sleep();
    }
  }

  boost::shared_ptr<KobukiRos> kobuki_; // lib ဖြစ်တဲ့ KobukiRos class object ရဲ့ ptr
  ecl::Thread update_thread_;
  bool shutdown_requested_;
};

} // namespace kobuki

PLUGINLIB_EXPORT_CLASS(kobuki::boboNodelet, nodelet::Nodelet);
