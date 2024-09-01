/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>
#include "popo_node/kobuki_ros.hpp"


namespace kobuki
{

class popoNodelet : public nodelet::Nodelet
{
public:
  popoNodelet() : shutdown_requested_(false) {};
  ~popoNodelet()  // ojbect ဖျက်တဲ့ အခါ thread ကို ပြီးအောင်စောင့်သွားမယ်။
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
    
    // callback() တွေကြောင့် latency issues တွေဖြစ်ရင် , callback process တချို့ကို  
    // multiple Thread တွေ ထပ်သုံးသင့်တယ်။ (use MTPrivateNodeHandle)
    if (kobuki_->init(this->getPrivateNodeHandle(), this->getNodeHandle()))
    {
      update_thread_.start(&popoNodelet::update, *this);
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
  // lib ဖြစ်တဲ့ KobukiRos class object ရဲ့ ptr
  // KobukiRos object တွင် init(), update(), authTimer(), 
  // splitString(), advertiseTopics(), subscribeTopics() စသည်ဖန်ရှင်များ ပါဝင်သလို
  // Objects များအနေဖြင့် Kobuki, joint_states, odometry များပါဝင်သည်။

  // var များအနေဖြင့်  cmd_vel_timed_out_, serial_timed_out_, auth_timer တို့ပါဝင်သည်။
  // ros အနေနဲ့ publisher ၂၀ ခု ၊ subscriber ၁၄ ခု ပါဝင်သည်။
  // publisher callback နဲ့ တွဲထားတဲ့ slot ၂၁ ခုပါဝင်တယ်။ slot_stream_data က publisher နဲ့ မတွဲတဲ့ slot အလွတ်
  // နောက် debug များလည်းပါဝင်သည်။
  boost::shared_ptr<KobukiRos> kobuki_; 

  ecl::Thread update_thread_;
  bool shutdown_requested_;
};

} // namespace kobuki

PLUGINLIB_EXPORT_CLASS(kobuki::popoNodelet, nodelet::Nodelet);
