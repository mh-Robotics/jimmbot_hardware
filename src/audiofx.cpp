#include "jimmbot_audiofx/audiofx.hpp"
#include <ros/package.h>

namespace jimmbot_audiofx
{
  AudioFX::AudioFX(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
  {
    this->extn_data_sub_ = nh->subscribe<jimmbot_msgs::ExtnDataStamped>("extn_data", 1, &AudioFX::extnDataMsgCallback, this);
    this->audio_buffer_pub_ = nh->advertise<audio_common_msgs::AudioData>("audio", 1, false);
  }
  
  void AudioFX::extnDataMsgCallback(const jimmbot_msgs::ExtnDataStamped::ConstPtr &extn_data_msg)
  {
    if(extn_data_msg->extn_data.horn && !is_called_)
    {
      this->is_called_ = true;
      //This should repeat when wav is finished.
      this->sound_client_.startWave(ros::package::getPath("jimmbot_hardware") + "/media/horn.wav");
    }
    else if(!extn_data_msg->extn_data.horn && is_called_)
    {
      this->is_called_ = false;
      this->sound_client_.stopAll();
    }
  }
}//end namespace jimmbot_audiofx
