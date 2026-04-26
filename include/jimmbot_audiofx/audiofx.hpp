/**
 * @file audiofx.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-05-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef AUDIO_FX_H
#define AUDIO_FX_H

#include <ros/ros.h>

#include <mutex>

#include <sound_play/sound_play.h>
#include <audio_common_msgs/AudioData.h>
#include <jimmbot_msgs/ExtnDataStamped.h>
#include <jimmbot_audiofx/audiofx.hpp>

namespace jimmbot_audiofx
{
  class AudioFX
  {
  public:
    AudioFX(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

    /**
     * @brief 
     * 
     * @param extn_data_msg 
     */
    void extnDataMsgCallback(const jimmbot_msgs::ExtnDataStamped::ConstPtr &extn_data_msg);

  private:
    ros::Subscriber extn_data_sub_;
    ros::Publisher audio_buffer_pub_;
    sound_play::SoundClient sound_client_;
    bool is_called_{false};
  };
}//end namespace jimmbot_audiofx
#endif//end AUDIO_FX_H