/**
 * @file audiofx_loader.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-05-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef AUDIO_FX_LOADER_H
#define AUDIO_FX_LOADER_H

#include "audiofx.hpp"

namespace jimmbot_audiofx
{
  class AudioFxLoader
  {
  public:
    AudioFxLoader(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

  private:
    AudioFX* audiofx_;
  };
}//end namespace jimmbot_audiofx
#endif//end AUDIO_FX_H