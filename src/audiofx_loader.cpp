#include "jimmbot_audiofx/audiofx_loader.hpp"

namespace jimmbot_audiofx
{
  AudioFxLoader::AudioFxLoader(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
  {
    audiofx_ = new AudioFX(nh, nh_param);
  }

}//end namespace jimmbot_audiofx

int main(int argc, char **argv)
{
  ros::init(argc, argv, "audio_effects_loader");

  ros::NodeHandle nh(""), nh_param("~");
  jimmbot_audiofx::AudioFxLoader audio_fx_loader(&nh, &nh_param);

  ros::spin();

  return EXIT_SUCCESS;
}