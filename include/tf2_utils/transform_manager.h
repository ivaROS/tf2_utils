#ifndef TF2_UTILS_TRANSFORM_MANAGER_H
#define TF2_UTILS_TRANSFORM_MANAGER_H

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

namespace tf2_utils
{

  class TransformManager :
  {
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::Listener> listener_;
  
public:
  
    TransformManager(double buffer_size=10) :
      buffer_(std::make_shared<tf2_ros::Buffer>(buffer_size))
      listener_(std::make_shared<tf2_ros::TransformListener>(*buffer))
    {}
    
    std::shared_ptr<tf2_ros::Buffer> getBuffer()
    {
      return buffer_;
    }
    
  };

} //end namespace tf2_utils

#endif //TF2_UTILS_TRANSFORM_MANAGER_H
