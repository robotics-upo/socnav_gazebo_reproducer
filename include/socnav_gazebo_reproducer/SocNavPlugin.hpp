#ifndef SOCNAV_PLUGIN_HPP
#define SOCNAV_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace socnav_gazebo_reproducer
{
class SocNavPluginPrivate;

class SocNavPlugin : public gazebo::WorldPlugin
{
public:
  SocNavPlugin();
  virtual ~SocNavPlugin();
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;
  void Reset() override;

private:
  std::unique_ptr<SocNavPluginPrivate> impl_;
};

} // namespace socnav_gazebo_reproducer

#endif
