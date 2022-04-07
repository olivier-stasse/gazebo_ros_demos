#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {

  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    gazebo_ros::Node::SharedPtr ros_node = gazebo_ros::Node::Get(_sdf);
    
    // Make sure the ROS node for Gazebo has already been initialized
    if (!rclcpp::ok())
    {
      RCLCPP_FATAL(ros_node->get_logger(),
		   "A ROS node for Gazebo has not been initialized, unable to load plugin. ");
      return;
    }

    RCLCPP_INFO(ros_node->get_logger(), "Hello World!");
    
  }

};
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
