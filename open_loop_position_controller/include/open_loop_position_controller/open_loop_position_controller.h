#ifndef open_loop_position_controller__20188101642
#define open_loop_position_controller__20188101642

# include <controller_interface/controller.h>
# include <hardware_interface/joint_command_interface.h>
# include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <sensor_msgs/JointState.h>
# include <pluginlib/class_list_macros.h>

# include <ros/callback_queue.h>


namespace control
{

class OpenLoopPositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

protected:

  ros::NodeHandle m_root_nh;
  ros::NodeHandle m_controller_nh;
  ros::Subscriber m_target_subscriber;
  ros::CallbackQueue m_queue;

  hardware_interface::PositionJointInterface* m_hw;
  hardware_interface::JointHandle m_jh;

  std::string m_joint_name;
  double m_pos_cmd;
  double m_default_value;

  void callback(const sensor_msgs::JointStateConstPtr msg);
  bool extractJoint(const sensor_msgs::JointState msg, const std::string name, double& pos);
};


}

# endif
