#ifndef pid_effort_controller__20230223
#define pid_effort_controller__20230223

# include <controller_interface/controller.h>
# include <hardware_interface/joint_command_interface.h>
# include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <sensor_msgs/JointState.h>
# include <pluginlib/class_list_macros.h>

# include <ros/callback_queue.h>


namespace control
{

class PidEffortController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);



protected:

  ros::NodeHandle m_root_nh;
  ros::NodeHandle m_controller_nh;
  ros::CallbackQueue m_queue;
  ros::Subscriber m_target_subscriber;

  hardware_interface::EffortJointInterface* m_hw;
  hardware_interface::JointHandle m_jh;



  std::string m_joint_name;
  double m_position_reference;
  double m_velocity_reference;
  double m_effort_feedforward;
  double m_max_effort;
  double m_position_gain;  //Kp
  double m_velocity_gain;  //Kd
  double m_integral_gain;  //Ki
  double m_integral_state; //xi

  double m_pos;
  double m_vel;
  double m_eff;
  double m_position_error;
  double m_velocity_error;
  double m_effort_command;



  void callback(const sensor_msgs::JointStateConstPtr msg);
  bool extractJoint(const sensor_msgs::JointState msg,
                    const std::string name,
                    double& pos,
                    double& vel,
                    double& eff);

};


}

# endif
