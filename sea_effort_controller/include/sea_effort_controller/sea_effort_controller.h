#ifndef sea_state_space_effort_controller__20230223
#define sea_state_space_effort_controller__20230223

# include <controller_interface/controller.h>
# include <controller_interface/multi_interface_controller.h>
# include <hardware_interface/joint_command_interface.h>
# include <thread>
# include <mutex>
# include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <sensor_msgs/JointState.h>
# include <pluginlib/class_list_macros.h>
# include <ros/callback_queue.h>


namespace control
{

class SeaEffortController : public controller_interface::MultiInterfaceController<
    hardware_interface::EffortJointInterface,
    hardware_interface::JointStateInterface>
{
public:
  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);


  std::string getJointName()
  {
    return m_joint_name;
  };

protected:
  ros::NodeHandle m_root_nh;
  ros::NodeHandle m_controller_nh;
  ros::Subscriber m_target_subscriber;

  hardware_interface::RobotHW* m_hw;
  hardware_interface::EffortJointInterface* m_effort_hw;
  hardware_interface::JointStateInterface* m_js_hw;

  std::string m_joint_name;
  hardware_interface::JointHandle m_motor_joint_handle;

  std::string m_link_joint_name;
  hardware_interface::JointStateHandle m_link_joint_handle;

  ros::CallbackQueue m_queue;

  double m_position_reference;
  double m_velocity_reference;
  double m_effort_feedforward;
  double m_max_effort;


  double m_motor_pos;
  double m_motor_vel;
  double m_link_pos;
  double m_link_vel;


  double m_position_error;
  double m_integral_state; //xi
  double m_integral_gain;  //Ki

  double m_motor_eff;
  double m_effort_command;

  bool m_difference_state;
  // if true  :  effort=K*(r-x)+xi
  //             with x=[motor_pos,link_pos,motor_vel,link_vel]
  //             and  r=[0,pos_reference,0,vel_reference]
  // if false :  effort=K*(r-x)+xi=-K*x+xi
  //             with x=[motor_pos,link_pos,motor_vel,link_vel]
  //             and  r=[0,0,0,0]
  std::vector<double> m_state;
  std::vector<double> m_state_gains;
  std::vector<double> m_r;


  void callback(const sensor_msgs::JointStateConstPtr msg);
  bool extractJoint(const sensor_msgs::JointState msg,
                    const std::string name,
                    double& pos,
                    double& vel,
                    double& eff);

};


}

# endif
