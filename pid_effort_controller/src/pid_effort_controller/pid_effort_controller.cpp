#include <pid_effort_controller/pid_effort_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(control::PidEffortController, controller_interface::ControllerBase);

namespace control
{


bool PidEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_hw=hw;
  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_controller_nh.setCallbackQueue(&m_queue);

  // Get controlled_joint and check if it is managed by EffortJointInterface
  if (!controller_nh.getParam("controlled_joint",m_joint_name))
  {
    ROS_ERROR("controlled_joint is not defined");
    return false;
  }
  bool flag=false;
  for (unsigned idx=0;idx<m_hw->getNames().size();idx++)
  {
    if (!m_hw->getNames().at(idx).compare(m_joint_name))
    {
      m_jh=m_hw->getHandle(m_joint_name);
      flag=true;
      break;
    }
  }
  if (!flag)
  {
    ROS_ERROR("controlled_joint is not part of the hardware_interface");
    return false;
  }


  // Subscribe setpoint topic
  std::string setpoint_topic_name;
  if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  m_target_subscriber=m_controller_nh.subscribe<sensor_msgs::JointState>(setpoint_topic_name, 1,boost::bind(&PidEffortController::callback,this,_1));
  /* NOTE
   *
   * boost::bind is the equivalent of the matlab @(x)function(x,2). Nowadays std::bind should be preferred, but ROS uses boost::bind for back-compatibility.
   *
   * Why we are using boost::bind?
   * 1) The subscribed wants a callback that receives only an argument (const sensor_msgs::JointStateConstPtr msg)
   * 2) 'this' is the pointer to the object (https://www.tutorialspoint.com/cplusplus/cpp_this_pointer.htm)
   * 3) C++ Methods implicitly received the object itself as first argument:
   *    this->callback(msg) is equivalent of calling PidEffortController::callback(this,msg) which has two arguments
   * 4) with the instruction boost::bind(&PidEffortController::callback,this,_1) we create an anonymous function
   *    with one argument (_1). This anonymous function calls PidEffortController::callback providing 'this' as
   *    the first argument and '_1' as the second one
  */

  ROS_DEBUG("Controller '%s' controls the following joint: %s",m_controller_nh.getNamespace().c_str(),m_joint_name.c_str());

  // Read controller parameters
  if (!m_controller_nh.getParam("maximum_torque",m_max_effort))
  {
    ROS_WARN("no maximum_torque specified for joint %s, set equal to zero",m_joint_name.c_str());
    m_max_effort=0;
  }

  if (!m_controller_nh.getParam("position_gain", m_position_gain))
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/'position_gain' does not exist");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (m_position_gain<0.0)
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/'position_gain' cannot be negative");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }

  if (!m_controller_nh.getParam("velocity_gain", m_velocity_gain))
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/'velocity_gain' does not exist");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (m_velocity_gain<0.0)
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/'velocity_gain' cannot be negative");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }

  if (!m_controller_nh.getParam("integral_gain", m_integral_gain))
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/'integral_gain' does not exist");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (m_integral_gain<0.0)
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/'integral_gain' cannot be negative");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }

  ROS_INFO("Controller '%s' well initialized",m_controller_nh.getNamespace().c_str());

  return true;
}


void PidEffortController::starting(const ros::Time& /*time*/)
{
  ros::Time t0=ros::Time::now();

  m_pos=m_jh.getPosition();
  m_vel=m_jh.getVelocity();
  m_eff=m_jh.getEffort();


  // initialize reference with actual position
  m_position_reference=m_pos;
  m_velocity_reference=0.0;
  m_effort_feedforward=0.0;

  // managed received messages
  m_queue.callAvailable();

  // compute tracking errors
  m_position_error=m_position_reference-m_pos;
  m_velocity_error=m_velocity_reference-m_vel;

  // initialized integral state to guarantee
  // Control action = Kp*position_error+Kv*velocity_error+xi+effort_feedforward == actual joint effort
  double pd_term=m_position_gain*m_position_error+m_velocity_gain*m_velocity_error+m_effort_feedforward;
  m_integral_state=m_eff-pd_term;
  ROS_DEBUG("Controller '%s' started in %f seconds",m_controller_nh.getNamespace().c_str(),(ros::Time::now()-t0).toSec());
}

void PidEffortController::stopping(const ros::Time& /*time*/)
{
  ROS_INFO("[ %s ] Stopping controller",  m_controller_nh.getNamespace().c_str());
}

void PidEffortController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
  try
  {
    // managed received messages
    m_queue.callAvailable();

    m_pos=m_jh.getPosition();
    m_vel=m_jh.getVelocity();
    m_eff=m_jh.getEffort();

    // compute tracking errors
    m_position_error=m_position_reference-m_pos;
    m_velocity_error=m_velocity_reference-m_vel;

    // compute command
    m_effort_command=m_position_gain*m_position_error+m_velocity_gain*m_velocity_error+m_integral_state+m_effort_feedforward;

    // check saturation and apply conditional integration
    if (m_effort_command>m_max_effort)
    {
      m_effort_command=m_max_effort;
      if (m_position_error<0.0)  // conditional integration
        m_integral_state+=period.toSec()*m_integral_gain*m_position_error;
    }
    else if (m_effort_command<-m_max_effort)
    {
      m_effort_command=-m_max_effort;
      if (m_position_error>0.0)  // conditional integration
        m_integral_state+=period.toSec()*m_integral_gain*m_position_error;
    }
    else
    {
      m_integral_state+=period.toSec()*m_integral_gain*m_position_error;
    }

    // apply command
    m_jh.setCommand(m_effort_command);
  }
  catch (...)
  {
    ROS_WARN_THROTTLE(0.1,"something wrong: Controller '%s'",m_controller_nh.getNamespace().c_str());
    m_jh.setCommand(0);
  }

}

bool PidEffortController::extractJoint(const sensor_msgs::JointState msg,
                                       const std::string name,
                                       double& pos,
                                       double& vel,
                                       double& eff)
{
  for (unsigned int iJoint=0;iJoint<msg.name.size();iJoint++)
  {
    // check if name is an element of msg.name
    if (!msg.name.at(iJoint).compare(name)) // compare return false if the two strings are equal
    {
      // check if the vectors have the right dimensions
      if (msg.position.size()>(iJoint) && msg.velocity.size()>(iJoint) && msg.effort.size()>(iJoint))
      {
        // get the values
        pos=msg.position.at(iJoint);
        vel=msg.velocity.at(iJoint);
        eff=msg.effort.at(iJoint);
        return true;
      }
    }
  }
  return false;
}

void PidEffortController::callback(const sensor_msgs::JointStateConstPtr msg)
{
  if (!extractJoint(*msg,
                   m_joint_name,
                   m_position_reference,
                   m_velocity_reference,
                   m_effort_feedforward))
  {
    ROS_ERROR_STREAM_THROTTLE(1,m_controller_nh.getNamespace()+" target message is wrong, skip it. msg =\n"<<msg);
  }
  return;
}


}
