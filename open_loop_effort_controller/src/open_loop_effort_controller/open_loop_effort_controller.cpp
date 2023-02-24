#include <open_loop_effort_controller/open_loop_effort_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(control::OpenLoopEffortController, controller_interface::ControllerBase);

namespace control
{


bool OpenLoopEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_hw=hw;
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

  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_controller_nh.setCallbackQueue(&m_queue);

  std::string setpoint_topic_name;

  if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  m_target_subscriber=m_controller_nh.subscribe<sensor_msgs::JointState>(setpoint_topic_name, 1,boost::bind(&OpenLoopEffortController::callback,this,_1));

  ROS_DEBUG("Controller '%s' controls the following joint: %s",m_controller_nh.getNamespace().c_str(),m_joint_name.c_str());


  if (!m_controller_nh.getParam("maximum_torque",m_max_effort))
  {
    ROS_WARN("no maximum_torque specified for joint %s, set equal to zero",m_joint_name.c_str());
    m_max_effort=0;
  }
  ROS_INFO("Controller '%s' well initialized",m_controller_nh.getNamespace().c_str());

  return true;
}

void OpenLoopEffortController::starting(const ros::Time& /*time*/)
{
  ros::Time t0=ros::Time::now();
  m_eff_cmd=0;
  ROS_DEBUG("Controller '%s' started in %f seconds",m_controller_nh.getNamespace().c_str(),(ros::Time::now()-t0).toSec());

}

void OpenLoopEffortController::stopping(const ros::Time& /*time*/)
{
  ROS_INFO("[ %s ] Stopping controller",  m_controller_nh.getNamespace().c_str());
  m_eff_cmd=0;
}

void OpenLoopEffortController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  try
  {
    m_queue.callAvailable();


    if (m_eff_cmd>m_max_effort)
    {
      m_eff_cmd=m_max_effort;
    }
    else if (m_eff_cmd<-m_max_effort)
    {
      m_eff_cmd=-m_max_effort;
    }

    m_jh.setCommand(m_eff_cmd);
  }
  catch (...)
  {
    ROS_WARN("something wrong: Controller '%s'",m_controller_nh.getNamespace().c_str());
    m_jh.setCommand(0);
  }

}

bool OpenLoopEffortController::extractJoint(const sensor_msgs::JointState msg, const std::string name, double& eff)
{
  for (unsigned int iJoint=0;iJoint<msg.name.size();iJoint++)
  {
    if (!msg.name.at(iJoint).compare(name))
    {
      if (msg.effort.size()>(iJoint ))
        eff=msg.effort.at(iJoint);
      else
        return false;

      return true;
    }
  }
  return false;
}

void OpenLoopEffortController::callback(const sensor_msgs::JointStateConstPtr msg)
{
  if (!extractJoint(*msg,m_joint_name,m_eff_cmd))
  {
    ROS_ERROR_STREAM_THROTTLE(1,m_controller_nh.getNamespace()+" target message dimension is wrong");
  }
  return;
}


}
