#include <open_loop_position_controller/open_loop_position_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(control::OpenLoopPositionController, controller_interface::ControllerBase);

namespace control
{


bool OpenLoopPositionController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
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
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/setpoint_topic_name does not exist");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  m_target_subscriber=m_controller_nh.subscribe<sensor_msgs::JointState>(setpoint_topic_name, 1,boost::bind(&OpenLoopPositionController::callback,this,_1));

  if (!m_controller_nh.getParam("default_value", m_default_value))
  {
    ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/default_value does not exist. It set to 0.");
    m_default_value = 0;
  }
  {
      ROS_INFO_STREAM("default_value: " << m_default_value);
  }

  ROS_DEBUG("Controller '%s' controls the following joint: %s",m_controller_nh.getNamespace().c_str(),m_joint_name.c_str());

  ROS_INFO("Controller '%s' well initialized",m_controller_nh.getNamespace().c_str());

  return true;
}

void OpenLoopPositionController::starting(const ros::Time& /*time*/)
{
  ros::Time t0=ros::Time::now();
  m_pos_cmd=m_default_value;
  ROS_DEBUG("Controller '%s' started in %f seconds",m_controller_nh.getNamespace().c_str(),(ros::Time::now()-t0).toSec());

}

void OpenLoopPositionController::stopping(const ros::Time& /*time*/)
{
  ROS_INFO("[ %s ] Stopping controller",  m_controller_nh.getNamespace().c_str());
  m_pos_cmd=0;
}

void OpenLoopPositionController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  try
  {
    m_queue.callAvailable();
    m_jh.setCommand(m_pos_cmd);
  }
  catch (...)
  {
    ROS_WARN("something wrong: Controller '%s'",m_controller_nh.getNamespace().c_str());
    m_jh.setCommand(0);
  }

}

bool OpenLoopPositionController::extractJoint(const sensor_msgs::JointState msg, const std::string name, double& pos)
{
  for (unsigned int iJoint=0;iJoint<msg.name.size();iJoint++)
  {
    if (!msg.name.at(iJoint).compare(name))
    {
      if (msg.position.size()>(iJoint ))
        pos=msg.position.at(iJoint);
      else
        return false;

      return true;
    }
  }
  return false;
}

void OpenLoopPositionController::callback(const sensor_msgs::JointStateConstPtr msg)
{
  if (!extractJoint(*msg,m_joint_name,m_pos_cmd))
  {
    ROS_ERROR_STREAM_THROTTLE(1,m_controller_nh.getNamespace()+" target message dimension is wrong");
  }
  return;
}


}
