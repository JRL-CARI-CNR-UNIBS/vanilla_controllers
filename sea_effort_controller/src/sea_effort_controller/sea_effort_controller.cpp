#include <sea_effort_controller/sea_effort_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(control::SeaEffortController, controller_interface::ControllerBase);

namespace control
{


bool SeaEffortController::init(hardware_interface::RobotHW *hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_hw=hw;
  m_effort_hw=hw->get<hardware_interface::EffortJointInterface>();
  m_js_hw=hw->get<hardware_interface::JointStateInterface>();
  if (!controller_nh.getParam("controlled_joint",m_joint_name))
  {
    ROS_ERROR("controlled_joint is not defined");
    return false;
  }
  if (!controller_nh.getParam("link_joint",m_link_joint_name))
  {
    ROS_ERROR("link_joint is not defined");
    return false;
  }

  bool flag=false;

  ROS_INFO("effort hw has %zu handles",m_effort_hw->getNames().size());
  for (unsigned idx=0;idx<m_effort_hw->getNames().size();idx++)
  {
    ROS_INFO(" handle %s",m_effort_hw->getNames().at(idx).c_str() );
    if (!m_effort_hw->getNames().at(idx).compare(m_joint_name))
    {
      m_motor_joint_handle=m_effort_hw->getHandle(m_joint_name);
      flag=true;
      break;
    }
  }
  if (!flag)
  {
    ROS_ERROR("controlled_joint %s is not part of the hardware_interface",m_joint_name.c_str());
    return false;
  }

  flag=false;
  ROS_INFO("state hw has %zu handles",m_js_hw->getNames().size());
  for (unsigned idx=0;idx<m_js_hw->getNames().size();idx++)
  {
    if (!m_js_hw->getNames().at(idx).compare(m_link_joint_name))
    {
      m_link_joint_handle=m_js_hw->getHandle(m_link_joint_name);
      flag=true;
      break;
    }
  }
  if (!flag)
  {
    ROS_ERROR("link_joint %s is not part of the hardware_interface",m_link_joint_name.c_str());
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
  m_target_subscriber=m_controller_nh.subscribe<sensor_msgs::JointState>(setpoint_topic_name, 1,boost::bind(&SeaEffortController::callback,this,_1));

  ROS_DEBUG("Controller '%s' controls the following joint: %s",m_controller_nh.getNamespace().c_str(),m_joint_name.c_str());


  if (!m_controller_nh.getParam("maximum_torque",m_max_effort))
  {
    ROS_WARN("no maximum_torque specified for joint %s, set equal to zero",m_joint_name.c_str());
    m_max_effort=0;
  }

  if (!m_controller_nh.getParam("state_gains", m_state_gains))
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/'state_gains' does not exist");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (m_state_gains.size()!=4)
  {
    ROS_ERROR("%s/'state_gains' has dimension %zu instead of 4",m_controller_nh.getNamespace().c_str(),m_state_gains.size());
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }

  if (!m_controller_nh.getParam("integral_gain", m_integral_gain))
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/'integral_gain' does not exist");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }

  if (!m_controller_nh.getParam("use_difference_state", m_difference_state))
  {
    ROS_INFO_STREAM(m_controller_nh.getNamespace()+"/'use_difference_state' does not exist, set false");
    m_difference_state=false;
  }



  if (m_integral_gain<0.0)
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace()+"/'integral_gain' cannot be negative");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }

  ROS_INFO("Controller '%s' well initialized",m_controller_nh.getNamespace().c_str());


  m_state.resize(4,0.0);
  m_r.resize(4,0.0);
  return true;
}

void SeaEffortController::starting(const ros::Time& /*time*/)
{
  ros::Time t0=ros::Time::now();

  m_motor_pos=m_motor_joint_handle.getPosition();
  m_motor_vel=m_motor_joint_handle.getVelocity();
  m_motor_eff=m_motor_joint_handle.getEffort();
  m_link_pos=m_link_joint_handle.getPosition();
  m_link_vel=m_link_joint_handle.getVelocity();


  m_position_reference=m_link_pos;
  m_velocity_reference=0.0;
  m_effort_feedforward=0.0;

  m_state.at(0)=m_motor_pos;
  m_state.at(1)=m_link_pos;
  m_state.at(2)=m_motor_vel;
  m_state.at(3)=m_link_vel;

  if (m_difference_state)
  {
    m_r.at(1)=m_position_reference;
    m_r.at(3)=m_velocity_reference;
  }

  m_state.at(0)=m_motor_pos;
  m_state.at(1)=m_link_pos;
  m_state.at(2)=m_motor_vel;
  m_state.at(3)=m_link_vel;
  // state_feedback_term=K*(r-x)
  // torque= integral_state +state_feedback_term ==> integral_state=torque-state_feedback_term

  double state_feedback_term=0.0;
  for (size_t idx=0;idx<4;idx++)
    state_feedback_term+=m_state_gains.at(idx)*(m_r.at(idx)-m_state.at(idx));

  m_integral_state=m_motor_eff-state_feedback_term;
  ROS_DEBUG("Controller '%s' started in %f seconds",m_controller_nh.getNamespace().c_str(),(ros::Time::now()-t0).toSec());
}

void SeaEffortController::stopping(const ros::Time& /*time*/)
{
  ROS_INFO("[ %s ] Stopping controller",  m_controller_nh.getNamespace().c_str());
  m_effort_command=0;
}

void SeaEffortController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
  try
  {
    m_queue.callAvailable();

    m_motor_pos=m_motor_joint_handle.getPosition();
    m_motor_vel=m_motor_joint_handle.getVelocity();
    m_motor_eff=m_motor_joint_handle.getEffort();

    m_link_pos=m_link_joint_handle.getPosition();
    m_link_vel=m_link_joint_handle.getVelocity();

    m_state.at(0)=m_motor_pos;
    m_state.at(1)=m_link_pos;
    m_state.at(2)=m_motor_vel;
    m_state.at(3)=m_link_vel;
    if (m_difference_state)
    {
      m_r.at(0)=m_position_reference;
      m_r.at(1)=m_position_reference;
      m_r.at(2)=m_velocity_reference;
      m_r.at(3)=m_velocity_reference;
    }

    double state_feedback_term=0.0;
    for (size_t idx=0;idx<4;idx++)
      state_feedback_term+=m_state_gains.at(idx)*(m_r.at(idx)-m_state.at(idx));

    m_effort_command=state_feedback_term+m_integral_state+m_effort_feedforward;

    m_position_error=m_position_reference-m_link_pos;

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

    ROS_INFO_THROTTLE(1,"command=%f, integral_state=%f, error=%f",m_effort_command,m_integral_state,m_position_error);
    m_motor_joint_handle.setCommand(m_effort_command);
  }
  catch (...)
  {
    ROS_WARN("something wrong: Controller '%s'",m_controller_nh.getNamespace().c_str());
    m_motor_joint_handle.setCommand(0);
  }
}

bool SeaEffortController::extractJoint(const sensor_msgs::JointState msg,
                                       const std::string name,
                                       double& pos,
                                       double& vel,
                                       double& eff)
{
  for (unsigned int iJoint=0;iJoint<msg.name.size();iJoint++)
  {
    if (!msg.name.at(iJoint).compare(name))
    {
      if (msg.effort.size()>(iJoint ))
      {
        pos=msg.position.at(iJoint);
        vel=msg.velocity.at(iJoint);
        eff=msg.effort.at(iJoint);
      }
      else
      {
        return false;
      }
      return true;
    }
  }
  return false;
}

void SeaEffortController::callback(const sensor_msgs::JointStateConstPtr msg)
{
  if (!extractJoint(*msg,
                   m_joint_name,
                   m_position_reference,
                   m_velocity_reference,
                   m_effort_feedforward))
  {
    ROS_ERROR_STREAM_THROTTLE(1,m_controller_nh.getNamespace()+" target message dimension is wrong");
  }
  return;
}


}
