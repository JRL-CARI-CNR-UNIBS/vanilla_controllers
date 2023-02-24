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

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param hw The specific hardware interface used by this controller.
   *
   * \param controller_nh A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  /** \brief This is called periodically by the realtime thread when the controller is running
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref update
   */
  virtual void update(const ros::Time& time, const ros::Duration& period) override;

  /** \name Real-Time Safe Functions
   *\{*/

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  virtual void starting(const ros::Time& time) override;

  /** \brief This is called from within the realtime thread just after the last
   * update call before the controller is stopped
   *
   * \param time The current time
   */
  virtual void stopping(const ros::Time& time) override;

protected:

  ros::NodeHandle m_root_nh;           // NodeHandle with hardware_interface namespace
  ros::NodeHandle m_controller_nh;     // NodeHandle with private namespace
  ros::Subscriber m_target_subscriber; // Subscribes the reference topic
  ros::CallbackQueue m_queue;          // the reference topic is managed by a private queue. New messages are processed by using m_queue.callAvailable()

  hardware_interface::EffortJointInterface* m_hw; // Interface for sending effort commands
  hardware_interface::JointHandle m_jh;           // Handle to read joint feedback and apply commands

  std::string m_joint_name;    // Name of the controlled joint
  double m_position_reference; // Position reference (received from the reference topic)
  double m_velocity_reference; // Velocity reference (received from the reference topic)
  double m_effort_feedforward; // Effort Feedforward (received from the reference topic)
  double m_max_effort;         // Maximum value of the controller command

  double m_pos;                // actual position of the joint
  double m_vel;                // actual velocity of the joint
  double m_eff;                // actual effort of the joint
  double m_position_error;     // position_reference-position
  double m_velocity_error;     // velocity_reference-velocity
  double m_effort_command;     // control action

  // Control action = Kp*position_error+Kv*velocity_error+xi+effort_feedforward
  // xi = xi + sample_period*Ki*(position_reference-position)
  double m_position_gain;      // Kp
  double m_velocity_gain;      // Kv
  double m_integral_gain;      // Ki
  double m_integral_state;     // xi

  /** \brief Callback for receiving a reference message (JointState type).
   * The reference is kept constant until a new message is received
   * Before the first message the actual joint position is kept constant
   *
   * \param msg the reference message
   */
  void callback(const sensor_msgs::JointStateConstPtr msg);


  /** \brief Method to extract the reference position/velocity/effort from the message
   * return false if message does not contain the controller joint
   *
   * \param msg the reference message
   * \param name the name of the desired joint
   * \param pos the position of the desired joint
   * \param vel the velocity of the desired joint
   * \param eff the effort of the desired joint
   *
   * \returns True if the desired joint is in the reference message
   */
  bool extractJoint(const sensor_msgs::JointState msg,
                    const std::string name,
                    double& pos,
                    double& vel,
                    double& eff);

};


}

# endif
