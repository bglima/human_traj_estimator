#include <human_traj_estimation/human_traj_estimation.h>
#include <human_traj_estimation/utils.h>
#include <ros/package.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <franka_msgs/FrankaState.h>


TrajEstimator::TrajEstimator(ros::NodeHandle nh)
:nh_(nh)
{
  w_b_     .setZero();
  w_bias_  .setZero();
  dW_      .setZero();
  velocity_.setZero();
  alpha_ = 0.95;
  init_pos_ok = false;
  first_cb_ = false;

  if (!nh.getParam( "wrench_topic", wrench_topic))
  {
    wrench_topic = "filtered_wrench_base";
    ROS_WARN_STREAM (nh.getNamespace() << " /wrench_topic not set. default " << wrench_topic );
  }
  
  if ( !nh_.getParam ( "sampling_time", dt_) )
  {
    dt_=0.008;
    ROS_WARN_STREAM (nh_.getNamespace() << " /sampling_time set. default : "<< dt_);
  }
  
  if ( !nh_.getParam ( "K_tras", K_tras_) )
  {
    K_tras_=0.0001;
    ROS_WARN_STREAM (nh_.getNamespace() << " /K_tras set. default : " << K_tras_);
  }
  
  if ( !nh_.getParam ( "K_rot", K_rot_) )
  {
    K_rot_=0.000001;
    ROS_WARN_STREAM (nh_.getNamespace() << " /K_tras set. default : " << K_rot_);
  }

  if ( !nh_.getParam ( "base_link", base_link_) )
  {
    base_link_="base_link";
    ROS_WARN_STREAM (nh_.getNamespace() << " /base_link set. default : " << base_link_);
  }
  if ( !nh_.getParam ( "tool_link", tool_link_) )
  {
    tool_link_="tool_link";
    ROS_WARN_STREAM (nh_.getNamespace() << " /tool_link set. default : " << tool_link_);
  }

  ROS_INFO_STREAM (nh_.getNamespace() << " /K_tras set to: " << K_tras_);
  ROS_INFO_STREAM (nh_.getNamespace() << " /K_rot set to : " << K_rot_);
  
  if ( !nh_.getParam ( "max_fl", max_fl_) )
  {
    max_fl_ = 0.007;
    ROS_WARN_STREAM (nh_.getNamespace() << " /max_fl set. default: " << max_fl_);
  }
  
}

Eigen::Vector6d TrajEstimator::getVel() {return velocity_;}
Eigen::Vector6d TrajEstimator::getDwrench() {return dW_;}

void TrajEstimator::wrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  // We expect the robot-felt force measurement.
  // Then, we convert it to work as the human-applied force
  w_b_(0) = -msg->wrench.force.x;
  w_b_(1) = -msg->wrench.force.y;
  w_b_(2) = -msg->wrench.force.z;
  w_b_(3) = -msg->wrench.torque.x;
  w_b_(4) = -msg->wrench.torque.y;
  w_b_(5) = -msg->wrench.torque.z;

  w_b_ += w_bias_;

  // Exctract the quaternion orientation
  Eigen::Quaterniond current_quaternion;
  tf2::fromMsg(cur_pos_.pose.orientation, current_quaternion);

  // Convert to a transformation in 3d
  Eigen::Isometry3d rotation_transform = Eigen::Isometry3d::Identity();
  rotation_transform.rotate(current_quaternion);

  // Extract Eigen vectors
  Eigen::Vector3d forces(w_b_.head(3));
  Eigen::Vector3d torques(w_b_.tail(3));

  // Update forces and torques
  forces = rotation_transform * forces;
  torques = rotation_transform * torques; // Assumes all the mass is concentrated at the EE frame

  // Give it back to the original attribute
  w_b_.head(3) = forces;
  w_b_.tail(3) = torques;
}

void TrajEstimator::alphaCallback(const std_msgs::Float32ConstPtr& msg)
{
  alpha_ = msg->data;
}

void TrajEstimator::dWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg )
{
  dW_(0) = msg->wrench.force.x;
  dW_(1) = msg->wrench.force.y;
  dW_(2) = msg->wrench.force.z;
  dW_(3) = msg->wrench.torque.x;
  dW_(4) = msg->wrench.torque.y;
  dW_(5) = msg->wrench.torque.z;
}


void TrajEstimator::velocityCallback(const geometry_msgs::TwistStampedConstPtr& msg )
{
  velocity_(0) = msg->twist.linear.x;
  velocity_(1) = msg->twist.linear.y;
  velocity_(2) = msg->twist.linear.z;
  velocity_(3) = msg->twist.angular.x;
  velocity_(4) = msg->twist.angular.y;
  velocity_(5) = msg->twist.angular.z; 
}

void TrajEstimator::currPoseCallback(const franka_msgs::FrankaStateConstPtr& msg)
{   


  // Modified part. cur_pos_ is the variable that needs to assigned after the procedure.
  // A matrix with the values passed by msg is initialized, recreating the rotation matrix 
  Eigen::Matrix3d rot_mat;
  rot_mat << msg->O_T_EE[0], msg->O_T_EE[4], msg->O_T_EE[8],
             msg->O_T_EE[1], msg->O_T_EE[5], msg->O_T_EE[9],
             msg->O_T_EE[2], msg->O_T_EE[6], msg->O_T_EE[10];

  // std::cout << "rot_mat: \n" << rot_mat << "\n";

  // Once the matrix has been defined, we need to transform it into a quaternion form
  Eigen::Quaterniond rot_mat_in_quaternion(rot_mat);

  // std::cout << "rot_mat_in_quaternion matrix form: \n" << rot_mat_in_quaternion.matrix() << "\n";
  // std::cout << "rot_mat_in_quaternion.vec() form: \n" << rot_mat_in_quaternion.vec() << "\n";
  // std::cout << "rot_mat_in_quaternion.w() form: \n" << rot_mat_in_quaternion.w() << "\n";

  // Assignment of the relevant parameters to a geometry_msgs::PoseStamped type
  cur_pos_.header = msg->header;
  cur_pos_.pose.position.x = msg->O_T_EE[12];
  cur_pos_.pose.position.y = msg->O_T_EE[13];
  cur_pos_.pose.position.z = msg->O_T_EE[14];
  cur_pos_.pose.orientation.x = rot_mat_in_quaternion.vec()[0];
  cur_pos_.pose.orientation.y = rot_mat_in_quaternion.vec()[1];
  cur_pos_.pose.orientation.z = rot_mat_in_quaternion.vec()[2];
  cur_pos_.pose.orientation.w = rot_mat_in_quaternion.w();

  if (!init_pos_ok)
  {
    init_pose_ = cur_pos_;
    last_pose_ = cur_pos_;
    init_pos_ok = true;
  }
  
  // tf2::fromMsg can convert a PoseStamped message to its equivalent tf2 representation.
  tf2::fromMsg(cur_pos_.pose, T_robot_base_targetpose_);

  // std::cout << "cur_pos_: \n" << cur_pos_ << "\n";
  // std::cout << "T_robot_base_target_pose_ matrix form: \n" << T_robot_base_targetpose_.matrix() << "\n";  
  // std::cout << "T_robot_base_target_pose_ translation form: \n" << T_robot_base_targetpose_.translation() << "\n";  
  // std::cout << "T_robot_base_target_pose_ rotation form: \n" << T_robot_base_targetpose_.rotation() << "\n";  
}

bool TrajEstimator::computeWrenchBias(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  nh_.getParam("n_of_wrench_bias_samples", n_of_wrench_bias_samples_);
  
  Eigen::Vector6d wrench_bias_cumulator = Eigen::Vector6d::Zero();

  ROS_INFO("wrench topic is: %s", wrench_topic.c_str());

  for (int i=0; i<n_of_wrench_bias_samples_; i++)
  {
    auto wrench_msg_ptr = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(wrench_topic, nh_);
    
    if(wrench_msg_ptr != NULL){
      
      geometry_msgs::WrenchStamped wrench_msg = *wrench_msg_ptr;

      Eigen::Vector6d wrench;
      

      wrench(0) = wrench_msg.wrench.force.x;
      wrench(1) = wrench_msg.wrench.force.y;
      wrench(2) = wrench_msg.wrench.force.z;
      wrench(3) = wrench_msg.wrench.torque.x;
      wrench(4) = wrench_msg.wrench.torque.y;
      wrench(5) = wrench_msg.wrench.torque.z;

      wrench_bias_cumulator += wrench;

    }
    else
    {
      ROS_ERROR("No message was received for the wrench bias computation!");
    }
  }

  w_bias_ = wrench_bias_cumulator / n_of_wrench_bias_samples_;

  ROS_INFO_STREAM("resetting wrench bias. this wrench: \n" << w_bias_);
  
  res.success=true;
  return true;
}



bool TrajEstimator::updatePoseEstimate(geometry_msgs::PoseStamped& ret)
{
  if (init_pos_ok)
  {
    
//     ret.pose.orientation = init_pose_.pose.orientation;
    if (alpha_>0.5)
      ret.pose = last_pose_.pose;
    else
      ret.pose = cur_pos_.pose;
    
    if(isnan(w_b_(0)))
      ROS_FATAL_STREAM("w_b_(0) : " << w_b_(0));
    if(isnan(w_b_(1)))
      ROS_FATAL_STREAM("w_b_(1) : " << w_b_(1));
    if(isnan(w_b_(2)))
      ROS_FATAL_STREAM("w_b_(2) : " << w_b_(2));
    if(isnan(w_b_(3)))
      ROS_FATAL_STREAM("w_b_(3) : " << w_b_(3));
    if(isnan(w_b_(4)))
      ROS_FATAL_STREAM("w_b_(4) : " << w_b_(4));
    if(isnan(w_b_(5)))
      ROS_FATAL_STREAM("w_b_(5) : " << w_b_(5));

    // std::cout << "w_b_:\n" << w_b_ << "\n";
    
    if (!isnan (w_b_(0)/std::fabs(w_b_(0))))
    {
      ret.pose.position.x += K_tras_ * w_b_(0);
      ret.pose.position.y += K_tras_ * w_b_(1);
      ret.pose.position.z += K_tras_ * w_b_(2);

      // I've modified this part since the already developed script had only the delta_z component for the upgrade of the rotation.
      double delta_x = K_rot_ * w_b_(3);
      double delta_y = K_rot_ * w_b_(4);
      double delta_z = K_rot_ * w_b_(5);

      Eigen::Quaterniond rotation_quaternion(T_robot_base_targetpose_.rotation());

      // std::cout << "rotation_quaternion matrix form: \n" << rotation_quaternion.matrix() << "\n";
      // std::cout << "rotation_quaternion.vec() form: \n" << rotation_quaternion.vec() << "\n";
      // std::cout << "rotation_quaternion.w() form: \n" << rotation_quaternion.w() << "\n";

      // A rotation matrix based on the product of all the three rotational component (defined as matrices starting from AngleAxis form) is defined.
      Eigen::Matrix3d m;
      m = Eigen::AngleAxisd(delta_x, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(delta_y, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(delta_z, Eigen::Vector3d::UnitZ());
      
      // std::cout << "m matrix: \n" << m << "\n";
      
      Eigen::Quaterniond additional_rotation_quaternion(m);

      // std::cout << "additional_rotation_quaternion matrix form: \n" << additional_rotation_quaternion.matrix() << "\n";
      // std::cout << "additional_rotation_quaternion.vec() form: \n" << additional_rotation_quaternion.vec() << "\n";
      // std::cout << "additional_rotation_quaternion.w() form: \n" << additional_rotation_quaternion.w() << "\n";

      // Upgrade of the rotation_matrix with the new values obtained from the human force/torque.
      rotation_quaternion = additional_rotation_quaternion * rotation_quaternion;

      // Updating the orientation part of the PoseStamped message type ret and printing it
      tf2::convert(rotation_quaternion, ret.pose.orientation);
      // ROS_INFO_STREAM("pose:\n"<<ret);
    }
    last_pose_ = ret;
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(5.0,"pose not initialized !");
    return false;
  }
  return true;
}




// bool TrajEstimator::updateKestSrv(pbo_service::updateKest::Request  &req,
//                                   pbo_service::updateKest::Response &res)
// {
//   K_tras_ = req.K_assist;
//   last_pose_ = cur_pos_;
//   ROS_INFO_STREAM("K_tras updated ! new K_tras_: " << K_tras_);
//   res.res = true;
//   return true;
// }




bool TrajEstimator::resetPose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  init_pos_ok = false;
//   tf::TransformListener listener_;
//   tf::StampedTransform transform_;
//   ROS_INFO_STREAM("reading transform from " << base_link_ << " to " << tool_link_);
//   listener_.waitForTransform(base_link_, tool_link_, ros::Time::now(), ros::Duration(1.0));
//   listener_.lookupTransform (base_link_, tool_link_, ros::Time(0)    , transform_);
//   
//   Eigen::Affine3d tmp;
//   tf::poseTFToEigen(transform_,tmp
//   tf::poseEigenToMsg (tmp, cur_pos_.pose);
  
  // TODO::might not be needed while, check
  while(!init_pos_ok)
  {
    ROS_INFO_STREAM_THROTTLE(1.0,"waiting for current pose to be read");
    ros::Duration(0.01);
  }
  init_pos_ok = true;
  
  init_pose_ = cur_pos_;
  last_pose_ = cur_pos_;
  
  ROS_INFO_STREAM("resetting estimation pose. this pose: \n" << init_pose_);
  
  res.success=true;
  return true;
}













