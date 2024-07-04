#include <human_traj_estimation/human_traj_estimation.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <franka_msgs/FrankaState.h>
#include <std_msgs/Bool.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_estimation_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(5);
  spinner.start();

  TrajEstimator te(nh);  

  ros::Subscriber wrench_sub    = nh.subscribe(te.wrench_topic , 10, &TrajEstimator::wrenchCallback, &te);
  ros::Subscriber dwrench_sub   = nh.subscribe(te.dwrench_topic, 10, &TrajEstimator::dWrenchCallback, &te);
  ros::Subscriber velocity_sub  = nh.subscribe(te.vel_topic    , 10, &TrajEstimator::velocityCallback, &te);
  ros::Subscriber pose_sub      = nh.subscribe(te.pos_topic    , 10, &TrajEstimator::currPoseCallback, &te);
  ros::Subscriber alpha_sub     = nh.subscribe(te.alpha_topic  , 10, &TrajEstimator::alphaCallback, &te);

  // Publishers part
  ros::Publisher assistance_pub = nh.advertise<geometry_msgs::TwistStamped>(te.assistance_topic, 10);
  ros::Publisher trajectory_pub = nh.advertise<geometry_msgs::PoseStamped>(te.reference_traj_topic, 10);
  ros::Publisher r_trajectory_pub = nh.advertise<geometry_msgs::PoseStamped>("/target_cart_pose", 10);
  ros::Publisher bool_pub = nh.advertise<std_msgs::Bool>(te.bool_topic, 10);

  // Boolean variable definition
  std_msgs::Bool check_flag;
  
//   ros::ServiceServer update_Kest_server = nh.advertiseService(update_Kest, &TrajEstimator::updateKestSrv, &te);
  
  ros::ServiceServer reset_pose_srv = nh.advertiseService("reset_pose_estimation", &TrajEstimator::resetPose, &te);
  ros::ServiceServer compute_wrench_bias_srv = nh.advertiseService("compute_wrench_bias", &TrajEstimator::computeWrenchBias, &te);
  
  ros::Duration(0.1).sleep();
  
  ros::Rate rate(25);
  
  static tf::TransformBroadcaster br;

  // The inizializaion part is complete. Now the computational part starts
  std::cout << "Completed the inizialization part. Now the computational part starts...\n";

  while (ros::ok())
  {
    check_flag.data = te.init_pos_ok;
    bool_pub.publish(check_flag); // In this case, it is true

    geometry_msgs::PoseStamped updated_human_pose_stamped;
    
    if(!te.updatePoseEstimate(updated_human_pose_stamped))
    {
      ROS_ERROR_STREAM_THROTTLE(5.0,"error in updating the estimated pose . Is the pose initialized?");
    }
    else
    {
      updated_human_pose_stamped.header.stamp = ros::Time::now();
      trajectory_pub.publish(updated_human_pose_stamped);
      // check_flag.data = te.init_pos_ok;
      // bool_pub.publish(check_flag); // In this case, it is true
      
      tf::Transform human_reference_tf;
      tf::poseMsgToTF(updated_human_pose_stamped.pose, human_reference_tf);
      br.sendTransform(tf::StampedTransform(human_reference_tf, ros::Time::now(), te.base_link, "human_trg_pose"));
      // te.init_pos_ok = false;
      // check_flag.data = te.init_pos_ok;
      // bool_pub.publish(check_flag); // In this case, it is false
    }
    
    // ROS_INFO_STREAM_THROTTLE(5.0,"looping .");
    
    rate.sleep();
  }
  ros::waitForShutdown();
  
  return 0;
  
}

