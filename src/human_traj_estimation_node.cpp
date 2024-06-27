#include <human_traj_estimation/human_traj_estimation.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <franka_msgs/FrankaState.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_estimation_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  TrajEstimator te(nh);

  std::string wrench_topic, dwrench_topic, vel_topic, pos_topic, assistance_topic, reference_traj, update_Kest;

  if (!nh.getParam( "wrench_topic", wrench_topic))
  {
    wrench_topic = "filtered_wrench_base";
    ROS_WARN_STREAM (nh.getNamespace() << " /wrench_topic not set. default " << wrench_topic );
  }
  if ( !nh.getParam ( "dwrench_topic", dwrench_topic) )
  {
    dwrench_topic = "delta_force";
    ROS_WARN_STREAM (nh.getNamespace() << " /dwrench_topic not set. default " << dwrench_topic );
  }
    if ( !nh.getParam ( "vel_topic", vel_topic) )
  {
    vel_topic = "current_velocity";
    ROS_WARN_STREAM (nh.getNamespace() << " /vel_topic not set. default "<< vel_topic);
  }
  if ( !nh.getParam ( "pos_topic", pos_topic) )
  {
    pos_topic = "current_pose";
    ROS_WARN_STREAM (nh.getNamespace() << " /pos_topic not set. default "<< pos_topic);
  }
  if ( !nh.getParam ( "assistance_topic", assistance_topic) )
  {
    assistance_topic = "assistance";
    ROS_WARN_STREAM (nh.getNamespace() << " /assistance_topic not set. default " << assistance_topic);
  }
  if ( !nh.getParam ( "reference_traj", reference_traj) )
  {
    reference_traj = "reference_traj";
    ROS_WARN_STREAM (nh.getNamespace() << " /reference_traj not set. default " << reference_traj);
  }
  bool robot_ref;
  if ( !nh.getParam ( "robot_has_human_reference", robot_ref) )
  {
    robot_ref = false;
    ROS_WARN_STREAM (nh.getNamespace() << " /robot_has_human_reference not set. default " << robot_ref);
  }
  if (!nh.getParam("update_Kest", update_Kest))
  {
    update_Kest = "update_Kest";
    ROS_WARN_STREAM (nh.getNamespace() << " /update_Kest not set. default " << update_Kest);
  }


  ros::Subscriber wrench_sub    = nh.subscribe(wrench_topic , 10, &TrajEstimator::wrenchCallback, &te);
  ros::Subscriber dwrench_sub   = nh.subscribe(dwrench_topic, 10, &TrajEstimator::dWrenchCallback, &te);
  ros::Subscriber velocity_sub  = nh.subscribe(vel_topic    , 10, &TrajEstimator::velocityCallback, &te);
  ros::Subscriber pose_sub      = nh.subscribe(pos_topic    , 10, &TrajEstimator::currPoseCallback, &te);
  ros::Subscriber alpha_sub     = nh.subscribe("/alpha"     , 10, &TrajEstimator::alphaCallback, &te);

  // Publishers part
  ros::Publisher assistance_pub = nh.advertise<geometry_msgs::TwistStamped>(assistance_topic,10);
  ros::Publisher trajectory_pub = nh.advertise<geometry_msgs::PoseStamped>(reference_traj,10);
  ros::Publisher r_trajectory_pub = nh.advertise<geometry_msgs::PoseStamped>("/target_cart_pose",10);
  
//   ros::ServiceServer update_Kest_server = nh.advertiseService(update_Kest, &TrajEstimator::updateKestSrv, &te);
  
  ros::ServiceServer reset_pose_srv = nh.advertiseService("reset_pose_estimation", &TrajEstimator::resetPose, &te);
  
  ros::Duration(0.1).sleep();
  
  ros::Rate rate(125);
  
  static tf::TransformBroadcaster br;

  // The inizializaion part is complete. Now the computational part starts
  std::cout << "Completed the inizialization part. Now the computational part starts...\n";

  while (ros::ok())
  {
    geometry_msgs::PoseStamped p;
    if (!te.updatePoseEstimate(p))
    {
      ROS_ERROR_STREAM_THROTTLE(5.0,"error in updating the estimated pose . Is the pose initialized?");
    }
    else
    {
      p.header.stamp = ros::Time::now();
      trajectory_pub.publish(p);
      if(robot_ref)
        r_trajectory_pub.publish(p);
      
      
      tf::Transform transform;
      
      tf::poseMsgToTF(p.pose,transform);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "human_trg_pose"));
      te.init_pos_ok = false;
    }
    
    ROS_INFO_STREAM_THROTTLE(5.0,"looping .");
    
    rate.sleep();
  }
  ros::waitForShutdown();
  
  return 0;
  
}

