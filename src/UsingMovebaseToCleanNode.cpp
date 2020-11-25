#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include "CleaningPathPlanner.h"
#include "tf/tf.h"
#include <math.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "cleaning_using_movebase");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal nextGoal;

  //load the global path.
  tf::TransformListener tl_listener(ros::Duration(10));
  costmap_2d::Costmap2DROS lcr("cleaning_costmap", tl_listener);
  CleaningPathPlanning *pathPlanner = new CleaningPathPlanning(&lcr);

  //full coverage path.
  std::vector<geometry_msgs::PoseStamped> fullCoverPath = pathPlanner->GetPathInROS();


  std::vector<geometry_msgs::PoseStamped> better_points;
  for(int i = 0; i<(int)fullCoverPath.size()-3; i++)
  {
    std::vector<float> line_1(2);
    std::vector<float> line_2(2);
    line_1[0] = fullCoverPath[i].pose.position.x - fullCoverPath[i+1].pose.position.x;
    line_1[1] = fullCoverPath[i].pose.position.y - fullCoverPath[i+1].pose.position.y;

    line_2[0] = fullCoverPath[i+1].pose.position.x - fullCoverPath[i+2].pose.position.x;
    line_2[1] = fullCoverPath[i+1].pose.position.y - fullCoverPath[i+2].pose.position.y;

    float th = sqrt(line_1[0]*line_1[0] + line_1[1]*line_1[1]) * sqrt(line_2[0]*line_2[0] + line_2[1]*line_2[1]) ;
    float costheta = (line_1[0]*line_2[0] + line_1[1]*line_2[1])/ th;
    float theta = acos(costheta) / 3.1415 * 180;
    // std::cout<<theta<<std::endl;
    if(theta > 3 && theta < 177)
    {
      better_points.push_back(fullCoverPath[i+1]);
    }

  }
  //border tracing path.
//  std::vector<geometry_msgs::PoseStamped> borderTrackingPath = pathPlanner->GetBorderTrackingPathInROS();
//  for(int i = 0;i<borderTrackingPath.size();i++)
//  {
//      fullCoverPath.push_back(borderTrackingPath[i]);
//  }

  //main loop
  ros::Rate r(10);

  for(int i=0; i< fullCoverPath.size(); i++)
  {
  std::cout<<fullCoverPath[i].pose.position.x<<"*******"<<fullCoverPath[i].pose.position.y<<std::endl;
  }

  int det_flag = 0;
  for(int i = 0; i < better_points.size(); i++)
  {
      nextGoal.target_pose.header.frame_id = "map";
      nextGoal.target_pose.header.stamp = ros::Time::now();

      geometry_msgs::PoseStamped posestamped = better_points[i];

      //call move base to plan a long distance.
      nextGoal.target_pose.pose.position.x = posestamped.pose.position.x;
      nextGoal.target_pose.pose.position.y = posestamped.pose.position.y;
      nextGoal.target_pose.pose.position.z = 0;
      nextGoal.target_pose.pose.orientation.w = 1;
      nextGoal.target_pose.pose.orientation.x = 0;
      nextGoal.target_pose.pose.orientation.y = 0;
      nextGoal.target_pose.pose.orientation.z = posestamped.pose.orientation.z;

      ROS_INFO("Sending next goal!");
      ac.sendGoal(nextGoal);
      ac.waitForResult(ros::Duration(90));

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
          ROS_INFO("Hooray, the base moved a point forward in full path!");
        //   pathPlanner->SetCoveredGrid(posestamped.pose.position.x,posestamped.pose.position.y);
        //   pathPlanner->PublishGrid();
      }
      else
      {
          ROS_INFO("The base failed to move forward to the next path for some reason!");
          ac.cancelGoal();
          // continue;
      }

      if(det_flag == 10)
      {
        delete pathPlanner;
      }
      if(det_flag < 10){pathPlanner->PublishCoveragePath(); }
      det_flag = det_flag+1;
      ros::spinOnce();
      r.sleep();
    }
    

  return 0;
}
