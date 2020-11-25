#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <math.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <tx2_cover_clean/FullCoveredPathPlanner.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cleaning_using_movebase");

  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal nextGoal;

  tf::TransformListener tl_listener(ros::Duration(10));
  costmap_2d::Costmap2DROS lcr("cleaning_costmap", tl_listener);
  FullCoveredPathPlanner *fullpathPlanner = new FullCoveredPathPlanner(&lcr);

  // std::map<int, std::vector<geometry_msgs::PoseStamped> > fullCoverPath = fullpathPlanner->GetMultiPathsInRos();
  // std::map<int, std::vector<geometry_msgs::PoseStamped> > nostopPath = fullpathPlanner->GetNoStopPaths(fullCoverPath);

  std::map<int, std::vector<geometry_msgs::PoseStamped> > OderPath = fullpathPlanner->GetOderPathsInRos();
  std::map<int, std::vector<geometry_msgs::PoseStamped> > nostopOderPath = fullpathPlanner->GetNoStopPaths(OderPath);
  std::vector<int> clean_oder = fullpathPlanner->GetRegionOder();

  std::cout<<"**&&&& init is normal **&&&"<<std::endl;
  for(int i = 0; i<clean_oder.size(); i++)
  {
      std::cout<<"the best "<< i << "clean oder is: "<<clean_oder[i]<<std::endl;
  }
  for(std::map<int, std::vector<geometry_msgs::PoseStamped> >::iterator iter = nostopOderPath.begin();
      iter != nostopOderPath.end();
      iter++)
  {
    std::cout<<"the region in map path"<<iter->first << std::endl;
  }

  std::cout<<"the oderpath size"<<OderPath.size()<<std::endl;

  ros::Rate r(10);

  for(int i = 0; i<clean_oder.size(); i++)
  {
    for(int j = 0; j<nostopOderPath[clean_oder[i]].size(); j++)
    {
        nextGoal.target_pose.header.frame_id = "map";
        nextGoal.target_pose.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped posestamped = nostopOderPath[clean_oder[i]][j];

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
        }
        else
        {
            ROS_INFO("The base failed to move forward to the next path for some reason!");
        }

        fullpathPlanner->publishOderPlan(OderPath);
        std::cout<<"**&&&& publish is normal ! **&&&"<<std::endl;

        ros::spinOnce();
        r.sleep();
    }
  }


/*  int det_flag = 0;
  std::map<int, std::vector<geometry_msgs::PoseStamped> >::iterator iter;
  for(iter = nostopPath.begin(); iter!= nostopPath.end(); iter++)
  {
    
    for(int i = 0; i < (iter->second).size(); i++)
    {
        nextGoal.target_pose.header.frame_id = "map";
        nextGoal.target_pose.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped posestamped = (iter->second)[i];

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
          delete fullpathPlanner;
        }
        if(det_flag < 10)
        {
          // fullpathPlanner->publishPlan(fullCoverPath); 
          fullpathPlanner->publishOderPlan(OderPath);
        }
        det_flag = det_flag+1;
        ros::spinOnce();
        r.sleep();
      }
  }*/

  return 0;
}
