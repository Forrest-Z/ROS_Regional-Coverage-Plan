#ifndef FULLCLEANINGPATHPLANNING_H
#define FULLCLEANINGPATHPLANNING_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include<algorithm>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <costmap_2d_kinetic/costmap_2d.h>
#include <costmap_2d_kinetic/costmap_2d_ros.h>
#include <costmap_2d_kinetic/layered_costmap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace cv;

struct pointIndex
{
    int row;
    int col;
    double theta; 
};
struct Polygon
{
	std::vector<cv::Point> polygon;
};

class FullCoveredPathPlanner
{
public:
	FullCoveredPathPlanner(costmap_2d::Costmap2DROS *costmap2d_ros);
	FullCoveredPathPlanner(bool simple);

	virtual ~FullCoveredPathPlanner();

/*********************The main execution function *************************/
	void InitRosParameters();
	void InitCvCostImg();
	void InitRectRegions();
	void InitPolygonRegions();

	void CalcPathInCv();
	void CalcPathsInRos();
	void CalcCleanOrder();
	void InitRegionOder();

	void CheckBorder();


/******************* main fictions of planning the Path ********************/
	//Convert point coordinates in OpenCv to point coordinates in CostMap
	std::vector<pointIndex> CvImageToCostMap(std::vector<pointIndex> &cvpoints);

	//Plan the full coverage path in OpenCv with a given rectangle
	std::vector<pointIndex> FromRectGetPath(const cv::Rect &rect);
	std::vector<pointIndex> FromRectGetPathRightCorner(const cv::Rect &rect); //Ditto, but start planning from the top right

	//Plan the full coverage path in OpenCv with a given polygon
	std::vector<pointIndex> FromPolygonGetPath(const Polygon &pg);
	std::vector<pointIndex> FromPolygonGetPathRightCorner(const Polygon &pg);

	std::map<int, cv::Rect> AutoRectZoning();

	//Convert the path planned in OpenCv to the ROS standard path
	std::map<int, std::vector<geometry_msgs::PoseStamped> > CalcMultiRegionPathInROS( std::map<int, std::vector<pointIndex> > &cvRegionsPath);
	std::map<int, std::vector<geometry_msgs::PoseStamped> > CalcMultiRegionPathInROS( std::map<int, std::vector<pointIndex> > &cvRegionsPath, std::map<int, std::vector<pointIndex> > &cvRegionsPathR);



/*******The interface that user get the planning results or show them********/
	std::map<int, std::vector<geometry_msgs::PoseStamped> > GetMultiPathsInRos(){return multiRegionPathVecInROS_;};
	std::map<int, std::vector<geometry_msgs::PoseStamped> > GetOderPathsInRos(){return multiOderPathInROS_;};

	std::map<int, std::vector<geometry_msgs::PoseStamped> > GetNoStopPaths(const std::map<int, std::vector<geometry_msgs::PoseStamped> > &paths);
	std::vector<int> GetRegionOder(){ return region_oder_;}
	
	void publishPlan(const std::map<int, std::vector<geometry_msgs::PoseStamped> >& paths);
	void publishOderPlan(const std::map<int, std::vector<geometry_msgs::PoseStamped> >& paths);


/************** The drawing function *******************/
	cv::Mat DrawExample(const std::vector<pointIndex> &path, const cv::Rect &rect);
	void DrawPolygon(cv::Mat &image, const std::vector<pointIndex> &path, const Polygon &pg);


protected:
	/*******************Other auxiliary functions*********************/
	int whatRectContainsPoint(const cv::Point pt, const std::map<int, cv::Rect> &rects);
	float pointLineDistance(const cv::Point &pt1, const cv::Point &pt2);
	float vectorDot(const cv::Point2i pt1, const cv::Point2i pt2);
	std::string starAndEndCov(std::string soe);
	std::map<bool, std::vector<int> > ParallelogramCheck(const Polygon &pg);

	
	std::vector<int> pg_closed_;
	cv::Mat draw_image_;
	cv::Mat drawploygon_left_;
	cv::Mat drawploygon_right_;
	


private:
	bool use_rect_;

	/****************A set of rectangular or polygonal regions****************/
	std::map<int, cv::Rect> cvRegionsRect_;
	std::map<int, Polygon> cvRegionsPolygon_;


	/********Store the robot's initial coordinates and the planned path*******/
	tf::Stamped<tf::Pose> initPose_;
	std::map<int, std::vector<pointIndex> > pathInCv_;
	std::map<int, std::vector<pointIndex> > pathInCvRightCorner_;
	std::map<int, std::vector<geometry_msgs::PoseStamped> > multiRegionPathVecInROS_;
	std::map<int, std::vector<geometry_msgs::PoseStamped> > multiOderPathInROS_;



	/************The publisher object that publish planned paths**************/
	ros::Publisher plan_pub_;	


	/***************Used to plan the sequence of areas cleared****************/
	std::vector<int> clean_oder_;								//Order of cleaning areas
	std::vector<std::map<int, std::string> > best_clean_oder_; //entrance points
	std::vector<int> region_oder_;


	/***************** Robot and path planning parameters *******************/
	int map_rotation_angle_;
	double robot_radius_;
	double plan_interval_;
	double step_forward_;
	double robot_length_;
	double robot_width_;


/*******************Define the necessary costmap object******************/
    costmap_2d::Costmap2DROS* costmap2d_ros_;
    costmap_2d::Costmap2D* costmap2d_;
	costmap_2d::LayeredCostmap* layered_costmap_;


/****************The costmap descripted by OpenCv format****************/
	cv::Mat srcCVCostMap_;
	double resolution_; 	//The resolution of costmap and the srcCVCostMap_ ,in m/pixel.
	

/********Stores the coordinates of the points at Freespace in the CostMap ********/
	//The origin of opencv coordinates is in the upper left corner and the Costmap is in the lower left corner*****
	std::map<int, std::vector<pointIndex> > cvFreeSpaceVecR_;
	std::map<int, std::vector<pointIndex> > cvFreeSpaceVecC_;
	std::map<int, std::vector<cv::Point> > cvPointVecR_;

	std::map<int, std::vector<pointIndex> > costFreeSpaceVecR_;
	std::map<int, std::vector<pointIndex> > costFreeSpaceVecC_;


};

#endif