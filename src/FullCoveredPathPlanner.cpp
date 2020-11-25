#include <tx2_cover_clean/FullCoveredPathPlanner.h>

FullCoveredPathPlanner::FullCoveredPathPlanner(costmap_2d::Costmap2DROS *costmap2d_ros)
{
    costmap2d_ros_ = costmap2d_ros;
    costmap2d_ = costmap2d_ros_->getCostmap();
    layered_costmap_ = costmap2d_ros_->getLayeredCostmap();

    InitRosParameters();
    InitCvCostImg();

    InitRectRegions();
    InitPolygonRegions();

    CalcPathInCv();
    CalcCleanOrder();
    InitRegionOder();
    CalcPathsInRos();

}
FullCoveredPathPlanner::FullCoveredPathPlanner(bool simple)
{
    InitPolygonRegions();
    robot_radius_ = 0.7;
    plan_interval_ = 0.5;
    step_forward_ = 0.2;
    robot_length_ = 0.8;
    robot_width_ = 0.9;
    resolution_ = 0.05;
    draw_image_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
    drawploygon_left_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
    drawploygon_right_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));

    for(std::map<int, Polygon>::iterator iter = cvRegionsPolygon_.begin(); iter!=cvRegionsPolygon_.end(); iter++)
    {
        std::vector<pointIndex> pg_path0 ;
        pg_path0 = FromPolygonGetPath(iter->second);
        DrawPolygon(drawploygon_left_, pg_path0, iter->second);
    }
    for(std::map<int, Polygon>::iterator iter = cvRegionsPolygon_.begin(); iter!=cvRegionsPolygon_.end(); iter++)
    {
        std::vector<pointIndex> pg_path0 ;
        pg_path0 = FromPolygonGetPathRightCorner(iter->second);
        DrawPolygon(drawploygon_right_, pg_path0, iter->second);
    }

    std::vector<cv::Point> polygon0 = {cv::Point(129,94),cv::Point(478,91),cv::Point(601,383),cv::Point(47,380)};
    Polygon pg;
    pg.polygon = polygon0;
    cv::Mat demoimage = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
    std::vector<pointIndex> pg_path0 ;
    pg_path0 = FromPolygonGetPathRightCorner(pg);
    DrawPolygon(demoimage, pg_path0, pg);
    

    imshow("ploygon_left", drawploygon_left_);
    imshow("ploygon_right", drawploygon_right_);
    imshow("sourcepoint", draw_image_);
    imshow("demoimage", demoimage);

    waitKey(0);
}
FullCoveredPathPlanner::~FullCoveredPathPlanner()
{
}

/*********************The main execution function *************************/
void FullCoveredPathPlanner::InitRosParameters()
{
    ros::NodeHandle private_nh("~/cleaning_plan_nodehandle");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("cleaning_path", 1);
    std::string robotRadius,planInterval, stepForward ,robotLength, robotWidth; 
    std::string useRect;

    robot_radius_ = 0.5;
    if(private_nh.searchParam("robot_radius",robotRadius))
        private_nh.param("robot_radius",robot_radius_,0.5);
    plan_interval_ = 0.3;
    if(private_nh.searchParam("plan_interval",planInterval))
        private_nh.param("plan_interval",plan_interval_,0.3);
    step_forward_ = 0.2;
    if(private_nh.searchParam("step_forward",stepForward))
        private_nh.param("step_forward",step_forward_,0.2);
    robot_length_ = 0.8;
    if(private_nh.searchParam("robot_length",robotLength))
        private_nh.param("robot_length",robot_length_,0.8);
    robot_width_ = 0.6;
    if(private_nh.searchParam("robot_width",robotWidth))
        private_nh.param("robot_width", robot_width_, 0.6);
    use_rect_ = false;
    if(private_nh.searchParam("use_rect",useRect))
        private_nh.param("use_rect", use_rect_, false);


    resolution_ = costmap2d_->getResolution();
    std::cout<<"The resolution of map is "<<resolution_<<std::endl;

    draw_image_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
    drawploygon_left_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
    drawploygon_right_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
}
void FullCoveredPathPlanner::InitCvCostImg()
{
    int sizex = costmap2d_->getSizeInCellsX();
    //Accessor for the x size of the costmap in cells.
    int sizey = costmap2d_->getSizeInCellsY();
    std::cout<<"The size of map is "<<sizex<<"  "<<sizey<<std::endl;

    srcCVCostMap_ = cv::Mat(sizey,sizex,CV_8U);
    for(int r = 0; r < sizey; r++){
      for(int c = 0; c < sizex; c++ ){
          unsigned char cost_value = costmap2d_->getCost(c, sizey-1-r);
          srcCVCostMap_.at<uchar>(r,c) = cost_value;
          //??sizey-r-1 caution: costmap's origin is at left bottom ,while opencv's pic's origin is at left-top.
          //costmap的原点在左下方，opencv的图片的原点在左上方。

          if(cost_value == 0)
          {
              pointIndex free_index ; free_index.theta = 0;
              free_index.row = r; free_index.col = c;
              cvFreeSpaceVecR_[r].push_back(free_index);
              cvFreeSpaceVecC_[c].push_back(free_index);
              cv::Point self_pt; self_pt.x = c; self_pt.y = r;
              cvPointVecR_[r].push_back(self_pt);

              free_index.row = sizey-1-r; free_index.col = c;
              costFreeSpaceVecC_[c].push_back(free_index);
              costFreeSpaceVecR_[sizey-1-r].push_back(free_index);
          }
      }
    }

    imshow("debugMapImage",srcCVCostMap_);
    waitKey(0);
}
void FullCoveredPathPlanner::InitRectRegions()
{
    cv::Rect myregion(210,243,190,44);
    cv::Rect myregion1(172,270,59,136);
    cv::Rect myregion2(233,337,89,49);
    cv::Rect myregion3(286,357,156,45);
    cv::Rect myregion4(379,252,61,148);

    cvRegionsRect_[0] = myregion;
    cvRegionsRect_[1] = myregion1;
    cvRegionsRect_[2] = myregion2;
    cvRegionsRect_[3] = myregion3;
    cvRegionsRect_[4] = myregion4;
}
void FullCoveredPathPlanner::InitPolygonRegions()
{
    std::vector<cv::Point>  polygon0, polygon1, polygon2, polygon3, polygon4;

    polygon0 = {cv::Point(199,238),cv::Point(404,228),cv::Point(405,286),cv::Point(203,294)};
    polygon1 = {cv::Point(164,263),cv::Point(225,258),cv::Point(234,409),cv::Point(173,415)};
    polygon2 = {cv::Point(231,334),cv::Point(321,329),cv::Point(323,382),cv::Point(235,387)};
    polygon3 = {cv::Point(281,360),cv::Point(442,353),cv::Point(445,400),cv::Point(282,407)};
    polygon4 = {cv::Point(376,256),cv::Point(438,251),cv::Point(443,401),cv::Point(383,403)};

    cvRegionsPolygon_[0].polygon = polygon0;
    cvRegionsPolygon_[1].polygon = polygon1;
    cvRegionsPolygon_[2].polygon = polygon2;
    cvRegionsPolygon_[3].polygon = polygon3;
    cvRegionsPolygon_[4].polygon = polygon4;

}
void FullCoveredPathPlanner::CalcPathInCv()
{
    std::map<int, std::vector<pointIndex> > cv_paths;
    std::map<int, std::vector<pointIndex> > cv_paths_right_corner;
    if(use_rect_ == true)
    {
        std::map<int, cv::Rect>::iterator iter_rect;
        for(iter_rect = cvRegionsRect_.begin(); iter_rect!=cvRegionsRect_.end(); iter_rect++)
        {
            cv_paths[iter_rect->first] = FromRectGetPath(iter_rect->second);
            //Compute the path in Opencv from the given rectangle.
            cv_paths_right_corner[iter_rect->first] = FromRectGetPathRightCorner(iter_rect->second);
        }
    }
    else
    {
        std::map<int, Polygon>::iterator iter_rect;
        for(iter_rect = cvRegionsPolygon_.begin(); iter_rect!=cvRegionsPolygon_.end(); iter_rect++)
        {
            cv_paths[iter_rect->first] = FromPolygonGetPath(iter_rect->second);
            //Compute the path in Opencv from the given rectangle.
            cv_paths_right_corner[iter_rect->first] = FromPolygonGetPathRightCorner(iter_rect->second);
        }   
    }

    pathInCv_ = cv_paths;
    pathInCvRightCorner_ = cv_paths_right_corner;
    // std::cout<<"the size of pathInCv_ is :"<< pathInCv_.size()<<std::endl;
    // std::cout<<"the size of pathInCvRightCorner_ is :"<< pathInCvRightCorner_.size()<<std::endl;
}
void FullCoveredPathPlanner::CalcPathsInRos()
{
    multiRegionPathVecInROS_ = CalcMultiRegionPathInROS(pathInCv_);
    //Calculate the path on the actual map from the path in Opencv.

    /*A map is used to ensure that a region corresponds to its number*/
    multiOderPathInROS_ = CalcMultiRegionPathInROS(pathInCv_, pathInCvRightCorner_);
    // std::cout<<"the size of multiOderPathInROS_ is :"<< multiOderPathInROS_.size()<<std::endl;
}
void FullCoveredPathPlanner::CalcCleanOrder()
{
    bool isok = costmap2d_ros_->getRobotPose(initPose_);//Gets the current robot world coordinates
    if(!isok)
    {
        ROS_INFO("Failed to get robot location! Please check where goes wrong!");
        return;
    }
    unsigned int mx,my;
    double wx = initPose_.getOrigin().x() ;
    double wy = initPose_.getOrigin().y() ;

    std::cout<<wx<<"**********"<<wy<<std::endl;

    bool getmapcoor = costmap2d_->worldToMap(wx,wy,mx,my);

    std::cout<<"the robot's mx is:"<<mx<<"&&&&&&&&&&&&&"<<"the robot's my is:"<<my<<std::endl;
    if(!getmapcoor)
    {
        ROS_INFO("Failed to get robot location in map! Please check where goes wrong!");
        return;
    }
    cv::Point robot_cvpoint; //The initial position of the robot in OpencV
    int sizey = costmap2d_->getSizeInCellsY();
    robot_cvpoint.x = mx;
    robot_cvpoint.y = sizey - 1 - my;
    


    std::map<int, std::vector<cv::Point> >  regions_se_point;
    std::map<int, std::vector<cv::Point> >  regions_se_point_r;
    std::map<int, std::map<std::string, cv::Point> > regions_se_point_lr;
    //Store the starting and ending points of each area

    //Calculate the starting and ending points of each area
    for(std::map<int, std::vector<pointIndex> >::iterator iter = pathInCv_.begin(); iter != pathInCv_.end(); iter++)
    {
        cv::Point p_start( (iter->second)[0].col, (iter->second)[0].row );
        regions_se_point[iter->first].push_back( p_start );
        regions_se_point_lr[iter->first]["left_start"] = p_start;

        cv::Point p_end( (iter->second)[(iter->second).size()-1].col, (iter->second)[(iter->second).size()-1].row );
        regions_se_point[iter->first].push_back( p_end );
        regions_se_point_lr[iter->first]["left_end"] = p_end;
    }
    for(std::map<int, std::vector<pointIndex> >::iterator iter = pathInCvRightCorner_.begin(); iter != pathInCvRightCorner_.end(); iter++)
    {
        cv::Point p_start( (iter->second)[0].col, (iter->second)[0].row );
        regions_se_point_r[iter->first].push_back( p_start );
        regions_se_point_lr[iter->first]["right_start"] = p_start;

        cv::Point p_end( (iter->second)[(iter->second).size()-1].col, (iter->second)[(iter->second).size()-1].row );
        regions_se_point_r[iter->first].push_back( p_end );
        regions_se_point_lr[iter->first]["right_end"] = p_end;
    }
    //Calculate the starting and ending points of each area

    int robot_init_region  = 0;
    robot_init_region = whatRectContainsPoint(robot_cvpoint, cvRegionsRect_);
    //Get the robot's starting area

    //Calculate the order of cleaning
    clean_oder_.push_back(robot_init_region);
    for(int i = 1; i < cvRegionsRect_.size(); i++)
    {
        std::map<int, float> distances;
        std::vector<float> dist;
        int linshi_region;
        for(std::map<int, std::vector<cv::Point> >::iterator iter_se = regions_se_point.begin(); iter_se != regions_se_point.end(); iter_se++)
        {
            if(std::find(clean_oder_.begin(),clean_oder_.end(), iter_se->first) == clean_oder_.end())
            {
                float d = pointLineDistance(regions_se_point[clean_oder_[i-1] ][0], (iter_se->second)[0]);
                distances[iter_se->first] = d;
                dist.push_back(d);

            }
            else
            {
                continue;
            }
        }
        float min_d = *(std::min_element(dist.begin(), dist.end()));
        for(std::map<int,float>::iterator it = distances.begin();it!=distances.end();it++) 
        {   
            if(std::abs(it->second - min_d) <= 0.01 )
            {
                linshi_region = it->first;
                break;
            }
        } 

        clean_oder_.push_back(linshi_region);
    }
    //Calculate the order of cleaning


    best_clean_oder_.clear();
    float min_d = 0, last_min_d=0;
    std::string min_str;
    std::map<int,std::string> last_start;
    std::map<std::map<int,std::string>, std::map<float, std::map<int,std::string> >  > io_dist; 
    //The min distance to every regions.

    std::map<std::string, float> single_re_dist;
    std::vector<float> single_re_dist_vec;

    for(int i = 1; i < cvRegionsRect_.size(); i++)
    {   io_dist.clear();
        
        if(i == 1 && best_clean_oder_.empty())
        {
            for(std::map<std::string, cv::Point>::iterator iter 
            = regions_se_point_lr[robot_init_region].begin(); 
            iter != regions_se_point_lr[robot_init_region].end(); //region's exit
            iter++)
            {
                for(std::map<int, std::map<std::string, cv::Point> >::iterator iter_re
                = regions_se_point_lr.begin();
                iter_re != regions_se_point_lr.end(); iter_re++) //region's entrance
                {
                    if(iter_re->first != robot_init_region)
                    {
                        single_re_dist_vec.clear();
                        single_re_dist.clear();

                        for(std::map<std::string, cv::Point>::iterator iter_p = (iter_re->second).begin(); 
                        iter_p != (iter_re->second).end();
                        iter_p++)
                        {
                            float d = pointLineDistance(iter->second, iter_p->second);
                            single_re_dist_vec.push_back(d);
                            single_re_dist[iter_p->first] = d;
                        }

                        min_d = *(std::min_element(single_re_dist_vec.begin(), single_re_dist_vec.end()));
                        for(std::map<std::string, float>::iterator iter_min = single_re_dist.begin();
                        iter_min != single_re_dist.end();
                        iter_min++)
                        {
                            if(std::abs(iter_min->second - min_d) <= 0.01);
                            {
                                min_str = iter_min->first;
                                break;
                            }                           
                        }

                        if(io_dist.empty())
                        {
                            std::map<int,std::string> io_first;
                            io_first[robot_init_region] = iter->first; //The first regional starting point is variable

                            std::map<float, std::map<int,std::string> > io_sencond;
                            std::map<int,std::string> io_sencond_sencond;

                            io_sencond_sencond[iter_re->first] = min_str;
                            io_sencond[min_d] = io_sencond_sencond;

                            io_dist[io_first] = io_sencond;

                            last_start = io_first;
                            last_min_d = min_d;
                        }
                        else
                        {
                            if(min_d < last_min_d )
                            {
                                io_dist.clear();
                                std::map<int,std::string> io_first;
                                io_first[robot_init_region] = iter->first;

                                std::map<float, std::map<int,std::string> > io_sencond;
                                std::map<int,std::string> io_sencond_sencond;

                                io_sencond_sencond[iter_re->first] = min_str;
                                io_sencond[min_d] = io_sencond_sencond;

                                io_dist[io_first] = io_sencond;

                                last_start = io_first;
                                last_min_d = min_d;
                            }

                        }
                    }
                }
            }
            std::map<int, std::string> best_entrance;
            best_entrance[robot_init_region] = starAndEndCov(last_start[robot_init_region]);
            best_clean_oder_.push_back(best_entrance);

            best_entrance.clear();
            best_entrance = (io_dist[last_start])[last_min_d];
            best_clean_oder_.push_back(best_entrance);

            last_start.clear();
            last_min_d = 0;

        }
        else
        {
            std::map<int, std::string> start_region;
            start_region = best_clean_oder_[best_clean_oder_.size()-1];
            int last_end_region = (start_region.begin())->first;
            std::string last_end_str = starAndEndCov( (start_region.begin())->second );

            for(std::map<int, std::map<std::string, cv::Point> >::iterator iter_re
            = regions_se_point_lr.begin();
            iter_re != regions_se_point_lr.end();
            iter_re++)
            {
                bool check_flag = false;
                for(int i = 0; i < best_clean_oder_.size(); i++)
                {
                    if(best_clean_oder_[i].find(iter_re->first) == best_clean_oder_[i].end())
                    {
                        check_flag = true;
                    }
                    else
                    {
                        check_flag = false; break;
                    }

                }

                if(check_flag == true)
                {   single_re_dist.clear();
                    single_re_dist_vec.clear();

                    for(std::map<std::string, cv::Point>::iterator iter_p = (iter_re->second).begin(); 
                    iter_p != (iter_re->second).end();
                    iter_p++)
                    {
                        float d = pointLineDistance( (regions_se_point_lr[last_end_region]).at(last_end_str), iter_p->second);
                        single_re_dist_vec.push_back(d);
                        single_re_dist[iter_p->first] = d;
                    }

                    min_d = *(std::min_element(single_re_dist_vec.begin(), single_re_dist_vec.end()));
                    for(std::map<std::string, float>::iterator iter_min = single_re_dist.begin();
                    iter_min != single_re_dist.end();
                    iter_min++)
                    {
                        if(std::abs(iter_min->second - min_d) <= 0.01);
                        {
                            min_str = iter_min->first;
                            break;
                        }                           
                    }

                    if(io_dist.empty())
                    {
                        std::map<int,std::string> io_first;
                        io_first[last_end_region] = last_end_str;

                        std::map<float, std::map<int,std::string> > io_sencond;
                        std::map<int,std::string> io_sencond_sencond;

                        io_sencond_sencond[iter_re->first] = min_str;
                        io_sencond[min_d] = io_sencond_sencond;

                        io_dist[io_first] = io_sencond;

                        last_start = io_first;
                        last_min_d = min_d;
                    }
                    else
                    {
                        if(min_d < last_min_d )
                        {
                            io_dist.clear();
                            std::map<int,std::string> io_first;
                            io_first[last_end_region] = last_end_str;

                            std::map<float, std::map<int,std::string> > io_sencond;
                            std::map<int,std::string> io_sencond_sencond;

                            io_sencond_sencond[iter_re->first] = min_str;
                            io_sencond[min_d] = io_sencond_sencond;

                            io_dist[io_first] = io_sencond;

                            last_start = io_first;
                            last_min_d = min_d;
                        }
                    }
                    
                }
            }
            std::map<int, std::string> best_entrance;
            best_entrance = (io_dist.at(last_start) ).at(last_min_d);
            best_clean_oder_.push_back(best_entrance);

            last_start.clear();
            last_min_d = 0;
        }
    }

    for(int i = 0; i<clean_oder_.size(); i++)
    {
        std::cout<<"the old "<< i <<"chean oder is : "<<clean_oder_[i]<<std::endl;
    }
}
void FullCoveredPathPlanner::InitRegionOder()
{
    for(int i = 0; i < best_clean_oder_.size(); i++)
    {
        region_oder_.push_back( (best_clean_oder_[i].begin())->first);
    }
    // for(int i = 0; i<region_oder_.size(); i++)
    // {
    //     std::cout<<"the best "<< i << "clean oder is: "<<region_oder_[i]<<std::endl;
    // }
}
void FullCoveredPathPlanner::CheckBorder()
{  
}


/******************* main fictions of planning the Path ********************/
std::vector<pointIndex> FullCoveredPathPlanner::CvImageToCostMap(std::vector<pointIndex> &cvpoints)
{
    int sizex = costmap2d_->getSizeInCellsX();  //The width of the costmap
    int sizey = costmap2d_->getSizeInCellsY();  //The height of the costmap
    std::vector<pointIndex> map_points;
    for(int i = 0; i<cvpoints.size(); i++)
    {
        pointIndex po;
        po.col = cvpoints[i].col;   po.row = sizey-1-cvpoints[i].row;   po.theta = cvpoints[i].theta;
        //The origin of Opencv is in the upper left corner and the origin of CostMap is in the lower left corner
        map_points.push_back(po);
    }
    return map_points;
}

std::vector<pointIndex> FullCoveredPathPlanner::FromRectGetPath(const cv::Rect &rect)
{
    int step_forward = (int)(step_forward_/resolution_);   //The interval between navigation points on a path
    int plan_interval = (int)(plan_interval_/resolution_);  //The interval between paths, in pixels
    int robot_radius = (int)(robot_radius_/resolution_) ;  //The rotation radius of robot, in pixels
    int robot_length = (int)(robot_length_/resolution_) ;
    int robot_width = (int)(robot_width_/resolution_);
    std::vector<pointIndex> pathRect;
    if(rect.width >= rect.height)
    {
        int ran_x = (rect.width - 2*robot_radius) / step_forward;    //The number of iterations in the x or width direction
        int ran_y = (rect.height - robot_width) / plan_interval;  //The number of iterations in the y or height direction
        for(int i_y = 0; i_y <= ran_y; i_y++)
        {
            for(int i_x = 0; i_x <= ran_x; i_x++)
            {
                pointIndex point;
                point.row = i_y*plan_interval + (int)(robot_width/2) + rect.y;
                if(i_y%2 == 0)
                {
                    point.col = i_x*step_forward + robot_radius + rect.x;
                }
                else
                {
                    point.col = rect.x + rect.width - robot_radius - i_x*step_forward;
                }
                point.theta = 90;
                pathRect.push_back(point);
            }
        }
    }
    else
    {
        int ran_x = (rect.width - robot_width) / plan_interval;
        int ran_y = (rect.height - 2*robot_radius) / step_forward;
        for(int i_x = 0; i_x <= ran_x; i_x++)
        {
            for(int i_y = 0; i_y <= ran_y; i_y++)
            {
                pointIndex point;
                point.col = i_x*plan_interval + (int)(robot_width/2) + rect.x;
                if(i_x%2 == 0)
                {
                    point.row = i_y*step_forward + robot_radius + rect.y;
                }
                else
                {
                    point.row = rect.y + rect.height - robot_radius - i_y*step_forward;
                }
                point.theta = 90;
                pathRect.push_back(point);
            }
        }
    }
    return pathRect;

}
std::vector<pointIndex> FullCoveredPathPlanner::FromRectGetPathRightCorner(const cv::Rect &rect)
{
    int step_forward = (int)(step_forward_/resolution_);   //The interval between navigation points on a path
    int plan_interval = (int)(plan_interval_/resolution_);  //The interval between paths, in pixels
    int robot_radius = (int)(robot_radius_/resolution_) ;  //The rotation radius of robot, in pixels
    int robot_length = (int)(robot_length_/resolution_) ;
    int robot_width = (int)(robot_width_/resolution_);
    std::vector<pointIndex> pathRect;
    if(rect.width >= rect.height)
    {
        int ran_x = (rect.width - 2*robot_radius) / step_forward;    //The number of iterations in the x or width direction
        int ran_y = (rect.height - robot_width) / plan_interval;  //The number of iterations in the y or height direction
        for(int i_y = 0; i_y <= ran_y; i_y++)
        {
            for(int i_x = 0; i_x <= ran_x; i_x++)
            {
                pointIndex point;
                point.row = i_y*plan_interval + (int)(robot_width/2) + rect.y;
                if(i_y%2 != 0)
                {
                    point.col = i_x*step_forward + robot_radius + rect.x;
                }
                else
                {
                    point.col = rect.x + rect.width - robot_radius - i_x*step_forward;
                }
                point.theta = 90;
                pathRect.push_back(point);
            }
        }
    }
    else
    {
        int ran_x = (rect.width - robot_width) / plan_interval;
        int ran_y = (rect.height - 2*robot_radius) / step_forward;
        for(int i_x = 0; i_x <= ran_x; i_x++)
        {
            for(int i_y = 0; i_y <= ran_y; i_y++)
            {
                pointIndex point;
                point.col = i_x*plan_interval + (int)(robot_width/2) + rect.x;
                if(i_x%2 != 0)
                {
                    point.row = i_y*step_forward + robot_radius + rect.y;
                }
                else
                {
                    point.row = rect.y + rect.height - robot_radius - i_y*step_forward;
                }
                point.theta = 90;
                pathRect.push_back(point);
            }
        }
    }
    return pathRect;  
}

std::vector<pointIndex> FullCoveredPathPlanner::FromPolygonGetPath(const Polygon &pg)
{
    int step_forward = (int)(step_forward_/resolution_);   //The interval between navigation points on a path
    int plan_interval = (int)(plan_interval_/resolution_);  //The interval between paths, in pixels
    int robot_radius = (int)(robot_radius_/resolution_) ;  //The rotation radius of robot, in pixels
    int robot_length = (int)(robot_length_/resolution_) ;
    float robot_width = (robot_width_/resolution_);
    std::vector<pointIndex> result;

    std::map<bool, std::vector<int> > mysb = ParallelogramCheck(pg);
    std::map<bool, std::vector<int> >::iterator iter = mysb.begin();
    bool pg_isok = iter->first;
    if(pg_isok == true)
    {
        std::vector<int> sequence = iter->second;
        float line0_L =  pointLineDistance(pg.polygon[sequence[0]], pg.polygon[sequence[1]]);
        float line2_L =  pointLineDistance(pg.polygon[sequence[1]], pg.polygon[sequence[2]]);
        cv::Point2f pt1, pt2, pt_left, pt_right, pt_down1, pt_down2;

        if(line0_L >= line2_L)
        {
            std::cout<<"&&&&&&&&&line0 is the base line&&&&&&&&&&&"<<std::endl<<std::endl;
            pt1 = pg.polygon[sequence[0]];  pt2 = pg.polygon[sequence[1]];
            pt_left =  pg.polygon[sequence[3]]; pt_right = pg.polygon[sequence[2]];
            pt_down1 = pg.polygon[sequence[3]]; pt_down2 = pg.polygon[sequence[2]];
        }
        else
        {
            //belong to line2
            std::cout<<"&&&&&&&&&line2 is the base line&&&&&&&&&&&"<<std::endl<<std::endl;
            pt1 = pg.polygon[sequence[1]]; pt2 = pg.polygon[sequence[2]]; 
            pt_left =  pg.polygon[sequence[0]]; pt_right = pg.polygon[sequence[3]];
            pt_down1 = pg.polygon[sequence[0]]; pt_down2 = pg.polygon[sequence[3]];
        }
        
        /*Line1 is the active baseline*/
        /*Left,Right and down line is a fixed boundary line*/
        float line1_k ;
        if( abs(pt1.x - pt2.x) <= 0.1) {std::cout<<"dx=0"<<std::endl; line1_k = ( (float)(pt1.y - pt2.y) )/1;}
        else {line1_k = ( (float)(pt1.y - pt2.y) )/( (float)(pt1.x - pt2.x) );}
        float line1_b = pt1.y - line1_k * pt1.x;
        std::cout<<"pt1"<<pt1<<"pt2"<<pt2<<"line2_k = "<<line1_k<<"   "<<"line2_b = "<<line1_b<<std::endl<<std::endl;

        float left_k;
        if(abs(pt1.x - pt_left.x) <= 0.1) {left_k = (pt1.y - pt_left.y)/1 ;}
        else {left_k = ( (float)(pt1.y - pt_left.y) )/( (float)(pt1.x - pt_left.x) ) ;}
        float left_b = pt1.y - left_k * pt1.x ;
        std::cout<<"left_point"<<pt_left<<"left_k = "<<left_k<<"   "<<"left_b = "<<left_b<<std::endl<<std::endl;

        float right_k;
        if( abs(pt2.x - pt_right.x) <= 0.1) {right_k = ( (float)(pt2.y - pt_right.y) )/1;}
        else { right_k = ( (float)(pt2.y - pt_right.y) )/( (float)(pt2.x - pt_right.x) );}
        float right_b = pt2.y - right_k * pt2.x;
        std::cout<<"right_point"<<pt_right<<"right_k = "<<right_k<<"   "<<"right_b = "<<right_b<<std::endl<<std::endl;
        
        float down_k;
        if( abs(pt_down1.x - pt_down2.x) <= 0.1) {down_k = ( (float)(pt_down1.y - pt_down2.y) )/1;}
        else { down_k = ( (float)(pt_down1.y - pt_down2.y) )/( (float)(pt_down1.x - pt_down2.x) );}
        float down_b = pt_down1.y - down_k * pt_down1.x;
        std::cout<<"pt_down1"<<pt_down1<<"down_k = "<<down_k<<"   "<<"down_b = "<<down_b<<std::endl<<std::endl;

        /*The equation of the line is y = kx+b */
        float line_distance1 = abs(line1_k*pt_left.x - pt_left.y + line1_b)/sqrt(line1_k * line1_k + 1);
        float line_distance2 = abs(line1_k*pt_right.x - pt_right.y + line1_b)/sqrt(line1_k * line1_k + 1);//d = |b-b1|/sqrt(k*k+1)
        float line_distance;
        float line1_end_b;
        float line1_ts_b;//line1's b at the boundary turn
        if(line_distance1 >= line_distance2 )
        {
            line_distance = line_distance1;
            line1_end_b = pt_left.y - line1_k * pt_left.x;
            line1_ts_b = pt_right.y - line1_k * pt_right.x;//b = y - kx
        } 
        else 
        {
            line_distance = line_distance2;
            line1_end_b = pt_right.y - line1_k * pt_right.x;
            line1_ts_b =  pt_left.y - line1_k * pt_left.x;
        }
        std::cout<<"the line_distance is: "<<line_distance<<std::endl<<std::endl;
        std::cout<<"the line1_end_b is: "<<line1_end_b<<std::endl<<std::endl;
        int iter_count = (int)( (line_distance - robot_width)/plan_interval) + 1; //计算迭代次数
        std::cout<<"the iter_count is: "<<iter_count<<std::endl<<std::endl;
        //At the end of the b
        float left_offset_b = pt2.y - left_k * pt2.x;
        float right_offset_b = pt1.y - right_k * pt1.x;
        float down_offset_b = pt1.y - down_k* pt1.x;
        //B corresponds to the line after the expansion of the robot radius
        float line1_b1 = robot_width/2*sqrt(line1_k*line1_k + 1)*( (line1_end_b-line1_b)/abs(line1_end_b-line1_b) ) + line1_b;
        float left_b1 = robot_radius*sqrt(left_k*left_k + 1)*((left_offset_b-left_b)/abs(left_offset_b-left_b)) + left_b;
        float right_b1 = robot_radius*sqrt(right_k*right_k + 1)*((right_offset_b-right_b)/abs(right_offset_b-right_b)) + right_b;
        float down_b1 = robot_radius*sqrt(down_k*down_k) * ((down_offset_b-down_b)/abs(down_offset_b-down_b)) + down_b;

        float k_angle = atan(line1_k);//line1's angle
        std::cout<<"the line1 angle is: "<<k_angle<<std::endl<<std::endl;
        std::cout<<"the left_offset_b is: "<<left_offset_b<<std::endl<<std::endl;
        std::cout<<"the right_offset_b is: "<<right_offset_b<<std::endl<<std::endl;
        std::cout<<"the line1_b1 is: "<<line1_b1<<std::endl<<std::endl;
        std::cout<<"the left_b1 is: "<<left_b1<<std::endl<<std::endl;
        std::cout<<"the right_b1 is: "<<right_b1<<std::endl<<std::endl;

        pointIndex path_point;
        float x_start,y_start;
        float angle_diff = abs( (atan(down_k)-atan(line1_k)) / 3.1415 * 180);
        std::cout<<"angle_diff is : "<<angle_diff<<std::endl;
        std::cout<<"the iter is begin "<<std::endl<<std::endl<<std::endl;
        for(int i = 0; i< iter_count; i++)
        {
            float line1_offset_b = line1_b1 + i * plan_interval *sqrt(line1_k*line1_k + 1)*( (line1_end_b-line1_b)/abs(line1_end_b-line1_b) ) ;
            
            /*Find the intersection point of each baseline and edge*/
            float x0;
            float x1;
            if(line_distance1>line_distance2)
            {
                x0 = (left_b1 - line1_offset_b)/( line1_k - left_k);
                float b_direction = line1_end_b-line1_b ;
                if(b_direction >= 0)
                {
                    if(line1_offset_b <= line1_ts_b){ x1 = (right_b1 - line1_offset_b)/( line1_k - right_k); }
                    else { x1 = (down_b1 - line1_offset_b)/( line1_k - down_k); }
                }
                else
                {
                    if(line1_offset_b >= line1_ts_b){ x1 = (right_b1 - line1_offset_b)/( line1_k - right_k);  }
                    else { x1 = (down_b1 - line1_offset_b)/( line1_k - down_k); }                       
                }
            }
            else
            {
                x1 = (right_b1 - line1_offset_b)/( line1_k - right_k);
                float b_direction = line1_end_b-line1_b ;
                if(b_direction >= 0)
                {
                    if(line1_offset_b <= line1_ts_b){ x0 = (left_b1 - line1_offset_b)/( line1_k - left_k); }
                    else { x0 = (down_b1 - line1_offset_b)/( line1_k - down_k); }
                }
                else
                {
                    if(line1_offset_b >= line1_ts_b){ x0 = (left_b1 - line1_offset_b)/( line1_k - left_k); }
                    else { x0 = (down_b1 - line1_offset_b)/( line1_k - down_k); }                       
                }
            }
            if(angle_diff < 5 || angle_diff>175)
            {
                x0 = (left_b1 - line1_offset_b)/( line1_k - left_k);
                x1 = (right_b1 - line1_offset_b)/( line1_k - right_k); 
            }
            float y0 = line1_k * x0 + line1_offset_b;
            float y1 = line1_k * x1 + line1_offset_b;

            /*    std::cout<<"the iter is : "<<i<<std::endl;
            std::cout<<"the line1_offset_b is: "<<line1_offset_b<<std::endl<<std::endl;
            std::cout<<"(x0, y0)= "<<"("<<x0<<", "<<y0<<")"<<std::endl<<std::endl;
            std::cout<<"(x1, y1)= "<<"("<<x1<<", "<<y1<<")"<<std::endl<<std::endl;*/

            /*Solving iteration times*/
            float path_length = sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
            int path_iter = (int)(path_length/step_forward) + 1;
            
            /***The path points are generated from the line***/
            if(i%2 == 0) //pt1 to pt2
            {
                if(i!=0){cv::line(draw_image_, cv::Point(x_start,y_start), cv::Point(x0,y0), cv::Scalar(0,255,0) ) ;}
                cv::line(draw_image_, cv::Point(x0,y0), cv::Point(x1,y1), cv::Scalar(0,255,0) ) ;
                x_start = x1; y_start = y1;

                for(int u = 0; u<path_iter ;u++)
                {
                    float t = u * step_forward;
                    float x_direction = 1; float y_direction = 1;
                    if(x1-x0 == 0) x_direction = 0;
                    else x_direction = (x1-x0)/abs(x1-x0);

                    if(y1-y0 == 0) y_direction = 0;
                    else y_direction = (y1-y0)/abs(y1-y0);
                    int p_x = (int)(x0 + t * cos(k_angle) * x_direction);
                    int p_y = (int)(y0 + t * abs(sin(k_angle)) * y_direction);
                    path_point.col = p_x;
                    path_point.row = p_y;
                    path_point.theta = 90;
                    result.push_back(path_point);
                }
            }
            else
            {
                if(i!=0){cv::line(draw_image_, cv::Point(x_start,y_start), cv::Point(x1,y1), cv::Scalar(0,255,0) ) ;}
                cv::line(draw_image_, cv::Point(x1,y1), cv::Point(x0,y0), cv::Scalar(0,255,0) ) ;
                x_start = x0; y_start = y0;

                for(int u = 0; u<path_iter ;u++)
                {
                    float t = u * step_forward;
                    float x_direction = 1; float y_direction = 1;
                    if(x1-x0 == 0) x_direction = 0;
                    else x_direction = (x0-x1)/abs(x0-x1);

                    if(y1-y0 == 0) y_direction = 0;
                    else y_direction = (y0-y1)/abs(y0-y1);

                    int p_x = (int)(x1 + t * cos(k_angle) * x_direction);
                    int p_y = (int)(y1 + t * abs(sin(k_angle)) * y_direction);
                    path_point.col = p_x;
                    path_point.row = p_y;
                    path_point.theta = 90;
                    result.push_back(path_point);
                }
            }
        }
    
        return result;

    }
    else
    {
        return result;
    }
}
std::vector<pointIndex> FullCoveredPathPlanner::FromPolygonGetPathRightCorner(const Polygon &pg)
{
    int step_forward = (int)(step_forward_/resolution_);   //The interval between navigation points on a path
    int plan_interval = (int)(plan_interval_/resolution_);  //The interval between paths, in pixels
    int robot_radius = (int)(robot_radius_/resolution_) ;  //The rotation radius of robot, in pixels
    int robot_length = (int)(robot_length_/resolution_) ;
    float robot_width = (robot_width_/resolution_);
    std::vector<pointIndex> result;

    std::map<bool, std::vector<int> > mysb = ParallelogramCheck(pg);
    std::map<bool, std::vector<int> >::iterator iter = mysb.begin();
    bool pg_isok = iter->first;
    if(pg_isok == true)
    {
        std::vector<int> sequence = iter->second;
        float line0_L =  pointLineDistance(pg.polygon[sequence[0]], pg.polygon[sequence[1]]);
        float line2_L =  pointLineDistance(pg.polygon[sequence[1]], pg.polygon[sequence[2]]);
        cv::Point2f pt1, pt2, pt_left, pt_right, pt_down1, pt_down2;

        if(line0_L >= line2_L)
        {
            std::cout<<"&&&&&&&&&line0 is the base line&&&&&&&&&&&"<<std::endl<<std::endl;
            pt1 = pg.polygon[sequence[0]];  pt2 = pg.polygon[sequence[1]];
            pt_left =  pg.polygon[sequence[3]]; pt_right = pg.polygon[sequence[2]];
            pt_down1 = pg.polygon[sequence[3]]; pt_down2 = pg.polygon[sequence[2]];
        }
        else
        {
            //belong to line2
            std::cout<<"&&&&&&&&&line2 is the base line&&&&&&&&&&&"<<std::endl<<std::endl;
            pt1 = pg.polygon[sequence[1]]; pt2 = pg.polygon[sequence[2]]; 
            pt_left =  pg.polygon[sequence[0]]; pt_right = pg.polygon[sequence[3]];
            pt_down1 = pg.polygon[sequence[0]]; pt_down2 = pg.polygon[sequence[3]];
        }
        
        /*Line1 is the active baseline*/
        /*Left,Right and down line is a fixed boundary line*/
        float line1_k ;
        if( abs(pt1.x - pt2.x) <= 0.1) {std::cout<<"dx=0"<<std::endl; line1_k = ( (float)(pt1.y - pt2.y) )/1;}
        else {line1_k = ( (float)(pt1.y - pt2.y) )/( (float)(pt1.x - pt2.x) );}
        float line1_b = pt1.y - line1_k * pt1.x;
        std::cout<<"pt1"<<pt1<<"pt2"<<pt2<<"line2_k = "<<line1_k<<"   "<<"line2_b = "<<line1_b<<std::endl<<std::endl;

        float left_k;
        if(abs(pt1.x - pt_left.x) <= 0.1) {left_k = (pt1.y - pt_left.y)/1 ;}
        else {left_k = ( (float)(pt1.y - pt_left.y) )/( (float)(pt1.x - pt_left.x) ) ;}
        float left_b = pt1.y - left_k * pt1.x ;
        std::cout<<"left_point"<<pt_left<<"left_k = "<<left_k<<"   "<<"left_b = "<<left_b<<std::endl<<std::endl;

        float right_k;
        if( abs(pt2.x - pt_right.x) <= 0.1) {right_k = ( (float)(pt2.y - pt_right.y) )/1;}
        else { right_k = ( (float)(pt2.y - pt_right.y) )/( (float)(pt2.x - pt_right.x) );}
        float right_b = pt2.y - right_k * pt2.x;
        std::cout<<"right_point"<<pt_right<<"right_k = "<<right_k<<"   "<<"right_b = "<<right_b<<std::endl<<std::endl;
        
        float down_k;
        if( abs(pt_down1.x - pt_down2.x) <= 0.1) {down_k = ( (float)(pt_down1.y - pt_down2.y) )/1;}
        else { down_k = ( (float)(pt_down1.y - pt_down2.y) )/( (float)(pt_down1.x - pt_down2.x) );}
        float down_b = pt_down1.y - down_k * pt_down1.x;
        std::cout<<"pt_down1"<<pt_down1<<"down_k = "<<down_k<<"   "<<"down_b = "<<down_b<<std::endl<<std::endl;

        /*The equation of the line is y = kx+b */
        float line_distance1 = abs(line1_k*pt_left.x - pt_left.y + line1_b)/sqrt(line1_k * line1_k + 1);
        float line_distance2 = abs(line1_k*pt_right.x - pt_right.y + line1_b)/sqrt(line1_k * line1_k + 1);//d = |b-b1|/sqrt(k*k+1)
        float line_distance;
        float line1_end_b;
        float line1_ts_b;//line1's b at the boundary turn
        if(line_distance1 >= line_distance2 )
        {
            line_distance = line_distance1;
            line1_end_b = pt_left.y - line1_k * pt_left.x;
            line1_ts_b = pt_right.y - line1_k * pt_right.x;//b = y - kx
        } 
        else 
        {
            line_distance = line_distance2;
            line1_end_b = pt_right.y - line1_k * pt_right.x;
            line1_ts_b =  pt_left.y - line1_k * pt_left.x;
        }
        std::cout<<"the line_distance is: "<<line_distance<<std::endl<<std::endl;
        std::cout<<"the line1_end_b is: "<<line1_end_b<<std::endl<<std::endl;
        int iter_count = (int)( (line_distance - robot_width)/plan_interval) + 1; //计算迭代次数
        std::cout<<"the iter_count is: "<<iter_count<<std::endl<<std::endl;
        //At the end of the b
        float left_offset_b = pt2.y - left_k * pt2.x;
        float right_offset_b = pt1.y - right_k * pt1.x;
        float down_offset_b = pt1.y - down_k* pt1.x;
        //B corresponds to the line after the expansion of the robot radius
        float line1_b1 = robot_width/2*sqrt(line1_k*line1_k + 1)*( (line1_end_b-line1_b)/abs(line1_end_b-line1_b) ) + line1_b;
        float left_b1 = robot_radius*sqrt(left_k*left_k + 1)*((left_offset_b-left_b)/abs(left_offset_b-left_b)) + left_b;
        float right_b1 = robot_radius*sqrt(right_k*right_k + 1)*((right_offset_b-right_b)/abs(right_offset_b-right_b)) + right_b;
        float down_b1 = robot_radius*sqrt(down_k*down_k) * ((down_offset_b-down_b)/abs(down_offset_b-down_b)) + down_b;

        float k_angle = atan(line1_k);//line1's angle
        std::cout<<"the line1 angle is: "<<k_angle<<std::endl<<std::endl;
        std::cout<<"the left_offset_b is: "<<left_offset_b<<std::endl<<std::endl;
        std::cout<<"the right_offset_b is: "<<right_offset_b<<std::endl<<std::endl;
        std::cout<<"the line1_b1 is: "<<line1_b1<<std::endl<<std::endl;
        std::cout<<"the left_b1 is: "<<left_b1<<std::endl<<std::endl;
        std::cout<<"the right_b1 is: "<<right_b1<<std::endl<<std::endl;

        pointIndex path_point;
        float x_start,y_start;
        float angle_diff = abs( (atan(down_k)-atan(line1_k)) / 3.1415 * 180);
        std::cout<<"angle_diff is : "<<angle_diff<<std::endl;
        std::cout<<"the iter is begin "<<std::endl<<std::endl<<std::endl;
        for(int i = 0; i< iter_count; i++)
        {
            float line1_offset_b = line1_b1 + i * plan_interval *sqrt(line1_k*line1_k + 1)*( (line1_end_b-line1_b)/abs(line1_end_b-line1_b) ) ;
            
            /*Find the intersection point of each baseline and edge*/
            float x0;
            float x1;
            if(line_distance1>line_distance2)
            {
                x0 = (left_b1 - line1_offset_b)/( line1_k - left_k);
                float b_direction = line1_end_b-line1_b ;
                if(b_direction >= 0)
                {
                    if(line1_offset_b <= line1_ts_b){ x1 = (right_b1 - line1_offset_b)/( line1_k - right_k); }
                    else { x1 = (down_b1 - line1_offset_b)/( line1_k - down_k); }
                }
                else
                {
                    if(line1_offset_b >= line1_ts_b){ x1 = (right_b1 - line1_offset_b)/( line1_k - right_k);  }
                    else { x1 = (down_b1 - line1_offset_b)/( line1_k - down_k); }                       
                }
            }
            else
            {
                x1 = (right_b1 - line1_offset_b)/( line1_k - right_k);
                float b_direction = line1_end_b-line1_b ;
                if(b_direction >= 0)
                {
                    if(line1_offset_b <= line1_ts_b){ x0 = (left_b1 - line1_offset_b)/( line1_k - left_k); }
                    else { x0 = (down_b1 - line1_offset_b)/( line1_k - down_k); }
                }
                else
                {
                    if(line1_offset_b >= line1_ts_b){ x0 = (left_b1 - line1_offset_b)/( line1_k - left_k); }
                    else { x0 = (down_b1 - line1_offset_b)/( line1_k - down_k); }                       
                }
            }
            if(angle_diff < 5 || angle_diff>175)
            {
                x0 = (left_b1 - line1_offset_b)/( line1_k - left_k);
                x1 = (right_b1 - line1_offset_b)/( line1_k - right_k); 
            }
            float y0 = line1_k * x0 + line1_offset_b;
            float y1 = line1_k * x1 + line1_offset_b;

            /*    std::cout<<"the iter is : "<<i<<std::endl;
            std::cout<<"the line1_offset_b is: "<<line1_offset_b<<std::endl<<std::endl;
            std::cout<<"(x0, y0)= "<<"("<<x0<<", "<<y0<<")"<<std::endl<<std::endl;
            std::cout<<"(x1, y1)= "<<"("<<x1<<", "<<y1<<")"<<std::endl<<std::endl;*/

            /*Solving iteration times*/
            float path_length = sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
            int path_iter = (int)(path_length/step_forward) + 1;
            
            /***The path points are generated from the line***/
            if(i%2 != 0) //pt1 to pt2
            {
                if(i!=0){cv::line(draw_image_, cv::Point(x_start,y_start), cv::Point(x0,y0), cv::Scalar(0,255,0) ) ;}
                cv::line(draw_image_, cv::Point(x0,y0), cv::Point(x1,y1), cv::Scalar(0,255,0) ) ;
                x_start = x1; y_start = y1;

                for(int u = 0; u<path_iter ;u++)
                {
                    float t = u * step_forward;
                    float x_direction = 1; float y_direction = 1;
                    if(x1-x0 == 0) x_direction = 0;
                    else x_direction = (x1-x0)/abs(x1-x0);

                    if(y1-y0 == 0) y_direction = 0;
                    else y_direction = (y1-y0)/abs(y1-y0);
                    int p_x = (int)(x0 + t * cos(k_angle) * x_direction);
                    int p_y = (int)(y0 + t * abs(sin(k_angle)) * y_direction);
                    path_point.col = p_x;
                    path_point.row = p_y;
                    path_point.theta = 90;
                    result.push_back(path_point);
                }
            }
            else
            {
                if(i!=0){cv::line(draw_image_, cv::Point(x_start,y_start), cv::Point(x1,y1), cv::Scalar(0,255,0) ) ;}
                cv::line(draw_image_, cv::Point(x1,y1), cv::Point(x0,y0), cv::Scalar(0,255,0) ) ;
                x_start = x0; y_start = y0;

                for(int u = 0; u<path_iter ;u++)
                {
                    float t = u * step_forward;
                    float x_direction = 1; float y_direction = 1;
                    if(x1-x0 == 0) x_direction = 0;
                    else x_direction = (x0-x1)/abs(x0-x1);

                    if(y1-y0 == 0) y_direction = 0;
                    else y_direction = (y0-y1)/abs(y0-y1);

                    int p_x = (int)(x1 + t * cos(k_angle) * x_direction);
                    int p_y = (int)(y1 + t * abs(sin(k_angle)) * y_direction);
                    path_point.col = p_x;
                    path_point.row = p_y;
                    path_point.theta = 90;
                    result.push_back(path_point);
                }
            }
        }
    
        return result;

    }
    else
    {
        return result;
    }
}

std::map<int, cv::Rect> FullCoveredPathPlanner::AutoRectZoning()
{
}

std::map<int, std::vector<geometry_msgs::PoseStamped> > FullCoveredPathPlanner::CalcMultiRegionPathInROS( std::map<int, std::vector<pointIndex> > &cvRegionsPath)
{
    std::map<int, std::vector<geometry_msgs::PoseStamped> > RectROSpaths;//The temporary variable to be returned

    std::map<int, std::vector<pointIndex> > paths_in_costmap;
    std::map<int, std::vector<pointIndex> >::iterator cv_path_iter;
    for(cv_path_iter = cvRegionsPath.begin(); cv_path_iter != cvRegionsPath.end(); cv_path_iter++)
    {
        paths_in_costmap[cv_path_iter->first] = CvImageToCostMap(cv_path_iter->second);
    }
    //Convert path points in Opencv to costMap

    /*Convert the target points in the CostMap to the actual target points to be published*/
    double PI = 3.141592;
    std::map<int, std::vector<pointIndex> >::iterator cost_iter;
    for(cost_iter = paths_in_costmap.begin(); cost_iter!= paths_in_costmap.end(); cost_iter++)
    {
        geometry_msgs::PoseStamped posestamped;
        geometry_msgs::Pose pose;

        std::vector<pointIndex>::iterator iter ;
        for(iter = (cost_iter->second).begin(); iter != (cost_iter->second).end(); iter++)
        {
            costmap2d_->mapToWorld((*iter).col, (*iter).row,  pose.position.x, pose.position.y);
            pose.orientation.w = cos((*iter).theta * PI / 180 / 2); //(sizey-(*iter).row-1)
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = sin((*iter).theta * PI / 180 / 2);
            posestamped.header.stamp= ros::Time::now();
            posestamped.header.frame_id = "map";
            posestamped.pose = pose;

            RectROSpaths[cost_iter->first].push_back(posestamped);           
        }
    }

    return RectROSpaths;
}
std::map<int, std::vector<geometry_msgs::PoseStamped> > FullCoveredPathPlanner::CalcMultiRegionPathInROS( std::map<int, std::vector<pointIndex> > &cvRegionsPath, std::map<int, std::vector<pointIndex> > &cvRegionsPathR)
{
    //The temporary variable to be returned
    std::map<int, std::vector<geometry_msgs::PoseStamped> > RectROSpaths;

    //Convert path points in Opencv to costMap
    std::map<int, std::vector<pointIndex> > paths_in_costmap;
    for(std::map<int, std::vector<pointIndex> >::iterator cv_path_iter = cvRegionsPath.begin(); cv_path_iter != cvRegionsPath.end(); cv_path_iter++)
    {
        paths_in_costmap[cv_path_iter->first] = CvImageToCostMap(cv_path_iter->second);
    }
    std::map<int, std::vector<pointIndex> > paths_in_costmap_r;
    for(std::map<int, std::vector<pointIndex> >::iterator cv_path_iter = cvRegionsPathR.begin(); cv_path_iter != cvRegionsPathR.end(); cv_path_iter++)
    {
        paths_in_costmap_r[cv_path_iter->first] = CvImageToCostMap(cv_path_iter->second);
    }
    // std::cout<<"the size of paths_in_costmap is :"<< paths_in_costmap.size()<<std::endl;
    // std::cout<<"the size of paths_in_costmap_r is :"<< paths_in_costmap_r.size()<<std::endl;
    for(int i = 0; i<best_clean_oder_.size(); i++)
    {
        std::map<int, std::string>::iterator iter = best_clean_oder_[i].begin();
        std::cout<<"the bestCleanOder is: "<<iter->first<< "the point is "<<iter->second<<std::endl;
    }


    /*Convert the target points in the CostMap to the actual target points to be published*/
    double PI = 3.141592;
    for(int i = 0; i < best_clean_oder_.size(); i++)
    {
        std::map<int, std::string>::iterator iter = best_clean_oder_[i].begin();
        std::vector<pointIndex> temp_path;
        if(iter->second == "left_start")
        {
            temp_path = paths_in_costmap[iter->first];
        }
        if(iter->second == "left_end")
        {
            temp_path = paths_in_costmap[iter->first];
            std::reverse(temp_path.begin(), temp_path.end());
        }

        if(iter->second == "right_start")
        {
            temp_path = paths_in_costmap_r[iter->first];
        }
        if(iter->second == "right_end")
        {
            temp_path = paths_in_costmap_r[iter->first];
            std::reverse(temp_path.begin(), temp_path.end());
        }

        geometry_msgs::PoseStamped posestamped;
        geometry_msgs::Pose pose;
        for(std::vector<pointIndex>::iterator iter_path = temp_path.begin(); iter_path != temp_path.end(); iter_path++)
        {
            costmap2d_->mapToWorld((*iter_path).col, (*iter_path).row,  pose.position.x, pose.position.y);
            pose.orientation.w = cos((*iter_path).theta * PI / 180 / 2); //(sizey-(*iter).row-1)
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = sin((*iter_path).theta * PI / 180 / 2);
            posestamped.header.stamp= ros::Time::now();
            posestamped.header.frame_id = "map";
            posestamped.pose = pose;

            RectROSpaths[iter->first].push_back(posestamped);           
        }

    }

    return RectROSpaths;
}



/*******************Other auxiliary functions*********************/
float FullCoveredPathPlanner::vectorDot(const cv::Point2i pt1, const cv::Point2i pt2)
{
    return (float)(pt1.x * pt2.x + pt1.y * pt2.y);
}

std::string FullCoveredPathPlanner::starAndEndCov(std::string soe)
{
    if(soe == "left_start"){ return "left_end";}
    if(soe == "left_end"){ return "left_start";}

    if(soe == "right_start"){ return "right_end";}
    if(soe == "right_end"){ return "right_start";}
}

std::map<bool, std::vector<int> > FullCoveredPathPlanner::ParallelogramCheck(const Polygon &pg)
{
    std::vector<int> index;
    std::vector<int> closed = {0, 0, 0, 0};
    std::map<bool, std::vector<int> >  result;
    if(pg.polygon.size() == 4)
    {
        for(int i = 0; i < pg.polygon.size() - 1; i++)
        {
            for(int j = i+1; j <  pg.polygon.size(); j++)
            {
                cv::Point2i vec1(pg.polygon[i].x - pg.polygon[j].x, pg.polygon[i].y - pg.polygon[j].y);
                
                index.clear();
                for(int k = 0; k < pg.polygon.size(); k++)
                {
                    if(k != i && k != j)
                    {
                        index.push_back(k);
                    }
                }
                std::cout <<"the i and j is : "<< i <<"&&"<<j<<" the shenxia is: "<<index[0]<<"&&"<<index[1] << std::endl;

                cv::Point2i vec2(pg.polygon[index[0]].x - pg.polygon[index[1]].x, pg.polygon[index[0]].y - pg.polygon[index[1]].y);
                // float theta = vec1.dot(vec2) / (vec1.dot(vec1) * vec2.dot(vec2) );
                // std::cout<<"the vec1 is: "<<vec1<<"*****"<<"the vec2 is"<<vec2<<std::endl;
                float theta = vectorDot(vec1, vec2) / sqrt(vectorDot(vec1, vec1) * vectorDot(vec2,vec2));
                float degree = acos(theta) / 3.1415 * 180;

                // std::cout <<"vectorDot(vec1, vec2) is : "<< vectorDot(vec1, vec2) << std::endl;
                // std::cout <<"vectorDot(vec1, vec1) is : "<< vectorDot(vec1, vec1) << std::endl;
                // std::cout <<"vectorDot(vec2,vec2) is : "<< vectorDot(vec2,vec2) << std::endl;
                // std::cout <<"the costheta is : "<< theta << std::endl;
                // std::cout <<"the angle is : "<< degree << std::endl;
                // std::cout<<std::endl<<std::endl<<std::endl;
                if(degree < 10 || degree > 170)
                {   
                    if(theta >= 0 && theta < 1)
                    {
                        closed[0] = j; closed[1] = i; closed[2] = index[0]; closed[3] = index[1];
                    }
                    else
                    {
                        closed[0] = j; closed[1] = i; closed[2] = index[1]; closed[3] = index[0];
                    }
                    std::cout <<"the closed is : "<<closed[0]<<closed[1]<<closed[2]<<closed[3]<< std::endl;
                    pg_closed_ = closed;
                    result[true] = closed;
                    return result;
                }
                index.clear();
            }
        }
        result[false] = closed;
        return result;
    }
    else
    {
        result[false] = closed;
        return result;
    }
}

int FullCoveredPathPlanner::whatRectContainsPoint(const cv::Point pt, const std::map<int, cv::Rect> &rects)
{
    int region;
    std::map<int, cv::Rect>::const_iterator iter;
    for(iter = rects.begin(); iter != rects.end(); iter++)
    {
        if( (iter->second).contains(pt) )
        {
            std::cout<<"The robot is located in region : "<<iter->first<<std::endl;
            return iter->first;
        }
    }
    std::cout<<"Could not find the robot in which area, please check what went wrong"<<std::endl;
    return 0;
}

float FullCoveredPathPlanner::pointLineDistance(const cv::Point &pt1, const cv::Point &pt2)
{
    float a = sqrt( (pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y) );
    return a;
}



/*******The interface that user get the planning results or show them********/
void FullCoveredPathPlanner::publishPlan(const std::map<int, std::vector<geometry_msgs::PoseStamped> > &paths)
{
    if (paths.empty()) {
        ROS_ERROR(
           "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    std::map<int, std::vector<geometry_msgs::PoseStamped> >::const_iterator iter;
    nav_msgs::Path gui_path;

    for(iter = paths.begin(); iter!= paths.end(); iter++)
    {
            //create a message for the plan
        // gui_path.poses.resize((iter->second).size());
        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < (iter->second).size(); i++) 
        {
            gui_path.poses.push_back( (iter->second)[i] ) ;
        }
    }
    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();
    plan_pub_.publish(gui_path);
}

void FullCoveredPathPlanner::publishOderPlan(const std::map<int, std::vector<geometry_msgs::PoseStamped> >& paths)
{
    if (paths.empty()) {
        ROS_ERROR(
           "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    nav_msgs::Path gui_path;
    for(int i = 0; i < region_oder_.size(); i++)
    {
        std::vector<geometry_msgs::PoseStamped> allpath = (paths.find(region_oder_[i]))->second ;
        for(int j = 0; j<allpath.size(); j++)
        {
            gui_path.poses.push_back(allpath[j]);
        }

    }

    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();
    plan_pub_.publish(gui_path);
}

std::map<int, std::vector<geometry_msgs::PoseStamped> > FullCoveredPathPlanner::GetNoStopPaths(const std::map<int, std::vector<geometry_msgs::PoseStamped> > &paths)
{
    std::map<int, std::vector<geometry_msgs::PoseStamped> > new_plans;
    std::map<int, std::vector<geometry_msgs::PoseStamped> >::const_iterator iter;
    for(iter = paths.begin(); iter!=paths.end(); iter++)
    {
        
        for(int i = 0; i<(int)(iter->second).size()-3; i++)
        {
        std::vector<float> line_1(2);
        std::vector<float> line_2(2);
        line_1[0] = (iter->second)[i].pose.position.x - (iter->second)[i+1].pose.position.x;
        line_1[1] = (iter->second)[i].pose.position.y - (iter->second)[i+1].pose.position.y;

        line_2[0] = (iter->second)[i+1].pose.position.x - (iter->second)[i+2].pose.position.x;
        line_2[1] = (iter->second)[i+1].pose.position.y - (iter->second)[i+2].pose.position.y;

        float th = sqrt(line_1[0]*line_1[0] + line_1[1]*line_1[1]) * sqrt(line_2[0]*line_2[0] + line_2[1]*line_2[1]) ;
        float costheta = (line_1[0]*line_2[0] + line_1[1]*line_2[1])/ th;
        float theta = acos(costheta) / 3.1415 * 180;
        // std::cout<<theta<<std::endl;
        if(theta > 20 && theta < 160 || (i==0 || i==(int)(iter->second).size()-4 ))
        {
            if(i == 0)
            {
                new_plans[iter->first].push_back((iter->second)[i]);
                continue;
            }
            if(i == (int)(iter->second).size()-3)
            {
                new_plans[iter->first].push_back( (iter->second)[(iter->second).size()-1] );
                continue;
            }
            new_plans[iter->first].push_back((iter->second)[i+1]);
        }

        }
    }

    return new_plans;
}


/************** The drawing function *******************/
cv::Mat FullCoveredPathPlanner::DrawExample(const std::vector<pointIndex> &path, const cv::Rect &rect)
{
    cv::Mat image = cv::Mat(640, 480, CV_8UC3, cv::Scalar(255,255,255));
    cv::rectangle(image, rect, cv::Scalar(0, 0, 255));
    for(int i=0; i<path.size()-1; i++)
    {
        cv::line(image, cv::Point(path[i].col, path[i].row), cv::Point(path[i+1].col, path[i+1].row), cv::Scalar(0,255,0));

    }
    return image;
}

void FullCoveredPathPlanner::DrawPolygon(cv::Mat &image, const std::vector<pointIndex> &path, const Polygon &pg)
{
    std::map<bool, std::vector<int> > mysb = ParallelogramCheck(pg);
    std::map<bool, std::vector<int> >::iterator iter = mysb.begin();
    std::vector<int> sq = iter->second;
    bool isok = iter->first;


    if(isok == false)
    {   std::cout<<"check is innormol"<<std::endl;
        return ;
    }

    for(int i=0; i< pg.polygon.size()-1; i++)
    {
        cv::line(image, pg.polygon[sq[i] ], pg.polygon[sq[i+1] ], cv::Scalar(255,0,0),4 ) ;
        if(i == pg.polygon.size()-2)
        {
            cv::line(image, pg.polygon[sq[i+1] ], pg.polygon[sq[0] ], cv::Scalar(255,0,0),4 ) ;
        }

    }
    if(path.size() == 0)
    {
        return ;
    }
    for(int i=0; i<path.size()-1; i++)
    {
        cv::line(image, cv::Point(path[i].col, path[i].row), cv::Point(path[i+1].col, path[i+1].row), cv::Scalar(0,255,0));
        // cv::imshow("myimage", image);
        // cv::waitKey(100);
    }
}




