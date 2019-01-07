#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <algorithm>
#include "geometry_msgs/Twist.h"


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//bool ipmLeftDone;
bool ipmDone;

/// Intrisic Camera parameters
/*
    [   alpha_x ,   beta    ,   x0
        0       ,   alpha_Y ,   y0
        0       ,   0       ,   1   ]
    alpha_x, alpha_y -> focal length
    x0, y0           -> principal point
*/
//------------------------Camera parameters----------------------------------------------
cv::Mat camera_intrinsic = (cv::Mat_<double>(3,3) << \
                                 130.81305          , 0                     , 79.81980 ,\
                                 0                  , 131.22421             , 58.91209,\
                                 0                  , 0                     , 1);

cv::Mat camera_T_chess_robot = (cv::Mat_<double>(4,4) <<  \
                                     -1,    0,  0,  0.612,\
                                     0,   -1,  0,  0.0,\
                                     0,    0,  1,  -0.004,\
                                     0,    0,  0,  1);

cv::Mat camera_T_cam_chess = (cv::Mat_<double>(4,4) << \
                                   0.330007,      0.918154,       0.219294,     -0.540327012,\
                                   0.562501,     -0.004706,      -0.826783,     -0.018409465,\
                                   -0.758082,      0.396197,      -0.518016,      1.038223574,\
                                   0       ,      0       ,       0       ,      1);
// Distortion coeficients
cv::Mat camera_dist_coef = (cv::Mat_<double>(1,4) << -0.275678598507515 , 0.045106260288961 ,
                                 0.004883645512607 , 0.001092737340199);


ros::Publisher dist_angle_pub;
ros::Publisher crossWalk_pub;

///--------------------------------------------------------------------------------------------------
/*void imageMergeAndTrack()
{
	double distIPM = 0.375;// meters
	double angleIPM = 0; // rad
	
	/// Publish Distance and Angle Info
	std_msgs::Float64MultiArray array;
	array.data.clear();
	array.data.push_back(distIPM);
	array.data.push_back(angleIPM);//angleIPM
	ros::Rate loop_rate(200);
	dist_angle_pub.publish(array);
	ros::spinOnce();
	loop_rate.sleep();

}*/

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
        
//Line detection
        //cv::Mat dst, cdst;
        //Canny(img_rgb, dst, 50, 200, 3); //Edge detector
        //cv::cvtColor(dst, cdst, CV_GRAY2BGR);
        
        //std::vector<cv::Vec2f> lines;
        //cv::HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 ); //Hough Line Transform 
                                                        //(dst binary image; 
                                                        //lines with parameters r, theta
                                                        //resolution of r 1 pixel
                                                        //resolution of theta (radians) 1degree
                                                        //threshold -minimum number of intersections to “detect” a line
                                                        // minLinLength minimum number of points for a line
                                                        //maxLineGap maximum gap between two points to be considered in the same line.
        //Draw the lines
        //for( size_t i = 0; i < lines.size(); i++ )
        //{
        //    float rho = lines[i][0], theta = lines[i][1];
        //    cv::Point pt1, pt2;
        //    double a = cos(theta), b = sin(theta);
        //   double x0 = a*rho, y0 = b*rho;
        //    pt1.x = cvRound(x0 + 1000*(-b));
        //    pt1.y = cvRound(y0 + 1000*(a));
        //    pt2.x = cvRound(x0 - 1000*(-b));
        //    pt2.y = cvRound(y0 - 1000*(a));
        //    cv::line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
        //}
        
        cv::imshow("rgb", img_rgb);
        //cv::imshow("detected lines", cdst);
        ipmDone = true;
        
        uint8_t k = cv::waitKey(1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    /*if(ipmRightDone && ipmLeftDone)
    {
        ipmLeftDone = false;
        ipmRightDone = false;
        imageMergeAndTrack();
    }*/
    return;
}

///--------------------------------------------------------------------------------------------------
///--------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{

    /// init variables
    ros::init(argc, argv, "alphabot2_tracking_node");
    ros::NodeHandle nh("~");
    cv::namedWindow("rgb");

    std::string cameraTopic;

    if(!(nh.getParam("camera", cameraTopic))){
        std::cerr << "Parameter (camera_) not found" << std::endl;
        return 0;
    }
    std::cout << "Parameter camera: " << cameraTopic <<  std::endl;

    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(cameraTopic, 1, imageCallback);

    dist_angle_pub = nh.advertise<std_msgs::Float64MultiArray>("/alphabot2_dist_angle", 1);
    crossWalk_pub = nh.advertise<std_msgs::Bool>("/crossWalk", 1);

        ros::Rate r(100);
        while(ros::ok())
        {
           ros::spinOnce();
            r.sleep();
        }
        
    cv::destroyWindow("view");
}
