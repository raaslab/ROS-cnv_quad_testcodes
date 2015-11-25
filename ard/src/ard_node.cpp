#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include "apriltags_ros/AprilTagDetectionArray.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>


using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;
ros::Publisher pioneer_pub;
geometry_msgs::Twist  pub_point;
geometry_msgs::Pose2D point1;
int first_image_frame=1;
cv::VideoWriter writer_apr;
cv::Mat image;

/*
void vanish_callback(const  geometry_msgs::Pose2D::ConstPtr& msg)
{
	  geometry_msgs::Pose2D point1;
	  geometry_msgs::Twist  pub_point;
	  //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'


	  //if(!(msg->x.emp)
	  //{
		  point1.x = msg->x;
		  point1.y = msg->y;

		  float px= 329.6735;
		  float py= 187.2286;
		  float error= (float)point1.x - px;
		  float k= 0.1;

		  pub_point.linear.x = 0.1;//'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]';
		  pub_point.linear.y = 0.0;
		  pub_point.linear.z = 0.0;
		  pub_point.angular.x = 0.0;
		  pub_point.angular.y = 0.0;
		  //pub_point.angular.z = 0.0; //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.1]';
		  pub_point.angular.z = -(k*error);

		  pioneer_pub.publish(pub_point);

	  //}

	  cout <<  "  X   " <<  (float)point1.x << endl;

	  cout <<  "                Y   " << (float)point1.y << endl;
}
*/
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	if (first_image_frame)
	{
		string filename_apr = "/home/ashishkb/Videos/april_tag_videos/aprtag.avi";
		int fcc = CV_FOURCC('D','I','V','3');
		int fps = 1;
		cv::Size frameSize(640,360);
		writer_apr = VideoWriter(filename_apr,fcc,fps,frameSize);

		while (!writer_apr.isOpened()) {
		cout<<"ERROR OPENING FILE FOR WRITE"<<endl; }


		first_image_frame = 0;
	}

	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_ptr_resized;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}


	image=cv_ptr->image;
	cv::namedWindow("image", CV_WINDOW_NORMAL);
	cv::resizeWindow("image", 500,800);


	circle(image, Point2f(point1.x,point1.y), 2, Scalar(255,0,255), 2, CV_AA);//Point2f(pub_point.linear.x,pub_point.linear.y)

	cv::imshow("image", image);
	writer_apr.write(image);

	cv::waitKey(10);


}

void vanish_callback(const  geometry_msgs::Pose2D& msg)
{

}


void chatterCallback(const  apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  //geometry_msgs::PoseArray april_data;
	//april_data->poses.size
  //std_msgs::Float64 x;
  //std_msgs::Float64 y;


  //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
  float error;
  float k= 0.00;

  if(!(msg->detections.empty()))
  {
	  point1.x = msg->detections[0].pose.pose.position.x;
	  point1.y = msg->detections[0].pose.pose.position.y;

	  float px= 329.6735;
	  float py= 187.2286;

	  error= (float)point1.x - px;
	  pub_point.linear.x = 0.0;//'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]';
	  pub_point.linear.y = 0.0;
	  pub_point.linear.z = 0.0;
	  pub_point.angular.x = 0.0;
	  pub_point.angular.y = 0.0;
	  //pub_point.angular.z = 0.0; //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.1]';
	  pub_point.angular.z = -(k*error);

	  pioneer_pub.publish(pub_point);

  }

  cout <<  "  X   " <<  (float)point1.x;

  cout <<  "                Y   " << (float)point1.y;
  cout <<  "                             (-k*(error))   " << (float)-(k*error) << endl;


}

int main(int argc, char **argv)
{
	 ros::init(argc, argv, "ard_node");
	 ros::NodeHandle n;
	 ros::Subscriber sub = n.subscribe("/tag_detections", 1, chatterCallback);
	 ros::Subscriber sub2 = n.subscribe("/vanishing_point", 1, vanish_callback);
	 ros::Subscriber image_sub = n.subscribe("/ardrone/front/image_rect_color", 1, imageCb); // /ardrone/front/image_rect_color
	// ros::Subscriber sub_vanish = n.subscribe("/vanishing_point", 1, vanish_callback);
	 //   /RosAria/cmd_vel
	 pioneer_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); //ros::Publisher
	 ros::spin();


	 return 0;
}
