#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
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
#include <signal.h>


using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;
float previous_error=0.0;

static const std::string OPENCV_WINDOW1 = "April window";
int first_image_frame=1;
sig_atomic_t volatile g_request_shutdown = 0;
int pubtake_done = 1;

int Glob_Kp_max = 1000;
int Glob_Kd_max = 1000;

int Kp = 390;
int Kd = 100;



/*cv::namedWindow(OPENCV_WINDOW1, CV_WINDOW_NORMAL);
cv::resizeWindow(OPENCV_WINDOW1, 500,800);*/
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


class code_ard
{




public:
	ros::NodeHandle nh_;

	image_transport::ImageTransport it_;
	image_transport::Subscriber im_sub_;
	ros::Subscriber sub_tagdetect;
	ros::Subscriber sub_vanish;
	ros::Publisher pub_ardrone_cmdvel;
	ros::Publisher pub_ardrone_takeoff;
	ros::Publisher pub_ardrone_land;
	ros::Publisher pub_ardrone_reset;
	ros::Publisher pub_check_destructor;
	ros::Publisher pub_check_speed;

	geometry_msgs::Twist  pub_point;
	geometry_msgs::Pose2D point1;

	cv::VideoWriter writer_apr;
	cv::Mat image;
	std_msgs::Empty doit;
	std_msgs::String desctruct_var;
	code_ard()
	:it_(nh_)
	{

		im_sub_ = it_.subscribe("/ardrone/front/image_rect_color", 1, &code_ard::imageCb, this);

		sub_tagdetect = nh_.subscribe("/tag_detections", 1, &code_ard::aprilCallback,this);
		sub_vanish = nh_.subscribe("/vanishing_point", 1, &code_ard::vanish_callback,this);
		pub_ardrone_cmdvel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		//pub_ardrone_takeoff = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
		pub_ardrone_land = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
	//	pub_ardrone_reset = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);

		pub_check_destructor = nh_.advertise<std_msgs::String>("/testdestruct",1);
		pub_check_speed = nh_.advertise<std_msgs::Empty>("/checkspeed",1);

		std::stringstream ss;
		ss << "destruction" << cout;
		desctruct_var.data = ss.str();
		cv::namedWindow("Kp Kd Tuning", 1);

		createTrackbar( "Kp ", "Kp Kd Tuning", &Kp, Glob_Kp_max);
		createTrackbar( "Kd ", "Kp Kd Tuning", &Kd, Glob_Kd_max);

		cv::namedWindow(OPENCV_WINDOW1);

//		  std_msgs::String msg;
//		  88
//		  89     std::stringstream ss;
//		  90     ss << "hello world " << count;
//		  91     msg.data = ss.str();
//		  92
//		  93     ROS_INFO("%s", msg.data.c_str());
	}
	~code_ard()
	{
		cout<<"in_distructer"<<endl;
		//pub_ardrone_reset.publish(doit); // doit is a empty msg
		//pub_ardrone_land.publish(doit);

		//ros::spinOnce();
	    cv::destroyWindow(OPENCV_WINDOW1);

	    cv::destroyWindow("Kp Kd Tuning");

	   // ros::Duration(1).sleep();

	}


	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
//		if (first_image_frame)
//		{
//			string filename_apr = "/home/ashishkb/Desktop/aprtag.avi";
//			int fcc = CV_FOURCC('D','I','V','3');
//			int fps = 10;
//			cv::Size frameSize(640,360);
//			writer_apr = VideoWriter(filename_apr,fcc,fps,frameSize);
//
//			while (!writer_apr.isOpened()) {
//			cout<<"ERROR OPENING FILE FOR WRITE"<<endl; }
//
//
//			first_image_frame = 0;
//		}

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
		cv::waitKey(3);

	//	circle(image, Point2f(point1.x,point1.y), 2, Scalar(255,0,255), 2, CV_AA);//Point2f(pub_point.linear.x,pub_point.linear.y)

	//	cv::imshow(OPENCV_WINDOW1, image);

		//writer_apr.write(image);

	//	cv::waitKey(3);


	}

	void vanish_callback(const  geometry_msgs::Pose2D& msg)
	{
		float error1; //
		float k2 = 0.00;

	//	geometry_msgs::Pose2D point1;
		//geometry_msgs::Twist  pub_point;
		  //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'


		  //if(!(msg->x.emp)
		  //{
		point1.x = msg.x;
		point1.y = msg.y;




		float px= 329.6735;
		float py= 187.2286;
		float current_error= (float)point1.x - px;
		float diff_error =previous_error-current_error;
		previous_error=current_error;
		float kp=0;//= 0.0037;  // 100 slider 10 -> 0.001 --- 100 -> 0.01
		float kd=0;//= 0.001;  /// 1000 slider 10 -> 0.0001  100 -> 0.001   1000 -> 0.01




		//waitKey(3);


		kp = (float)Kp/100000;  //370/100000 =
		kd = (float)Kd/100000;

		pub_point.linear.x = 0.0;//'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]';
		pub_point.linear.y = 0.0;
		pub_point.linear.z = 0.0;
		pub_point.angular.x = 0.0;
		pub_point.angular.y = 0.0;
		//pub_point.angular.z = 0.0; //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.1]';
		pub_point.angular.z = -(kp*current_error)+(kd*diff_error);//0.0;//
		if(pub_point.angular.z<-1)
		{
			pub_point.angular.z=-1;
		}
		else if(pub_point.angular.z>1)
		{
			pub_point.angular.z=1;
		}

		pub_ardrone_cmdvel.publish(pub_point);
		pub_check_speed.publish(doit);

		  //}
		cout <<   pub_point.angular.z << "   v   " << kp<< "       " <<kd <<endl; //kd*diff_error
		 // cout <<  "  X   " <<  (float)point1.x;

		//  cout <<  "                Y   " << (float)point1.y << endl;


		//ardrone_cmdvel.publish();
//		if(pubtake_done == 1)
//		{
//			pub_ardrone_reset.publish(doit);
//			pub_ardrone_reset.publish(doit);
//			ros::Duration(1).sleep();
//			pub_ardrone_takeoff.publish(doit);
//			cout<< "takeoff" << endl;
//			ros::Duration(3).sleep();
//			pubtake_done = 0;
//		}
		//cout<< "in_vanish_callback"<< endl;

	}

	void aprilCallback(const  apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
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

		  float current_error= (float)point1.x - px;
		  float diff_error =previous_error-current_error;
		  previous_error=current_error;
		  float kp=0;//= 0.0037;  // 100 slider 10 -> 0.001 --- 100 -> 0.01
		  float kd=0;//= 0.001;  /// 1000 slider 10 -> 0.0001  100 -> 0.001   1000 -> 0.01




		  		//waitKey(3);


		  kp = (float)Kp/1000;
		  kd = (float)Kd/1000;


		  error= (float)point1.x - px;
		  pub_point.linear.x = 0.0;//'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]';
		  pub_point.linear.y = 0.0;
		  pub_point.linear.z = 0.0;
		  pub_point.angular.x = 0.0;
		  pub_point.angular.y = 0.0;
		  //pub_point.angular.z = 0.0; //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.1]';
		  pub_point.angular.z = -(kp*current_error)+(kd*diff_error);

		  //pub_ardrone_cmdvel.publish(pub_point);
		  //pub_check_speed.publish(doit);
		  //pioneer_pub.publish(pub_point);
		  cout <<   pub_point.angular.z << "      " << kp << "       " << kd << endl;
	  }


	  //cout <<  "  X   " <<  (float)point1.x;

	  //cout <<  "                Y   " << (float)point1.y;
	  //cout <<  "                             (-k*(error))   " << (float)-(k*error) << endl;


	}

};


void mySigIntHandler(int sig)
{
		  g_request_shutdown = 1;
		  cout<<"in_sigint"<< endl;
		  code_ard ic_sig;
		//  ic_sig.pub_check_destructor.publish(ic_sig.desctruct_var);

		  ic_sig.pub_ardrone_land.publish(ic_sig.doit);

		  ros::Duration(3).sleep();
		 // ic_sig.pub_ardrone_reset.publish(ic_sig.doit);

		  ros::shutdown();
}


int main(int argc, char **argv)
{
	 ros::init(argc, argv, "ard_node", ros::init_options::NoSigintHandler);

//	 ros::NodeHandle n;
//	 ros::Subscriber sub = n.subscribe("/tag_detections", 1, aprilCallback);
//	 ros::Subscriber sub2 = n.subscribe("/vanishing_point", 1, vanish_callback);
//	 ros::Subscriber image_sub = n.subscribe("/ardrone/front/image_rect_color", 1, imageCb); // /ardrone/front/image_rect_color
//	// ros::Subscriber sub_vanish = n.subscribe("/vanishing_point", 1, vanish_callback);
//	 //   /RosAria/cmd_vel
//	 ardrone_cmdvel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //ros::Publisher
//	 ardrone_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
//	 ardrone_land = n.advertise<std_msgs::Empty>("/ardrone/land",1);;

	 code_ard ic;

	 signal(SIGINT, mySigIntHandler);
	 ros::spin();
	 //while(ros::ok());
	 ///ic.pub_check_destructor.publish(ic.desctruct_var);

	 return 0;
}
