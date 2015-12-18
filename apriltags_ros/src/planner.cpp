#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <stdlib.h>
#include <string>
#include <iostream>

#define DEBUG

//#define TEST_IMAGE
#define LAB

#define PLOT_PATH

#define NO_BOTS 5
#define NO_STATES 5

#define THRESHOLD 5

#include <iostream>
#include <stdio.h>
#include "PID.h"

#define SEND_LIGHT_CMD 0
#define WAIT_LIGHT 1
#define RUN_GSO 2
#define ROTATE 3
#define MOVE_FWD 4

#define PI 3.141592653

#define sensor_trim  1000
//#define sensor_b_trim  2000
//#define sensor_c_trim  3000

//#define sensor_max_reading_i  420
//#define sensor_max_reading_j  435
//#define sensor_max_reading_k  420

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

//int first_image_frame_planner = 1;

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

	ros::Subscriber bot_pose_sub[NO_BOTS];

	ros::Publisher nxt_command_pub[NO_BOTS];
	ros::Publisher nxt_light_pub[NO_BOTS];

	cv::Mat image;
	cv::Mat temp_result, temp_path;

	cv::VideoWriter writer_dynamic;
	cv::VideoWriter writer_static;
	cv::VideoWriter writer_path;
	cv::VideoWriter writer_bot_plan[NO_BOTS];

	string filename_bot_plan[NO_BOTS];

	string bot_pose_topic[NO_BOTS];
	geometry_msgs::Pose2D bot_pose[NO_BOTS];

	int target_intensity;
	int min_delta, min_delta_bot;

	int heading_correct_counter[NO_BOTS];

	int text_size_height;
	int iter_count;

	Point2f source_point;
	Point2f goal_point[NO_BOTS];

	int light_intensity[NO_BOTS], front_data[NO_BOTS];
	bool light_intensity_flag[NO_BOTS];
	bool front_move_flag[NO_BOTS];

	#ifdef LAB
		ros::Subscriber bot_intensity_sub[NO_BOTS];
	#endif
	ros::Subscriber bot_front_sub[NO_BOTS];

	bool bot_goal[NO_BOTS];
	bool goal;
	float goal_heading[NO_BOTS];

	String bot_window[NO_BOTS];

	int repelling_radius;

	int frame_count,callback_count, flag_cb, first_fwd, counter_waitlight;
	int collision_count[NO_BOTS];

	geometry_msgs::Twist nxt_command[NO_BOTS];

	int state;
	int first_image_frame;
	int frame_count_vector[NO_STATES];

	#ifdef PLOT_PATH
		cv::Point2f old_position[NO_BOTS];
		cv::Mat path;
		bool plot_start;
	#endif

	public:
	ImageConverter()
		: it_(nh_)
	{

		first_image_frame = 1;
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
		char temp[25];
		for(int i=0;i<NO_BOTS;i++)
		{
			light_intensity_flag[i]=false;
			front_move_flag[i]=false;

			sprintf(temp,"/nxt%d/pose",i);
			bot_pose_topic[i]=temp;
			sprintf(temp,"bot%d_plan",i);
			bot_window[i]=temp;
			#ifdef DEBUG
				cv::namedWindow(bot_window[i], CV_WINDOW_NORMAL);
			#endif
		}

		// Scale up
		bot_pose_sub[0]=nh_.subscribe(bot_pose_topic[0],1,&ImageConverter::bot0PoseCb,this);
		bot_pose_sub[1]=nh_.subscribe(bot_pose_topic[1],1,&ImageConverter::bot1PoseCb,this);
		bot_pose_sub[2]=nh_.subscribe(bot_pose_topic[2],1,&ImageConverter::bot2PoseCb,this);
		bot_pose_sub[3]=nh_.subscribe(bot_pose_topic[3],1,&ImageConverter::bot3PoseCb,this);
		bot_pose_sub[4]=nh_.subscribe(bot_pose_topic[4],1,&ImageConverter::bot4PoseCb,this);

		cv::namedWindow("result", CV_WINDOW_NORMAL);
		cv::resizeWindow("result", 500,800);
		cv::namedWindow("gso_result", CV_WINDOW_NORMAL);
		cv::resizeWindow("gso_result", 500,800);
		cv::namedWindow("Path", CV_WINDOW_NORMAL);
		cv::resizeWindow("Path", 500,800);
		#ifdef TEST_IMAGE
			for(int i=0;i<NO_BOTS;i++)
			{
				sprintf(temp,"bot%d_intensity",i);
				string label=temp;
				light_intensity[i]=(i+15)*10;
				cv::createTrackbar(label, "result", &light_intensity[i], 255, 0);	//Hue varies between 0 and 180
			}
		#endif
// capture video

//		cv::Size frameSize(cap.get(CV_CAP_PROP_FRAME_WIDTH),cap.get(CV_CAP_PROP_FRAME_HEIGHT));

		#ifdef LAB
//			for(int i=0;i<NO_BOTS;i++)
//			{
//				sprintf(temp,"Bot%dIntensityCb",i);
//				string callback=temp;
//				sprintf(temp,"Bot%dIntensityCb",i);
//				string callback=temp;
//
//			}
			bot_intensity_sub[0]=nh_.subscribe("/nxt0/intensity",1,&ImageConverter::bot0IntensityCb,this);
			bot_intensity_sub[1]=nh_.subscribe("/nxt1/intensity",1,&ImageConverter::bot1IntensityCb,this);
			bot_intensity_sub[2]=nh_.subscribe("/nxt2/intensity",1,&ImageConverter::bot2IntensityCb,this);
			bot_intensity_sub[3]=nh_.subscribe("/nxt3/intensity",1,&ImageConverter::bot3IntensityCb,this);
			bot_intensity_sub[4]=nh_.subscribe("/nxt4/intensity",1,&ImageConverter::bot4IntensityCb,this);

			bot_front_sub[0] = nh_.subscribe("/nxt0/front",1,&ImageConverter::bot0FrontCb,this);
			bot_front_sub[1] = nh_.subscribe("/nxt1/front",1,&ImageConverter::bot1FrontCb,this);
			bot_front_sub[2] = nh_.subscribe("/nxt2/front",1,&ImageConverter::bot2FrontCb,this);
			bot_front_sub[3] = nh_.subscribe("/nxt3/front",1,&ImageConverter::bot3FrontCb,this);
			bot_front_sub[4] = nh_.subscribe("/nxt4/front",1,&ImageConverter::bot4FrontCb,this);

		#endif

		#ifdef PLOT_PATH
			plot_start=false;
		#endif

		target_intensity=75;
		cv::createTrackbar( "Target Intensity", "result", &target_intensity, 1000, 0);

		repelling_radius=80;
		cv::createTrackbar( "Repelling Radius", "result", &repelling_radius, 250, 0);

		text_size_height=12;
//		bot_goal[0]=false; bot_goal[1]=false; bot_goal[2]=false;

		for (int i=0;i<NO_BOTS;i++)
		{
			collision_count[i] = 0;
			bot_goal[i] = false;
			heading_correct_counter[i] = 0;
		}

		frame_count=0;
		callback_count=0;
		counter_waitlight = 0;
		//flag_cb=1;
		first_fwd=1;

		for(int i=0;i<NO_BOTS;i++)
		{
			sprintf(temp,"/nxt%d/cmd_vel",i);
			string nxt_command_topic=temp;
			nxt_command_pub[i]=nh_.advertise<geometry_msgs::Twist>(nxt_command_topic,1);
		}
		for(int i=0;i<NO_BOTS;i++)
		{
			sprintf(temp,"/nxt%d/find_intensity",i);
			string nxt_light_topic=temp;
			nxt_light_pub[i]=nh_.advertise<std_msgs::Empty>(nxt_light_topic,1);
		}

		state=SEND_LIGHT_CMD;
	}

	~ImageConverter()
	{
		cv::destroyWindow("result");
	}

// Publishing front move acknowledgement on the same topic as reading light_intensity

	void bot0IntensityCb(const std_msgs::UInt16ConstPtr& msg) // scale up
	{
		light_intensity[0]=msg->data;
		cout<<"light 0:"<<light_intensity[0];
		light_intensity_flag[0]=true;

		if(light_intensity[0] >= target_intensity)
		{
			bot_goal[0] = false;
		}
		else
		{
			bot_goal[0] = true;
		}
	}

	void bot1IntensityCb(const std_msgs::UInt16ConstPtr& msg)
	{
		light_intensity[1]=msg->data;
		cout<<"light 1:"<<light_intensity[1];
		light_intensity_flag[1]=true;

		if(light_intensity[1] >= target_intensity)
		{
			bot_goal[1] = false;
		}
		else
		{
			bot_goal[1] = true;
		}
	}

	void bot2IntensityCb(const std_msgs::UInt16ConstPtr& msg)
	{
		light_intensity[2]=msg->data;
		cout<<"light 2:"<<light_intensity[2];
		light_intensity_flag[2]=true;

		if(light_intensity[2] >= target_intensity)
		{
			bot_goal[2] = false;
		}
		else
		{
			bot_goal[2] = true;
		}
	}

	void bot3IntensityCb(const std_msgs::UInt16ConstPtr& msg)
		{
			light_intensity[3]=msg->data;
			cout<<"light 3:"<<light_intensity[3];
			light_intensity_flag[3]=true;

			if(light_intensity[3] >= target_intensity)
			{
				bot_goal[3] = false;
			}
			else
			{
				bot_goal[3] = true;
			}
		}

	void bot4IntensityCb(const std_msgs::UInt16ConstPtr& msg)
		{
			light_intensity[4]=msg->data;
			cout<<"light 4:"<<light_intensity[4];
			light_intensity_flag[4]=true;

			if(light_intensity[4] >= target_intensity)
			{
				bot_goal[4] = false;
			}
			else
			{
				bot_goal[4] = true;
			}
		}

	void bot0PoseCb(const geometry_msgs::Pose2DConstPtr& msg) // Scale up
	{
		bot_pose[0].x=msg->x;
		bot_pose[0].y=msg->y;
		bot_pose[0].theta=msg->theta;
	}
	void bot1PoseCb(const geometry_msgs::Pose2DConstPtr& msg)
	{
		bot_pose[1].x=msg->x;
		bot_pose[1].y=msg->y;
		bot_pose[1].theta=msg->theta;
	}
	void bot2PoseCb(const geometry_msgs::Pose2DConstPtr& msg)
	{
		bot_pose[2].x=msg->x;
		bot_pose[2].y=msg->y;
		bot_pose[2].theta=msg->theta;
	}

	void bot3PoseCb(const geometry_msgs::Pose2DConstPtr& msg)
	{
		bot_pose[3].x=msg->x;
		bot_pose[3].y=msg->y;
		bot_pose[3].theta=msg->theta;
	}
	void bot4PoseCb(const geometry_msgs::Pose2DConstPtr& msg)
	{
		bot_pose[4].x=msg->x;
		bot_pose[4].y=msg->y;
		bot_pose[4].theta=msg->theta;
	}

	void bot0FrontCb(const std_msgs::UInt16ConstPtr& msg) // Scale up
	{
		front_data[0]=msg->data;
		cout<<"Front Move Flag for Bot 0 is TRUE with DATA "<<front_data[0]<<endl;
		front_move_flag[0]=true;
	}

	void bot1FrontCb(const std_msgs::UInt16ConstPtr& msg)
	{
		front_data[1]=msg->data;
		cout<<"Front Move Flag for Bot 1 is TRUE with DATA "<<front_data[1]<<endl;
		front_move_flag[1]=true;
	}

	void bot2FrontCb(const std_msgs::UInt16ConstPtr& msg)
	{
		front_data[2]=msg->data;
		cout<<"Front Move Flag for Bot 2 is TRUE with DATA "<<front_data[2]<<endl;
		front_move_flag[2]=true;
	}

	void bot3FrontCb(const std_msgs::UInt16ConstPtr& msg)
	{
		front_data[3]=msg->data;
		cout<<"Front Move Flag for Bot 3 is TRUE with DATA "<<front_data[3]<<endl;
		front_move_flag[3]=true;
	}

	void bot4FrontCb(const std_msgs::UInt16ConstPtr& msg)
	{
		front_data[4]=msg->data;
		cout<<"Front Move Flag for Bot 4 is TRUE with DATA "<<front_data[4]<<endl;
		front_move_flag[4]=true;
	}


	float convertToPi(float angle2pi)
	{
		float anglepi;
		if((angle2pi >= -PI) && (angle2pi <= PI)) {
         		anglepi = angle2pi; }
		else if((angle2pi >= -2*PI) && (angle2pi < -PI)) {
         		anglepi = angle2pi + 2*PI; }
		else if((angle2pi <= 2*PI) && (angle2pi > PI)) {
        		anglepi = angle2pi - 2*PI; }

        	return anglepi;
	}

	void insertText(Mat img, Point2f target, char text[], Scalar color)
	{

		int fontFace = CV_FONT_HERSHEY_PLAIN;
		double fontScale = 1.25;
		int thickness = 1.5;
		int baseline=0;
		Size textSize = getTextSize(text, fontFace,fontScale, thickness, &baseline);
		//baseline += thickness;
		// then put the text itself
		text_size_height=textSize.height;
		putText(img, text, target, fontFace, fontScale, color, thickness, 8);
	}
	int randomIndex(int delta[], int no)
	{
		int sum=0;
		for(int i=0;i<no;i++)
			sum+=delta[i];
		int random=(rand() % sum) + 1;
		sum=0;
		for(int i=0;i<no;i++)
		{
			sum+=delta[i];
			if(random<sum)
				return i;
		}
	}
	//Function to draw an arrow to represent vectors
	void drawArrow(Mat image, Point2f p, Point2f q, Scalar color, int arrowMagnitude = 9, int thickness=1)
	{
		//Draw the principle line
		line(image, p, q, color, thickness, CV_AA);
		//compute the angle alpha
		double angle = atan2((double)p.y-q.y, (double)p.x-q.x);
		//compute the coordinates of the first segment
		p.x = (q.x+arrowMagnitude*cos(angle+(PI/6)));
		p.y = (q.y+arrowMagnitude*sin(angle+(PI/6)));
		//Draw the first segment
		line(image, p, q, color, thickness, CV_AA);
		//compute the coordinates of the second segment
		p.x = (q.x+arrowMagnitude*cos(angle-(PI/6)));
		p.y = (q.y+arrowMagnitude*sin(angle-(PI/6)));
		//Draw the second segment
		line(image, p, q, color, thickness, CV_AA);
	}
	//Calculates the sum of two vectors (p1p2 and p3p4) and translates the tail of the resultant to point p5;
	Point2f vectorAddTranslate(Point2f p1, Point2f p2, Point2f p3, Point2f p4, Point2f p5)
	{
		p4=p4-(p3-p2);
		p4=p4-(p1-p5);
		return p4;
	}
	//Function to aid in the drawing of the heading vector;
	Point2f headingPointCalc(Point2f bot_pos, float heading)
	{
		//x'=xcos(theta)-ysin(theta)
		//y'=xsin(theta)+ycos(theta)
		Point2f temp;
		temp.x=50.0f*cosf(heading);
		temp.y=50.0f*sinf(heading);
		temp+=bot_pos;
		return temp;
	}
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		/*
		if(frame_count<15)
		{
			frame_count++;
			return;
		}
		elsewrapToPi
		{
			frame_count=0;
		}
		*/

		//Get a pointer to the image
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		#ifdef LAB
			image=cv_ptr->image;
		#endif
		#ifdef TEST_IMAGE
			image=imread("/home/paloma/Pictures/swarm_1.png");
		#endif

		#ifdef PLOT_PATH
			if(!plot_start)
			{
				path=Mat::zeros(image.size(), CV_8UC3);
				path=cv::Scalar(255,255,255);
			}
		#endif
		Mat result=image.clone();
		cv::Mat bot_plan[NO_BOTS];

		imshow("source", image);

		if(first_image_frame)
		{
			string filename_dynamic = "/home/ashish/Pictures/gso_videos/gso_result_dynamic.avi";
			string filename_static = "/home/ashish/Pictures/gso_videos/gso_result_static.avi";
			string filename_path = "/home/ashish/Pictures/gso_videos/gso_result_path.avi";


			int fcc = CV_FOURCC('D','I','V','3');
			int fps = 1;
			cv::Size frameSize(640,480);

			writer_dynamic = VideoWriter(filename_dynamic,fcc,fps,frameSize);
			while (!writer_dynamic.isOpened()) {
			cout<<"ERROR OPENING FILE FOR WRITE"<<endl; }

			writer_static = VideoWriter(filename_static,fcc,fps,frameSize);
			while (!writer_static.isOpened()) {
			cout<<"ERROR OPENING FILE FOR WRITE"<<endl; }

			writer_path = VideoWriter(filename_path,fcc,fps,frameSize);
			while (!writer_path.isOpened()) {
			cout<<"ERROR OPENING FILE FOR WRITE"<<endl; }

			filename_bot_plan[0] = "/home/ashish/Pictures/gso_videos/gso_bot0_plan.avi";
			filename_bot_plan[1] = "/home/ashish/Pictures/gso_videos/gso_bot1_plan.avi";
			filename_bot_plan[2] = "/home/ashish/Pictures/gso_videos/gso_bot2_plan.avi";
			filename_bot_plan[3] = "/home/ashish/Pictures/gso_videos/gso_bot3_plan.avi";
			filename_bot_plan[4] = "/home/ashish/Pictures/gso_videos/gso_bot4_plan.avi";


			for(int p=0; p<NO_BOTS; p++)
			{
				writer_bot_plan[p] = VideoWriter(filename_bot_plan[p],fcc,fps,frameSize);
				while (!writer_bot_plan[p].isOpened()) {
				cout<<"ERROR OPENING FILE FOR WRITE"<<endl; }
			}

			temp_result = result;
			temp_path = result;

			first_image_frame = 0;
		}

		//Mark the location of the bots
		for(int i=0;i<NO_BOTS;i++)
		{
			bot_plan[i]=image.clone();
			int delta=target_intensity-light_intensity[i];
			//Label each bot
			char label[30];
			sprintf(label,"B: %d",i);
			insertText(result, Point2f(bot_pose[i].x+5, bot_pose[i].y), label, Scalar(0,0,0));
			sprintf(label,"I: %d",light_intensity[i]);
			insertText(result, Point2f(bot_pose[i].x+5, bot_pose[i].y+(text_size_height)+2), label, Scalar(0,0,0));
		}
		//Main logic begins here
		//TODO:Replace the following with something more scalable
		Scalar bot_color[]={Scalar(0,0,255), Scalar(255,0,0), Scalar(0,255,0), Scalar(0,255,255), Scalar(255,0,255), Scalar(255,255,0), Scalar(128,128,255)};
		//Core logic begins here.

		bool light_initialised, front_initialised;

		if((state == SEND_LIGHT_CMD) && (callback_count == 1))
		{

			for(int i=0;i<NO_BOTS;i++)	//GSO Algorithm starts here. Calculate heading for intermediate goals.
			{
//				if(!light_intensity_flag[i])
//				{
					while (nxt_light_pub[i].getNumSubscribers() == 0);
//					ros::Duration(1).sleep();
					nxt_light_pub[i].publish(std_msgs::Empty());
					cout<<"publish "<<i<<endl;
//				}
			}

			state=WAIT_LIGHT;
			#ifdef DEBUG
				cout<<"state: SEND_LIGHT_COMMAND"<<endl;
			#endif
		}
		else if(state==SEND_LIGHT_CMD)
		{
			callback_count = 1;
		}

		if((state==WAIT_LIGHT) && (callback_count==2))
		{
			light_initialised = true;
			for(int i=0;i<NO_BOTS;i++)
			{
				#ifdef LAB
//					if(!bot_goal[i])
						light_initialised &= light_intensity_flag[i];

				#endif
			}
			if(light_initialised)
			{
				for(int i=0;i<NO_BOTS;i++)
				{
					for(int j=i+1;j<NO_BOTS;j++)
					{
						if(light_intensity[i] == light_intensity[j])
						{
							light_intensity[j] = light_intensity[j] + 1;
						}
					}
				}
				for(int i=0;i<NO_BOTS;i++)	//Reset new light intensities flag
				{
//					if(!bot_goal[i])
						light_intensity_flag[i] = false;
					Scalar bot_color[]={Scalar(0,0,255), Scalar(255,0,0), Scalar(0,255,0), Scalar(0,255,255), Scalar(255,0,255), Scalar(255,255,0), Scalar(128,128,255)};
					#ifdef PLOT_PATH
						if(plot_start && !bot_goal[i])
						{
							drawArrow(path, old_position[i], Point2f(bot_pose[i].x, bot_pose[i].y), bot_color[i]);
							imshow("Path",path);
							writer_path.write(path);
						}
					#endif
				}
				state = RUN_GSO;

			}
			#ifdef DEBUG
				cout<<"state: WAIT_LIGHT"<<endl;
			#endif
		}
		else if (state==WAIT_LIGHT)
		{
			callback_count = 2;
		}

		if((state == RUN_GSO) && (callback_count == 3))
		{
			for(int i=0;i<NO_BOTS;i++)
			{
				goal_heading[i] = 0;
				collision_count[i] = 0;
			}

			for(int i=0;i<NO_BOTS;i++)	//GSO Algorithm starts here. Calculate heading for intermediate goals.
			{
				Point2f p[4]={Point2f(0.0f,0.0f)};
				int delta_bot=abs(target_intensity-light_intensity[i]);
				if(light_intensity[i] < target_intensity)
				{
					#ifdef PLOT_PATH
					if(!bot_goal[i])
						circle(path, Point2f(bot_pose[i].x, bot_pose[i].y), 2, Scalar(0,0,0), 2, CV_AA);
					#endif
					writer_path.write(path);
					bot_goal[i]=true;
				}
				cout<<"----------"<<i<<" delta "<<delta_bot;

				bool flag_pull=false,flag_push=false;
				int pull_count=0,push_count=0;
				int pull_delta[NO_BOTS],push_delta[NO_BOTS];
				Point2f pull_bots[NO_BOTS],push_bots[NO_BOTS];
				for(int j=1;j<NO_BOTS;j++)
				{
					int index=(i+j)%NO_BOTS;
					if(light_intensity[i] > light_intensity[index])
					{
						//drawArrow(result, Point2f(bot_pose[i].x, bot_pose[i].y), Point2f(bot_pose[index].x, bot_pose[index].y), Scalar(255,255,0));
						#ifdef DEBUG
							drawArrow(bot_plan[i], Point2f(bot_pose[i].x, bot_pose[i].y), Point2f(bot_pose[index].x, bot_pose[index].y), Scalar(255,255,0));
						#endif
						pull_bots[pull_count]=Point2f(bot_pose[index].x, bot_pose[index].y);
						pull_delta[pull_count++]=abs(light_intensity[i] - light_intensity[index]);
						flag_pull=true;
					}
					else
					{
						drawArrow(result, Point2f(bot_pose[index].x, bot_pose[index].y), Point2f(bot_pose[i].x, bot_pose[i].y), Scalar(0,0,255));
						#ifdef DEBUG
							drawArrow(bot_plan[i], Point2f(bot_pose[index].x, bot_pose[index].y), Point2f(bot_pose[i].x, bot_pose[i].y), Scalar(0,0,255));
						#endif
						push_bots[push_count]=Point2f(bot_pose[index].x, bot_pose[index].y);
						push_delta[push_count++]=abs(light_intensity[i]-light_intensity[index]);
						flag_push=true;
					}
				}
				Point2f resultant;
				if(flag_pull)
				{
					p[0]=Point2f(bot_pose[i].x, bot_pose[i].y);
					p[1]=pull_bots[randomIndex(pull_delta,pull_count)];
					resultant=p[1];
				}
				if(flag_push)
				{
					p[2]=push_bots[randomIndex(push_delta,push_count)];
					p[3]=Point2f(bot_pose[i].x, bot_pose[i].y);
					resultant=(p[3]-p[2])+p[3];
				}
				if(flag_pull&flag_push)
				{
					//resultant=vectorAddTranslate(p[0],p[1],p[2],p[3],Point2f(bot_pose[i].x, bot_pose[i].y));
					resultant=p[1] + (p[3]-p[2])+p[3] - p[3];
				}
				bool collision_flag=false;
				for(int j=1;j<NO_BOTS;j++)
				{
					int index=(i+j)%NO_BOTS;
					if(sqrt(pow((bot_pose[index].x-bot_pose[i].x),2)+pow((bot_pose[index].y-bot_pose[i].y),2)) < repelling_radius)
					{
						if(light_intensity[i] > light_intensity[index])
						{
							if (collision_count[i] == 0)  // scale up
							{
								goal_heading[i]=convertToPi(atan2f((bot_pose[index].y-bot_pose[i].y),(bot_pose[index].x-bot_pose[i].x))+PI/2);
							}

							else
							{
								int perp_angle = convertToPi(atan2f((bot_pose[index].y-bot_pose[i].y),(bot_pose[index].x-bot_pose[i].x))+PI/2);

								if ((goal_heading[i]>=-PI) && (goal_heading[i]<=0))
								{
									goal_heading[i] = goal_heading[i] + 2*PI;
								}

								if ((perp_angle>=-PI) && (perp_angle<=0))
								{
									goal_heading[i] = goal_heading[i] + 2*PI;
								}

								goal_heading[i]=convertToPi(0.5*(goal_heading[i]+perp_angle));

							}
//							goal_heading[i]=atan2f((bot_pose[index].y-bot_pose[i].y),(bot_pose[index].x-bot_pose[i].x))-atan2f(tanf(PI/2),0.0f);
							collision_flag = true;
							collision_count[i]++;
							cout<<"Bot "<<i<<" under collision "<<" with goal heading "<<goal_heading[i];
						}
					}
				}
				if(!collision_flag)
					goal_heading[i]=atan2f((resultant.y-bot_pose[i].y),(resultant.x-bot_pose[i].x));
				//Mark the center of the bot
				circle(result, Point2f(bot_pose[i].x, bot_pose[i].y), 2, bot_color[i], 2, CV_AA);
				//Draw the heading vector
				drawArrow(result, Point2f(bot_pose[i].x, bot_pose[i].y), headingPointCalc(Point2f(bot_pose[i].x, bot_pose[i].y), bot_pose[i].theta), Scalar(0,0,0));
				//Draw the repelling circle
				circle(result, Point2f(bot_pose[i].x, bot_pose[i].y), repelling_radius, bot_color[i], 1, CV_AA);
//				float error_angle=goal_heading[i]-bot_pose[i].theta;
				drawArrow(result, Point2f(bot_pose[i].x, bot_pose[i].y), headingPointCalc(Point2f(bot_pose[i].x, bot_pose[i].y), goal_heading[i]), bot_color[i]);

				imshow(bot_window[i],bot_plan[i]);
				writer_bot_plan[i].write(bot_plan[i]);
			}
			imshow("gso_result",result);
			temp_result = result;
			writer_static.write(temp_result);
			writer_dynamic.write(temp_result);

			state=ROTATE;
			#ifdef PLOT_PATH
				if(plot_start)
				{
					for(int i=0;i<NO_BOTS;i++)	 //Once heading correct is complete, bots are facing the right direction. Move one step forward
					{
						old_position[i]=Point2f(bot_pose[i].x, bot_pose[i].y);
					}
				}
				else
				{
					for(int i=0;i<NO_BOTS;i++)	 //Once heading correct is complete, bots are facing the right direction. Move one step forward
					{
						old_position[i]=Point2f(bot_pose[i].x, bot_pose[i].y);
						circle(path, Point2f(bot_pose[i].x, bot_pose[i].y), 2, Scalar(0,0,0), 2, CV_AA);
						plot_start = true;
					}
				}
			#endif
			#ifdef DEBUG
				cout<<"state: RUN_GSO"<<endl;
			#endif
		}
		else if (state == RUN_GSO)
		{
			callback_count = 3;
		}
		bool heading_correct[NO_BOTS];


		bool heading_flag=false;

		if((state == ROTATE) && (callback_count == 4) && (frame_count > 2))
		{

			for (int p=0;p<NO_BOTS;p++)
			{
			//			cout<<"Heading Counter "<<p<<heading_correct_counter[p]<<endl;
						if ((heading_correct_counter[p] < 5) && (!bot_goal[p]))
						{
							heading_correct[p] = false;
						}
						else
						{
							heading_correct[p] = true;
						}
			}
			frame_count=0;
			for(int i = 0;i < NO_BOTS; i++)
			{
				//Mark the center of the bot
				circle(result, Point2f(bot_pose[i].x, bot_pose[i].y), 2, bot_color[i], 2, CV_AA);
				//Draw the heading vector
				drawArrow(result, Point2f(bot_pose[i].x, bot_pose[i].y), headingPointCalc(Point2f(bot_pose[i].x, bot_pose[i].y), bot_pose[i].theta), Scalar(0,0,0));
				//Draw the repelling circle
				circle(result, Point2f(bot_pose[i].x, bot_pose[i].y), repelling_radius, bot_color[i], 1, CV_AA);

				float error_angle_2pi = goal_heading[i] - bot_pose[i].theta;
				float error_angle = convertToPi(error_angle_2pi);

				drawArrow(result, Point2f(bot_pose[i].x, bot_pose[i].y), headingPointCalc(Point2f(bot_pose[i].x, bot_pose[i].y), goal_heading[i]), bot_color[i]);
				float error_angle_degrees=error_angle*57.29;

				if((abs(error_angle_degrees) > 5) && (heading_correct_counter[i] < 5))
				{
					//Correct heading;
//					cout<<"Bot "<<i<<" turn "<<error_angle_degrees<<endl;
					nxt_command[i].linear.x = 0.0;
					nxt_command[i].linear.y = 0.0;
					nxt_command[i].linear.z = 0.0;
					nxt_command[i].angular.x = 0.0f;
					nxt_command[i].angular.y = 0.0f;
					if(error_angle_degrees < 0.0f)
						nxt_command[i].angular.z=(error_angle_degrees/180.0f);//-0.5f;
					else
						nxt_command[i].angular.z=(error_angle_degrees/180.0f);//+0.5f;
					if(!bot_goal[i])
					{
						nxt_command_pub[i].publish(nxt_command[i]);
						ros::Duration(1).sleep();
					}
					else
					{
						heading_correct[i]=true;
					}

						heading_correct_counter[i] = 0;
				}
				else
				{
//					heading_correct[i]=true;
					heading_correct_counter[i] += 1;
				}
				#ifdef DEBUG
					cout<<i<<" error "<<(error_angle*57.29)<<"  "<<heading_correct_counter[i]<<endl;
					//drawArrow(bot_plan[i], Point2f(bot_pose[i].x, bot_pose[i].y), headingPointCalc(Point2f(bot_pose[i].x, bot_pose[i].y), goal_heading[i]), Scalar(0,255,0));
					//imshow(bot_window[i],bot_plan[i]);
				#endif
//				cout<<"Heading Correct for Bot "<<i<<" is "<<heading_correct[i]<<endl;
			}
			heading_flag=true;
			for(int j=0;j<NO_BOTS;j++)	//rotate robots until heading error is 0. Then translate one step.
			{
				heading_flag &= heading_correct[j];
			}
			if(heading_flag)
			{
				state=MOVE_FWD;
//				ros::Duration(2).sleep();
			}
			#ifdef DEBUG
				cout<<"state: ROTATE"<<endl;
			#endif

			// Write Image Frames to a Video File
			writer_dynamic.write(result);

			imshow("result", result);
		}
		else if (state == ROTATE)
		{
//			if (flag_cb == 1) {
			callback_count = 4;
//			flag_cb = 0;}

			frame_count++;
		}

		if(state==MOVE_FWD && callback_count==5)
		{
			front_initialised = true;

			for(int i=0;i<NO_BOTS;i++)	 //Once heading correct is complete, bots are facing the right direction. Move one step forward
			{
				heading_correct_counter[i] = 0;
				//Move  one step towards intermediate goal

				nxt_command[i].linear.x=1.0;
				nxt_command[i].linear.y=0.0;
				nxt_command[i].linear.z=0.0;
				nxt_command[i].angular.x=0.0f;
				nxt_command[i].angular.y=0.0f;
				nxt_command[i].angular.z=0.0;

				if(!bot_goal[i])
				{
//					ros::Duration(2).sleep();
//					while (nxt_command_pub[i].getNumSubscribers() == 0);
					if (first_fwd == 1)
					{
		//				while (nxt_command_pub[i].getNumSubscribers() == 0);
						nxt_command_pub[i].publish(nxt_command[i]);
						cout<<"Bot "<< i <<" move forward publish " << endl;
						nxt_command[i].linear.x=0.0;
						nxt_command[i].linear.y=0.0;
						nxt_command[i].linear.z=0.0;
						nxt_command[i].angular.x=0.0f;
						nxt_command[i].angular.y=0.0f;
						nxt_command[i].angular.z=0.0;
					}

					front_initialised &= front_move_flag[i];
				}
			}

			first_fwd = 0;

			if(front_initialised)
			{
				for (int p=0;p<NO_BOTS;p++)
				{
					front_move_flag[p]=false;
				}
				first_fwd = 1;
//				ros::Duration(1).sleep();
				state = SEND_LIGHT_CMD;
				callback_count = 0;
				//flag_cb = 1;

			}

			#ifdef DEBUG
				cout<<"state: MOVE_FWD"<<endl;
			#endif

		}
		else if (state == MOVE_FWD)
		{
			callback_count = 5;
		}

		goal = bot_goal[0]&bot_goal[1]&bot_goal[2]&bot_goal[3]&bot_goal[4]; // scale up
		if(goal)
		{
			insertText(result, Point2f(400.0f,(text_size_height)+2), "GOAL STATE REACHED!", Scalar(0,0,255));
		}

		if ((state != ROTATE) && (state != RUN_GSO))
		{
			writer_dynamic.write(result);
//			writer_static.write(temp_result);
//			writer_path.write(temp_path);
		}

		cv::waitKey(3);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "glow_planner");
	ImageConverter ic;
	ros::spin();
	return 0;
}
