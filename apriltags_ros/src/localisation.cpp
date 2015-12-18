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
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "structures.h"


#define DEBUG

//#define TEST_IMAGE
#define LAB

#define POSE_IP
//#define POSE_MAGNETO

#define NO_BOTS 5
#define NO_COLORS 2

#include <stdio.h>
#include "PID.h"


using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

///int first_image_frame_localization = 1;

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

	ros::Publisher bot_pose_pub[NO_BOTS];

	cv::Mat image;
	Mat bot_mask[NO_COLORS];
	dsfsdfdd
	cv::VideoWriter writer_localization;
	cv::VideoWriter writer_source;

	//Variables for thresholding
	int min_hue_bot[NO_COLORS];
	int max_hue_bot[NO_COLORS];
	int min_sat_bot[NO_COLORS];
	int max_sat_bot[NO_COLORS];
	int min_val_bot[NO_COLORS];
	int max_val_bot[NO_COLORS];

	string bot_window[NO_COLORS];
	string bot_pose_topic[NO_BOTS];

	geometry_msgs::Pose2D bot_pose[NO_BOTS];

	geometry_msgs::Pose2D temp_pose;
	int binary_value_array[NO_BOTS];

	int min_contour_area[2];

	blob location_blob[2*NO_BOTS+6];
	blob binary_blob[12];
	int front_index, rear_index, binary_index;

	robot bot[8];

	int first_image_frame;

	bool bot_blob_match[NO_BOTS];

	#ifdef POSE_IP
		int pose_ip_threshold;
	#endif
	public:
	ImageConverter()
		: it_(nh_)
	{

		first_image_frame = 1;
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
		char temp[15];
		for(int i=0;i<NO_BOTS;i++)
		{
			sprintf(temp,"/nxt%d/pose",i);
			bot_pose_topic[i]=temp;
			bot_pose_pub[i]=nh_.advertise<geometry_msgs::Pose2D>(bot_pose_topic[i],100);
		}
		cv::namedWindow("result", CV_WINDOW_NORMAL);
		cv::resizeWindow("result", 500,800);
		cv::namedWindow("source", CV_WINDOW_NORMAL);
		cv::resizeWindow("source", 500,800);
		cv::namedWindow("color0_detect", CV_WINDOW_NORMAL);
		cv::resizeWindow("color0_detect", 500,800);
		cv::namedWindow("color1_detect", CV_WINDOW_NORMAL);
		cv::resizeWindow("color1_detect", 500,800);

		//cv::namedWindow("bot2_detect", CV_WINDOW_NORMAL);

		#ifdef LAB
			//Bot1: red
			min_hue_bot[0]=152;
			max_hue_bot[0]=180;
			min_sat_bot[0]=95;
			max_sat_bot[0]=243;
			min_val_bot[0]=113;
			max_val_bot[0]=253;

			//Bot2: blue
			min_hue_bot[1]=95;
			max_hue_bot[1]=132;
			min_sat_bot[1]=86;
			max_sat_bot[1]=255;
			min_val_bot[1]=168;//133=night;
			max_val_bot[1]=255;

			/*
			//Bot3: yellow
			min_hue_bot[2]=29;
			max_hue_bot[2]=74;
			min_sat_bot[2]=71;
			max_sat_bot[2]=217;
			min_val_bot[2]=0;
			max_val_bot[2]=255;
			*/
		#endif

		#ifdef TEST_IMAGE
			//Bot1: red
			min_hue_bot[0]=151;
			max_hue_bot[0]=180;
			min_sat_bot[0]=115;
			max_sat_bot[0]=218;
			min_val_bot[0]=0;//106;
			max_val_bot[0]=255;

			//Bot2: blue
			min_hue_bot[1]=103;
			max_hue_bot[1]=144;
			min_sat_bot[1]=100;
			max_sat_bot[1]=211;
			min_val_bot[1]=140;//118=night;
			max_val_bot[1]=255;

			/*
			//Bot3: yellow
			min_hue_bot[2]=14;
			max_hue_bot[2]=43;
			min_sat_bot[2]=149;
			max_sat_bot[2]=201;
			min_val_bot[2]=0;
			max_val_bot[2]=255;
			*/
		#endif

		for(int i=0;i<NO_COLORS;i++)
		{
			sprintf(temp,"color%d_detect",i);
			bot_window[i]=temp;
		}
		for(int i=0;i<NO_COLORS;i++)
		{
			cv::createTrackbar( "Min Hue", bot_window[i], &min_hue_bot[i], 180, 0);	//Hue varies between 0 and 180
			cv::createTrackbar( "Max Hue", bot_window[i], &max_hue_bot[i], 180, 0);
			cv::createTrackbar( "Min Saturation", bot_window[i], &min_sat_bot[i], 255, 0);
			cv::createTrackbar( "Max Saturation", bot_window[i], &max_sat_bot[i], 255, 0);
			cv::createTrackbar( "Min Value", bot_window[i], &min_val_bot[i], 255, 0);
			cv::createTrackbar( "Max Value", bot_window[i], &max_val_bot[i], 255, 0);
		}

		min_contour_area[0]=20;
		min_contour_area[1]=20;
		cv::createTrackbar( "Color0: Minimum contour area", "result", &min_contour_area[0], 3000, 0);
		cv::createTrackbar( "Color1: Minimum contour area", "result", &min_contour_area[1], 3000, 0);

		//index has been configured to run with 7 robots
		front_index=0;
		rear_index=NO_BOTS;
		binary_index=0;

		#ifdef POSE_IP
			pose_ip_threshold=808;
			cv::createTrackbar( "Pose IP threshold", "result", &pose_ip_threshold, 6000, 0);
		#endif
		for(int i=0;i<NO_BOTS;i++)
			bot_blob_match[NO_BOTS]=false;
	}

	~ImageConverter()
	{
		cv::destroyWindow("result");
		cv::destroyWindow("source");
		cv::destroyWindow("color0_detect");
		cv::destroyWindow("color1_detect");
		//cv::destroyWindow("bot2_detect");
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
		int text_size_height;
		text_size_height=textSize.height;
		putText(img, text, target, fontFace, fontScale, color, thickness, 8);
	}



	float calculateDistance(Point2f p1, Point2f p2)
	{
		return sqrt(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2));
	}

	int findMedian(float input_vector[2*NO_BOTS])
	{
		int qMin, median;
		float temp_vector;
		for(int p=0;p<(2*NO_BOTS-1);p++)
		{
			qMin = p;
			for(int q=p+1;q<2*NO_BOTS;q++)
			{
				if (input_vector[q]<input_vector[qMin]) {
				qMin = q; }
			}
			if (qMin != p)
			{
				temp_vector=input_vector[p];
				input_vector[p]=input_vector[qMin];
				input_vector[qMin]=temp_vector;
			}
		}
		median = 0.5*(input_vector[NO_BOTS-1] + input_vector[NO_BOTS]);
		cout<<"Median "<<median<<endl;
		return median;
	}

	void findBlobs(Mat image,Mat result)
	{
		for(int i=0;i< NO_COLORS;i++)
		{
			bot_mask[i] = Mat::zeros(image.size(), CV_8UC1);
			inRange(image, cv::Scalar(min_hue_bot[i], min_sat_bot[i], min_val_bot[i]), cv::Scalar(max_hue_bot[i], max_sat_bot[i], max_val_bot[i]), bot_mask[i]);
			//dilate(bot_mask[i], bot_mask[i], Mat(Size(10, 10), CV_8UC1)); // Erode with a 30 x 30 kernel
			//erode(bot_mask[i], bot_mask[i], Mat(Size(10, 10), CV_8UC1)); // Erode with a 30 x 30 kernel
			imshow(bot_window[i], bot_mask[i]);

			vector<vector<Point> > contours;
			vector<vector<Point> > contours_accepted;

			findContours(bot_mask[i], contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );
			contours_accepted = contours;

			#ifdef POSE_IP
				Point2f bot_front, bot_rear;
			#endif
			//cout<<"Countours.size"<<"******"<<contours.size()<<"******"<<endl;
			int Rejected = 0;
			int Accepted = 0;
			int frontaccepted = 0;
			int backaccepted = 0;
			int frontarea = 0;
			int backarea = 0;

			if (i==1) {
			float contours_area_vector[NO_BOTS*2];
			int contours_index = 0;
			for(size_t j = 0; j < contours.size(); j++)
			{
				size_t count = contours[j].size();
				if( count < 6 ) {//  rejecting those countours which have less than 6 points
					cout<<"Rejected  <6 countour points"<< Rejected++ <<endl;
					continue;
					}
				//Reject contours with area lower than threshold


				if(contourArea(contours[j])<min_contour_area[i])
				{

					cout<<"Rejected  <min_contour_area"<< Rejected++ <<endl;
					continue;
				}
				else
				{
					contours_accepted[contours_index] = contours[j];
					contours_area_vector[contours_index]=contourArea(contours[j]);
//					cout<<"Contour Area Vector "<<contours_index<<" "<<contours_area_vector[contours_index]<<endl;
					contours_index++;
				}
			}

/*			for(size_t j = 0; j < contours.size(); j++)
			{

				size_t count = contours[j].size();
				if( count < 6 ) //  rejecting those countours which have less than 6 points
					continue;
				//Reject contours with area lower than threshold
				float contour_area=contourArea(contours[j]);
				if(contourArea(contours[j])<min_contour_area[i])
				{

					cout<<"Rejected  "<< Rejected++ <<endl;
					continue;
				}
				else
				{
					//cout<<"Accepted"<<endl;
				}
*/
			pose_ip_threshold = findMedian(contours_area_vector);
			cout<<"Pose IP Threshold "<<pose_ip_threshold<<endl;
		if (contours_index>=NO_BOTS*2) {

			for (int j=0; j<NO_BOTS*2;j++)
			{
				Mat pointsf,matellipse;
				Mat(contours_accepted[j]).convertTo(pointsf, CV_32F);
				RotatedRect box = fitEllipse(pointsf);
				#ifdef POSE_IP
				if(i==1)//&&j<(2*NO_BOTS))
				{
					Accepted++;
					cout<<"                                Accepted  "<< Accepted <<endl;
					int index;
					if (contourArea(contours_accepted[j])<pose_ip_threshold)
					{
						index=front_index;
						bot_front.x=(float)box.center.x;
						bot_front.y=(float)box.center.y;
						circle(result, box.center, 2, Scalar(0,255,255), 2, CV_AA);
						location_blob[front_index].location=box.center;
						location_blob[front_index++].size=contourArea(contours_accepted[j]);
						frontarea = contourArea(contours_accepted[j]);
						backarea = 0;
						frontaccepted++;
					}
					else
					{
						index=rear_index;
						bot_rear.x=(float)box.center.x;
						bot_rear.y=(float)box.center.y;
						circle(result, box.center, 2, Scalar(255,0,255), 2, CV_AA);
						location_blob[rear_index].location=box.center;
						location_blob[rear_index++].size=contourArea(contours_accepted[j]);
						backarea = contourArea(contours_accepted[j]);
						frontarea = 0;
						backaccepted++;

					}
					cout<<"                                              frontaccepted  "<< frontaccepted << "  " << frontarea<<endl;
					cout<<"                                                                      backaccepted  "<< backaccepted << "  "<< backarea<<endl;
					Point2f vtx[4];
					box.points(vtx);
					for(int k=0;k<4;k++)
					{
						line(result, vtx[(k%4)], vtx[((k+1)%4)], Scalar(0,255,0), 4, CV_AA);
						location_blob[index].vertex[k]=vtx[k];
						char label[30];
						sprintf(label,"%d",k);
						insertText(result, vtx[k], label, Scalar(0,0,0));
					}
					location_blob[index].slope=atan2f((vtx[0].y-vtx[1].y),(vtx[0].x-vtx[1].x));
					char label[30];
					sprintf(label,"%f",contours_area_vector[j]);
					//insertText(result, box.center, label, Scalar(0,0,0));
				}
				/*else if(i==0)
				{

					if (contourArea(contours[j])<pose_ip_threshold&&contourArea(contours[j])>10)
					{
						binary_blob[binary_index].location=box.center;
						binary_blob[binary_index++].size=contour_area;
					}
				}
				*/
				else
				{
					continue;
				}
				#endif
				#ifdef POSE_MAGNETO
				{
					bot_pose[i].x=(float)box.center.x;
					bot_pose[i].y=(float)box.center.y;
					bot_pose[i].theta=0.0f;
				}
				#endif
				char label[30];
//				sprintf(label,"B: %d, S: %f",j,contourArea(contours_accepted[j]));
				//insertText(result, box.center, label, Scalar(0,0,0));
				//ROS_INFO("location %f %f, iterator: %d, area: %f",box.center.x,box.center.y,j,contourArea(contours[j]));
			}
		}
//		else {}

			//for(int a=0;a<14;a++)
			//	ROS_INFO("location blob structure location: %f %f, area: %f",location_blob[a].location.x,location_blob[a].location.y,location_blob[a].size);
			//for(int b=0;b<9;b++)
			//	ROS_INFO("binary blob structure location: %f %f, area: %f",binary_blob[b].location.x,binary_blob[b].location.y,binary_blob[b].size);

			} // if (i==1) added as extra condition because nothing is happening for i=0
		}
		front_index=0;
		rear_index=NO_BOTS;
		binary_index=0;

	}

	void swapBlobs(blob *b1, blob *b2)
	{
		blob *temp;
		temp=b1;
		b1=b2;
		b2=temp;
	}

	Point2f findMidPoint(Point2f p1, Point2f p2)
	{
		Point2f result;
		result.x=(float)((p1.x+p2.x)/2.0f);
		result.y=(float)((p1.y+p2.y)/2.0f);
		return result;
	}

	void matchBlobsRobot(Mat result)
	{
		#ifdef POSE_IP
			//match the front and rear blobs of the robot
			for(int i=0;i<NO_BOTS;i++)
			{
				bool first_positive=true;
				float minimum_distance;
				int rear_match_index;
				for(int rear=NO_BOTS;rear<((NO_BOTS*2));rear++)
				{
					//if(abs(location_blob[i].slope-location_blob[rear].slope)<0.4)
					//{
						if(first_positive)
						{
							bot_blob_match[i]=true;
							minimum_distance=calculateDistance(location_blob[i].location,location_blob[rear].location);
							//ROS_INFO("F: %d R: %d D: %f",i,rear,minimum_distance);
							rear_match_index=rear;
							first_positive=false;
						}
						else
						{
//							ROS_INFO("F: %d R: %d D: %f ",i,rear,calculateDistance(Point2f(1.0f,1.0f),Point2f(2.0f,2.0f)));
							//ROS_INFO("F: %d R: %d D: %f %f %f",i,rear,calculateDistance(location_blob[i].location,location_blob[rear].location), location_blob[i].location.x, location_blob[i].location.y);
							if(calculateDistance(location_blob[i].location,location_blob[rear].location)<minimum_distance)
							{
								minimum_distance=calculateDistance(location_blob[i].location,location_blob[rear].location);
								rear_match_index=rear;
							}
						}
//						cout << i << " " << calculateDistance(location_blob[i].location,location_blob[rear_match_index].location)<< endl;
					//}
				}
				if(bot_blob_match[i]==true)
				{
					bot[i].front_index=i;
					bot[i].rear_index=rear_match_index;
					bot[i].position.x=(float)((location_blob[i].location.x+location_blob[rear_match_index].location.x)/2.0f);
					bot[i].position.y=(float)((location_blob[i].location.y+location_blob[rear_match_index].location.y)/2.0f);
					bot[i].heading=atan2f((location_blob[i].location.y-location_blob[rear_match_index].location.y),(location_blob[i].location.x-location_blob[rear_match_index].location.x));
	//				cout<<i<<" "<<(bot_pose[i].theta*57.29f)<<endl;
					circle(result, Point2f(bot[i].position.x,bot[i].position.y), 2, Scalar(0,0,255), 2, CV_AA);
					char label[30];
					sprintf(label,"%d",i);
					insertText(result, location_blob[i].location, label, Scalar(0,0,0));
					insertText(result, location_blob[rear_match_index].location, label, Scalar(0,0,0));
	//				swapBlobs(&location_blob[rear_match_index],&location_blob[(NO_BOTS+i)]);
				}
			}

			//match robot with binary code and publish location

//			int index1[]={0, 0, 3, 3};
//			int index2[]={0, 3, 0, 3};

			int index1[]={0, 3, 0, 2};
			int index2[]={3, 0, 1, 3};

			bool first_positive=true;
			int binary_front_index, binary_rear_index;
			for(int i=0;i<NO_BOTS;i++)
			{
			if(bot_blob_match[i]==true)
			{
				float distance;
				for(int j=0;j<4;j++)
				{
					if(first_positive)
					{
						distance=calculateDistance(location_blob[bot[i].front_index].vertex[index1[j]], location_blob[bot[i].rear_index].vertex[index2[j]]);
						binary_front_index=index1[j];
						binary_rear_index=index2[j];
						first_positive=false;
						//cout<<"bot: "<<i<<" "<<binary_front_index<<" "<<binary_rear_index<<" "<<distance<<endl;
					}
					else
					{
						if(calculateDistance(location_blob[bot[i].front_index].vertex[index1[j]], location_blob[bot[i].rear_index].vertex[index2[j]])<distance)
						{
							distance=calculateDistance(location_blob[bot[i].front_index].vertex[index1[j]], location_blob[bot[i].rear_index].vertex[index2[j]]);
							binary_front_index=index1[j];
							binary_rear_index=index2[j];
							//cout<<"bot: "<<i<<" "<<binary_front_index<<" "<<binary_rear_index<<" "<<distance<<endl;
						}
					}
				}
				first_positive=true;
//				cout<<"bot: "<<i<<" "<<binary_front_index<<" "<<binary_rear_index<<endl;
				Point2f midpoint_front, midpoint_rear, binary_zero, binary_one, binary_two;
				if(binary_front_index==0)
					midpoint_front=findMidPoint(location_blob[bot[i].front_index].vertex[0], location_blob[bot[i].front_index].vertex[1]);
				else
					midpoint_front=findMidPoint(location_blob[bot[i].front_index].vertex[2], location_blob[bot[i].front_index].vertex[3]);
				if ((binary_rear_index==0) || (binary_rear_index==1))
					midpoint_rear=findMidPoint(location_blob[bot[i].rear_index].vertex[0], location_blob[bot[i].rear_index].vertex[1]);
				else
					midpoint_rear=findMidPoint(location_blob[bot[i].rear_index].vertex[2], location_blob[bot[i].rear_index].vertex[3]);

				if(binary_front_index==0&&binary_rear_index==3)	//quadrants 3 and 4
				{
					binary_zero=findMidPoint(findMidPoint(location_blob[bot[i].rear_index].vertex[3], midpoint_rear), findMidPoint(location_blob[bot[i].front_index].vertex[0], midpoint_front));
					binary_two=findMidPoint(findMidPoint(location_blob[bot[i].rear_index].vertex[2], midpoint_rear), findMidPoint(location_blob[bot[i].front_index].vertex[1], midpoint_front));
				}
				else if(binary_front_index==3&&binary_rear_index==0)	//quadrants 1 and 2
				{
					binary_two=findMidPoint(findMidPoint(location_blob[bot[i].rear_index].vertex[0], midpoint_rear), findMidPoint(location_blob[bot[i].front_index].vertex[3], midpoint_front));
					binary_zero=findMidPoint(findMidPoint(location_blob[bot[i].rear_index].vertex[1], midpoint_rear), findMidPoint(location_blob[bot[i].front_index].vertex[2], midpoint_front));
				}

				// Additional 2 cases
				else if(binary_front_index==0&&binary_rear_index==1)
				{
					binary_two=findMidPoint(findMidPoint(location_blob[bot[i].rear_index].vertex[0], midpoint_rear), findMidPoint(location_blob[bot[i].front_index].vertex[1], midpoint_front));
					binary_zero=findMidPoint(findMidPoint(location_blob[bot[i].rear_index].vertex[1], midpoint_rear), findMidPoint(location_blob[bot[i].front_index].vertex[0], midpoint_front));
				}


				else if(binary_front_index==2&&binary_rear_index==3)
				{
					binary_two=findMidPoint(findMidPoint(location_blob[bot[i].rear_index].vertex[2], midpoint_rear), findMidPoint(location_blob[bot[i].front_index].vertex[3], midpoint_front));
					binary_zero=findMidPoint(findMidPoint(location_blob[bot[i].rear_index].vertex[3], midpoint_rear), findMidPoint(location_blob[bot[i].front_index].vertex[2], midpoint_front));
				}

				binary_one=findMidPoint(midpoint_front, midpoint_rear);
				#ifdef DEBUG
					circle(result, binary_zero, 2, Scalar(255,0,0), 2, CV_AA);
					circle(result, binary_one, 2, Scalar(255,0,0), 2, CV_AA);
					circle(result, binary_two, 2, Scalar(255,0,0), 2, CV_AA);
				#endif

				bot_pose[i].x=bot[i].position.x;
				bot_pose[i].y=bot[i].position.y;
				bot_pose[i].theta=bot[i].heading;
//				bot_pose_pub[i].publish(bot_pose[i]);

				int point_transform[5][2]={{1,0},{-1,0},{0,1},{0,-1},{0,0}};
				bool threshold_flag=false;
				int true_count=0;
				int binary_value = 0;
				Point2f binary_points[]={binary_zero, binary_one, binary_two};
				for(int i=0;i<3;i++)
				{
					for(int transform_index=0;transform_index<5;transform_index++)
					{
						Point2f check_point=Point2f(binary_points[i].x+point_transform[transform_index][1], binary_points[i].y+point_transform[transform_index][0]);
						//circle(result, check_point, 2, Scalar(0,255,0), 2, CV_AA);
						int value=bot_mask[0].at<uchar>(check_point.y,check_point.x);
						//cout<<value<<endl;
						if(value>0)
							true_count++;
					}
					if(true_count>0)
					{
						circle(result, binary_points[i], 2, Scalar(0,255,0), 2, CV_AA);
						binary_value+=pow(2,i);
					}
//					cout<<true_count<<endl;
					true_count=0;
				}
//				cout<<binary_value<<endl;
				binary_value_array[i] = binary_value;
			}
			}

/*	cout<<"Before Sorting"<<endl;
	for(int p=0;p<NO_BOTS;p++){
		cout<<binary_value_array[p]<<"     ";
		cout<<bot_pose[p].x<<" "<<bot_pose[p].y<<" "<<bot_pose[p].theta<<" "<<endl;
		}
*/

/*
	binary_value_array[0] = 5;
	binary_value_array[1] = 1;
	binary_value_array[2] = 3;
*/

	int qMin, temp_binary;

	// Sorting
	for(int p=0;p<(NO_BOTS-1);p++)
	{
		qMin = p;
		for(int q=p+1;q<NO_BOTS;q++)
		{
			if (binary_value_array[q]<binary_value_array[qMin]) {
				qMin = q; }
		}
		if (qMin != p)
		{
			temp_binary=binary_value_array[p];
			binary_value_array[p]=binary_value_array[qMin];
			binary_value_array[qMin]=temp_binary;

			temp_pose.x=bot_pose[p].x;
			bot_pose[p].x=bot_pose[qMin].x;
			bot_pose[qMin].x=temp_pose.x;

			temp_pose.y=bot_pose[p].y;
			bot_pose[p].y=bot_pose[qMin].y;
			bot_pose[qMin].y=temp_pose.y;

			temp_pose.theta=bot_pose[p].theta;
			bot_pose[p].theta=bot_pose[qMin].theta;
			bot_pose[qMin].theta=temp_pose.theta;
		}
	}

	cout<<"After Sorting"<<endl;
	for(int p=0;p<NO_BOTS;p++){
		cout<<binary_value_array[p]<<"     ";
		cout<<bot_pose[p].x<<" "<<bot_pose[p].y<<" "<<bot_pose[p].theta<<" "<<endl;
		bot_pose_pub[p].publish(bot_pose[p]);
		}


			for(int i=0;i<NO_BOTS;i++)
				bot_blob_match[i]=false;
		#endif
	}

	void localiseRobots(Mat image, Mat result)
	{
		findBlobs(image,result);
		matchBlobsRobot(result);

//		bot_pose_pub[i].publish(bot_pose[i]);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		if (first_image_frame)
		{
			string filename_localization = "/home/ashishkb/Videos/april_tag_videos/result.avi";
			string filename_source = "/home/ashish/Pictures/gso_videos/localization_source.avi";
			int fcc = CV_FOURCC('D','I','V','3');
			int fps = 1;
			cv::Size frameSize(640,480);
			writer_localization = VideoWriter(filename_localization,fcc,fps,frameSize);
			writer_source= VideoWriter(filename_source,fcc,fps,frameSize);

			while (!writer_localization.isOpened()) {
			cout<<"ERROR OPENING FILE FOR WRITE"<<endl; }

			while (!writer_source.isOpened()) {
			cout<<"ERROR OPENING FILE FOR WRITE"<<endl; }

			first_image_frame = 0;
		}


		//Get a pointer to the image
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

		#ifdef LAB
			//cv_ptr_resized =
			image=cv_ptr->image;
			//image=cv_ptr_resized->image;
		#endif
		#ifdef TEST_IMAGE
			image=imread("/home/achalarvind/Pictures/swarm_edit.jpg");
		#endif

		Mat result=image.clone();

		writer_source.write(result);
		imshow("source", image);
		GaussianBlur( image, image, Size( 5, 5 ), 0, 0 );
		cv::cvtColor(image, image, CV_BGR2HSV);

		localiseRobots(image,result);
		//Find the pair of surfaces and then decode which pair is associated with each robot

		writer_localization.write(result);
		imshow("result", result);
		cv::waitKey(3);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "glow_localisation");
	ImageConverter ic;
	ros::spin();
	return 0;
}
