
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>

using namespace cv;
using namespace std;

int Hough_slider=50;
int CannyThres1=50;
int CannyThres2=200;
int first_image_frame=1;

static const std::string OPENCV_WINDOW = "Image window";

class codebridge
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	geometry_msgs::Pose2D vanish_point;
	ros::Publisher vanish_pub;
	cv::VideoWriter writer_vanish;


public:
	codebridge():it_(nh_)
            {
		       // Subscrive to input video feed and publish output video feed
		       image_sub_ = it_.subscribe("/ardrone/front/image_rect", 1,&codebridge::imageCb, this);
		       vanish_pub = nh_.advertise<geometry_msgs::Pose2D>("/vanishing_point", 1);;

		       //image_pub_ = it_.advertise("/codebridge/output_video", 1);
		       cv::namedWindow(OPENCV_WINDOW);
		    }

	~codebridge()
	     {
	       cv::destroyWindow(OPENCV_WINDOW);
	     }

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		if (first_image_frame)
		{
			string filename_apr = "/home/ashishkb/Desktop/vanish_vid.avi";
			int fcc = CV_FOURCC('D','I','V','3');
			int fps = 10;
			cv::Size frameSize(640,360);
			writer_vanish = VideoWriter(filename_apr,fcc,fps,frameSize);

			while (!writer_vanish.isOpened()) {
			cout<<"ERROR OPENING FILE FOR WRITE"<<endl; }


			first_image_frame = 0;
		}
		cv_bridge::CvImagePtr cv_ptr;
		 try
         {
	         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
         }
		 catch (cv_bridge::Exception& e)
		 {
			 ROS_ERROR("cv_bridge exception: %s", e.what());
			 return;
		 }

//////////////////////////////////////MY CODE///////////////////////////////////////////////////

		 Mat gra,The_Vid;

		The_Vid=cv_ptr->image;
		GaussianBlur( cv_ptr->image,cv_ptr-> image, Size( 5, 5 ), 0, 0 );
		 cvtColor( cv_ptr->image, gra, CV_BGR2GRAY );

		////trackbar

			/*  namedWindow("Manual Tuning", 1);

			  createTrackbar( "Hough line Threshold", "Manual Tuning", &Hough_slider, hough_slider_max);
			  createTrackbar( "Canny Threshold 1", "Manual Tuning", &CannyThres1, canny1_slider_max);
			  createTrackbar( "Canny Threshold 2", "Manual Tuning", &CannyThres2, canny2_slider_max);

			  waitKey(0);*/

		////edge detection through canny and display the binary image
		     Mat dst,cdst, edgevideo;
			 Canny(gra, dst, CannyThres1,CannyThres2, 3);
			 cdst=dst.clone();

		////Detect lines using hough transform function
			  vector<Vec2f> lines, vidlines;
				   HoughLines(cdst, lines, 1, CV_PI/180, Hough_slider, 0, 0 );
				   for( size_t i = 0; i < lines.size(); i++ )
				   {
				      float rho = lines[i][0], theta = lines[i][1];
				      Point pt1, pt2;
				      double a = cos(theta), b = sin(theta);
				      double x0 = a*rho, y0 = b*rho;
				       //Draw the lines on the color image
				      if((theta>(10+90)*(CV_PI/180) && theta<(90+80)*(CV_PI/180))||(theta>10*(CV_PI/180) && theta<80*(CV_PI/180)))
				      {

				      pt1.x = cvRound(x0 + 6000*(-b));

				      pt1.y = cvRound(y0 + 6000*(a));

				      pt2.x = cvRound(x0 - 6000*(-b));

				      pt2.y = cvRound(y0 - 6000*(a));

				      line( cv_ptr->image, pt1, pt2, Scalar(255,255,255), 1, CV_AA);

				      }
				   }

		///TEST//////////////////////////////////////////////////DON'T SEE//////////////////////////


		///TEST////////////////////////////////////////////////////////////DON'T SEE ABOVE//////////////////////



		////initialize an matrix for storing b-m values
		      int ct=0;
		      Mat B(10000, 2, DataType<float>::type);
		      for(int i=0;i<10000;i++)
		           {
		         	  for(int j=0;j<2;j++)
		         	  {
		         		 B.at<float>(i,j)=-1;
		         	  }
		           }




		////Storing values of b and m for every value of rho and theta
			   for( size_t i = 0; i < lines.size(); i++ )
			   {
				   float rho = lines[i][0], theta=lines[i][1];
				   Point pt3, pt4;
				   float b,m;
				   if((theta>(10+90)*(CV_PI/180) && theta<(90+80)*(CV_PI/180))||(theta>10*(CV_PI/180) && theta<80*(CV_PI/180)))
				  {
				   b=rho/sin(theta);
		           B.at<float>(ct,0)=b;
		     	   m=cos(theta)/sin(theta);
				   B.at<float>(ct,1)=m;
				//   cout<<b<<"             "<<m<<"_____________________________"<<B.at<float>(ct,0)<<"          "<<B.at<float>(ct,1)<<"\n";
				   ct=ct+1;

				   }
			   }

		////Reading the size of the b-m matrix
		       int sz=0;
			   int numofpt;
			   for(int i=0;i<10000;i++)
		       {
		     	  for(int j=0;j<2;j++)
		     	  {
		     		  if(B.at<float>(i,j)!=-1)
			     	  {
			     		   sz++;
			     	  }

				  }
			   }

			   numofpt=sz/2;


		////RANSAC implementation

			   int ct2;                              //count variable
			   int numinlier;                        //number of inliers
			   int itr=500;                          //required iterations
		       int maxInliers=0;                     //Max number of inliers initialization
		       int maxInliers_num;

		       float x1,y1,x2,y2,xa,ya,dp;           //(x1,y1) & (x2,y2) points connecting a line. (xa,ya) chosen point. dp=length of the line connecting 2 points

		       float slopeBest=0;                    //initialization of slope of best fit line
		       float slope,LineIntercept;            //slope and intercepts of lines
		       float LineInterceptbest=0;            //intercept of best line
		       float inlierThresh=0.3;               //percentage of rejected inliers
		       float inlierRatio=0.4;            //percentage of points chosen as inliers
		       int k;                                //iteration variable upto value of 'itr'
		       int ex=1;                             //a parameter to ensure generation of different random variables
		       int r1,r2;                            //two random variables denotiong index of chosen points
		       Mat chosenpt(2, 2, DataType<float>::type),bestpt(2, 2, DataType<float>::type);
		       Mat inlierind(numofpt, 1, DataType<int>::type);
		       Mat bestinlierind(numofpt, 1, DataType<int>::type);
		       Mat distmat(numofpt, 1, DataType<float>::type),distmatab(numofpt, 1, DataType<float>::type);

		       //Initializing values to -1 in the matrix inlier indexes



if(ct>6)
{
		      //Part of code to find the best line fit
		      srand(time(0));
		      for(k=0;k<=itr;k++)
		       {
		    	//re-initialize inlier indexes
		    	  for(int i=0;i<numofpt;i++)
		    	   {
		    	      inlierind.at<int>(i,0)= (int)-1; //the value of -1 appears for unused positions

		    	   }
		    	  for(int i=0;i<numofpt;i++)
		    	  {
		    	  		distmat.at<float>(i)=-1; //the value of -1 appears for unused positions
		    	  }
		      //Generate two random numbers between 0 to numofpt
		            while(ex==1)
		            {
		    	       r1=rand() %(numofpt) ;
		               r2=rand() %(numofpt) ;
		               if(r1!=r2)
		   		       {
		   	        	  ex=0;
		   		       }
		            }
		        //       cout<<"\n"<<r1<<"\n"<<r2<<"\n";
		            ex=1;
		      //Store the values of the chosen points
		           for(int i=0;i<numofpt;i++)
		           {
		        	   for(int j=0;j<2;j++)
		        	   {
		        		   if(i==r1)
		        		   {
		        			   chosenpt.at<float>(0,0)=B.at<float>(i,0);
		        			   x1=chosenpt.at<float>(0,0);
		        			   chosenpt.at<float>(0,1)=B.at<float>(i,1);
		        			   y1=chosenpt.at<float>(0,1);
		        		   }
		        		   if(i==r2)
		        		   {
		        			   chosenpt.at<float>(1,0)=B.at<float>(i,0);
		        			   x2=chosenpt.at<float>(1,0);
		        			   chosenpt.at<float>(1,1)=B.at<float>(i,1);
		        			   y2=chosenpt.at<float>(1,1);
		        		   }
		        	   }

		           }

		      //Calculate the distance of every point from the lines formed by connecting two points

		           for(int i=0;i<numofpt;i++)
		           {
		            	if(i==r1) //(i!=r1)||(i!=r2))
		            	{
		            		distmat.at<float>(i,0)=(float)0.0;

		            	}
		            	else if(i==r2)
		            	{
		            		distmat.at<float>(i,0)=(float)0.0;
		            	}
		            	else
		            	{
							xa=B.at<float>(i,0);
							ya=B.at<float>(i,1);
							dp=sqrt(pow((y2-y1),2)+pow((x2-x1),2));
							distmat.at<float>(i,0)=abs(((y2-y1)*xa)-((x2-x1)*ya)+(x2*y1)-(y2*x1))/dp;
		            	}

		           }


		      //store the values of inliers in an array that are close the selected line. The closeness is defined by a threshold
		           ct2=0;
		           for(int i=0;i<numofpt;i++)
		           {
		        	   if(distmat.at<float>(i,0)<=inlierThresh)
		        	   {
		        		   inlierind.at<int>(ct2,0)= (int)i;
		        		   ct2=ct2+1;
		        	   }
		           }
		          // cout<<inlierind<<endl;
		      //Find the number of inliers


		           numinlier=0;
		           for(int i=0; i<numofpt;i++)
		           {
		        	   if(inlierind.at<int>(i,0)!=(int)-1)
		        		   numinlier=numinlier+1;
		           }

		      //Error showing the need to increase the threshold. This is because of
		          if((numinlier<round(inlierRatio*numofpt)))
		          {
		        	//  cout<<"\nplease reduce the inlier ratio or increase the number of points by increasing the threshold\n";
		              //break;
		          }
		      //properties of the chosen line
		          if((numinlier>=round(inlierRatio*numofpt))&&(numinlier>maxInliers))
		          {
		        	  maxInliers=numinlier;
		        	  slope=(y2-y1)/(x2-x1);
		        	  LineIntercept = y1-(slope*x1);
		        	  slopeBest=slope;
		        	  LineInterceptbest = LineIntercept;
		        	  maxInliers_num = maxInliers;
		        	  bestpt=chosenpt.clone();
		        	  bestinlierind=inlierind.clone();

		          }
		         // cout << "maxInliers " << maxInliers << endl;


		       }
		      int ct4=0;

		       for(int i=0;i<numofpt;i++)
		       {
		    	   if(bestinlierind.at<int>(i)!=-1)
		         {

		    		   ct4=ct4+1;

		         }
		       }

		////Least Square Fitting implementation
		      Mat chosenInlier(ct4, 2, DataType<float>::type);
		      Mat reqLine(1, 4, DataType<float>::type);
		      Mat mpt(2, 2, DataType<float>::type);
		      Mat bpt(2, 1, DataType<float>::type);
		      Mat XYpoints(2, 1, DataType<double>::type);
		      //store the values of the chosen inliers in an array

		          int ix,ct3=0;
		          for(int i=0;i<numofpt;i++)
		          {
		        		  if(bestinlierind.at<int>(i)!=-1)
		        		  {
		        			      ix=bestinlierind.at<int>(i);
		        			  	  chosenInlier.at<float>(ct3,0)=B.at<float>(ix,0);
		        			  	  chosenInlier.at<float>(ct3,1)=B.at<float>(ix,1);
		        			  	  ct3=ct3+1;

		        		  }
		          }

		      //declare variables for least square fit output
		          float xp,yp,vx,vy; //(xp,yp)- value of b and m in bm space.
		                             //the vector along the chosen line is resolved into vx and vy


		      //Finding the parameters of the best fit line.
		          cv::fitLine(chosenInlier,reqLine,CV_DIST_L2,0,0.01,0.01);

		      //Storing the parameters in their respective variables
		         vx=reqLine.at<float>(0,0);
				 vy=reqLine.at<float>(0,1);
				 xp=reqLine.at<float>(0,2);
				 yp=reqLine.at<float>(0,3);

		      //forming a 2x2 matrix made from two equations -b1=m1 x - y & -b2=m2 x - y
				 mpt.at<float>(0,0)=1;
				 mpt.at<float>(0,1)=-1;
				 mpt.at<float>(1,0)=vy+yp;  //m2
				 mpt.at<float>(1,1)=-yp;    //m1

		      //forming a 2x1 matrix with values of -b1 and -b2
				 bpt.at<float>(0,0)=-xp;
				 bpt.at<float>(1,0)=-(xp+vx);

		      //Finding the values of (x,y) from the given pair of (b1,m1) and (b2,m2)
				 XYpoints= abs((1/(vy))*mpt*bpt);//this array stores the (x,y) value of vanishing point



		      //displaying the vanishing point
				 int rows,cols;

				 cv::Size s = (cv_ptr->image).size();
				 rows = s.height;
				 cols = s.width;


				 Point pt3,pt4,pt5,pt6,pt7;

				 pt3.x = 0;
				 pt3.y = 0;
				 pt4.x = cvRound(XYpoints.at<float>(0,0));
				 pt4.y = 0;
		         pt5.x = cvRound(XYpoints.at<float>(0,0));
		         pt5.y = rows;
		         pt6.x = 0;
		         pt6.y = cvRound(XYpoints.at<float>(1,0));
		         pt7.x = cols;
		         pt7.y = cvRound(XYpoints.at<float>(1,0));


				 line( cv_ptr->image, pt4, pt5, Scalar(0,0,255), 2, CV_AA);

				 line( cv_ptr->image, pt6, pt7, Scalar(0,0,255), 2, CV_AA);

		    	// imshow("vanish point Image",cv_ptr->image);

				 vanish_point.x  = cvRound(XYpoints.at<float>(0,0));
				 vanish_point.y  = cvRound(XYpoints.at<float>(1,0));

				 vanish_pub.publish(vanish_point);

				 cv::imshow(OPENCV_WINDOW, cv_ptr->image);
				 writer_vanish.write(cv_ptr->image);
				 cv::waitKey(3);
           }
		   else
		   {
			   cv::imshow(OPENCV_WINDOW, cv_ptr->image);
			   writer_vanish.write(cv_ptr->image);
			   cv::waitKey(3);
		   }

//////////////////////////////////////MY CODE//////////////////////////////////////////////////


	}
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "hough_test");
	 codebridge ic;
	 ros::spin();
	 return 0;
}


























/*
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "cvplot.h"
//#include <koolplot.h>

#include <iostream>
//#include
//#define rowPtr(imagePtr, dataType, lineIndex) \
	//    (dataType *)(imagePtr->imageData + (lineIndex) * imagePtr->widthStep)

using namespace cv;
using namespace std;

void help()
{
 cout << "\nThis program demonstrates line finding with the Hough transform.\n"
         "Usage:\n"
         "./houghlines <image_name>, Default is pic1.jpg\n" << endl;
}

int main(int argc, char** argv)
{
 const char* filename = argc >= 2 ? argv[1] : "/home/ashishkb/catkin_ws/src/hough_test/src/corridor.jpg";

 Mat src = imread(filename, 0);
 if(src.empty())
 {
     help();
     cout << "can not open " << filename << endl;
     return -1;
 }

 Mat dst, cdst;
 Canny(src, dst, 50, 200, 3);
 cvtColor(dst, cdst, CV_GRAY2BGR);

 #if 0
  vector<Vec2f> lines;
  HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

  for( size_t i = 0; i < lines.size(); i++ )
  {
     float rho = lines[i][0], theta = lines[i][1];
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  }
 #else
  vector<Vec4i> lines;
  HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }
 #endif
 cv::namedWindow("source", CV_WINDOW_NORMAL);
 cv::resizeWindow("source", 500,800);

 cv::namedWindow("detectline test", CV_WINDOW_NORMAL);
 cv::resizeWindow("detectline test", 500,800);
 imshow("source", src);
 imshow("detectline test", cdst);

 //int the_line = 100;
 unsigned char  pb[10]= {1,2,3,4,5,6,7,8,9,10};
 int width = 0;

// *width = 0;
 //template<typename T>
// const * p;

// CvPlot::plot("RGB", pb, 10, 1, 255, 0, 0);
 //CvPlot::label("B");


 waitKey();

 //while(1);

 return 0;
}
*/
