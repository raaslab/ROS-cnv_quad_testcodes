#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



#define TEST_IMAGE

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
//static const std::string OPENCV_WINDOW2 = "detectline test";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat image;
  
  int usegradient;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_NORMAL);
    cv::resizeWindow(OPENCV_WINDOW, 500,800);

    cv::namedWindow("detectline test", CV_WINDOW_NORMAL);
    cv::resizeWindow("detectline test", 500,800);

    cv::namedWindow("test", CV_WINDOW_NORMAL);
    cv::resizeWindow("test", 500,800);


    usegradient = 1;
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void detectLine(Mat input_im, int useGradient )
  {
	  cv::Size sz_input_im = input_im.size();

	//  matrix = Mat(numOfRows,numOfCols,CV_64FC1,&theArray);
	  Mat output_im = input_im.clone();
//	  Mat mat_P;
//	  Mat mat_drt;
	  Mat output_im8u;
	  //float P[][3] = {{1146.097731597490, 0, 695.7216576708486}, {0, 1150.201496341596, 376.7037880110635}, {0, 0, 1}};
	  //float drt[] = {-0.550900359006328, 0.425551830340219,  0.00227361036471774,	-0.00401725789453718, -0.219813690451924};
//	  float P[][3] = [1146.0977, 0, 695.721;0, 1150.2014, 376.7037;0, 0, 1];
	  Mat mat_P = (Mat_<float>(3,3) << 1146.0977, 0, 695.721,0, 1150.2014, 376.7037,0, 0, 1);
	    cout << "mat_P = " << endl << " " << mat_P << endl << endl;
	  //float P[][3] = {{1146.0977, 0, 695.7216}, {0, 1150.2014, 376.7037}, {0, 0, 1}};
	//  float drt[] = {-0.5509, 0.4255,  0.0022,	-0.0040, -0.2198};
	  Mat mat_drt = (Mat_<float>(1,5) << -0.5509, 0.4255,  0.0022,	-0.0040, -0.2198);
	  cout << "mat_drt = " << endl << " " << mat_drt << endl << endl;
	  //mat_P = Mat(3,3,CV_64FC1,&P);
	 // mat_drt = Mat(1,5,CV_64FC1,&drt);

	  cv::imshow("detectline test", output_im);
	  output_im.convertTo(output_im8u,CV_8UC3);
	  cv::imshow("test", output_im8u);

	//  GaussianBlur( input_im, output_im, Size( 9, 9 ), 0, 0 );
	  cv::undistort(input_im, output_im8u, mat_P, mat_drt); // drt is distortion matrix
	  output_im8u.convertTo(output_im8u,CV_8UC1);
	  cv::imshow("test", output_im8u);
	//  cv::imshow("detectline test", output_im8u);

	  //Mat M(2,2, CV_8UC3, Scalar(0,0,255));
//	  cout << "P = " << endl << " " << P << endl << endl;
//	  cout << "drt = " << endl << " " << drt << endl << endl;



  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
	#ifdef CAM_STREAM
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(0,0,255));

    // Update GUI Window
       cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	#endif

	#ifdef TEST_IMAGE
    	image = imread("/home/ashishkb/catkin_ws/src/testmycode/src/undistort.jpg");
    	if (image.rows > 60 && image.cols > 60)
    	      cv::circle(image, cv::Point(50, 50), 10, CV_RGB(0,0,255));
    	 // Update GUI Window
    	    cv::imshow(OPENCV_WINDOW, image);
	#endif
    detectLine(image, usegradient);

    cv::waitKey();
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spinOnce();
  return 0;
}
