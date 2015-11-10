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

 CvPlot::plot("RGB", pb, 10, 1, 255, 0, 0);
 //CvPlot::label("B");


 waitKey();

 //while(1);

 return 0;
}
