#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

//Structue to store the location and size of the blobs detected
typedef struct b	//Sonar data.
{
	Point2f location;
	float size;
	float slope;
	Point2f vertex[4];
}blob;


//Structure to store the state of the robot (location, orientation and intensity of the robot)
typedef struct r	//Sonar data.
{
	Point2f position;
	float heading;
	int front_index;
	int rear_index;
}robot;
