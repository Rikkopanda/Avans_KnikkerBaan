//
// Simple opencv blob detector example
//
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	//
	cout << "Welcome to OpenCV " << CV_VERSION << std::endl;

	// Lees input
	Mat src_image = imread("pcb_1.jpg", IMREAD_GRAYSCALE);

	//
	SimpleBlobDetector::Params params;

	// Thresholds
	params.minThreshold = 10;
	params.maxThreshold = 200;

	// Oppervlakte (Area).
	params.filterByArea = true;
	params.minArea = 10;
	params.maxArea = 1500;

	// Circularity (Rondheid)
	params.filterByCircularity = true;
	params.minCircularity = 0.8;

	// Convexity
	params.filterByConvexity = false;
	params.minConvexity = 0.87;

	// Filter byInertia
	params.filterByInertia = false;
	params.minInertiaRatio = 0.01;

	std::vector<KeyPoint> keypoints;
	cv::Ptr<cv::SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	detector->detect(src_image, keypoints);

	// Nieuw image net zo groot als source
	//Mat result_image(src_image.rows, src_image.cols, CV_8UC3, Scalar(0, 0, 0));
	Mat result_image = imread("pcb_1.jpg", IMREAD_COLOR);
	for (int idx = 0; idx < keypoints.size(); idx++)
	{
		Point pt;
		pt.x = keypoints[idx].pt.x;
		pt.y = keypoints[idx].pt.y;
		drawMarker(result_image, pt, Scalar(0, 0, 255), MARKER_CROSS, 10, 4, 8);
	}
	//drawKeypoints(src_image, keypoints, result_image, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	//
	imshow("Image", src_image);
	imshow("Blob detector", result_image);
	cout << keypoints.size() << endl;

	waitKey(0);
	return 0;
}
