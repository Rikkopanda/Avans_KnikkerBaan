// OpenCV live camera example
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    VideoCapture cap;
    int deviceId = 0;
    int apiId = cv::CAP_ANY;
    cap.open(deviceId + apiId);

    // Check of camera kan worden geopend ...
    if (!cap.isOpened()) {
        cerr << "Can't open camera" << endl;
        return -1;
    }

    while (true) {
        Mat src;

        // ... zag ik soms bij trage camera ???
        cap.read(src);
        if (src.empty()) {
            cerr << "Error! blank frame" << endl;
        }

        Mat dest;
        threshold(src, dest, 130, 255, THRESH_BINARY);

        imshow("Source", src);
        imshow("Threshold", dest);

        if (waitKey(5) >= 0) {
            break;
        }

    }

    return 0;
}
