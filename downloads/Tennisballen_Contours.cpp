
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

using namespace std;
using namespace cv;

// User interface elementen toevoegen
const int max_value_H = 360 / 2;
const int max_value = 255;
const String window_capture_name = "Capture";
const String window_detection_name = "Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

static void on_low_H_thresh_trackbar(int, void*)
{
    low_H = min(high_H - 1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void*)
{
    high_H = max(high_H, low_H + 1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void*)
{
    low_S = min(high_S - 1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void*)
{
    high_S = max(high_S, low_S + 1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void*)
{
    low_V = min(high_V - 1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void*)
{
    high_V = max(high_V, low_V + 1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

int main()
{
    std::cout << "Voorbeeld Tennisballen\n";

    //
    namedWindow(window_detection_name);
    namedWindow(window_capture_name);

    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    
    while (true) {

        // Source
        Mat im_src = imread("tennisball_2.jpg", IMREAD_COLOR);
        
        // Eventueel een Resize, keep aspect ration (moet veel slimmer!)
        Mat im_resize;
        resize(im_src, im_resize, Size(600,600), INTER_LINEAR);

        // Blur?
        Mat im_blurred;
        GaussianBlur(im_resize, im_blurred, Size(3, 3), 0);

        // Omzetten naar andere colorspace 
        Mat im_hsv;
        cvtColor(im_blurred, im_hsv, COLOR_BGR2HSV);

        //
        Mat im_masked;
        //inRange(im_hsv, Scalar(23, 0, 0), Scalar(155, 255, 255), im_masked);
        inRange(im_hsv, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), im_masked);
        erode(im_masked, im_masked, 2);
        dilate(im_masked, im_masked, 2);

        // Find alle contouren
        vector<vector<Point>> contours;
        findContours(im_masked.clone(), contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // Maak er een array van polyline's van ...
        vector<vector<Point> > polylines(contours.size());
        for (size_t idx = 0; idx < contours.size(); idx++)
        {
            approxPolyDP(contours[idx], polylines[idx], 3, true);
        }

        // contouren placeholder
        Mat im_contour = Mat::zeros(im_resize.size(), CV_8UC3);

        // Teken alle countouren
        for (size_t idx = 0; idx < contours.size(); idx++) {
            Scalar color = Scalar(0, 255, 0);
            drawContours(im_contour, polylines, (int)idx, color);
        }

        // Eerste sorteren en dan alleen de grootste tekenen 
        std::sort(
            polylines.begin(),
            polylines.end(),
            [](const vector<Point>& a, const vector<Point>& b) { return a.size() < b.size(); }
        );
        drawContours(im_contour, polylines, (int) polylines.size()-1, Scalar(0, 0, 255));

        // Manier 1: vinden van een enclosingCircle algorithme
        Point2f center;
        float radius;
        minEnclosingCircle(polylines[polylines.size() - 1], center, radius);
        if (radius > 50) {
            circle(im_resize, center, radius, Scalar(0, 255, 255));
            circle(im_resize, center, 5, Scalar(0, 0, 255), FILLED);

        }
        cout << "center(x,y) = (" << center.x << "," << center.y << ")" << endl;

         
        // Manier 2: vinden van CG (Moments in opencv)
        std::sort(
            contours.begin(),
            contours.end(),
            [](const vector<Point>& a, const vector<Point>& b) { return a.size() < b.size(); }
        );
        Moments m = moments(contours[contours.size() - 1], false);
        Point m_center(m.m10 / m.m00, m.m01 / m.m00);
        circle(im_contour, m_center, 10, Scalar(255, 255, 255), FILLED);


        // Afbeelden in window
        imshow("Resized", im_resize);
        imshow("Blurred", im_blurred);
        imshow("hsv", im_hsv);
        //
        imshow(window_detection_name, im_masked);
        
        //
        imshow("Countour", im_contour);

        // While(true) lus doorbreken om programma te stoppen
        if (waitKey(5) >= 0)
            break;
    }
}
