//objectTrackingTutorial.cpp

//Written by  Kyle Hounslow 2013

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.

#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <opencv4/opencv2/opencv.hpp>

using namespace cv;
using namespace std;
#include <deque>
//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 179;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";
const string controlsWindowName = "Controls";

double beamDistanceCm = 30.0;

int SETPOINT_TRACKBAR = 160;
double setpointCm = 16.0;
int KP_TRACKBAR = 30;
int KI_TRACKBAR = 150;
int KD_TRACKBAR = 35;
int SERVO_NEUTRAL_TRACKBAR = 84;
int SERVO_TRAVEL_TRACKBAR = 35;
int CONTROL_DIRECTION_TRACKBAR = 0;
int SETTLE_ERROR_TRACKBAR = 80;
int SETTLE_DERIVATIVE_TRACKBAR = 50;
int SERVO_RATE_TRACKBAR = 150;
int SERVO_DEADBAND_TRACKBAR = 1;

Point2f beamStart(-1.0f, -1.0f);
Point2f beamEnd(-1.0f, -1.0f);
bool beamHasStart = false;
bool beamDefined = false;
double ballPositionCm = -1.0;
double beamAngleDeg = 0.0;
double beamLengthPx = 0.0;

class SerialPort {
public:
	SerialPort() = default;
	~SerialPort() { close(); }

	bool openPort(const string &path, int baudRate)
	{
		fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
		if (fd_ < 0) {
			return false;
		}

		struct termios tty{};
		if (tcgetattr(fd_, &tty) != 0) {
			close();
			return false;
		}

		speed_t speed = B115200;
		if (baudRate == 57600) speed = B57600;
		else if (baudRate == 38400) speed = B38400;
		else if (baudRate == 19200) speed = B19200;
		else if (baudRate == 9600) speed = B9600;

		cfsetospeed(&tty, speed);
		cfsetispeed(&tty, speed);
		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
		tty.c_iflag &= ~IGNBRK;
		tty.c_lflag = 0;
		tty.c_oflag = 0;
		tty.c_cc[VMIN]  = 0;
		tty.c_cc[VTIME] = 1; // short timeout
		tty.c_iflag &= ~(IXON | IXOFF | IXANY);
		tty.c_cflag |= (CLOCAL | CREAD);
		tty.c_cflag &= ~(PARENB | PARODD);
		tty.c_cflag &= ~CSTOPB;
		tty.c_cflag &= ~CRTSCTS;

		if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
			close();
			return false;
		}

		return true;
	}

	void writeLine(const string &line)
	{
		if (fd_ < 0) {
			return;
		}
		string payload = line + "\n";
		const char *data = payload.data();
		size_t remaining = payload.size();
		while (remaining > 0) {
			ssize_t written = ::write(fd_, data, remaining);
			if (written <= 0) return;
			data += written;
			remaining -= (size_t)written;
		}
	}

	// Read available data and extract full lines (without trailing newline).
	// Appends found lines to the provided vector and returns true if any data was read.
	bool readLines(vector<string> &out)
	{
		if (fd_ < 0) return false;
		char buf[256];
		ssize_t n = ::read(fd_, buf, sizeof(buf));
		if (n <= 0) return false;
		readBuf_.append(buf, (size_t)n);
		size_t pos = 0;
		while ((pos = readBuf_.find('\n')) != string::npos) {
			string line = readBuf_.substr(0, pos);
			// strip CR if present
			if (!line.empty() && line.back() == '\r') line.pop_back();
			out.push_back(line);
			readBuf_.erase(0, pos + 1);
		}
		return true;
	}

	bool isOpen() const { return fd_ >= 0; }

private:
	void close()
	{
		if (fd_ >= 0) {
			::close(fd_);
			fd_ = -1;
		}
	}

	int fd_ = -1;
	string readBuf_;
};

SerialPort* activeSerialPort = nullptr;

void sendSerialCommand(const string &command)
{
	if (activeSerialPort != nullptr && activeSerialPort->isOpen()) {
		activeSerialPort->writeLine(command);
	}
}

void sendSerialFloat(const string &prefix, double value, int decimals)
{
	std::ostringstream payload;
	payload << prefix << ":" << fixed << setprecision(decimals) << value;
	sendSerialCommand(payload.str());
}

void sendSerialInt(const string &prefix, int value)
{
	std::ostringstream payload;
	payload << prefix << ":" << value;
	sendSerialCommand(payload.str());
}

void pushCurrentControlState()
{
	sendSerialCommand("source:vision");
	sendSerialFloat("set", setpointCm, 2);
	sendSerialFloat("kp", KP_TRACKBAR / 10.0, 2);
	sendSerialFloat("ki", KI_TRACKBAR / 1000.0, 3);
	sendSerialFloat("kd", KD_TRACKBAR / 10.0, 2);
	sendSerialFloat("neutral", SERVO_NEUTRAL_TRACKBAR, 1);
	sendSerialFloat("travel", SERVO_TRAVEL_TRACKBAR, 1);
	sendSerialInt("dir", CONTROL_DIRECTION_TRACKBAR ? 1 : -1);
	sendSerialFloat("settleerror", SETTLE_ERROR_TRACKBAR / 100.0, 2);
	sendSerialFloat("settlespeed", SETTLE_DERIVATIVE_TRACKBAR / 100.0, 2);
	sendSerialFloat("servorate", SERVO_RATE_TRACKBAR / 100.0, 2);
	sendSerialFloat("servodead", SERVO_DEADBAND_TRACKBAR / 100.0, 2);
}

void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed





}

void on_setpoint_trackbar(int, void*)
{
	setpointCm = SETPOINT_TRACKBAR / 10.0;
	sendSerialFloat("set", setpointCm, 2);
}

void on_kp_trackbar(int, void*)
{
	sendSerialFloat("kp", KP_TRACKBAR / 10.0, 2);
}

void on_ki_trackbar(int, void*)
{
	sendSerialFloat("ki", KI_TRACKBAR / 1000.0, 3);
}

void on_kd_trackbar(int, void*)
{
	sendSerialFloat("kd", KD_TRACKBAR / 10.0, 2);
}

void on_neutral_trackbar(int, void*)
{
	sendSerialFloat("neutral", SERVO_NEUTRAL_TRACKBAR, 1);
}

void on_travel_trackbar(int, void*)
{
	sendSerialFloat("travel", SERVO_TRAVEL_TRACKBAR, 1);
}

void on_dir_trackbar(int, void*)
{
	sendSerialInt("dir", CONTROL_DIRECTION_TRACKBAR ? 1 : -1);
}

void on_settleerr_trackbar(int, void*)
{
	sendSerialFloat("settleerror", SETTLE_ERROR_TRACKBAR / 100.0, 2);
}

void on_settlederiv_trackbar(int, void*)
{
	sendSerialFloat("settlespeed", SETTLE_DERIVATIVE_TRACKBAR / 100.0, 2);
}

void on_servorate_trackbar(int, void*)
{
	sendSerialFloat("servorate", SERVO_RATE_TRACKBAR / 100.0, 2);
}

void on_servodead_trackbar(int, void*)
{
	sendSerialFloat("servodead", SERVO_DEADBAND_TRACKBAR / 100.0, 2);
}

void onMouse(int event, int x, int y, int, void*)
{
	if (event == EVENT_RBUTTONDOWN) {
		beamHasStart = false;
		beamDefined = false;
		beamStart = Point2f(-1.0f, -1.0f);
		beamEnd = Point2f(-1.0f, -1.0f);
		beamLengthPx = 0.0;
		ballPositionCm = -1.0;
		return;
	}

	if (event != EVENT_LBUTTONDOWN) {
		return;
	}

	if (!beamHasStart || beamDefined) {
		beamStart = Point2f((float)x, (float)y);
		beamHasStart = true;
		beamDefined = false;
		beamEnd = Point2f(-1.0f, -1.0f);
	} else {
		beamEnd = Point2f((float)x, (float)y);
		beamLengthPx = norm(beamEnd - beamStart);
		beamAngleDeg = atan2(beamEnd.y - beamStart.y, beamEnd.x - beamStart.x) * 180.0 / CV_PI;
		beamDefined = beamLengthPx > 1.0;
	}
}

Point2f projectPointToBeam(const Point2f &point, double &t)
{
	Point2f beamVec = beamEnd - beamStart;
	double len2 = beamVec.dot(beamVec);
	if (len2 <= 0.0) {
		t = 0.0;
		return beamStart;
	}

	Point2f fromStart = point - beamStart;
	t = (fromStart.dot(beamVec)) / len2;
	t = max(0.0, min(1.0, t));
	return beamStart + beamVec * (float)t;
}

string intToString(int number){


	std::stringstream ss;
	ss << number;
	return ss.str();
}
void createTrackbars(){
	//create window for trackbars


    namedWindow(trackbarWindowName,0);
	namedWindow(controlsWindowName,0);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
	createTrackbar( "Setpoint cm", controlsWindowName, &SETPOINT_TRACKBAR, (int)(beamDistanceCm * 10.0), on_setpoint_trackbar );
	createTrackbar( "Kp x0.1", controlsWindowName, &KP_TRACKBAR, 500, on_kp_trackbar );
	createTrackbar( "Ki x0.001", controlsWindowName, &KI_TRACKBAR, 5000, on_ki_trackbar );
	createTrackbar( "Kd x0.1", controlsWindowName, &KD_TRACKBAR, 200, on_kd_trackbar );
	createTrackbar( "Neutral", controlsWindowName, &SERVO_NEUTRAL_TRACKBAR, 180, on_neutral_trackbar );
	createTrackbar( "Travel", controlsWindowName, &SERVO_TRAVEL_TRACKBAR, 60, on_travel_trackbar );
	createTrackbar( "Dir", controlsWindowName, &CONTROL_DIRECTION_TRACKBAR, 1, on_dir_trackbar );
	createTrackbar( "SetErr x0.01", controlsWindowName, &SETTLE_ERROR_TRACKBAR, 200, on_settleerr_trackbar );
	createTrackbar( "SetSpeed x0.01", controlsWindowName, &SETTLE_DERIVATIVE_TRACKBAR, 500, on_settlederiv_trackbar );
	createTrackbar( "SrvRate x0.01", controlsWindowName, &SERVO_RATE_TRACKBAR, 500, on_servorate_trackbar );
	createTrackbar( "SrvDead x0.01", controlsWindowName, &SERVO_DEADBAND_TRACKBAR, 500, on_servodead_trackbar );


}
void drawObject(int x, int y,Mat &frame){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame,Point(x,y),20,Scalar(0,255,0),2);
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}
void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
	


}
bool trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,RETR_CCOMP,CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}


			}
			//let user know you found an object
			if(objectFound ==true){
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(x,y,cameraFeed);}

		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
	return objectFound;
}
int main(int argc, char* argv[])
{
	string serialPortPath;
	int serialBaud = 115200;
	int cameraIndex = 0;
	for (int i = 1; i < argc; ++i) {
		string arg = argv[i];
		if (arg == "--serial" && i + 1 < argc) {
			serialPortPath = argv[++i];
		} else if (arg == "--baud" && i + 1 < argc) {
			serialBaud = atoi(argv[++i]);
		} else if (arg == "--camera" && i + 1 < argc) {
			cameraIndex = atoi(argv[++i]);
		} else if (arg == "--beam-cm" && i + 1 < argc) {
			beamDistanceCm = atof(argv[++i]);
		} else if (arg == "--help") {
			cout << "Usage: ./track [--serial /dev/ttyUSB0] [--baud 115200] "
			        "[--camera 0] [--beam-cm 30]\n";
			return 0;
		}
	}

	SerialPort serialPort;
	if (!serialPortPath.empty()) {
		if (!serialPort.openPort(serialPortPath, serialBaud)) {
			cerr << "Warning: could not open serial port " << serialPortPath << endl;
		}
	}
	activeSerialPort = &serialPort;

	//some boolean variables for different functionality within this
	//program
    bool trackObjects = true;
    bool useMorphOps = true;
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	//matrix storage for HSV image
	Mat HSV;
	//matrix storage for binary threshold image
	Mat threshold;
	//x and y values for the location of the object
	int x=0, y=0;
	setpointCm = SETPOINT_TRACKBAR / 10.0;
	//create slider bars for HSV filtering
	createTrackbars();
	//video capture object to acquire webcam feed
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(cameraIndex);
	if (!capture.isOpened()) {
		cerr << "Could not open camera " << cameraIndex << endl;
		return 1;
	}
	//set height and width of capture frame
	capture.set(CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	namedWindow(windowName, 1);
	setMouseCallback(windowName, onMouse, nullptr);

	// Serial console buffer shown in the Controls window
	deque<string> serialLog;
	const int maxSerialLines = 12;

	if (serialPort.isOpen()) {
		pushCurrentControlState();
	}
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while(1){
		//store image to matrix
		capture.read(cameraFeed);
		if (cameraFeed.empty()) {
			continue;
		}
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
		Mat thresholdDisplay = threshold.clone();
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if(useMorphOps)
		morphOps(threshold);
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		bool objectFound = trackObjects &&
			trackFilteredObject(x,y,threshold,cameraFeed);

		if (beamDefined && objectFound) {
			line(cameraFeed, beamStart, beamEnd, Scalar(255, 0, 0), 2);
			circle(cameraFeed, beamStart, 6, Scalar(255, 0, 0), FILLED);
			circle(cameraFeed, beamEnd, 6, Scalar(255, 0, 0), FILLED);
			Point2f beamVec = beamEnd - beamStart;
			double beamLen = norm(beamVec);
			Point2f unit = beamVec * (1.0f / (float)beamLen);
			Point2f setpointPoint = beamStart + unit * (float)((setpointCm / beamDistanceCm) * beamLen);
			circle(cameraFeed, setpointPoint, 8, Scalar(0, 255, 255), 2);

			double projectionT = 0.0;
			Point2f ballPoint((float)x, (float)y);
			Point2f projected = projectPointToBeam(ballPoint, projectionT);
			ballPositionCm = projectionT * beamDistanceCm;
			line(cameraFeed, ballPoint, projected, Scalar(0, 255, 255), 1);
			putText(cameraFeed, "Beam " + intToString((int)round(beamAngleDeg)) + " deg", Point(10, 80), 1, 1, Scalar(255, 255, 0), 2);
			putText(cameraFeed, "Ball " + intToString((int)round(ballPositionCm * 10.0) / 10) + " cm", Point(10, 110), 1, 1, Scalar(0, 255, 255), 2);
			putText(cameraFeed, "Set " + intToString((int)round(setpointCm * 10.0) / 10) + " cm", Point(10, 140), 1, 1, Scalar(0, 255, 255), 2);
		} else {
			putText(cameraFeed, "Click two points on the beam. Right click to reset.", Point(10, 80), 1, 1, Scalar(0, 255, 255), 2);
			ballPositionCm = -1.0;
		}

		if (serialPort.isOpen()) {
			if (beamDefined && objectFound && ballPositionCm >= 0.0) {
				std::ostringstream payload;
				payload << "vision:" << fixed << setprecision(2) << ballPositionCm;
				serialPort.writeLine(payload.str());
			} else {
				// send only the vision marker when no ball is detected
				serialPort.writeLine("vision:-1");
			}

			// Read any incoming serial lines and append to console buffer
			vector<string> newLines;
			if (serialPort.readLines(newLines)) {
				for (const auto &ln : newLines) {
					serialLog.push_back(ln);
					if ((int)serialLog.size() > maxSerialLines) serialLog.pop_front();
				}
			}
		}

		//show frames 
		imshow(windowName2,thresholdDisplay);
		imshow(windowName3,threshold);
		imshow(windowName,cameraFeed);
		imshow(windowName1,HSV);

		// Render serial console into the Controls window (trackbars remain attached)
		Mat controlsMat(400, 420, CV_8UC3, Scalar(40,40,40));
		int y = 20;
		// show serial port status at the top
		string status = "Serial: ";
		if (serialPortPath.empty()) {
			status += "(none)";
		} else {
			status += serialPortPath;
			status += serialPort.isOpen() ? " (OPEN)" : " (CLOSED)";
		}
		putText(controlsMat, status, Point(8, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200,200,200), 1);
		y += 18;
		putText(controlsMat, "Serial Console:", Point(8, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200,200,200), 1);
		y += 24;
		int lineIdx = 0;
		for (auto it = serialLog.rbegin(); it != serialLog.rend() && lineIdx < maxSerialLines; ++it, ++lineIdx) {
			string t = *it;
			// shorten long lines
			if (t.size() > 70) t = t.substr(0, 67) + "...";
			putText(controlsMat, t, Point(8, y), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(180,255,180), 1);
			y += 18;
		}
		imshow(controlsWindowName, controlsMat);
		

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		int key = waitKey(30);
		if (key == 27 || key == 'q') break;
	}






	return 0;
}
