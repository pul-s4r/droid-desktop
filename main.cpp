// C and C++ headers
#include <iostream>
#include <string>
#include <sys/time.h>
#include <cstdlib>
#include <list>

// OpenCV libraries
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Define processing mode: still images, or video
// #define STILL_IMAGES

// Define: output the processed image file
// #define OUTPUT_IMAGE

// Define to display all image output
#define DISP_TEST

// Streaming and Pi interaction to be added at a later date.


// Project-specific headers
// #include "include/Readimage.hpp"

// #include "main.hpp"

using namespace std;
using namespace cv;

/*
Variables
*/

// Image width and height constants
static const int iw = 320;
static const int ih = 240;

// File path for sample image processing
static string image_path = "./sample_images/";

// Window names
static string window_unmodified = "Unmodified image";
static string window_hsv_image = "HSV image";
static string window_camera = "Camera";
static string window_grey_path = "Grey with path superimposed";
static string window_canny = "Canny";
static string window_centrepath = "Centre path";

// Video capture containers
typedef VideoCapture camera_t;
typedef enum {RIGHT, LEFT, NEUTRAL} steering_dir_t;
typedef enum {YELLOW, BLUE, PURPLE, NONE} colour_t;

// Globals for correcting oblique perspective
Mat perspectiveMat;
const float aperture = 3.0; // makes 2*apeture - 1 scaling

// Function headers
void detect_path(Mat & grey, double & steeringAngle, double & speed);
void detect_obstacles(Mat hsv, vector<Rect2i> & obj);
colour_t sharp_corner(Mat middle);
// bool handle_remote_switch(DriveControl & control);
void camera_setup(camera_t & cam);
// void sleep(double seconds);

int main(int argc, char * argv[]) {
    /*
        Mat variables for storing images
        im      : Standard sized image, resized from imLarge
                    (default: 640 * 480)
        imHSV   : HSV representation
                    (intermediate format for converting to
                    BGR and Greyscale)
        imGrey  : Greyscale representation for image processing
        imLarge : Original image as read from file
        binaryPath : Path storage
    */
    Mat im, imHSV, imGrey, imLarge, binaryPath;

    // Vector for image channels
    vector<Mat> channels(3);

    // Vector for storing locations of objects
    vector<Rect2i> obstacles;

    // Region of interest for scanning for obstacles
	// Rect2i middleROI(2*iw/5, ih/3-1, iw/5, 2*ih/3);

    /*
        variables for iterating through sample images
    */
	stringstream filename;
	int imIndex = 462; // first jpeg in samples folder has index 0462

    // Instance variables for driving steering and speed
    double steeringAngle = 0;
    double speed = 0;
    int outAngle, outSpeed;

    /*
        perspective transformation matrix based on aperture skew
    */
	Point2f src[] = {Point2f(0, 0),
                    Point2f(iw - 1, 0),
                    Point2f(aperture * iw, ih - 1),
                    Point2f((1 - aperture) * iw, ih - 1)
                    };
	Point2f dst[] = {Point2f(0,0),
                    Point2f(iw - 1, 0),
                    Point2f(iw - 1, ih - 1),
                    Point2f(0, ih - 1)
                    };
	perspectiveMat = getPerspectiveTransform(src, dst);

    /*
    If using camera, open camera
    */
    #ifndef STILL_IMAGES
		// Instantiate camera object
		camera_t cam;

		// Configure camera before opening
		camera_setup(cam);

		// Open camera
		cam.open(0);

        // Check if camera is open, exit if not
		if (!cam.isOpened()){
			cout << "Camera not found" << endl;
			return -1;
		}
	#endif

    /*
        Time each iteration
    */
	double loopTime;
    clock_t begin, end;

    cout << "Entering main loop" << endl;

    /*
    Main loop: Read each image, apply path/object detection and plot on
    image
    */
    while (1) {
        /*
            get time of beginning of loop (reading/processing only)
            (uses ctime)
        */
		begin = clock();

        /*
            Get next image.
            If using camera, grab frame.
            If using still images, form input filename
        */
		#ifdef STILL_IMAGES
            // clear filename
            filename.clear();
            filename.str(string());

            // form input filename for reading images
            filename << image_path << "IMG_0" << imIndex << ".jpg";
            cout << "Reading " << filename.str() << endl;

            // read file and exit on end
            imLarge = imread(filename.str());
            if(imLarge.data==NULL){
                cout << "Reached end of images" << endl;
                return 0;
            }

            #ifdef OUTPUT_IMAGE
                // form output filename for saving images
                filename.clear();
                filename.str(string());
                filename << image_path << "out" << imIndex << ".jpg";
            #endif

            // increment file index
            imIndex++;
		#else
            cam.grab();

            cam.retrieve(imLarge);
		#endif

        // Scale original image (imLarge) to im for display
        resize(imLarge, im, Size(iw, ih), 0, 0, CV_INTER_LINEAR);

		// convert scaled image to HSV for greyscale conversion
		cvtColor(im, imHSV, COLOR_BGR2HSV);
		// convert hsv image to bgr->greyscale for processing
		cvtColor(imHSV, imGrey, COLOR_BGR2GRAY);

        //colour_t colourLine = sharp_corner(imHSV);
		colour_t colourLine = NONE;

        // find and plot obstacles in region of interest
        detect_obstacles(imHSV, obstacles);
        for(auto & el : obstacles){
            // el.x += ROI.x;
            // el.y += ROI.y;

            // draw box around each obstacle
            rectangle(im, el, Scalar(255,0,0));
            // rectangle(imGrey, el, Scalar(100));

            // draw triangle leading into obstacle
            // for path calculation
            int triangleHeight = min(el.width/2,
             ih - (el.y + el.height));
            Point2i vertex(el.x + el.width/2,
             el.y + el.height + triangleHeight);
            Point2i bottomL(el.x, el.y + el.height - 1);
            Point2i bottomR = el.br();
            line(im, bottomL, vertex, Scalar(255), 2);
            line(im, bottomR, vertex, Scalar(255), 2);
        }

        // determine path and steering angle, returns corrected image
        detect_path(imGrey, steeringAngle, speed);

        // plot estimated steering path on top of image as circular arc
		if(abs(steeringAngle) > 0.1){
			int radius = abs(1000 / steeringAngle);
			Point2i centre;
			if(steeringAngle > 0){
				centre = Point2i(iw/2 -1 + radius, ih - 1);
			} else {
				centre = Point2i(iw/2 - 1 - radius, ih - 1);
			}
			circle(im, centre, radius, Scalar(0,0,255));
			circle(imGrey, centre, radius, Scalar(255));
		}
		//cout << "Steering angle: " << steeringAngle << "	Speed: " << speed << endl;

		// CHANGE CONSTANTS FOR MODIFIED RESPONSE
		if(colourLine == NONE){
			if(steeringAngle > 0){
				outAngle = steeringAngle * 20; // right steering constant [change this]
			} else {
				outAngle = steeringAngle * 30; // left steering constant [change this]
				if(outAngle > 250){
					outAngle = 400;
				}
			}
			outSpeed = speed * 50;	// speed constant [change this]
		} else if(colourLine == YELLOW){
			outAngle = -500;
			outSpeed = 15;
			cout << "Yellow line in front!" << endl;
		} else if(colourLine == BLUE){
			outAngle = 500;
			outSpeed = 15;
			cout << "Blue line in front!" << endl;
		}

        // get time of end of loop
		end = clock();

        // Calculate and print execution time
        loopTime = double (end - begin) / CLOCKS_PER_SEC;
        cout << "Image size: " << im.size() << " Loop time: "
            << loopTime << endl;

        // Output the image
        #ifdef DISP_TEST
            // imshow(window_hsv_image, imGrey);
            imshow(window_unmodified, im);
            moveWindow(window_canny, iw, 0);
            moveWindow(window_grey_path, 2*iw, 0);
        #else
            // imshow(window_camera, im);
            // moveWindow(window_canny, im.cols, 0);
            // moveWindow(window_grey_path, im.rows, 0);
        #endif

        /*
            need to be clicked in to one of the image windows
            (not terminal) for this to work
        */
        #ifdef STILL_IMAGES
            // (original file outputs - imPath)
            #ifdef OUTPUT_IMAGE
    			cout << "Saving file " << filename.str() << endl;
    			imwrite(filename.str(), im);
            #endif
			cout << "Press any key for next image" << endl;
			waitKey();
		#elif defined DISP_TEST
			waitKey(5);
		#endif

        // Clear the obstacles vector for the next iteration
        obstacles.clear();

    }
    // End of main loop

}

colour_t sharp_corner(Mat hsv)
{
	Mat maskYellow;
	Mat checkYellow = hsv(Rect2i(0,0, iw/3, ih));
	inRange(checkYellow, Scalar(0, 60, 60), Scalar(60, 255, 255), maskYellow);

	//int n = 2;
	//Mat element = getStructuringElement(MORPH_RECT, Size(n*2+1, n*2+1), Point(n, n));
        //morphologyEx(maskYellow, maskYellow, MORPH_OPEN, element);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	findContours(maskYellow, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0) );

        // get rectangles of large obstacles
        for(size_t i = 0; i < contours.size(); ++i){
                if(contourArea(contours[i]) > 100){
                        return YELLOW;
                }
        }

	Mat maskBlue;

	contours.clear();
	hierarchy.clear();

	Mat checkBlue = hsv(Rect2i(2*iw/3-1, 0, iw/3, ih));
	inRange(checkBlue, Scalar(100,80,60), Scalar(155, 255, 255), maskBlue);

	//morphologyEx(maskBlue, maskBlue, MORPH_OPEN, element);

	findContours(maskBlue, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0) );

	for(size_t i = 0; i < contours.size(); ++i){
                if(contourArea(contours[i]) > 100){
                        return BLUE;
                }
        }


	return NONE;

}

void detect_obstacles(Mat hsv, vector<Rect2i> & obj) {
	Mat mask;

	// generate mask of purple colours
	//inRange(hsv, Scalar(110, 100, 80), Scalar(130, 255, 255), mask);
	// generate mask of all saturated colours
	inRange(hsv, Scalar(0, 50, 50), Scalar(255, 255, 255), mask);

	// eliminate noise
	int n = 2;
	Mat element = getStructuringElement(MORPH_RECT, Size(n*2+1, n*2+1), Point(n, n));
	morphologyEx(mask, mask, MORPH_OPEN, element);

	//imshow("Obstacle mask", mask);

	// variables for defining objects
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	// draw convex hulls around detected contours
	findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
	for(size_t i = 0; i < contours.size(); ++i){
		drawContours(mask, contours, (int)i, Scalar(255), CV_FILLED);
	}
	//n = 4;
	//element = getStructuringElement(MORPH_RECT, Size(n*2+1, n*2+1), Point(n,n));
	//erode(mask, mask, element);

	findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0) );

	//vector<vector<Point>> contours_poly( contours.size() );

	// get rectangles of large obstacles
	for(size_t i = 0; i < contours.size(); ++i){
		//approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true);
		if(contourArea(contours[i]) > 2000){
			Rect object = boundingRect( Mat(contours[i]) );
			obj.push_back(object);
		}
	}
}

void detect_path(Mat & grey, double & steeringAngle, double & speed) {
	// get edge binary mask from grey image
	Mat edges;

	/** PLEASE READ UP ON CANNY **/
	Canny(grey, edges, 80, 240); // note edge thresholding numbers

	// distort edge binary and grey image to correct for oblique perspective
	warpPerspective(edges, edges, perspectiveMat, edges.size());
	warpPerspective(grey, grey, perspectiveMat, grey.size());

	// Path matrix, which calculated path will be drawn on
	//Mat centrePath = Mat::zeros(grey.size(), CV_8UC1);

	// Integer variables for features of image
	int leftBorder, rightBorder, row;
	int height = grey.size().height;
	int width = grey.size().width;

	// effect of perspective increases for further away objects
	double perspectiveDistance;

	// doubles for inputs into the PD controller which generates the line
	double centre = width/2;
	double deltaF = 0;
	double oldF = 0;
	double difference = 0;
	steeringAngle = 0;
	double startAccel = 0;

	//steeringAngle = 0;
	double maxAccel = 0;
	int maxRow = height - 1;

	// keeps track of the presence of the left and right edges in each row of any frame
	bool rightFound, leftFound;

	// corner analysis
	bool turnInitiated = false;
	bool turnFinished = false;
	steering_dir_t steeringDirection = NEUTRAL;
	list<double> accels;

	// iterate from the bottom (droid) edge of the image to the top
	for(row = height - 1; row >= 0; --row){

		perspectiveDistance = width*(1-1/(2*aperture-1))*((double)row/(double)(1.5*height-1))/2;
		//perspectiveDistance = 0;
		//if(row == 30){
		//	cout << "Row: " << row << "	perspectivePixels: " << perspectiveCalc << endl;
		//}
		// start looking for left border from the centre path, iterate outwards
		leftBorder = centre;
		leftFound = false;
		while(leftBorder > perspectiveDistance){//(leftBorder > 0){
			leftBorder--;
			if(edges.at<uchar>(row, leftBorder) > 0){
				// if an edge is found, assume its the border, and break
				break;
			}
		}

		// start looking for right border from the centre path, iterate outwards
		rightBorder = centre;
		rightFound = false;
		while(rightBorder < width - perspectiveDistance){//(rightBorder < width){}
			rightBorder++;
			if(edges.at<uchar>(row, rightBorder) > 0){
				// if an edge is found, assume its the border, and break
				break;
			}
		}

		// feed calculated centre of the borders into the controller
		double deadCentre = (leftBorder + rightBorder)/2;
		// proportional term
		double ff = deadCentre - centre;
		// derivative term
		deltaF = ff - oldF;
		// cap derivative term in case of sudden jumps in path
		deltaF = max(deltaF, -3.0); deltaF = min(deltaF, 3.0);

		double accel;

		// feed terms into horizontal rate accross image
		if(row == height - 1){
			accel = 0;// ff * 0.005;
		} else {
			accel = ff * 0.001 + deltaF * 0.01;
		}

		// cap horizontal acceleration
		accel = min(accel, 0.3); accel = max(accel, -0.3);
		difference += accel;

		// if the end of the first turn hasn't been reached
		if(!turnFinished){
			// if the start of the first bend is detected
			if(!turnInitiated && abs(steeringAngle) > 0.12){
				// record initial turn direction
				turnInitiated = true;
				if(steeringAngle > 0){
					steeringDirection = RIGHT;
				} else {
					steeringDirection = LEFT;
				}
			}

			// record maximum acceleration in first turn
			if(abs(accel) > maxAccel){
				maxAccel = abs(accel);
				maxRow = row;
			}

			// generate the steering angle with a sum of accelerations
			steeringAngle += accel;

			// record the last few accelerations and produce an average
			accels.push_back(accel);
			if(accels.size() > 5){
				accels.pop_front();
			}
			double meanAccel = 0;
			for(auto el : accels){
				meanAccel += el;
			}
			meanAccel /= accels.size();

			// look for the end of a turn
			if(row < height - 200) {
				// no turn near
				turnFinished = true;
			} else if (meanAccel < 0 && steeringDirection == RIGHT){
				// acceleration to the left in a right turn
				turnFinished = true;
			} else if (meanAccel > 0 && steeringDirection == LEFT){
				// acceleration to the right in a left turn
				turnFinished = true;
			}

			if(turnFinished){
				// get the mean acceleration in the turn
				steeringAngle /= (height - row);

				// adjust steering angle with a signed square and other constants
				steeringAngle *= 10;
				if(steeringAngle >= 0){
					steeringAngle *= steeringAngle;
				} else {
					steeringAngle *= steeringAngle;
					steeringAngle = -steeringAngle;
				}
				//steeringAngle *= (steeringAngle * steeringAngle);
				steeringAngle *= 360;

				// slow down speed based on the maximum acceleration in the turn
				speed = 1 - 5*maxAccel;
				speed = min(0.75, speed);
			}
		}

		// adjust centre of path accordingly
		centre += difference;
		// bound path by edges of image
		centre = max(2.0, centre); centre = min(width-3.0, centre);
		// track past proportional term, used to calculate derivative term
		oldF = ff;

		// draw path pixels on images
		for(int i = - 1; i <=	1; ++i){
			grey.at<uchar>(row,	(int)deadCentre + i) = uchar(0);
			//centrePath.at<uchar>(row, (int)centre + i) = uchar(255);
			grey.at<uchar>(row, (int)centre + i) = uchar(255);
		}

	}

	// show binary mask
	#ifdef DISP_TEST
		imshow(window_canny, edges);
		//imshow(window_centrepath, centrePath);
		imshow(window_grey_path, grey);
	#endif

	//cout << "Max accel: " << maxAccel << " at row " << maxRow << endl;
}

// Configure camera settings
void camera_setup(camera_t & cam) {
    cam.set(CV_CAP_PROP_FORMAT, CV_8UC3); // 3 channel image
    cam.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    cam.set(CV_CAP_PROP_BRIGHTNESS, 80); // increased brightness
    cam.set(CV_CAP_PROP_SATURATION, 80); // increased saturation
    //cam.set(CV_CAP_PROP_AUTOFOCUS, 0); // set focus to manual
    //cam.set(CV_CAP_PROP_FOCUS, 255); // need to check if this works
    //cam.set(CV_CAP_PROP_EXPOSURE, 10);
}
