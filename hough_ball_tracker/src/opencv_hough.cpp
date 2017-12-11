/******************************************
 * OpenCV Tutorial: Ball Tracking using   *
 * Kalman Filter                          *
 ******************************************/
#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
// Module "core"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/features2d/features2d.hpp>
// Module "video"
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
// Output
#include <iostream>
#include <sstream>
// Vector
#include <vector>

using namespace std;
using namespace cv;
// >>>>> Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300

// Camera Index
int idx = 1;

int mouseflag = 0;
int mX = -1,minX,maxX;
int mY = -1,minY,maxY;
int maxG = 0,minG = 255;
int offset0L = 10, offset0H = 10, offset1L = 100, offset1H = 100, offset2L = 100, offset2H = 100;
int roffset0L = 75, roffset0H = 75, roffset1L = 75, roffset1H = 50, roffset2L = 75, roffset2H = 75;
float maxScale = 0;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
        mX = x;
        mY = y;
        mouseflag = 1;
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
}

int exposure = 120,hue = 46, gain = 16,saturation = 255;

void setV4lParameter(int idx, const char*data, int value){
    //Create string container
    stringstream ss;
    //Sub in camera id number (idx), and required parameter and value
    ss << "uvcdynctrl --device=/dev/video" << idx <<  " -s '" << data << "' -- " << value;
    //Print string to shell to call uvcdynctrl to set parameter in v4l 
    system( ss.str().c_str() );
    //Clear ss
    ss.str("");
}
void Hue(int, void *){
    setV4lParameter(idx, "Hue", hue);
    return;
}
void Saturation(int, void *){
    setV4lParameter(idx, "Saturation", saturation);
    return;
}
void Gain(int, void *){
    setV4lParameter(idx, "Gain", gain);
    return;
}
void Exposure(int, void *){
    setV4lParameter(idx, "Exposure", exposure);
    return;
}

void createTrackbars(void){
    /// Create Windows
    namedWindow("V4L parameters", 1);

    createTrackbar("Hue","V4L parameters", &hue, 90, Hue);
    createTrackbar("Saturation","V4L parameters", &saturation, 255, Saturation);
    createTrackbar("Gain","V4L parameters", &gain, 63, Gain);
    createTrackbar("Exposure","V4L parameters", &exposure, 255, Exposure);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_hough");
    ros::NodeHandle n;
    ros::Publisher Setpoint_Pub = n.advertise<geometry_msgs::Point>("coordinate_send_topic", 10);
    ros::Publisher Gripper_Control = n.advertise<std_msgs::Float64>("tilt_controller/command", 10);
    ros::Rate loop_rate(60);
    geometry_msgs::Point Setpoint;
    std_msgs::Float64 Gripper;
    // Camera frame
    cv::Mat frame;

    //Set parameters in qv4l. Hue, saturation, gain and exposure can be modified with trackbars
    setV4lParameter(idx, "White Balance, Automatic", 0);
    setV4lParameter(idx, "Gain, Automatic", 0);
    setV4lParameter(idx, "Auto Exposure", 0);
    setV4lParameter(idx, "Hue", hue);
    setV4lParameter(idx, "Saturation", saturation);
    setV4lParameter(idx, "Gain", gain);
    setV4lParameter(idx, "Exosure", exposure);

    createTrackbars();


    // Camera Capture
    cv::VideoCapture cap;

    // >>>>> Camera Settings
    if (!cap.open(idx))
    {
        cout << "Webcam not connected.\n" << "Please verify\n";
        return EXIT_FAILURE;
    }

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 4800);
    cap.set(CV_CAP_PROP_FPS, 60);
    //cap.set(CV_CAP_PROP_SATURATION,255);
    //brightness 57 contrast 31 saturation 255 hue 46 exposure 170 gain 15 //nighttime
    //daytime brightness 26 //contrast 32 saturation 255// hue 46 exposure 147 gain 0
   // cap.set(CV_CAP_PROP_HUE,46);
    //cap.set(CV_CAP_PROP_EXPOSURE,147);
    //cap.set(CV_CAP_PROP_BRIGHTNESS,26);
    //cap.set(CV_CAP_PROP_GAIN,0.23);


   
    int fWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int fHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    char ch = 0;
    Vec3b intensity;
    intensity.val[0] = 0;
    intensity.val[1] = 0;
    intensity.val[2] = 0;
    Vec3b intensity2;
    Vec3b intensitytemp;
    Vec3b intensity2temp;
    intensity2.val[0] = 0;
    intensity2.val[1] = 0;
    intensity2.val[2] = 0;
    Vec3b minintensity;
    Vec3b maxintensity;
    Vec3b minintensity2;
    Vec3b maxintensity2;
    Scalar grayness;
    Point2f lastpoint;
    int gripperclosed = 0;
            // >>>>> Contours detection

    // >>>>> Main loop
    while (ch != 'q' && ch != 'Q')
    {
        int div = 0;
        cap >> frame;
                    vector<vector<cv::Point> > contours;
                          vector<Vec3f> circles;
                        // >>>>> Filtering
            vector<vector<cv::Point> > balls;
            vector<cv::Rect> ballsBox;
        namedWindow("frame", WINDOW_NORMAL);
        imshow("frame",frame);
        setMouseCallback("frame", CallBackFunc, NULL);

        // >>>>> Noise smoothing
        cv::Mat blur;
        cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
        Mat frmHSV;
        cvtColor(blur, frmHSV,CV_BGR2HSV);

        // //cv::Mat HSV_split[3];   //destination array
        // vector<cv::Mat> HSV_split;
        // split(frmHSV,HSV_split);//split source  
        // HSV_split[0] += 42;
        // //HSV_spslit(0).setTo(new Scalar(42));
        // merge(HSV_split, frmHSV);
        // Mat rgb_hue_shifted;
        // cvtColor(frmHSV, rgb_hue_shifted, CV_HSV2BGR);

        // namedWindow("frame", WINDOW_NORMAL);
        // imshow("frame",frmHSV);
        // setMouseCallback("frame", CallBackFunc, NULL);

        Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
        Mat rangeRes2;
        Mat frmGray;
        Mat andRes;
        Mat frameRes;
        Mat rangeRes3;
        Mat andRes2;
        Mat frmRGB;
        cvtColor(blur,frmGray,CV_BGR2GRAY);
        cvtColor(blur, frmRGB, CV_BGR2RGB);
        frame.copyTo(frameRes);
        int index1 = 0;
        maxScale = 0;

        if (mouseflag == 2){
            inRange(frmHSV,Scalar(minintensity),Scalar(maxintensity),rangeRes);
            inRange(blur,Scalar(minintensity2),Scalar(maxintensity2),rangeRes3);
            // >>>>> Improving the result
            cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
            cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
                        cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
            //cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
                        cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
            //cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
            //cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
            //cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);

            cv::dilate(rangeRes3, rangeRes3, cv::Mat(), cv::Point(-1, -1), 2);
            cv::erode(rangeRes3, rangeRes3, cv::Mat(), cv::Point(-1, -1), 2);
            
            bitwise_and(rangeRes3,rangeRes,andRes);
            namedWindow("andres",WINDOW_NORMAL);
            imshow("andres",andRes);
            contours.clear();

            cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
            //cout << int(contours.size()) << endl;
            vector<Point2f>centers( contours.size() );
            vector<float>radius( contours.size() );
            vector<Point2f>centers2( contours.size() );
            vector<float>radius2( contours.size() );
            if (contours.size() !=0){
            for (size_t i = 0; i < contours.size(); i++)
            {
                cv::Rect bBox;
                bBox = cv::boundingRect(contours[i]);

                minEnclosingCircle( (Mat)contours[i], centers[i], radius[i] );

                float ratio = (float) bBox.width / (float) bBox.height;
                if (ratio > 1.0f)
                    ratio = 1.0f / ratio;

                // Searching for a bBox almost square
                if (ratio > 0.75 && bBox.area() >= 1000)
                {
                    balls.push_back(contours[i]);
                    ballsBox.push_back(bBox);

                }
            }

            for(size_t i = 0; i<balls.size();i++){
                minEnclosingCircle( (Mat)balls[i], centers[i], radius[i]);
                cv::drawContours(frameRes, balls, i, CV_RGB(20,150,20), 1);
                vector<float>scale(centers.size());
                scale.clear();
                scale[i] = 1/float(abs(lastpoint.x-centers[i].x)+(lastpoint.y-centers[i].y));
                vector<float> weighting(balls.size());
                if (maxScale<scale[i]){
                    maxScale = scale[i];
                    index1 = i;
                }

            }

            // intensity.val[0] = 0;
            // intensity.val[1] = 0;
            // intensity.val[2] = 0;
            // intensity2.val[0] = 0;
            // intensity2.val[1] = 0;
            // intensity2.val[2] = 0;

            // if ((centers[index1].x-5)<0)
            //     minX = 0;
            // else
            //     minX = centers[index1].x-5;
            // if ((mX+5)>fWidth)
            //     maxX = fWidth;
            // else
            //     maxX = centers[index1].x+5;

            // if ((centers[index1].y-5)<0)
            //     minY = 0;
            // else
            //     minY = centers[index1].y-5;
            // if ((centers[index1].y+5)>fHeight)
            //     maxY = fHeight;
            // else
            //     maxY = mY+5;    
            


            // //intensity += frmHSV.at<Vec3b>(mY, mX);
            // for (int i = minY; i<=maxY; i = i+1){
            //     for(int n = minX; n<=maxX; n=n+1){
            //         //cout << "value of i: " << i << endl;
            //         //cout << "value of n: " << n << endl;
            //         intensity += frmHSV.at<Vec3b>(i, n);
            //         intensity.val[0] = round(intensity.val[0]/2);
            //         intensity.val[1] = round(intensity.val[1]/2);
            //         intensity.val[2] = round(intensity.val[2]/2);


            //     }
            // }
            // minintensity.val[0] = intensity.val[0] - offset0L;
            // maxintensity.val[0] = intensity.val[0] + offset0H;
            // minintensity.val[1] = intensity.val[1] - offset1L;
            // maxintensity.val[1] = intensity.val[1] + offset1H;
            // minintensity.val[2] = intensity.val[2] - offset2L;
            // maxintensity.val[2] = intensity.val[2] + offset2H;
            // if (minintensity.val[0]<0)
            //     minintensity.val[0] = 0;

            // if (maxintensity.val[0]>255)
            //     maxintensity.val[0] = 255;

            if (centers.size()>=index1){
            lastpoint.x = centers[index1].x;
            lastpoint.y = centers[index1].y;
            cv::circle(frameRes, centers[index1],int(radius[index1]), CV_RGB(255,255,255),2,8,0);
            cout << int(radius[index1]) << endl;
            Setpoint.x = centers[index1].x;
            Setpoint.y = (480 - centers[index1].y);
            Setpoint.z = 0;
            Setpoint_Pub.publish(Setpoint);

            if (((int(radius[index1]) >=70)) && (gripperclosed == 0)) {
                Gripper.data = 3.0;
                Gripper_Control.publish(Gripper);
                gripperclosed = 1;
            }
            else if (ch == 'o'){
                Gripper.data = 3.4;
                Gripper_Control.publish(Gripper);
                gripperclosed = 0;          
        }
    }

            }
        
        else
        {
            Setpoint.x = 0;//centers[index1].x;
            Setpoint.y = 0;//center[index1].y;
            Setpoint.z = 1;
            Setpoint_Pub.publish(Setpoint);
 

        }
            if (ch == 'o'){
                Gripper.data = 3.4;
                Gripper_Control.publish(Gripper);
                gripperclosed = 0;
            } 
            namedWindow("frameres", WINDOW_NORMAL);
            imshow("frameres", frameRes);
            namedWindow("inrange", WINDOW_NORMAL);
            imshow("inrange", rangeRes);
            namedWindow("inrange2", WINDOW_NORMAL);
            imshow("inrange2", rangeRes3);
            
            
        }

        if (mouseflag == 1){
            intensity.val[0] = 0;
            intensity.val[1] = 0;
            intensity.val[2] = 0;
            // intensity2.val[0] = 0;
            // intensity2.val[1] = 0;
            // intensity2.val[2] = 0;

            if ((mX-10)<0)
                minX = 0;
            else
                minX = mX-10;
            if ((mX+10)>fWidth)
                maxX = fWidth;
            else
                maxX = mX+10;

            if ((mY-10)<0)
                minY = 0;
            else
                minY = mY-10;
            if ((mY+10)>fHeight)
                maxY = fHeight;
            else
                maxY = mY+10;    
            


            //intensity += frmHSV.at<Vec3b>(mY, mX);
            for (int i = minY; i<maxY; i = i+1){
                for(int n = minX; n<maxX; n=n+1){
                    //cout << "value of i: " << i << endl;
                    //cout << "value of n: " << n << endl;
                    intensitytemp = frmHSV.at<Vec3b>(i, n);
                    intensity2temp = blur.at<Vec3b>(i, n);

                    intensity.val[0] = int(intensitytemp.val[0]/2.0) + int(intensity.val[0]/2.0);

                    intensity.val[1] = int(intensitytemp.val[1]/2.0) + int(intensity.val[1]/2.0);
                    intensity.val[2] = int(intensitytemp.val[2]/2.0) + int(intensity.val[2]/2.0);
                    intensity2.val[0] = int(intensity2temp.val[0]/2.0) + int(intensity2.val[0]/2.0);
                    intensity2.val[1] = int(intensity2temp.val[1]/2.0) + int(intensity2.val[1]/2.0);
                    intensity2.val[2] = int(intensity2temp.val[2]/2.0) + int(intensity2.val[2]/2.0);
                    

                    // intensity.val[0] = int(int(intensity.val[0]) + int(frmHSV.at<Vec3b>(i, n).val[0])/2.0);
                    // intensity.val[1] = int(int(intensity.val[1]) + int(frmHSV.at<Vec3b>(i, n).val[1])/2.0);
                    // intensity.val[2] = int(int(intensity.val[2]) + int(frmHSV.at<Vec3b>(i, n).val[2])/2.0);
                    // intensity2.val[0] = int(int(intensity2.val[0]) + int(blur.at<Vec3b>(i, n).val[0])/2.0);
                    // intensity2.val[1] = int(int(intensity2.val[1]) + int(blur.at<Vec3b>(i, n).val[1])/2.0);
                    // intensity2.val[2] = int(int(intensity2.val[2]) + int(blur.at<Vec3b>(i, n).val[2])/2.0);
                    // grayness = frmGray.at<uchar>(i,n);

                    //minG = (minG>grayness.val[0] ? grayness.val[0] : minG);
                    //maxG = (maxG<grayness.val[0] ? grayness.val[0]: maxG);
                    // intensity.val[0] = round(int(intensity.val[0])/2.0);
                    // intensity.val[1] = round(int(intensity.val[1])/2.0);
                    // intensity.val[2] = round(int(intensity.val[2])/2.0);
                    // intensity2.val[0] = round(int(intensity2.val[0])/2.0);
                    // intensity2.val[1] = round(int(intensity2.val[1])/2.0);
                    // intensity2.val[2] = round(int(intensity2.val[2])/2.0);
                    
                                //cout << int(intensity2.val[1])  << endl;


                }
            }

            //cout << int(intensity2.val[0])  << endl;
            //cout << int(intensity2.val[1])  << endl;
            //cout << int(intensity2.val[2])  << endl;
            
            minintensity.val[0] = int(intensity.val[0]) - offset0L;
            maxintensity.val[0] = int(intensity.val[0]) + offset0H;
            minintensity.val[1] = int(intensity.val[1]) - offset1L;
            maxintensity.val[1] = int(intensity.val[1]) + offset1H;
            minintensity.val[2] = int(intensity.val[2]) - offset2L;
            maxintensity.val[2] = int(intensity.val[2]) + offset2H;
            minintensity2.val[0] = ((int(intensity2.val[0]) - roffset0L)<0) ? 0 : (int(intensity2.val[0]) - roffset0L);
            maxintensity2.val[0] = ((int(intensity2.val[0]) + roffset0H)>255) ? 255 : (int(intensity2.val[0]) + roffset0H);
            minintensity2.val[1] = ((int(intensity2.val[1]) - roffset1L)<0) ? 0 : (int(intensity2.val[1]) - roffset1L);
            maxintensity2.val[1] = ((int(intensity2.val[1]) + roffset1H)>255) ? 255 : (int(intensity2.val[1]) + roffset1H);
            minintensity2.val[2] = ((int(intensity2.val[2]) - roffset1L)<0) ? 0 : (int(intensity2.val[2]) - roffset2L);
            maxintensity2.val[2] = ((int(intensity2.val[2]) + roffset2H)>255) ? 255 : (int(intensity2.val[2]) + roffset2H);          
            if (int(minintensity.val[0])<0)
                minintensity.val[0] = 0;

            if (int(maxintensity.val[0])>255)
                maxintensity.val[0] = 255;
            
            // if (int(minintensity2.val[0])<0)
            // minintensity2.val[0] = 0;

            //  if (int(maxintensity2.val[0])>255)
            // maxintensity2.val[0] = 255;
            
            //  if (int(minintensity2.val[1])<0)
            //      minintensity2.val[1] = 0;

            //  if (int(maxintensity2.val[1])>255)
            //      maxintensity2.val[1] = 255;
            
            //  if (int(minintensity2.val[2])<0)
            //      minintensity2.val[2] = 0;

            //  if (int(maxintensity2.val[2])>255)
            //      maxintensity2.val[2] = 255;
            


            // if (int(maxintensity2.val[0])<int(minintensity2.val[0])){
            //     int temp = maxintensity2.val[0];
            //      maxintensity2.val[0] = minintensity2.val[0]; 
            //      minintensity2.val[0] = temp; 
            // }

            // if (int(maxintensity2.val[1])<int(minintensity2.val[1])){
            //     int temp = maxintensity2.val[1];
            //      maxintensity2.val[1] = minintensity2.val[1];
            //      minintensity2.val[1] = temp; 
            // }


            // if (int(maxintensity2.val[2])<int(minintensity2.val[2])){
            //     int temp = maxintensity2.val[2];
            //      maxintensity2.val[2] = minintensity2.val[2];
            //      minintensity2.val[2] = temp; 
            // }


            // if (int(maxintensity.val[0])<int(minintensity.val[0])){
            //     int temp = maxintensity.val[0];
            //      maxintensity.val[0] = minintensity.val[0];
            //      minintensity.val[0] = temp; 
            // }

            // if (int(maxintensity.val[1])<int(minintensity.val[1])){
            //     int temp = maxintensity.val[1];
            //      maxintensity.val[1] = minintensity.val[1];
            //      minintensity.val[1] = temp; 
            // }


            // if (int(maxintensity.val[2])<int(minintensity.val[2])){
            //     int temp = maxintensity.val[2];
            //      maxintensity.val[2] = minintensity.val[2];
            //      minintensity.val[2] = temp; 
            // }

            //cout << int(minintensity2.val[0]) << endl;
            //cout << int(minintensity2.val[1]) << endl;
            //cout << int(minintensity2.val[2]) << endl;
            //cout << int(maxintensity2.val[0]) << endl;
            //cout << int(maxintensity2.val[1]) << endl;
            //cout << int(maxintensity2.val[2]) << endl;

            //if (minintensity.val[1]<0)
                minintensity.val[1] = 0;

            //if (maxintensity.val[1]>255)
                maxintensity.val[1] = 255;
            
            //if (minintensity.val[2]<0)
                minintensity.val[2] = 0;

            //if (maxintensity.val[2]>255)
                maxintensity.val[2] = 255;
            inRange(frmHSV,Scalar(minintensity),Scalar(maxintensity),rangeRes);
            inRange(blur,Scalar(minintensity2),Scalar(maxintensity2),rangeRes3);
            cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
            cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
            cv::dilate(rangeRes3, rangeRes3, cv::Mat(), cv::Point(-1, -1), 2);
            cv::erode(rangeRes3, rangeRes3, cv::Mat(), cv::Point(-1, -1), 2);
            bitwise_and(rangeRes3,rangeRes,andRes);
            //bitwise_and(rangeRes2,andRes,andRes2);


                        //cout << "pass"  << endl;
            contours.clear();
            cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
            //cout << "pass" << endl;
            vector<vector<cv::Point> > balls;
            vector<cv::Rect> ballsBox;
            vector<Point2f>centers( contours.size() );
            vector<float>radius( contours.size() );
            vector<Point2f>centers2( contours.size() );
            vector<float>radius2( contours.size() );
            balls.clear();
            ballsBox.clear();
            centers.clear();
            radius.clear();
            for (size_t i = 0; i < contours.size(); i++)
            {
                cv::Rect bBox;
                bBox = cv::boundingRect(contours[i]);

 //               minEnclosingCircle( (Mat)contours[i], centers[i], radius[i] );

                float ratio = (float) bBox.width / (float) bBox.height;
                if (ratio > 1.0f)
                    ratio = 1.0f / ratio;

                // Searching for a bBox almost square
                if (ratio > 0.75 && bBox.area() >= 1000)
                {
                    balls.push_back(contours[i]);
                    ballsBox.push_back(bBox);

                }
            }
                        //cout << "pass"  << endl;

            for (size_t i = 0; i < balls.size(); i++)
            {
                cv::drawContours(frameRes, balls, i, CV_RGB(20,150,20), 1);
                cv::rectangle(frameRes, ballsBox[i], CV_RGB(0,255,0), 2);
                minEnclosingCircle( (Mat)balls[i], centers[i], radius[i] );
                cv::Point center;
                center.x = ballsBox[i].x + ballsBox[i].width / 2;
                center.y = ballsBox[i].y + ballsBox[i].height / 2;
                cv::circle(frameRes, center, 2, CV_RGB(20,150,20), -1);
                cv::circle(frameRes, centers[i],int(radius[i]), CV_RGB(0,255,0),2,8,0);
                // Setpoint.x = center.x;
                // Setpoint.y = center.y;
                // Setpoint.z = 0;
                // Setpoint_Pub.publish(Setpoint);
                stringstream sstr;
                sstr << "(" << center.x << "," << center.y << ")";
                cv::putText(frameRes, sstr.str(),
                cv::Point(center.x + 3, center.y - 3),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
            }
                        //cout << "pass"  << endl;


            namedWindow("frameres", WINDOW_NORMAL);
            imshow("frameres", frameRes);
            namedWindow("inrange", WINDOW_NORMAL);
            imshow("inrange", rangeRes);
            //             namedWindow("inrange2", WINDOW_NORMAL);
            // imshow("inrange2", rangeRes2);
            //             namedWindow("inrange3", WINDOW_NORMAL);
            // imshow("inrange3", rangeRes3);
            // namedWindow("andRes", WINDOW_NORMAL);
            // imshow("andRes", andRes);
            // namedWindow("andRes2", WINDOW_NORMAL);
            // imshow("andRes2", andRes2);
                        //cout << "pass"  << endl;

            mouseflag = 2;
            //lastpoint.y = mY;
            //lastpoint.x = mX;
        }







        ros::spinOnce();

        loop_rate.sleep();
        // User key
        ch = cv::waitKey(1);

    }
    // <<<<< Main loop

    return EXIT_SUCCESS;
}
