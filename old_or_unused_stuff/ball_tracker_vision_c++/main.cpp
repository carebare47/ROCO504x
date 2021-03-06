//#include <windows.h>    /// @todo remove - only dependency for it is the fps counter
#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

////////////////
// Namespaces //
////////////////
using namespace std;
using namespace cv;

/////////////
// Structs //
/////////////
struct minMaxPair
{
    int min;
    int max;
};
struct objParams
{
   minMaxPair BGR[3];
   minMaxPair HSV[3];
   bool shiftHue;
};

//////////////////////
// Custom functions //
//////////////////////

// Originally for mass increase algorithm
void shadeReduction(Mat &src, Mat &dst, float range, float mult);
void getImage(cv::VideoCapture &cap, cv::Mat &frameBGR, Mat &frameHSV, uint shrink = 1);
Mat calcHist(Mat &src, int channel);
void calcBackProject(Mat &src, Mat* hist, Mat &dst);

// Originally for colour detection algorithm
void getObjParams(Mat &srcBGR, Mat &srcHSV, vector<objParams> &objects, float stdDevs);
Rect getSubImageRoi(Mat &src, float percent);
void shiftHue(Mat &src, Mat &dst);
void getColourMask(Mat &src, Mat &mask, objParams params, bool isBGR);

//////////////////////
// Global Constants //
//////////////////////
const int imShrink = 2;
const int histSize = 256;
const int histReduce = 256 / histSize;

int main()
{
    // Create Video Reader & show an image
    VideoCapture cap(1);  // open the chosen camera feed
    if(!cap.isOpened())   // check if we succeeded
    {return -1;}

//    // Create Video Writer
//    VideoWriter maskVideo("C:\\Users\\Student\\Desktop\\Experiment.avi",
//                          -1, cap.get(CV_CAP_PROP_FPS), Size(w, h), false);
//    if(!maskVideo.isOpened())   // check if we succeeded
//    {return -1;}

    ///////////////
    // Variables //
    ///////////////
    Mat input[2], mask[2];
    vector<objParams> objects;
    Moments moment;

//    LARGE_INTEGER q1,q2,freqq;
//    double fps = 0.0;
//    double beta = 0.1;
    int var[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    ////////////////////
    // Initialisation //
    ////////////////////
//    QueryPerformanceCounter(&q1);
    getImage(cap, input[0], input[1], imShrink);
    imshow("frame", input[0]);
    createTrackbar("bm", "frame", &var[0], 255);
    createTrackbar("bM", "frame", &var[1], 255);
    createTrackbar("gm", "frame", &var[2], 255);
    createTrackbar("gM", "frame", &var[3], 255);
    createTrackbar("rm", "frame", &var[4], 255);
    createTrackbar("rM", "frame", &var[5], 255);
    createTrackbar("h?", "frame", &var[7], 1);

    /////////////////////
    // Start Main Code //
    /////////////////////
    for(;;)
    {
//        // Get FPS
//        QueryPerformanceCounter(&q2);
//        QueryPerformanceFrequency( &freqq );
//        fps = (1.0-beta)*fps + beta*(double)freqq.QuadPart/double(q2.QuadPart-q1.QuadPart);
//        QueryPerformanceCounter(&q1);
//        if(isfinite(fps) == false) {fps = 0.0;}
//        cout << "fps: " << fps << endl;

        // ###################################################################################
        // Main code #########################################################################
        // ###################################################################################

        // Get the next image frame
        getImage(cap, input[0], input[1], imShrink);
        imshow("frame", input[0]);

        // Select action based on the users input
        switch (waitKey(1))
        {
        // User pressed 'q' for 'quit'
        case 'q':
            goto end;
            break;

        // User pressed 'c' for 'capture'
        case 'c':
            getObjParams(input[0], input[1], objects, 1.5);
            break;

        // User pressed 'u' for 'update'
        case 'u':
            if(objects.size() < 1)
            {
                objParams temp;
                for(int i = 0; i < 3; i++)
                {
                    temp.BGR[i].min=0;
                    temp.BGR[i].max=255;
                    temp.HSV[i].max=0;
                    temp.HSV[i].max=255;
                    temp.shiftHue = false;
                }
                objects.push_back(temp);
            }

            if(var[7] == 0)
            {
                for(int i = 0; i < 3; i++)
                {objects[0].BGR[i].min=var[2*i]; objects[0].BGR[i].max=var[2*i+1];}
            }
            else
            {
                for(int i = 0; i < 3; i++)
                {objects[0].HSV[i].min=var[2*i]; objects[0].HSV[i].max=var[2*i+1];}
            }
            break;

        // For all other use cases, change nothing
        default:
            break;
        }

        // Detect all objects, show only the selected's mask
        var[6] = 0;
        for(objParams object : objects)
        {
            // Get the BGR & HSV masks separately (for possible viewing later)
            getColourMask(input[0], mask[0], object, true);
            getColourMask(input[1], mask[1], object, false);

            imshow("bgr", mask[0]);
            imshow("hsv", mask[1]);

            // Merge the results
            bitwise_or(mask[0], mask[1], mask[0]);
            input[1] = Mat::zeros(input[1].rows, input[1].cols, input[1].type());
            input[0].copyTo(input[1], mask[0]);
            imshow("all", input[1]);

            // Get the center of mass of any objects in the mask
            moment = moments(mask[0]);
            cout << var[6] << " x:y = " <<
                    moment.m10/moment.m00 << ":" <<
                    moment.m01/moment.m00 << endl;
            var[6]++;
            /// @todo link to rest of system
        }
    }

/// Immediate exit point from code
end: ;
}

//////////////////////###################################################################
// CUSTOM FUNCTIONS //###################################################################
//////////////////////###################################################################

// Mass Algorithm Functions
void shadeReduction(Mat &src, Mat &dst, float range, float mult)
{
    int ch = src.channels();
    Mat ori, temp, sub;
    src.convertTo(ori, CV_32FC(ch));

    // Get the flattening image
    cv::log(ori, temp);
    temp = (temp*range - ori) / range;  // Genius accident?!?!?!
    cv::exp(temp, temp);
//    sub = temp / 16;
//    imshow("log", sub);

    // Apply flattening image
    temp *= mult;
    sub = Mat(temp.size(), temp.type(), mean(temp));
    temp = temp + ori - sub;
    temp.convertTo(dst, CV_8UC(ch));
}

void getImage(cv::VideoCapture &cap, cv::Mat &frameBGR, cv::Mat &frameHSV, uint shrink)
{
    Mat frame;
    cap >> frame;
    resize(frame, frameBGR, Size(frame.cols/shrink, frame.rows/shrink));
    cvtColor(frameBGR, frameHSV, CV_BGR2HSV);
}

Mat calcHist(Mat &src, int channel)
{
    Mat hist;
    static const int nDims = 1;
    static const int nImgs = 1;
    static const int channels[] = {channel};
    static const int histSizes[] = {histSize};
    static const float range[] = { 0, 256 };// SET TO 256 FOR EXPECTED FUNCTIONALITY, size for thresholding effects
    static const float* ranges[] = { range };

    calcHist( &src, nImgs, channels, Mat(), // do not use mask
             hist, nDims, histSizes, ranges,
             true, // the histogram is uniform
             false // empty the hist before hand
             );
    return hist.t();
}

void calcBackProject(Mat &src, Mat* hist, Mat &dst)
{
    // Remember image dimensions
    int numCh = src.channels();
    int rows = src.rows;

    // Reshape into a row matrix
    dst = Mat::zeros(1, src.rows*src.cols, CV_8U);
//    dst += 255;
    src = src.reshape(0, 1);

    float* hp[] = {hist[0].ptr<float>(0), hist[1].ptr<float>(0), hist[2].ptr<float>(0)};
    uchar* dp = dst.ptr<uchar>(0);
    uchar* sp = src.ptr<uchar>(0);
    int id = 0;

    // Begin the back projection! (For each pixel)
    for (int i = 0; i < dst.cols; i++)
    {
        // For each channel
        for (int j = 0; j < numCh; j++)
        {
            if(j != 2){continue;}
            // Get the pixel + channel id
            id = i*numCh+j;
            // Find respective histogram entry and provide binary output
            dp[i] += (hp[j][sp[id]/histReduce] > 0.01) ? 85 : 0;
        }
    }

    // Return output back to the original dimensions
    dst = dst.reshape(0, rows);
    src = src.reshape(0, rows);
}

// Colour Description
void getObjParams(Mat &srcBGR, Mat &srcHSV, vector<objParams> &objects, float stdDevs)
{
    assert(srcBGR.size == srcHSV.size &&
           srcBGR.channels() == srcHSV.channels());

    // Variables
    objParams newObj;
    Mat miniImg, splinter3[3], splinter7[7], mean, stdDev;
    Rect roi = getSubImageRoi(srcBGR, 1.0);

    // Split each image into separate channels
    // BGR image
    split(srcBGR(roi), splinter3);
    for(int i = 0; i < 3; i++)
    {splinter3[i].copyTo(splinter7[i]);}

    // HSV image
    split(srcHSV(roi), splinter3);
    for(int i = 0; i < 3; i++)
    {splinter3[i].copyTo(splinter7[i+4]);}

    // Shifted hue channel
    shiftHue(splinter3[0], splinter7[3]);

    // Merge all channels into a single image, and get some statistics
    cv::merge(splinter7, 7, miniImg);
    meanStdDev(miniImg, mean, stdDev);

    // For each channels stats, find the most appropriate min and max values
    for(int i = 0; i < 3; i++)
    {
        // BGR
        newObj.BGR[i].min = int(mean.at<uchar>(i) - stdDev.at<uchar>(i) * stdDevs);
        newObj.BGR[i].max = int(mean.at<uchar>(i) + stdDev.at<uchar>(i) * stdDevs);

        // HSV - selecting the more optimal hue variant
        newObj.shiftHue = false;
        if((i == 0) && (stdDev.at<uchar>(3) < stdDev.at<uchar>(4)))
        {
            newObj.HSV[i].min = int(mean.at<uchar>(3) - stdDev.at<uchar>(3) * stdDevs);
            newObj.HSV[i].max = int(mean.at<uchar>(3) + stdDev.at<uchar>(3) * stdDevs);
            newObj.shiftHue = true;
        }
        else
        {
            newObj.HSV[i].min = int(mean.at<uchar>(i+4) - stdDev.at<uchar>(i+4) * stdDevs);
            newObj.HSV[i].max = int(mean.at<uchar>(i+4) + stdDev.at<uchar>(i+4) * stdDevs);
        }
    }

    // Add the newly created object to the vector list
    objects.clear();
    objects.push_back(newObj);
}

Rect getSubImageRoi(Mat &src, float percent)
{
    int rows = src.rows*percent;
    int cols = src.cols*percent;
    Rect roi = Rect( (src.cols-cols)/2, (src.rows-rows)/2, cols, rows );
    return roi;
}

void shiftHue(Mat &src, Mat &dst)
{
    assert(src.channels() == 1);

    // Setup
    int rows = src.rows;
    dst = src.reshape(0, 1);
    uchar* dp = dst.ptr<uchar>(0);

    // Begin the conversion
    for (int i = 0; i < dst.cols; i++)
    {dp[i] = (dp[i] < 90) ? dp[i]+90 : dp[i]-90;}

    // Return output back to the original dimensions
    dst = dst.reshape(0, rows);
}

// Colour Tracker
void getColourMask(Mat &src, Mat &mask, objParams params, bool isBGR)
{
    // Initialise
    minMaxPair limits;
    Mat splinter3[3], temp;
    split(src, splinter3);
    if((isBGR == true) && (params.shiftHue == true))
    {shiftHue(splinter3[0], splinter3[0]);}

    // Threshold everything
    for(int i = 0; i < 3; i++)
    {
        // Get channel parameters
        if(isBGR == true)
        {limits = params.BGR[i];}
        else
        {limits = params.HSV[i];}

        // Extract channel mask
        inRange(splinter3[i], limits.min, limits.max, splinter3[i]);
    }

    // Bitwise AND the channels together to generate the resultant mask
    bitwise_and(splinter3[0], splinter3[1], mask);
    bitwise_and(splinter3[2], mask, mask);

    cv::merge(splinter3, 3, temp);
    if(isBGR == true)
    {imshow("BGR_Mask", temp);}
    else
    {imshow("HSV_Mask", temp);}
}


























