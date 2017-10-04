#include <windows.h>
#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

using namespace std;
using namespace cv;

void shadeReduction(Mat &src, Mat &dst, float range, float mult);
void getImage(cv::VideoCapture &cap, cv::Mat &frame, uint shrink = 1);
Mat calcHist(Mat &src, int channel);
void calcBackProject(Mat &src, Mat* hist, Mat &dst);

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

    // Other Variables
    Mat input, expAvg, motion, incMass, splinter[3];
    Mat hist[3], histPrev[3], histVelo[3], histThr[3];
    LARGE_INTEGER q1,q2,freqq;
    double fps = 0.0;
    double beta = 0.1;
    int var[] = {74, 50, 2, 50, 10, 7, 75, 40, 0, 0, 0};

    // Initialise
    getImage(cap, input, imShrink);
    input.copyTo(expAvg);
    input.copyTo(incMass);
    for(int i = 0; i < 3; i++)
    {
        hist[i] = Mat::zeros(1,histSize,CV_32F);
        histPrev[i] = Mat::zeros(1,histSize,CV_32F);
        histVelo[i] = Mat::zeros(1,histSize,CV_32F);
        splinter[i] = Mat::zeros(input.rows, input.cols, CV_8UC1);
    }

    imshow("frame", input);
    createTrackbar("rng", "frame", &var[0], 100);
    createTrackbar("mul", "frame", &var[1], 100);
    createTrackbar("blr", "frame", &var[2], 20);
    createTrackbar("avg", "frame", &var[3], 100);
    createTrackbar("thr", "frame", &var[4], 255);
    createTrackbar("hst", "frame", &var[5], 100);
    createTrackbar("vhs", "frame", &var[6], 100);
    createTrackbar("vel", "frame", &var[7], 100);

    // Start Main Code
    QueryPerformanceCounter(&q1);
    for(;;)
    {
        // Get FPS
        QueryPerformanceCounter(&q2);
        QueryPerformanceFrequency( &freqq );
        fps = (1.0-beta)*fps + beta*(double)freqq.QuadPart/double(q2.QuadPart-q1.QuadPart);
        QueryPerformanceCounter(&q1);
        if(isfinite(fps) == false) {fps = 0.0;}
        cout << "fps: " << fps << endl;

        // ###################################################################################
        // Main code #########################################################################
        // ###################################################################################
        getImage(cap, input, imShrink);
        cvtColor(input, input, CV_BGR2HLS);
//        imshow("ori", input);

        // Normalise the image
        medianBlur(input, input, var[2]*2+1);
//        shadeReduction(input, input, double(var[0]), double(var[1]/10.0));
        medianBlur(input, input, var[2]*2+1);
        imshow("frame", input);

        // Apply moving average to remember motion, compare with current frame to get motion
        expAvg = (1.0-var[3]/100.0) * expAvg + (var[3]/100.0) * input;
//        absdiff(input, expAvg, motion);

        // Threshold to give a binary image of areas of motion
//        cvtColor(motion, motion, CV_BGR2GRAY);
//        threshold(motion, motion, var[4], 255, THRESH_BINARY);
//        imshow("motion", motion);

        // Obtain a histogram for each colour channel of the original image
        for(int i = 0; i < 3; i++)
        {
            // Get the histogram
            hist[i] = calcHist(expAvg, i);
            // Get the smoothed rate of change of "mass"
            histVelo[i] = (1.0-var[6]/100.0) * histVelo[i] + (var[6]/100.0) * (hist[i] - histPrev[i]);
            // Get a smoothed prior to compare with
            histPrev[i] = (1.0-var[7]/100.0) * histPrev[i] + (var[7]/100.0) * hist[i];
            // Threshold the rate of change of mass to remove -ve values
            threshold(histVelo[i], histThr[i], 10*var[5], 255, THRESH_TOZERO);
//            cout << i << ": " << histThr[i] << endl;
        }

        // Project the histogram findings backwards
        calcBackProject(input, histThr, incMass);
        imshow("incMass", incMass);

        // Merge the binary images
//        bitwise_and(incMass, motion, input);
//        imshow("mask", input);

        cout << "count: " << countNonZero(incMass) << "\t";

        char quit = waitKey(1);
        if(quit == 'q')
        {break;}
    }
}

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

void getImage(cv::VideoCapture &cap, cv::Mat &frame, uint shrink)
{
    cap >> frame;
    resize(frame, frame, Size(frame.cols/shrink, frame.rows/shrink));
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















