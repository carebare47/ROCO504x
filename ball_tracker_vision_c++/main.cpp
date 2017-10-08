#include <windows.h>    /// @todo remove - only dependency for it is the fps counter
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

//////////////////////
// Custom functions //
//////////////////////
// Original Functions
void shadeReduction(Mat &src, Mat &dst, float range, float mult);
void getImage(cv::VideoCapture &cap, cv::Mat &frame, uint shrink = 1);
Mat calcHist(Mat &src, int channel);

// New Functions
void calcXYImgs(int rows, int cols, Mat &x, Mat &y);
void calcFieldOfExpansion(Mat &posX, Mat &posY, Mat &velX, Mat &velY, Mat &foe);

//////////////////////
// Global Constants //
//////////////////////
const int imShrink = 2;

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
    Mat input, expAvg, flow, splinter2[2], splinter3[3], output;
    Mat posX, posY, foe;
    LARGE_INTEGER q1,q2,freqq;
    double fps = 0.0;
    double beta = 0.1;
    int var[] = {0, 0, 2, 100, 8, 1, 0, 0, 0, 0, 0};

    ////////////////////
    // Initialisation //
    ////////////////////
    // FPS
    QueryPerformanceCounter(&q1);

    // Initialise images
    getImage(cap, input, imShrink);
    input.copyTo(expAvg);
    calcXYImgs(input.rows, input.cols, posX, posY);

    // Initialise windows
    imshow("frame", input);
//    createTrackbar("_rng", "frame", &var[0], 100);
//    createTrackbar("_mul", "frame", &var[1], 100);
    createTrackbar("_blr", "frame", &var[2], 20);
    createTrackbar("_avg", "frame", &var[3], 100);
    createTrackbar("_lvl", "frame", &var[4], 8);
    createTrackbar("wins", "frame", &var[5], 20);
    createTrackbar("iter", "frame", &var[6], 9);

    /////////////////////
    // Start Main Code //
    /////////////////////
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

        // Get the next image frame & normalise
        getImage(cap, input, imShrink);
        medianBlur(input, input, var[2]*2+1);
//        shadeReduction(input, input, double(var[0]), double(var[1]/10.0));
        imshow("frame", input);

        // Get the optical flow between the previous image and the current image
        calcOpticalFlowFarneback(expAvg, input, flow, 0.5, var[4]+1,
                                var[5]*2+1, var[6]+1, 5, 1.2, OPTFLOW_FARNEBACK_GAUSSIAN );
        split(flow, splinter2);
        calcFieldOfExpansion(posX, posY, splinter2[0], splinter2[1], foe);
        cout << foe << endl;









//        for(int i = 0; i < 2; i++)
//        {
//            splinter2[i] *= 255;
//            splinter2[i].convertTo(splinter3[i], CV_8UC1);
//        }
//        splinter3[2] = Mat::zeros(input.rows, input.cols, CV_8UC1);
////        splinter3[2] += 255;
//        cv::merge(splinter3, 3, output);
//        imshow("opticalFlow", output);
















        // Apply moving average to remember motion
        expAvg = (1.0-var[3]/100.0) * expAvg + (var[3]/100.0) * input;
//        input.copyTo(expAvg);

        // Select action based on the users input
        switch (waitKey(1))
        {
        // User pressed 'q' for 'quit'
        case 'q':
            goto end;
            break;

        // For all other use cases, change nothing
        default:
            break;
        }
    }

/// Immediate exit point from code
end: ;
}

//////////////////////###################################################################
// CUSTOM FUNCTIONS //###################################################################
//////////////////////###################################################################
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
    cvtColor(frame, frame, CV_BGR2GRAY);
}

Mat calcHist(Mat &src, int channel)
{
    Mat hist;
    static const int nDims = 1;
    static const int nImgs = 1;
    static const int channels[] = {channel};
    static const int histSizes[] = {256};
    static const float range[] = { 0, 256 };// SET TO 256 FOR EXPECTED FUNCTIONALITY, size for thresholding effects
    static const float* ranges[] = { range };

    calcHist( &src, nImgs, channels, Mat(), // do not use mask
             hist, nDims, histSizes, ranges,
             true, // the histogram is uniform
             false // empty the hist before hand
             );
    return hist.t();
}

void calcXYImgs(int rows, int cols, Mat &x, Mat &y)
{
    x = Mat::zeros(rows, cols, CV_32FC1);
    y = Mat::zeros(rows, cols, CV_32FC1);

    // For each row
    for(int i = 0; i < rows; i++)
    {
        int* xp = x.ptr<int>(i);
        int* yp = y.ptr<int>(i);

        // For each column
        for(int j = 0; j < cols; j++)
        {
            xp[j] = j;  // Make an image where the intensity equals the x-coord
            yp[j] = i;  // Make an image where the intensity equals the y-coord
        }
    }
}

void calcFieldOfExpansion(Mat &posX, Mat &posY, Mat &velX, Mat &velY, Mat &foe)
{
    // Assume all sizes & types are identical, & all are single channeled
    int rows = posX.rows;
    int cols = posX.cols;

    // A = [Vy(:) -Vx(:)];
    // b = posX(:).*Vy(:)-posY(:).*Vx(:);
    // FOE_meas = (A'*A)\A'*b;
    // FOE = FOE_meas';

    // Convert all input images to vectors
    posX = posX.reshape(0, rows*cols);
    posY = posY.reshape(0, rows*cols);
    velX = velX.reshape(0, rows*cols);
    velY = velY.reshape(0, rows*cols);

    // Create a matrix containing both velocity vectors
    // A = [Vy(:) -Vx(:)];
    Mat A(rows*cols, 2, CV_32FC1);
    Mat left(A, Rect(0, 0, 1, A.rows));
    Mat right(A, Rect(1, 0, 1, A.rows));
    velY.copyTo(left);
    velX.copyTo(right);
    right *= -1;

    // b = posX(:).*Vy(:)-posY(:).*Vx(:);
    Mat b = posX.mul(velY) - posY.mul(velX);

    // FOE_meas = (A'*A)\A'*b;
    // FOE = FOE_meas';
    b = A.t() * b;
    A = A.t() * A;
    foe = A.inv() * b;
    foe = foe.t();
}























