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
void calcFieldOfExpansion(Mat &posX, Mat &posY, Mat &velX, Mat &velY, Point2f &foe);
void calcTimeToContact(Mat &velX, Mat &velY, Point2f &foe, Mat &ttc);
void sumErrAlongRow(Mat &src, Mat &dst);

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

    ///////////////
    // Variables //
    ///////////////
    Mat input;
    vector<Mat> pyramid, prevPyramid, diffPyramid;
    bool isFirstLoop = true;

    LARGE_INTEGER q1,q2,freqq;
    double fps = 0.0;
    double beta = 0.1;
    int var[] = {5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    ////////////////////
    // Initialisation //
    ////////////////////
    // FPS
    QueryPerformanceCounter(&q1);

    // Initialise images
    getImage(cap, input, imShrink);

    // Initialise windows
    imshow("frame", input);
    createTrackbar("gauss", "frame", &var[0], 10);
    createTrackbar("imgID", "frame", &var[1], 7);

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
        imshow("frame", input);

        // Create the image pyramid
        for(int i = 0; i < 7; i++)
        {
            pyramid.push_back(input);
            resize(input, input, Size(input.cols/2, input.rows/2));
        }
        pyramid.push_back(input);

        // Perform a difference of gaussians DoG
        for(int i = 0; i < 8; i++)
        {
            GaussianBlur(pyramid[i], input, Size(var[0]*2+1, var[0]*2+1), 0);
            absdiff(pyramid[i], input, pyramid[i]);
            cvtColor(pyramid[i], pyramid[i], CV_BGR2GRAY);
        }
        imshow("DoG", pyramid[0]);

        // Compare Different Scales
        for(int i = 1; i < 8; i++)
        {
            resize(pyramid[i-1], input, Size(pyramid[i].cols, pyramid[i].rows));
            pyramid[i-1] = input - pyramid[i];
        }
        imshow("Scale", pyramid[var[1]]);

        // Compare with previous values
        if(isFirstLoop == true)
        {goto skipTimeAnalysis;}



skipTimeAnalysis:


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

        // Reset the vector
        pyramid.clear();
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
//    cvtColor(frame, frame, CV_BGR2GRAY);
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
        float* xp = x.ptr<float>(i);
        float* yp = y.ptr<float>(i);

        // For each column
        for(int j = 0; j < cols; j++)
        {
            xp[j] = j;  // Make an image where the intensity equals the x-coord
            yp[j] = i;  // Make an image where the intensity equals the y-coord
        }
    }
}

void calcFieldOfExpansion(Mat &posX, Mat &posY, Mat &velX, Mat &velY, Point2f &foe)
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
    Mat foeCalc = A.inv() * b;
    foe = {foeCalc.at<float>(0), foeCalc.at<float>(1)};

    // Convert all input images back to their original dimensions
    posX = posX.reshape(0, rows);
    posY = posY.reshape(0, rows);
    velX = velX.reshape(0, rows);
    velY = velY.reshape(0, rows);
}

void calcTimeToContact(Mat &velX, Mat &velY, Point2f &foe, Mat &ttc)
{
    ttc = Mat::zeros(velX.rows, velX.cols, velX.type());

    // For each row
    for(int i = 0; i < velX.rows; i++)
    {
        float* xp = velX.ptr<float>(i);
        float* yp = velY.ptr<float>(i);
        float* tp =  ttc.ptr<float>(i);

        // For each column
        for(int j = 0; j < velX.cols; j++)
        {
            tp[j] = sqrt( pow( j - foe.x, 2 ) + pow( i - foe.y, 2 ) ) /
                    sqrt( pow( xp[j], 2 ) + pow( yp[j], 2 ) );
        }
    }
}

void sumErrAlongRow(Mat &src, Mat &dst)
{
    dst = Mat::zeros(1, src.rows, CV_32SC1);
    int* dp = dst.ptr<int>(0);

    // For each row get the sum(abs(x))-abs(sum(x))
    for(int i = 0; i < src.rows; i++)
    {
        int acc = 0;
        char* sp = src.ptr<char>(i);

        // For each column
        for(int j = 0; j < src.cols; j++)
        {
            dp[i] += sp[j];
            acc   += abs(sp[j]);
        }
        dp[i] = acc - abs(dp[i]);
    }
//    dst = dst.t();    // If we're being anal, then yes, this would be included
}



















