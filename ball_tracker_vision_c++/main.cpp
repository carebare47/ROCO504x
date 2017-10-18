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
void diff(Mat &src, cv::Mat &dst, bool isXDiff, int gap = 1);
void sqrt(Mat &src, Mat &dst);

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
    Mat input, diffX, diffY;
    vector<Mat> pyramid, prevPyramid, diffPyramid;
    bool isFirstLoop = true;

    LARGE_INTEGER q1,q2,freqq;
    double fps = 0.0;
    double beta = 0.1;
    int var[] = {10, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0};

    ////////////////////
    // Initialisation //
    ////////////////////
    // FPS
    QueryPerformanceCounter(&q1);

    // Initialise images
    getImage(cap, input, imShrink);
    diffPyramid.resize(8);

    // Initialise windows
    imshow("frame", input);
    createTrackbar("gauss", "frame", &var[0], 10);
    createTrackbar("imgID", "frame", &var[1], 6);
    createTrackbar("diffr", "frame", &var[2], 10);

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
//        imshow("DoG", pyramid[0]);

        // Compare Different Scales
        for(int i = 1; i < 8; i++)
        {
            resize(pyramid[i], input, Size(pyramid[i-1].cols, pyramid[i-1].rows));
            pyramid[i-1] -= input;// - pyramid[i];
        }
        imshow("Scale", pyramid[var[1]]);

        // Check if this is the first loop, if so, skip temporal analysis to allow previous data to exist
        if(isFirstLoop == true)
        {
            isFirstLoop = false;
            goto skipTimeAnalysis;
        }

        // Get temporal difference & remove lateral motions
        for(int i = 0; i < 7; i++)
        {
            // Convert to psuedo signed char for the subtraction
            input = pyramid[i] + 128;
            diffPyramid[i] = input - prevPyramid[i];

            // Convert to actual signed short to process the result
            diffPyramid[i].convertTo(diffPyramid[i], CV_16SC1);
            diffPyramid[i] -= 128;

            // Get the x & y differences & remove them
                // This *should* remove lateral motions whilst leaving z-blobs
            diff( diffPyramid[i], diffX, true,  var[2] );
            diff( diffPyramid[i], diffY, false, var[2] );
            diffX *= 100000;
            diffY *= 100000;

            if(i == var[1])
            {
                imshow("X", diffX);
                imshow("Y", diffY);
            }

            // Pythag
            diffX = diffX.mul(diffX) + diffY.mul(diffY);
            sqrt(diffX, diffX);

            // Difference
            diffPyramid[i] = cv::abs(diffPyramid[i]) - diffX;
            diffPyramid[i].convertTo(diffPyramid[i], CV_8UC1);
        }

        imshow("Motion", diffPyramid[var[1]]);

skipTimeAnalysis:
        prevPyramid = pyramid;

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

void diff(Mat &src, cv::Mat &dst, bool isXDiff, int gap)
{
    assert(src.type() == CV_16SC(src.channels()));

    // If x-diff, leave as is, else rotate the image 90 degrees
    if(isXDiff == false)
    {cv::rotate(src, src, ROTATE_90_COUNTERCLOCKWISE);}

    // Ensure dst is of correct dimensions
    dst = Mat::zeros(src.rows, src.cols, src.type());

    // Explicitly keep note of how many channels there are
    int ch = dst.channels();
    int cols = dst.cols;
    int rows = dst.rows;

    // Ensure that "gap" is always at least 1
    if(gap <= 0)
    {gap = 1;}
    int midPixle = gap / 2;

    // Begin!
    for (int i = 0; i < rows; i++)
    {
        short* dp = dst.ptr<short>(i);

        // Perform the differentiation
        for (int j = gap; j < cols; j++)
        {
            // Difference of pixels around and store in middle pixel
            for (int k = 0; k < ch; k++)
            {dp[(j-midPixle)*ch+k] = dp[j*ch+k] - dp[(j-gap)*ch+k];}
        }
    }

    // If y-diff then rotate everything to their correct orientations
    if(isXDiff == false)
    {
        cv::rotate(src, src, ROTATE_90_CLOCKWISE);
        cv::rotate(dst, dst, ROTATE_90_CLOCKWISE);
    }
}

void sqrt(Mat &src, Mat &dst)
{
    assert(src.type() == CV_16SC(src.channels()));

    // Explicitly keep note of how many channels there are
    int ch = src.channels();
    int cols = src.cols;
    int rows = src.rows;

    // Only tinker with 1 image
    if(&src != &dst)
    {src.copyTo(dst);}
    dst = dst.reshape(0, 1);

    short* dp = dst.ptr<short>(0);

    // Element-Wise Square Root
    for (int i = 0; i < rows * cols * ch; i++)
    {dp[i] = std::sqrt(dp[i]);}

    // Convert dst to the same dimensions as src
    dst = dst.reshape(0, rows);
}















