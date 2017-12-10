/////////////////////////////////
//////////////////////////@TODO'S
/////////////////////////////////
//@todo separate out current length and change in length


/////////////////////////////////
/////////////////////DEPENDENCIES
/////////////////////////////////
#include <vector>

/////////////////////////////////
////////////////NAMESPACES IN USE
/////////////////////////////////
using namespace std;

/////////////////////////////////
////////////FUNCTION DECLARATIONS
/////////////////////////////////
float pythag(float &x, float &y);
void coordToLength(vector<float> &coord, vector<float> &length);
void lengthToCoord(vector<float> &length, vector<float> &coord);
int convertLengthToSteps(float &length);
float convertStepsToLength(int &steps);

/////////////////////////////////
/////////////////GLOBAL CONSTANTS
/////////////////////////////////
const float gripperLength = 0.0;    // Assumes a square gripper
const float frameLengthX = 1.0;
const float frameLengthY = 1.0;
const float lengthToStepsRatio = 1.0;

// Simple model modification converts the square gripper into a point
const float xMax = frameLengthX - gripperLength;
const float yMax = frameLengthY - gripperLength;


/////////////////////////////////
///////SHOULD PROBABLY BE IN MAIN
/////////////////////////////////
// Initialise the system
vector<float> coord = {0.5, 0.5};
vector<float> length;
length.resize(4);


/////////////////////////////////
//////////////////FUNCTION BODIES
/////////////////////////////////
// Performs pythagorases theorem given 2 lengths
float pythag(float &x, float &y)
{
    return sqrt( x^2 + y^2 );
}

// Converts x & y gripper coordinates into cable lengths
void coordToLength(vector<float> &coord, vector<float> &length)
{
    length[0] = pythag( xMax-coord[0], yMax-coord[1] );
    length[1] = pythag(      coord[0], yMax-coord[1] );
    length[2] = pythag(      coord[0],      coord[1] );
    length[3] = pythag( xMax-coord[0],      coord[1] );
    
    return;
}

// Converts cable lengths into x & y gripper coordinates
void lengthToCoord(vector<float> &length, vector<float> &coord)
{
    // x = l3 * cos(theta3)
    // Cosine rule => cos(theta3) = (xMax^2 + l3^2 - l4^2) / (2 * xMax * l3)
    // Simplifying => x = (xMax/2) + (l3^2 - l4^2) / (2 * xMax)
    // Same can be applied to get y if l1 & l4 are used
    coord[0] = (xMax/2) + (length[3]^2 - length[4]^2) / (2*xMax);
    coord[1] = (yMax/2) + (length[4]^2 - length[1]^2) / (2*yMax);
    
    return;
}

// Converts a length into an integer number of steps for a stepper motor to make
int convertLengthToSteps(float &length)
{
    return int(length * lengthToStepsRatio);
}

// Convert a given number of stepper motor steps into a length
float convertStepsToLength(int &steps)
{
    return (float)steps / lengthToStepsRatio;
}





























