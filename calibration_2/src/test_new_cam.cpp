#include <ctime>
#include "opencv2/opencv.hpp"
#include <iostream>
 
using namespace cv;
using namespace std;
 
int main()
{
    VideoCapture inputVideo(0);
    if (!inputVideo.isOpened())
    {
        cout << "Could not open the input video: " << endl;
        return -1;
    }

    Mat frame;
    Mat frameCalibration;
 
    inputVideo >> frame;
    
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 971.057507 ;
    cameraMatrix.at<double>(0, 2) = 947.979129;
    cameraMatrix.at<double>(1, 1) = 974.701312;
    cameraMatrix.at<double>(1, 2) = 542.246776;
 
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = -0.313640;
    distCoeffs.at<double>(1, 0) = 0.079529;
    distCoeffs.at<double>(2, 0) = -0.001175 ;
    distCoeffs.at<double>(3, 0) = 0.000956;
    distCoeffs.at<double>(4, 0) = 0;
 
    Mat view, rview, map1, map2;
    Size imageSize;
    imageSize = frame.size();
    
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
        imageSize, CV_16SC2, map1, map2);
 


    size_t i=1;
    time_t begin,end;

    while (1) //Show the image captured in the window and repeat
    {
        inputVideo >> frame;              // read
        if (frame.empty()) break;         // check if at end
        
        begin = clock();
        remap(frame, frameCalibration, map1, map2, INTER_LINEAR);
        end = clock();
        cout<<(1.0*begin-end)/CLOCKS_PER_SEC<<endl;

        imshow("Origianl", frame);
        imshow("Calibration", frameCalibration);
        char key = waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')break;
    }
    return 0;
}