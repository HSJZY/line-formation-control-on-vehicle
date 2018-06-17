#ifndef CAMERAPOSITION_H
#define CAMERAPOSITION_H
// Simple example to demonstrate usage of ARToolKitPlus
// This sample does not open any graphics window. It just
// loads test images and shows how to use the ARToolKitPlus API.

#include <stdio.h>
#include <ARToolKitPlus/TrackerSingleMarker.h>
#include <opencv2/opencv.hpp>

#include <math.h>
#include <iostream>
#include <vector>

#include <fstream>
#include <strstream>
#include "dataLib.h"

using namespace cv;
using namespace std;
using ARToolKitPlus::TrackerSingleMarker;

using namespace cv;
using namespace std;
using ARToolKitPlus::TrackerSingleMarker;

#define pi 3.1415926


class Positing
{

public:

    std::ofstream outFile();

    int tryAutoThreshold(cv::Mat src, TrackerSingleMarker* tracker, ARToolKitPlus::ARMarkerInfo* & markInfo,double*outputMatrix, int tryNum =200);

    int oldCalcuate(Mat src, Mat & dst, ARToolKitPlus::TrackerSingleMarker *tracker, double *outputMatrix);

    int newCalcuate(Mat src, Mat & dst, ARToolKitPlus::TrackerSingleMarker &tracker, double *outputMatrix);

    void drawMarkerInfo(cv::Mat &image, ARToolKitPlus::ARMarkerInfo*  markInfo);

    int refractionRecover(ARToolKitPlus::ARMarkerInfo*  markInfo,double Dist);

    int opencvPnpSolve(ARToolKitPlus::ARMarkerInfo*  markInfo,double *outputMatrix);

    int getID(void);
private:
    static int m_ID;
};

void rotToEular_rad(double* rot,double* eular);
void rotToEular_angle(double* rot,double* eular);
void init_traker(ARToolKitPlus::TrackerSingleMarker *tracker, int maker_width = 100 );
void get_visual_pose(Mat frame_src, Mat* frame_dst, ARToolKitPlus::TrackerSingleMarker *tracker);

#endif // CAMERAPOSITION_H
