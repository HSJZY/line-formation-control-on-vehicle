#ifndef UNDISTORTION_H
#define UNDISTORTION_H
#include</usr/local/include/opencv2/opencv.hpp>
#include<cmath>
#include<iostream>


void imgUndistornation(cv::Mat src,cv::Mat dst);

void imgToAirScale(cv::Mat src,cv::Mat dst);

int pointToAirScale(cv::Point2f &point, double Dist);
#endif // UNDISTORTION_H
