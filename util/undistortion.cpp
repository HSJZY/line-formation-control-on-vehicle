#include "undistortion.h"
#define pi 3.1416
using namespace cv;
using namespace std;

void imgUndistornation(Mat src, Mat dst)
{
    Mat src_data,dst_data;
    src.copyTo(src_data);
    Mat cameraMatrix = Mat::eye(3,3,CV_64F);
    cameraMatrix.at<double>(0,0)=838.9983;
    cameraMatrix.at<double>(0,1)=1.8665;
    cameraMatrix.at<double>(0,2)=290.8101;
    cameraMatrix.at<double>(1,1)=834.2541;
    cameraMatrix.at<double>(1,2)=220.3223;

    Mat distortionCoeffs =Mat::zeros(5,1,CV_64F);
    distortionCoeffs.at<double>(0,0)=0.4148;
    distortionCoeffs.at<double>(1,0)=-0.2995;
    distortionCoeffs.at<double>(2,0)=-0.0169;
    distortionCoeffs.at<double>(3,0)=-0.0346;
    distortionCoeffs.at<double>(4,0)=0;
    Size imagesize =src_data.size();
    Mat map1,map2;
    initUndistortRectifyMap(cameraMatrix,distortionCoeffs,Mat(),cameraMatrix,imagesize,CV_16SC2,map1,map2);

    remap(src_data,dst_data,map1,map2,INTER_LINEAR);

    dst_data.copyTo(dst);
}

void imgToAirScale(Mat src, Mat dst)
{

    Mat src_frame ;
    Mat dst_frame;
    Mat map_x;
    Mat map_y;
    src.copyTo(src_frame);
    dst_frame.create(src_frame.size(),src_frame.type());
    map_x.create(src_frame.size(),CV_32FC1);
    map_y.create(src_frame.size(),CV_32FC1);


    for(int i=0;i<src_frame.rows;i++)
    {
        for(int j=0;j<src_frame.cols;j++)
        {
            cv::Point2f point;
            point.x=i;
            point.y=j;
            pointToAirScale(point,1000);
            map_x.at<float>(point.x,point.y)=j;
            map_y.at<float>(point.x,point.y)=i;
        }
    }

    remap(src_frame,dst_frame,map_x,map_y,INTER_LANCZOS4);

    dst_frame.copyTo(dst);

}

int pointToAirScale(cv::Point2f &point, double Dist)
{
    double f=618;
    double d1=10;
    double d2=3;
    double n_air=1.0;
    double n_water=1.33;
    double n_glass=1.49;
    double cameraCenter[2]={313.428,260.795};
    double thita1,thita2,thita3,thita4;
    double l1,l2,l3,l4,L,h;
    double src_r,dst_r;
    double src_r_x,src_r_y,dst_r_x,dst_r_y;
    src_r_x = point.x-cameraCenter[0];
    src_r_y = point.y-cameraCenter[1];
    src_r = sqrt(pow(src_r_x,2)+pow(src_r_y,2));
    thita3 = atan(src_r/f);
    thita2 = asin(n_air*sin(thita3)/n_glass);
    thita1 = asin(n_glass*sin(thita2)/n_water);
    l1 = d1/cos(thita3);
    l2 = d2/cos(thita2);
    l3 = (Dist-d1-d2)/cos(thita1);
    l4 = sqrt(pow(l2,2)+pow(l3,2)-2*cos(pi+thita2-thita1)*l2*l3);
    double buff;
    buff=pow(l2,2)+pow(l4,2)-pow(l3,2);
    thita4 = acos(buff/2/l2/l4);
    L = sqrt(pow(l1,2)+pow(l4,2)-2*cos(pi+thita4+thita2-thita3)*l1*l4);
    h = sqrt(pow(L,2)-pow(Dist,2));
    dst_r = f*h/Dist;
    dst_r_x=src_r_x*dst_r/src_r;
    dst_r_y=src_r_y*dst_r/src_r;
    point.x=dst_r_x+cameraCenter[0];
    point.y=dst_r_y+cameraCenter[1];

    return 0;
}
