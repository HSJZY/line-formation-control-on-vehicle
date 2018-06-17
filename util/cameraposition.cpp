#include"cameraposition.h"
#include "undistortion.h"
int Positing::m_ID;

//设置自动阈值模式，默认尝试100次, 这里的 markInfo为指针类型的引用传递
int Positing::tryAutoThreshold(cv::Mat src, TrackerSingleMarker* tracker,
                               ARToolKitPlus::ARMarkerInfo* & markInfo,double*outputMatrix, int tryNum )
{
    cv::Mat dst;
    src.copyTo(dst);
    if(dst.data == NULL)
    {
        printf("no image ");
        return 1;
    }
    if(dst.channels()==3)
    {
        printf("read color image,convert gray image ");
        cv::cvtColor(dst,dst,cv::COLOR_BGR2GRAY);
    }
    int markNum=0;
    std::vector<int> markerId; //store the transformation matrix information

    tracker->activateAutoThreshold(true);
    tracker->setNumAutoThresholdRetries(tryNum);

    markerId= tracker->calc(dst.data,&markInfo,&markNum);

    tracker->selectBestMarkerByCf();

    for(int i=0 ; i<16 ;i++)
    {

        outputMatrix[i]=tracker->getModelViewMatrix()[i];
        //cout<<outputMatrix[i];
        //printf("%.2f  %s", tracker->getModelViewMatrix()[i], (i % 4 == 3) ? "\n  " : "");
    }
    //printf("\n\n");

    return 0;

}

std::string  int2str(int num)
{

    std::stringstream ss;
    ss << num;
    std::string text = ss.str();
    return text;
}

void Positing::drawMarkerInfo(cv::Mat &image, ARToolKitPlus::ARMarkerInfo*  markInfo)
{

    cv::Point center,corner0,corner1,corner2,corner3 ;
    center=cv::Point(markInfo->pos[0],markInfo->pos[1]);
    corner0=cv::Point(markInfo->vertex[(4-markInfo->dir+0)%4][0],markInfo->vertex[(4-markInfo->dir+0)%4][1]);
    corner1=cv::Point(markInfo->vertex[(4-markInfo->dir+1)%4][0],markInfo->vertex[(4-markInfo->dir+1)%4][1]);
    corner2=cv::Point(markInfo->vertex[(4-markInfo->dir+2)%4][0],markInfo->vertex[(4-markInfo->dir+2)%4][1]);
    corner3=cv::Point(markInfo->vertex[(4-markInfo->dir+3)%4][0],markInfo->vertex[(4-markInfo->dir+3)%4][1]);

    cv::line(image,corner0,corner1,CV_RGB(255,0,0),1,8);
    cv::line(image,corner1,corner2,CV_RGB(255,0,0),1,8);
    cv::line(image,corner2,corner3,CV_RGB(255,0,0),1,8);
    cv::line(image,corner3,corner0,CV_RGB(255,0,0),1,8);
    cv::rectangle(image,cv::Point(center.x-1, center.y-1),cv::Point(center.x+1, center.y+1),CV_RGB(0,255,0),1,8); //圈取图像中心点

    //string dir_str = int2str(one_mark.dir);
    std::string tx0 = "0";
    std::string tx1 = "1";
    std::string tx2 = "2";
    std::string tx3 = "3";

    cv::putText(image,tx0,corner0,CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::putText(image,tx1,corner1,CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::putText(image,tx2,corner2,CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::putText(image,tx3,corner3,CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));


    std::string text ="Id:"+ int2str(markInfo->id);
    cv::putText(image,text,cv::Point(center.x+80,center.y),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));

}

int Positing::refractionRecover(ARToolKitPlus::ARMarkerInfo*  markInfo,double Dist)
{
    cv::Point2f center,corner0,corner1,corner2,corner3 ;
    center=cv::Point2f(markInfo->pos[0],markInfo->pos[1]);
    corner0=cv::Point2f(markInfo->vertex[(4-markInfo->dir+0)%4][0],markInfo->vertex[(4-markInfo->dir+0)%4][1]);
    corner1=cv::Point2f(markInfo->vertex[(4-markInfo->dir+1)%4][0],markInfo->vertex[(4-markInfo->dir+1)%4][1]);
    corner2=cv::Point2f(markInfo->vertex[(4-markInfo->dir+2)%4][0],markInfo->vertex[(4-markInfo->dir+2)%4][1]);
    corner3=cv::Point2f(markInfo->vertex[(4-markInfo->dir+3)%4][0],markInfo->vertex[(4-markInfo->dir+3)%4][1]);
    pointToAirScale(center,Dist);
    pointToAirScale(corner0,Dist);
    pointToAirScale(corner1,Dist);
    pointToAirScale(corner2,Dist);
    pointToAirScale(corner3,Dist);
    markInfo->pos[0]=center.x;
    markInfo->pos[1]=center.y;
    markInfo->vertex[(4-markInfo->dir+0)%4][0]=corner0.x;
    markInfo->vertex[(4-markInfo->dir+0)%4][1]=corner0.y;
    markInfo->vertex[(4-markInfo->dir+1)%4][0]=corner1.x;
    markInfo->vertex[(4-markInfo->dir+1)%4][1]=corner1.y;
    markInfo->vertex[(4-markInfo->dir+2)%4][0]=corner2.x;
    markInfo->vertex[(4-markInfo->dir+2)%4][1]=corner2.y;
    markInfo->vertex[(4-markInfo->dir+3)%4][0]=corner3.x;
    markInfo->vertex[(4-markInfo->dir+3)%4][1]=corner3.y;
    return 0;
}

int Positing::opencvPnpSolve(ARToolKitPlus::ARMarkerInfo*  markInfo,double *outputMatrix)
{
    double obj_point_arry[]={-0.0925,0.0925,0,0.0925,0.0925,0,0.0925,-0.0925,0,-0.0925,-0.0925,0};
    //double img_point_arry[]={0,0,185,0,185,185,185,0};
    double cameraMatrix_arrry[]={643.3848,0,310.1973,0,642.4595,265.6935,0,0,1};
    double distCoeffs_arry[]={0.1363,-0.2451,0.0012,0};
    Mat obj_point=Mat::eye(4,3,CV_32FC1);
    Mat img_point=Mat::zeros(4,2,CV_32FC1);
    Mat cameraMatrix=Mat::zeros(3,3,CV_32FC1);
    vector<double> distCoeffs;
    for(int i=0;i<4;i++)
    {
        distCoeffs.push_back(distCoeffs_arry[i]);
    }

    for(int i=0;i<4;i++)
    {
        for(int j=0;j<3;j++)
        {
            obj_point.at<float>(i,j)=obj_point_arry[i*3+j];
        }
    }
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<2;j++)
        {
            img_point.at<float>(i,j)=markInfo->vertex[i][j];
        }
    }
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cameraMatrix.at<float>(i,j)=cameraMatrix_arrry[i*3+j];
        }
    }
    Mat r_matrix;
    Mat t_matrix;
    double buff[2];
    solvePnP(obj_point,img_point,cameraMatrix,Mat(),r_matrix,t_matrix,false,CV_ITERATIVE);
    for(int i=0;i<3;i++)
    {
        buff[0]=r_matrix.at<double>(0,i);
        buff[1]=t_matrix.at<double>(0,i);
        r_matrix.at<double>(0,i)=buff[0]/3.1416*180;
        t_matrix.at<double>(0,i)=buff[1]*1000;

        outputMatrix[i]=r_matrix.at<double>(0,i);
        outputMatrix[i+3]=t_matrix.at<double>(0,i);
    }
    return 0;
}

int Positing::oldCalcuate(Mat src,Mat &dst,TrackerSingleMarker * tracker,double *outputMatrix)
{

    ARToolKitPlus::ARMarkerInfo* markInfo;

    cv::Mat image_gray(src.rows,src.cols,CV_8U);
    int rows, cols,cols_bits;
     rows = src.rows;
     cols = src.cols;
     cols_bits = cols*src.channels();
    for (int i = 1; i < rows-1; i++)
        {
            uchar *data = src.ptr<uchar>(i);
            uchar *data1 = image_gray.ptr<uchar>(i);
            for (int j = 1; j < cols-1; j++)
            {
                data1[j] = data[3 * j - 2];// -100;
                //data1[j] = data1[j]*1.7 ;
            }
        }
    //src.copyTo(image_gray);
//    if(image_gray.channels()==3)
//    {
//        cv::cvtColor(src,image_gray,cv::COLOR_BGR2GRAY);
//    }
    Positing::tryAutoThreshold(image_gray,tracker,markInfo,outputMatrix,150);

    Positing::drawMarkerInfo(dst,markInfo);

    m_ID=markInfo->id;

    return 0;
}

int Positing::getID()
{
    return this->m_ID;
}

int Positing::newCalcuate(Mat src, Mat & dst,TrackerSingleMarker &tracker, double *outputMatrix)
{

    const bool useBCH = false;
    ARToolKitPlus::ARMarkerInfo* markInfo;

   // TrackerSingleMarker tracker(src.cols, src.rows, 8, 6, 6, 6, 0);

    //设置图像格式为灰度图
    tracker.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

    // 载入自己相机的标定内参
    if (!tracker.init("Mydatas/lojiInAir.cal", 1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR: init() failed\n");
        return -1;
    }

    // 定义二维码大小，可定义成实际的二维码边框边长（以mm为单位）
    tracker.setPatternWidth(100);

    // the marker in the BCH test image has a thin border...
    tracker.setBorderWidth(useBCH ? 0.125 : 0.25);

    //如果采用自动阈值
    tracker.activateAutoThreshold(true);

    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker.setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);

    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    //这里图像像素为640 × 480,没考虑畸变
    tracker.setUndistortionMode(ARToolKitPlus::UNDIST_NONE);

    cv::Mat image_gray;

    src.copyTo(image_gray);

    if(image_gray.channels()==3)
    {
      //  printf("read color image,convert gray image ");
        cv::cvtColor(src,image_gray,cv::COLOR_BGR2GRAY);
    }
    //2.自动阈值尝试，尝试次数
    int markNum=0;
    std::vector<int> markerId; //store the transformation matrix information

    tracker.activateAutoThreshold(true);
    tracker.setNumAutoThresholdRetries(100);
    markerId= tracker.calc(image_gray.data,&markInfo,&markNum);

    Positing::refractionRecover(markInfo,1000);

    // Positing::opencvPnpSolve(markInfo,outputMatrix);

    tracker.selectBestMarkerByCf();//
    for(int i=0 ; i<16 ;i++)
    {
        outputMatrix[i]=tracker.getModelViewMatrix()[i];
        //printf("%.2f  %s", outputMatrix[i], (i % 4 == 3) ? "\n  " : "");
    }
    //printf("\n\n");

    Positing::drawMarkerInfo(dst,markInfo);

    return 0;
}
void rotToEular_rad(double *rot, double *eular)
{
    double R11=rot[0];
    double R21=rot[4];
    double R31=rot[8];
    double R32=rot[9];
    double R33=rot[10];
    double pitch=asin(R31);
    double yaw = atan2(R32,R33);
    double roll =atan2(R21,R11);
    eular[0]=pitch;
    eular[1]=yaw;
    eular[2]=roll;
}

void rotToEular_angle(double *rot, double *eular)
{
    double R11=rot[0];
    double R21=rot[4];
    double R31=rot[8];
    double R32=rot[9];
    double R33=rot[10];
    double pitch=asin(R31);
    double yaw = atan2(R32,R33);
    double roll =atan2(R21,R11);
    pitch=pitch/pi*180;
    yaw=yaw/pi*180;
    roll=roll/pi*180;
    eular[0]=pitch;
    eular[1]=yaw;
    eular[2]=roll;
}
//inite traker
void init_traker(TrackerSingleMarker *tracker,int maker_width )
{
    const bool useBCH = false;
    //设置图像格式为灰度图
    tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    // 载入自己相机的标定内参
    if (!tracker->init("/home/pi/dev/formationControl/parameter/lojiInWater.cal", 1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR: init() failed\n");
        return ;
    }
    // 定义二维码大小，可定义成实际的二维码边框边长（以mm为单位）
    tracker->setPatternWidth(maker_width);
    // the marker in the BCH test image has a thin border...
    tracker->setBorderWidth(useBCH ? 0.125 : 0.25);
    //如果采用自动阈值
    tracker->activateAutoThreshold(true);
    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker->setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    //这里图像像素为640 × 480,考虑畸变
    tracker->setUndistortionMode(ARToolKitPlus::UNDIST_STD);
}


void get_visual_pose(Mat frame_src,Mat* frame_dst,TrackerSingleMarker *tracker)
{
    double traslation_Matrix[16]={0};
    double eular[3]={0};
//    Mat frame_in;
//    Mat frame_out;
    Positing position;

//    frame_src.copyTo(frame_in);
//    frame_in.copyTo(frame_out);

   //position.newCalcuate(frame_src,*frame_dst,*tracker,traslation_Matrix);//main
    position.oldCalcuate(frame_src,*frame_dst,tracker,traslation_Matrix);//main
    rotToEular_angle(traslation_Matrix,eular);
    for(int j=0;j<16;j++)
    {
        Visual_Traslation_Matrix[j]= traslation_Matrix[j];       
    } 
    Visual_Position[0]= traslation_Matrix[14];
    Visual_Position[1]= traslation_Matrix[12];
    Visual_Position[2]= traslation_Matrix[13];
    Visual_Eular[0]=eular[2];
    Visual_Eular[1]=eular[1];
    Visual_Eular[2]=eular[0];

   // frame_out.copyTo(*frame_dst);
}
