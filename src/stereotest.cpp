//
// Created by jianping on 17-9-18.
//

#include <iostream>
#include <glog/logging.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <Eigen/Geometry>





void saveXYZ(const char* filename, const cv::Mat& mat,float tx)
{
    const double max_z = 2000;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            cv::Vec3f point = mat.at<cv::Vec3f>(y, x);
            if(fabs(fabs(point[2]) - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

void readParams(const std::string& intrinsicFile,
                Eigen::Matrix3d& K0,
                Eigen::Matrix3d& K1)
{
    std::ifstream ifs(intrinsicFile.c_str(),std::ios_base::in);
    char line[256];
    ifs.getline(line,256);
    sscanf(line,"cam0=[%lf %lf %lf; %lf %lf %lf; %lf %lf %lf]",
           &K0(0,0),&K0(0,1),&K0(0,2),&K0(1,0),&K0(1,1),&K0(1,2),&K0(2,0),&K0(2,1),&K0(2,2));
    std::cout<<"K0\n"<<K0<<"\n";

    ifs.getline(line,256);
    sscanf(line,"cam1=[%lf %lf %lf; %lf %lf %lf; %lf %lf %lf]",
           &K1(0,0),&K1(0,1),&K1(0,2),&K1(1,0),&K1(1,1),&K1(1,2),&K1(2,0),&K1(2,1),&K1(2,2));
    std::cout<<"K1\n"<<K1<<"\n";
}


const int scale = 1;
int main() {
    Eigen::Matrix3d K0;
    Eigen::Matrix3d K1;
    readParams("/home/jianping/workspace/datas/stereo/calib.txt",K0,K1);

    LOG(INFO)<<"matching"<<std::endl;



    cv::Mat image_ref_origin = cv::imread("/home/jianping/workspace/datas/stereo/im0.png");
    cv::Mat image_source_origin = cv::imread("/home/jianping/workspace/datas/stereo/im1.png");
    cv::StereoSGBM sgbm(0,640,
                        8,8*3*3*3,32*3*3*3,2,
                        16,5,100,
                        2,false
    );

    Eigen::Matrix3d C;
    C = Eigen::Matrix3d::Identity();
    Eigen::Vector3d r0,r1,r(100,0,0);

    cv::Mat Q;
    cv::Rect roi1, roi2;
    cv::Mat M0,M1,R01(3,3,CV_64FC1),t(3,1,CV_64FC1),R0__,R1__,P0__,P1__;
    cv::Mat D0(5,1,CV_64FC1),D1(5,1,CV_64FC1);
    D0.setTo(0);
    D1.setTo(0);
    cv::eigen2cv(K0,M0);
    cv::eigen2cv(K1,M1);
    cv::eigen2cv(C,R01);
    cv::eigen2cv(r,t);

    cv::Size size(image_ref_origin.cols, image_ref_origin.rows);
    cv::stereoRectify(M0,D0,M1,D1,
                      size,R01,t,
                      R0__,R1__,
                      P0__,P1__,
                      Q,cv::CALIB_FIX_INTRINSIC,-1,
                      size, &roi1, &roi2
    );

    cv::Mat disp,disp8,dispnorm(image_ref_origin.rows,image_ref_origin.cols,CV_64FC1);
    sgbm(image_ref_origin,image_source_origin,disp);

    cv::normalize(disp,dispnorm,255,0,cv::NORM_MINMAX);
    dispnorm.convertTo(disp8, CV_8U);
    cv::namedWindow("re",cv::WINDOW_NORMAL);
    cv::imshow("re",disp8);


    cv::namedWindow("ref",cv::WINDOW_NORMAL);
    cv::namedWindow("source",cv::WINDOW_NORMAL);

    for (int i = 0; i < image_ref_origin.rows; i += image_ref_origin.rows/20 ) {
        cv::line(image_ref_origin,cv::Point(0,i),cv::Point(image_ref_origin.cols,i),cv::Scalar(0,255,0),3);
        cv::line(image_source_origin,cv::Point(0,i),cv::Point(image_ref_origin.cols,i),cv::Scalar(0,255,0),3);

    }

    cv::imshow("ref",image_source_origin);
    cv::imshow("source",image_ref_origin);

    cv::Mat xyz,xyz8;
    reprojectImageTo3D(disp, xyz, Q, false);

    saveXYZ("./pts.xyz", xyz, r(0));
    cv::waitKey(0);
    return 0;
}