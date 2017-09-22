//
// Created by jianping on 17-9-11.
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

//image intrinsics
//reference image r,t
//source image r,t
//https://github.com/opencv/opencv/blob/master/samples/cpp/stereo_match.cpp




void saveXYZ(const char* filename, const cv::Mat& mat,float tx)
{
    const double max_z = 10000;
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
                Eigen::Matrix3d& K,
                Eigen::Matrix3d& C,
                Eigen::Vector3d& t)
{
    std::ifstream ifs(intrinsicFile.c_str(),std::ios_base::in);
    char line[256];
    ifs.getline(line,256);
    sscanf(line,"%lf %lf %lf",&K(0,0),&K(0,1),&K(0,2));
    ifs.getline(line,256);
    sscanf(line,"%lf %lf %lf",&K(1,0),&K(1,1),&K(1,2));
    ifs.getline(line,256);
    sscanf(line,"%lf %lf %lf",&K(2,0),&K(2,1),&K(2,2));
    ifs.getline(line,256);//useless
    ifs.getline(line,256);
    sscanf(line,"%lf %lf %lf",&C(0,0),&C(0,1),&C(0,2));
    ifs.getline(line,256);
    sscanf(line,"%lf %lf %lf",&C(1,0),&C(1,1),&C(1,2));
    ifs.getline(line,256);
    sscanf(line,"%lf %lf %lf",&C(2,0),&C(2,1),&C(2,2));
    ifs.getline(line,256);
    sscanf(line,"%lf %lf %lf",&t(0),&t(1),&t(2));
    std::cout<<K<<"\n"<<C<<"\n"<<t.transpose()<<"\n";

}


const int scale = 1;
int main()
{
    LOG(INFO)<<"matching"<<std::endl;
    cv::Mat image_ref_origin = cv::imread("rdimage.001.ppm");
    cv::Mat image_source_origin = cv::imread("rdimage.000.ppm");
    cv::Mat image_ref,image_source;

    cv::resize(image_ref_origin,image_ref,
               cv::Size(image_ref_origin.cols/scale,image_ref_origin.rows/scale),
               0,0,cv::INTER_CUBIC);
    cv::resize(image_source_origin,image_source,
               cv::Size(image_ref_origin.cols/scale,image_ref_origin.rows/scale),
               0,0,cv::INTER_CUBIC);

    Eigen::Matrix3d K0,C0,K1,C1,C;
    Eigen::Vector3d r0,r1,r;
    readParams("rdimage.001.ppm.camera",K0,C0,r0);
    readParams("rdimage.000.ppm.camera",K1,C1,r1);
    {
        K0(0,0)/=scale;
        K0(1,1)/=scale;
        K0(0,2)/=scale;
        K0(1,2)/=scale;
        K1(0,0)/=scale;
        K1(1,1)/=scale;
        K1(0,2)/=scale;
        K1(1,2)/=scale;
    }
    //C = C0.transpose()*C1;
    //r = C0.transpose()*(r1-r0);
    C = C1.transpose()*C0;// transform p from frame 0 to frame 1
    r = C1.transpose()*(r0-r1);
    Eigen::Quaterniond q(C);
    C = q.toRotationMatrix();
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

    cv::Size size(image_ref.cols, image_ref.rows);
    cv::stereoRectify(M0,D0,M1,D1,
                      size,R01,t,
                      R0__,R1__,
                      P0__,P1__,
                      Q,cv::CALIB_FIX_INTRINSIC,-1,
                      size, &roi1, &roi2
    );

    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(M0, D0, R0__, P0__, size, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(M1, D1, R1__, P1__, size, CV_16SC2, map21, map22);
    //std::cout<<P0__<<"\n"<<R1__<<std::endl;
    cv::Mat img1r, img2r;
    cv::remap(image_ref, img1r, map11, map12, cv::INTER_LINEAR);
    cv::remap(image_source, img2r, map21, map22, cv::INTER_LINEAR);

    image_ref = img1r;
    image_source = img2r;

//    cv::resize(img1r,image_ref,cv::Size(size.width/3,size.height/3));
//    cv::resize(img2r,image_source,cv::Size(size.width/3,size.height/3));


    //cv::waitKey(0);

    cv::StereoSGBM sgbm(-512,1024,
                        8,8*3*3*3,32*3*3*3,2,
                        16,5,100,
                        2,false
    );

    cv::Mat disp,disp8,dispnorm(image_ref.rows,image_ref.cols,CV_64FC1);
    sgbm(image_ref,image_source,disp);

//    cv::normalize(disp,dispnorm,255,0,cv::NORM_MINMAX);
//    dispnorm.convertTo(disp8, CV_8U);
//    cv::namedWindow("re",cv::WINDOW_NORMAL);
//    cv::imshow("re",disp8);


    cv::namedWindow("ref",cv::WINDOW_NORMAL);
    cv::namedWindow("source",cv::WINDOW_NORMAL);

    for (int i = 0; i < image_ref.rows; i += image_ref.rows/20 ) {
        cv::line(image_ref,cv::Point(0,i),cv::Point(image_ref.cols,i),cv::Scalar(0,255,0),1);
        cv::line(image_source,cv::Point(0,i),cv::Point(image_ref.cols,i),cv::Scalar(0,255,0),1);

    }

    cv::imshow("ref",image_ref);
    cv::imshow("source",image_source);

    cv::Mat xyz,xyz8;
    reprojectImageTo3D(disp, xyz, Q, false);

    saveXYZ("./pts.xyz", xyz, r(0));
    cv::waitKey(0);
    return 0;
}