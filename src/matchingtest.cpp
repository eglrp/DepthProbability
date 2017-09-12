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




void saveXYZ(const char* filename, const cv::Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            cv::Vec3f point = mat.at<cv::Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
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

int main()
{
    LOG(INFO)<<"matching"<<std::endl;

    cv::Mat image_ref = cv::imread("rdimage.002.ppm");
    cv::Mat image_source = cv::imread("rdimage.001.ppm");

    //cv::imshow("ref",image_ref);
    //cv::imshow("source",image_source);
    //cv::waitKey(0);
    //cv::StereoSGBM

    Eigen::Matrix3d K0,C0,K1,C1,C;
    Eigen::Vector3d r0,r1,r;
    readParams("rdimage.002.ppm.camera",K0,C0,r0);
    readParams("rdimage.001.ppm.camera",K1,C1,r1);
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
    cv::eigen2cv(K0,M0);
    cv::eigen2cv(K1,M1);
    cv::eigen2cv(C,R01);
    cv::eigen2cv(r,t);

    cv::Size size(image_ref.cols,image_ref.rows);
    cv::stereoRectify(M0,D0,M1,D1,
                      size,R01,t,
                      R0__,R1__,
                      P0__,P1__,
                      Q,cv::CALIB_FIX_INTRINSIC,-1,
                      size, &roi1, &roi2
    );
    //std::cout<<R01<<std::endl;

    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(M0, D0, R0__, P0__, size, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(M1, D1, R1__, P1__, size, CV_16SC2, map21, map22);
    //std::cout<<P0__<<"\n"<<R1__<<std::endl;
    cv::Mat img1r, img2r;
    cv::remap(image_ref, img1r, map11, map12, cv::INTER_LINEAR);
    cv::remap(image_source, img2r, map21, map22, cv::INTER_LINEAR);

    //image_ref = img1r;
    //image_source = img2r;

    cv::resize(img1r,image_ref,cv::Size(size.width/2,size.height/2));
    cv::resize(img2r,image_source,cv::Size(size.width/2,size.height/2));


    //cv::waitKey(0);

    cv::StereoSGBM sgbm(-0,256,
                        8,8*3*3*3,32*3*3*3,2,
                        16,5,100,
                        2,false
    );
    //cv::StereoSGBM sgbm(0,16,5);
    //StereoSGBM(int minDisparity, int numDisparities,
    // int SADWindowSize, int P1=0, int P2=0, int disp12MaxDiff=0,
    // int preFilterCap=0, int uniquenessRatio=0, int speckleWindowSize=0,
    // int speckleRange=0, bool fullDP=false)
//    sgbm.P1=(8*cn*sgbmWinSize*sgbmWinSize);
//    sgbm.P2=(32*cn*sgbmWinSize*sgbmWinSize);
//    sgbm.minDisparity = (0);
//    sgbm.numberOfDisparities = (numberOfDisparities);
//    sgbm->setUniquenessRatio(10);
//    sgbm->setSpeckleWindowSize(100);
//    sgbm->setSpeckleRange(32);
//    sgbm->setDisp12MaxDiff(1);

//    if(alg==STEREO_HH)
//        sgbm->setMode(StereoSGBM::MODE_HH);
//    else if(alg==STEREO_SGBM)
//        sgbm->setMode(StereoSGBM::MODE_SGBM);
//    else if(alg==STEREO_3WAY)
//        sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
    cv::Mat disp(image_ref.rows,image_ref.cols,CV_64FC1),disp8,dispnorm;
    sgbm(image_ref,image_source,disp);
    double min = 9999;
    for (int j = 0; j < disp.cols; ++j) {
        for (int i = 0; i < disp.rows; ++i) {
            if((disp.at<double>(i,j) <cv:: ) && disp.at<double>(i,j)<min)
            {
                min = disp.at<double>(i,j);
            }
            if(disp.at<double>(i,j)<0){
                disp.at<double>(i,j)*=-1;
            }
        }
    }
    std::cout<<min<<std::endl;
    disp.convertTo(disp8, CV_8U);
    //cv::normalize(disp8,dispnorm,0,255);
    //disp8*=255;
    cv::namedWindow("re",cv::WINDOW_NORMAL);
    cv::imshow("re",disp8);

    cv::namedWindow("ref",cv::WINDOW_NORMAL);
    cv::namedWindow("source",cv::WINDOW_NORMAL);

    for (int i = 0; i < image_ref.rows; i += image_ref.rows/20 ) {
        cv::line(image_ref,cv::Point(0,i),cv::Point(image_ref.cols,i),cv::Scalar(0,255,0),5);
        cv::line(image_source,cv::Point(0,i),cv::Point(image_ref.cols,i),cv::Scalar(0,255,0),5);

    }
    for (int i = 0; i < image_ref.cols; i += image_ref.cols/20 ) {
        cv::line(image_ref,cv::Point(i,0),cv::Point(i,image_ref.cols),cv::Scalar(0,255,0),5);
        cv::line(image_source,cv::Point(i,0),cv::Point(i,image_ref.cols),cv::Scalar(0,255,0),5);

    }

    cv::imshow("ref",image_ref);
    cv::imshow("source",image_source);

    cv::Mat xyz,xyz8;
    reprojectImageTo3D(disp, xyz, Q, true);
    xyz.convertTo(xyz8, CV_8U,1/2.0,0);
    cv::namedWindow("range",cv::WINDOW_NORMAL);

    cv::imshow("range",xyz8);
    saveXYZ("./pts.xyz", xyz);
    cv::waitKey(0);
    return 0;
}