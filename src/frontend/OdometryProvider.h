/*
 * This file is part of Kintinuous.
 *
 * Copyright (C) 2015 The National University of Ireland Maynooth and 
 * Massachusetts Institute of Technology
 *
 * The use of the code within this file and all code within files that 
 * make up the software that is Kintinuous is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.cs.nuim.ie/research/vision/data/kintinuous/code.php> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email commercialisation@nuim.ie.
 */

#ifndef ODOMETRYPROVIDER_H_
#define ODOMETRYPROVIDER_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cuda/internal.h"
#include "../utils/ConfigArgs.h"
#include "../utils/types.hpp"
#include "CloudSlice.h"

class OdometryProvider
{
    public:
        OdometryProvider()
        {}

        virtual ~OdometryProvider()
        {}

        virtual CloudSlice::Odometry getIncrementalTransformation(Vector3_t & trans,
                                                                  Matrix3_t & rot,
                                                                  const DeviceArray2D<unsigned short> & depth,
                                                                  const DeviceArray2D<PixelRGB> & image,
                                                                  uint64_t timestamp,
                                                                  unsigned char * rgbImage,
                                                                  unsigned short * depthData) = 0;

        virtual MatrixXd_t getCovariance() = 0;

        virtual void reset() = 0;

        inline static void computeProjectiveMatrix( const cv::Mat& ksi, cv::Mat& Rt )
        {
            CV_Assert( ksi.size() == cv::Size(1,6) && ksi.type() == CV_64FC1 );

            Rt = cv::Mat::eye(4, 4, CV_64FC1);

            cv::Mat R = Rt(cv::Rect(0,0,3,3));
            cv::Mat rvec = ksi.rowRange(3,6);

            cv::Rodrigues( rvec, R );

            Rt.at<double>(0,3) = ksi.at<double>(0);
            Rt.at<double>(1,3) = ksi.at<double>(1);
            Rt.at<double>(2,3) = ksi.at<double>(2);
        }
};

#endif /* ODOMETRYPROVIDER_H_ */
