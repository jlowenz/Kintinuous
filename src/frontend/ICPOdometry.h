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

#ifndef ICPODOMETRY_H_
#define ICPODOMETRY_H_

#include "../utils/types.hpp"
#include "OdometryProvider.h"

class ICPOdometry : public OdometryProvider
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ICPOdometry(vectors3_t & tvecs_,
                    matrices3_t & rmats_,
                    std::vector<DeviceArray2D<float> > & vmaps_g_prev_,
                    std::vector<DeviceArray2D<float> > & nmaps_g_prev_,
                    std::vector<DeviceArray2D<float> > & vmaps_curr_,
                    std::vector<DeviceArray2D<float> > & nmaps_curr_,
                    Intr & intr,
                    float distThresh = 0.10f,
                    float angleThresh = sin(20.f * 3.14159254f / 180.f));

        virtual ~ICPOdometry();

        CloudSlice::Odometry getIncrementalTransformation(Vector3_t & trans,
                                                          Matrix3_t & rot,
                                                          const DeviceArray2D<unsigned short> & depth,
                                                          const DeviceArray2D<PixelRGB> & image,
                                                          uint64_t timestamp,
                                                          unsigned char * rgbImage,
                                                          unsigned short * depthData);

        MatrixXd_t getCovariance();

        void reset();

        static const int LEVELS = 4;

    private:
        int icp_iterations_[LEVELS];

        vectors3_t & tvecs_;
        matrices3_t & rmats_;

        std::vector<DeviceArray2D<float> > & vmaps_g_prev_;
        std::vector<DeviceArray2D<float> > & nmaps_g_prev_;

        std::vector<DeviceArray2D<float> > & vmaps_curr_;
        std::vector<DeviceArray2D<float> > & nmaps_curr_;

        Intr & intr;

        Matrix6d_t lastA;

        DeviceArray<JtJJtrSE3> sumDataSE3;
        DeviceArray<JtJJtrSE3> outDataSE3;

        float distThres_;
        float angleThres_;
};

#endif /* ICPODOMETRY_H_ */
