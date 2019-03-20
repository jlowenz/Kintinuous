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

#ifndef ISAMINTERFACE_H_
#define ISAMINTERFACE_H_

#include <string>
#include <map>
#include <Eigen/Dense>
#include <isam.h>
#include <stdint.h>
#include "../utils/ConfigArgs.h"
#include "../utils/types.hpp"


typedef std::pair<uint64_t,Vector3d_t> ts_vector3d_t;
typedef std::pair<uint64_t,Matrix4_t> ts_matrix4_t;

typedef std::vector<ts_vector3d_t,
                    Eigen::aligned_allocator<ts_vector3d_t>> cam_position_t;
typedef std::vector<ts_matrix4_t,
                    Eigen::aligned_allocator<ts_matrix4_t>> cam_pose_t;

class iSAMInterface
{
 public:
  iSAMInterface();
  virtual ~iSAMInterface();

  void addCameraCameraConstraint(uint64_t time1, uint64_t time2,
                                 const Matrix3_t & Rprev, const Vector3_t & tprev,
                                 const Matrix3_t & Rcurr, const Vector3_t & tcurr);

  isam::Pose3d_Pose3d_Factor * addLoopConstraint(uint64_t time1,
                                                 uint64_t time2,
                                                 Matrix4d_t & loopConstraint);

  const std::list<isam::Factor* > & getFactors();

  void getCameraPositions(cam_position_t& positions);

  void getCameraPoses(cam_pose_t & poses);

  double optimise();

  Matrix4_t getCameraPose(uint64_t id);

  void removeFactor(isam::Pose3d_Pose3d_Factor * factor);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
 private:
  isam::Pose3d_Node * cameraNode(uint64_t time);

  isam::Slam * _slam;
  std::map<uint64_t, isam::Pose3d_Node*> _camera_nodes;

  Matrix4_t transformation2isam;
  std::map<std::pair<uint64_t, uint64_t>, bool> cameraCameraConstraints;
  std::map<std::pair<uint64_t, uint64_t>, bool> cameraPlaneConstraints;
};

#endif /* ISAMINTERFACE_H_ */
