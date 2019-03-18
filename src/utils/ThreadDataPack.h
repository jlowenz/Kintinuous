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
#pragma once
#ifndef THREADDATAPACK_H_
#define THREADDATAPACK_H_

#include "ConfigArgs.h"
#include "../frontend/KintinuousTracker.h"
#include "../backend/LoopClosureConstraint.h"
#include "../backend/IncrementalMesh.h"
#include "ThreadMutexObject.h"

#include <vector>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/keyboard_event.h>




class ThreadDataPack
{
public:
  static ThreadDataPack & get();
  static ThreadDataPack* getp();
  ~ThreadDataPack();
  void assignFrontend(KintinuousTracker * frontend);
  KintinuousTracker* getFrontend();
  void reset();        
  void notifyVariables();

  IncrementalMesh * incrementalMesh;
  std::vector<pcl::PolygonMesh *> triangles;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointPool;
  boost::mutex poolMutex;
  ThreadMutexObject<bool> poolLooped;
  ThreadMutexObject<Eigen::Matrix4f> loopOffset;
  ThreadMutexObject<Eigen::Matrix4f> isamOffset;

  boost::mutex incMeshMutex;
  ThreadMutexObject<bool> incMeshLooped;

  ThreadMutexObject<bool> finalised;

  ThreadMutexObject<bool> limit;

  KintinuousTracker * tracker;
  std::vector<CloudSlice *> cloudSlices;

  ThreadMutexObject<uint64_t> lastLoopTime;
  ThreadMutexObject<bool> readyForLoop;
  std::vector<LoopClosureConstraint *> loopClosureConstraints;

  ThreadMutexObject<int> latestLoopId;
  ThreadMutexObject<int> latestMeshId;
  ThreadMutexObject<int> latestPoseId;
  ThreadMutexObject<bool> trackerFinished;
  ThreadMutexObject<bool> cloudSliceProcessorFinished;
  ThreadMutexObject<bool> meshGeneratorFinished;
  ThreadMutexObject<bool> placeRecognitionFinished;
  ThreadMutexObject<bool> deformationFinished;
  ThreadMutexObject<int> trackerFrame;
  ThreadMutexObject<bool> pauseCapture;

private:
  ThreadDataPack()
    : incrementalMesh(0),
      pointPool(new pcl::PointCloud<pcl::PointXYZRGBNormal>),
      tracker(0)
  {
    reset();
  }

  ThreadDataPack(const ThreadDataPack&) = delete;
  ThreadDataPack& operator=(const ThreadDataPack&) = delete;
};

#endif /* THREADDATAPACK_H_ */
