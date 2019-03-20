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
#include "types.hpp"

#include <vector>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/keyboard_event.h>
#include <Eigen/Core>

class ThreadMutexObjectM
{
public:
  ThreadMutexObjectM()
    : object(new Matrix4_t),
      lastCopy(new Matrix4_t)
  {
    std::cout << "constructing specialized matrix mutex object" << std::endl;
  }

  ThreadMutexObjectM(const Matrix4_t& initialValue)
    : object(new Matrix4_t(initialValue)),
      lastCopy(new Matrix4_t(initialValue))
  {
    std::cout << "constructing specialized matrix mutex object" << std::endl;
  }

  ~ThreadMutexObjectM()
  {
    delete object;
    delete lastCopy;
  }
  
  void assignValue(const Matrix4_t& newValue)
  {
    boost::mutex::scoped_lock lock(mutex);

    *object = *lastCopy = newValue;
  }

  boost::mutex & getMutex()
  {
    return mutex;
  }

  Matrix4_t & getReference()
  {
    return *object;
  }

  void assignAndNotifyAll(const Matrix4_t& newValue)
  {
    boost::mutex::scoped_lock lock(mutex);

    *object = newValue;

    signal.notify_all();
  }
        
  void notifyAll()
  {
    boost::mutex::scoped_lock lock(mutex);

    signal.notify_all();
  }

  Matrix4_t getValue()
  {
    boost::mutex::scoped_lock lock(mutex);

    *lastCopy = *object;

    return *lastCopy;
  }

  Matrix4_t waitForSignal()
  {
    boost::mutex::scoped_lock lock(mutex);

    signal.wait(mutex);

    *lastCopy = *object;

    return *lastCopy;
  }

  Matrix4_t getValueWait(int wait = 33000)
  {
    boost::this_thread::sleep(boost::posix_time::microseconds(wait));

    boost::mutex::scoped_lock lock(mutex);

    *lastCopy = *object;

    return *lastCopy;
  }

  Matrix4_t & getReferenceWait(int wait = 33000)
  {
    boost::this_thread::sleep(boost::posix_time::microseconds(wait));

    boost::mutex::scoped_lock lock(mutex);

    *lastCopy = *object;

    return *lastCopy;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
private:
  Matrix4_t* object;
  Matrix4_t* lastCopy;
  boost::mutex mutex;
  boost::condition_variable_any signal;
};



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

  ThreadMutexObjectM loopOffset;
  ThreadMutexObjectM isamOffset;

  IncrementalMesh * incrementalMesh;
  std::vector<pcl::PolygonMesh *> triangles;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointPool;
  boost::mutex poolMutex;
  ThreadMutexObject<bool> poolLooped;

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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
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
