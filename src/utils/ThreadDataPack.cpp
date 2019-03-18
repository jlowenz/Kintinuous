#include "ThreadDataPack.h"

static ThreadDataPack* instance = NULL;
static boost::mutex inst_mutex;

ThreadDataPack &
ThreadDataPack::get()
{
  boost::mutex::scoped_lock lock(inst_mutex);
  if (instance == NULL) instance = new ThreadDataPack;
  return *instance;
}

ThreadDataPack*
ThreadDataPack::getp()
{
  boost::mutex::scoped_lock lock(inst_mutex);
  if (instance == NULL) instance = new ThreadDataPack;
  return instance;
}


ThreadDataPack::~ThreadDataPack()
{
  if(incrementalMesh)
  {
    delete incrementalMesh;
  }
}

void 
ThreadDataPack::assignFrontend(KintinuousTracker * frontend)
{
  std::cout << "tdp::assignFrontend: " << (unsigned long)frontend << std::endl;
  tracker = frontend;
  ThreadDataPack::get().tracker = frontend;
  std::cout << "tdp::assignFrontend: " << (unsigned long)tracker << std::endl;
}

KintinuousTracker*
ThreadDataPack::getFrontend()
{
  std::cout << "this: " << this << std::endl;
  std::cout << "this->tracker: " << this->tracker << std::endl;
  std::cout << "&this->tracker:" << &(this->tracker) << std::endl;
  //assert(tracker == ThreadDataPack::get().tracker);
  return tracker;
}


void 
ThreadDataPack::reset()
{
  //We only delete this because the first item is the initial pose slice
  //created by the CloudSliceProcessor, the rest of the pointers are owned
  //by the KintinuousTracker and dealt with there
  if(cloudSlices.size())
  {
    delete cloudSlices.at(0);
  }

  cloudSlices.clear();

  for(unsigned int i = 0; i < triangles.size(); i++)
  {
    delete triangles.at(i);
  }

  triangles.clear();

  for(unsigned int i = 0; i < loopClosureConstraints.size(); i++)
  {
    delete loopClosureConstraints.at(i);
  }
  loopClosureConstraints.clear();

  if(incrementalMesh)
  {
    delete incrementalMesh;
  }

  if(ConfigArgs::get().incrementalMesh)
  {
    incrementalMesh = new IncrementalMesh;
  }

  pauseCapture.assignValue(false);
  latestLoopId.assignValue(0);
  latestPoseId.assignValue(0);
  latestMeshId.assignValue(0);
  trackerFinished.assignValue(false);
  cloudSliceProcessorFinished.assignValue(false);
  meshGeneratorFinished.assignValue(false);
  placeRecognitionFinished.assignValue(false);
  deformationFinished.assignValue(false);
  trackerFrame.assignValue(0);
  finalised.assignValue(false);
  poolLooped.assignValue(false);
  limit.assignValue(true);
  incMeshLooped.assignValue(false);
  loopOffset.assignValue(Eigen::Matrix4f::Identity());
  isamOffset.assignValue(Eigen::Matrix4f::Identity());
  lastLoopTime.assignValue(0);
  readyForLoop.assignValue(true);

  boost::mutex::scoped_lock lock(poolMutex);
  pointPool->clear();
}
        
void 
ThreadDataPack::notifyVariables()
{
  latestLoopId.notifyAll();
  latestMeshId.notifyAll();
  latestPoseId.notifyAll();
}
