#pragma once

#include <memory>
#include <Eigen/Core>
#include "utils/ThreadMutexObject.h"

class Test
{
public:
  static Test& get();
  virtual ~Test();
  
  void setVal(int*);
  int* getVal();
  
  int* val1;
  int* val2;
  int* val3;
  int* val4;
  int* val5;
  int* val6;

  ThreadMutexObject<Eigen::Matrix4f> m1;
  ThreadMutexObject<Eigen::Matrix4f> m2;
  
  std::shared_ptr<Test> test;
  int* val;

private:
  Test() : val1(0), val2(0), val(0) {}
  Test(const Test&) = delete;
  Test& operator=(const Test&) = delete;
};
