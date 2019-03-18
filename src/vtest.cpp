
#include "vtest.hpp"

static Test* instance = NULL;

Test::~Test() {
  if (val1) delete val1;
}

Test& Test::get() {
  if (instance == NULL) instance = new Test;
  return *instance;
}

void Test::setVal(int* i) { val = i; }
int* Test::getVal() { return val; }
