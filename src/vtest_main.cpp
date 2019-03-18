#include <iostream>
#include "vtest.hpp"

int
main(int argc, char** argv)
{
  Test::get();
  Test& t = Test::get();

  t.setVal(reinterpret_cast<int*>(1));
  std::cout << "t.val     : " << t.val << std::endl;
  std::cout << "t.getVal(): " << t.getVal() << std::endl;

  t.val = reinterpret_cast<int*>(2);
  std::cout << "t.val     : " << t.val << std::endl;
  std::cout << "t.getVal(): " << t.getVal() << std::endl;
  
  return 0;
}
