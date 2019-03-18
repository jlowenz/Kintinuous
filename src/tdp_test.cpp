#include "utils/ThreadDataPack.h"

int
main(int argc, char** argv)
{
  ConfigArgs::get(argc, argv);
  ThreadDataPack::get();

  ThreadDataPack& tdp = ThreadDataPack::get();
  ThreadDataPack* tdpp = ThreadDataPack::getp();

  std::cout << "tdpp: " << tdpp << std::endl;
  
  std::cout << "tracker  type: " << typeid(tdp.tracker).name() << std::endl;
  std::cout << "frontend type: " << typeid(tdp.getFrontend()).name() << std::endl;

  std::cout << "&tdp / tdpp   : " << &tdp << " / " << tdpp << std::endl;
  std::cout << "tdp.tracker   : " << tdp.tracker << std::endl;
  std::cout << "tdpp->tracker : " << tdpp->tracker << std::endl;
  std::cout << "frontend      : " << tdp.getFrontend() << std::endl;
  tdp.assignFrontend(reinterpret_cast<KintinuousTracker*>(1));
  std::cout << "&tdp / tdpp   : " << &tdp << " / " << tdpp << std::endl;
  std::cout << "tdp.tracker   : " << tdp.tracker << std::endl;
  std::cout << "tdpp->tracker : " << tdpp->tracker << std::endl;
  std::cout << "frontend      : " << tdp.getFrontend() << std::endl;


  tdp.tracker = reinterpret_cast<KintinuousTracker*>(2);
  std::cout << "&tdp / tdpp   : " << &tdp << " / " << tdpp << std::endl;
  std::cout << "tdp.tracker   : " << tdp.tracker << std::endl;
  std::cout << "tdpp->tracker : " << tdpp->tracker << std::endl;
  std::cout << "frontend      : " << tdp.getFrontend() << std::endl;


  tdpp->tracker = reinterpret_cast<KintinuousTracker*>(3);
  std::cout << "&tdp / tdpp   : " << &tdp << " / " << tdpp << std::endl;
  std::cout << "tdp.tracker   : " << tdp.tracker << std::endl;
  std::cout << "tdpp->tracker : " << tdpp->tracker << std::endl;
  std::cout << "frontend      : " << tdp.getFrontend() << std::endl;

  tdpp->assignFrontend(reinterpret_cast<KintinuousTracker*>(4));
  std::cout << "&tdp / tdpp   : " << &tdp << " / " << tdpp << std::endl;
  std::cout << "&tdp.tracker  : " << &(tdp.tracker) << std::endl;
  std::cout << "&tdpp->tracker  : " << &(tdpp->tracker) << std::endl;
  std::cout << "tdp.tracker   : " << tdp.tracker << std::endl;
  std::cout << "tdpp->tracker : " << tdpp->tracker << std::endl;
  std::cout << "frontend      : " << tdp.getFrontend() << std::endl;

  
  return 0;
}
