#pragma once

#include "LogReader.h"
#include <boost/filesystem.hpp>

using namespace boost::filesystem;

typedef std::vector<std::string> paths_t;

struct RedwoodReader : public LogReader
{
  std::string root_path_;
  double depth_scale_;
  int current_frame_;
  int num_frames_;
  std::string obj_, seq_;
  paths_t rgb_, depth_;
  int width_, height_;
  
  RedwoodReader(const std::string& dataset_path, const std::string& obj,
                const std::string& seq, int width = 640, int height = 480);
  virtual ~RedwoodReader();

  bool grabNext(bool& returnVal, int& currentFrame);
  void readNext();
  bool hasMore();
  
  void get_rgb_dir(const std::string& seq_id,
                   const std::string& seq_class,
                   std::string& rgb_dir);
  void get_depth_dir(const std::string& seq_id,
                     const std::string& seq_class,
                     std::string& depth_dir);
  void get_rgb_names(const std::string& seq_id,
                     const std::string& seq_class,
                     std::vector<std::string>& rgb_names);

  void get_depth_names(const std::string& seq_id,
                       const std::string& seq_class,
                       std::vector<std::string>& depth_names);
};
