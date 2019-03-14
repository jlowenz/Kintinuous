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
  
  RedwoodReader(const string& dataset_path, const std::string& obj, const std::string& seq)
    : current_frame_(0), num_frames_(0), obj_(obj), seq_(seq)
  {
    root_path_ = dataset_path;
    depth_scale_ = 1000; // pixel values are in mm
    get_rgb_names(seq_, obj_, rgb_);
    get_depth_names(seq_, obj_, depth_);
    isCompressed = false;
  }
  virtual ~RedwoodDataset() {}

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
