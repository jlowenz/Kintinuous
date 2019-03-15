#include "RedwoodReader.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


RedwoodReader::RedwoodReader(const std::string& dataset_path, const std::string& obj,
                             const std::string& seq, int width, int height)
  : current_frame_(0), num_frames_(0), obj_(obj), seq_(seq), width_(width), height_(height)
{
  root_path_ = dataset_path;
  depth_scale_ = 1000; // pixel values are in mm
  get_rgb_names(seq_, obj_, rgb_);
  get_depth_names(seq_, obj_, depth_);
  isCompressed = false;

  decompressedImage = new unsigned char[width * height * 3];
  decompressedDepth = new unsigned short[width * height];

  num_frames_ = rgb_.size();
}

RedwoodReader::~RedwoodReader()
{
  delete decompressedDepth;
  delete decompressedImage;
}


bool
RedwoodReader::grabNext(bool& returnVal, int& currentFrame)
{
  if (hasMore() && currentFrame < ConfigArgs::get().totalNumFrames) {
    readNext();
    ThreadDataPack::get().trackerFrame.assignAndNotifyAll(currentFrame);
    return true;
  }
  returnVal = false;
  return false;
}

void
RedwoodReader::readNext()
{
  auto i = current_frame_;
  cv::Mat depth_img = cv::imread(depth_[i],
                                 CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
  depth_img.convertTo(depth_img, CV_16U);

  cv::Mat bgr_img = cv::imread(rgb_[i],
                               CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
  bgr_img.convertTo(bgr_img, CV_8UC3);
  cv::Mat rgb_img(bgr_img.rows, bgr_img.cols, CV_8UC3);
  cv::cvtColor(bgr_img, rgb_img, cv::COLOR_BGR2RGB);

  memcpy(decompressedImage, rgb_img.data, rgb_img.total());
  memcpy(decompressedDepth, depth_img.data, depth_img.total()*sizeof(unsigned short));
  current_frame_++;
}

bool
RedwoodReader::hasMore()
{
  return current_frame_ + 1 < num_frames_;
}

void 
RedwoodReader::get_rgb_dir(const std::string& seq_id,
                           const std::string& seq_class,
                           std::string& rgb_dir)
{
  path p = path(root_path_) / path(seq_class) / path(seq_id) / path("rgb");
  rgb_dir = p.string();
}

void 
RedwoodReader::get_depth_dir(const std::string& seq_id,
                             const std::string& seq_class,
                             std::string& depth_dir)
{
  path p = path(root_path_) / path(seq_class) / path(seq_id) / path("depth");
    depth_dir = p.string();
}

void 
RedwoodReader::get_rgb_names(const std::string& seq_id,
                             const std::string& seq_class,
                             std::vector<std::string>& rgb_names)
{
  std::string rgb_dir;
  get_rgb_dir(seq_id, seq_class, rgb_dir);
  path p(rgb_dir);
  assert(is_directory(p));
  for (auto& f : directory_iterator(p)) {
    if (f.path().string().find(std::string("jpg")) != std::string::npos) {
      rgb_names.push_back(f.path().string());
    }
  }
  std::sort(rgb_names.begin(), rgb_names.end());
}

void
RedwoodReader::get_depth_names(const std::string& seq_id,
                               const std::string& seq_class,
                               std::vector<std::string>& depth_names)
{
  std::string depth_dir;
  get_depth_dir(seq_id, seq_class, depth_dir);
  path p(depth_dir);
  assert(is_directory(p));
  for (auto& f : directory_iterator(p)) {
    if (f.path().string().find(std::string("png")) != std::string::npos) {
      depth_names.push_back(f.path().string());
    }
  }
  std::sort(depth_names.begin(), depth_names.end());
}

