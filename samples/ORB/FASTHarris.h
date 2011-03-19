#pragma once
#include <opencv2/features2d/features2d.hpp>
#include <vector>
namespace FASTHarris
{
struct HarrisResponse
{
  explicit HarrisResponse(const cv::Mat& image, double k = 0.04);
  void operator()(std::vector<cv::KeyPoint>& kpts) const;
  cv::Mat image;
  cv::Rect imgroi;
  double k;
};

inline bool keypointResponseLess(const cv::KeyPoint& lhs, const cv::KeyPoint& rhs)
{
  return lhs.response < rhs.response;
}

inline bool keypointResponseGreater(const cv::KeyPoint& lhs, const cv::KeyPoint& rhs)
{
  return lhs.response > rhs.response;
}

class SimpleFASTHarris : public cv::FeatureDetector
{
public:
  SimpleFASTHarris() :
    desired_n_features_(100)
  {
  }
  SimpleFASTHarris(size_t desired_n_features) :
    desired_n_features_(desired_n_features)
  {
  }
protected:
  virtual void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask = cv::Mat()) const;
  size_t desired_n_features_;
};
}
