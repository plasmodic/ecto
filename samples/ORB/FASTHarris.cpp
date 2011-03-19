#include "FASTHarris.h"

#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <iostream>
#include <boost/foreach.hpp>


namespace FASTHarris
{
inline float harris(const cv::Mat& patch, float k)
{
  cv::Matx<int, 9, 9> dX, dY;
  float a = 0, b = 0, c = 0;
  //seperable filter
  for (size_t v = 0; v < 9; v++)
  {
    for (size_t u = 0; u < 9; u++)
    {
      if (u >= 1 && u <= 7)
        dX(v, u) = (int)patch.at<uchar> (v, u - 1) - (int)patch.at<uchar> (v, u + 1);
      if (v >= 1 && v <= 7)
        dY(v, u) = (int)patch.at<uchar> (v - 1, u) - (int)patch.at<uchar> (v + 1, u);
    }
  }
  for (size_t v = 1; v < 8; v++)
  {
    for (size_t u = 1; u < 8; u++)
    {
      //float weight = 10 - std::sqrt((v - 4) * (v - 4) + (u - 4) * (u - 4));
      float weight = 1.0 / (9.0 * 9.0);
      int Ix = 3 * (dX(v - 1, u) + dX(v + 1, u)) + 11 * dX(v, u);
      int Iy = 3 * (dY(v, u - 1) + dY(v, u + 1)) + 11 * dY(v, u);
      Ix *= weight;
      Iy *= weight;
      a += Ix * Ix;
      b += Iy * Iy;
      c += Ix * Iy;
    }
  }
  return ((a * b - c * c) - (k * ((a + b) * (a + b))));
}


HarrisResponse::HarrisResponse(const cv::Mat& image, double k) :
  image(image), imgroi(0, 0, image.cols, image.rows), k(k)
{

}

void HarrisResponse::operator()(std::vector<cv::KeyPoint>& kpts) const
{
  for (size_t i = 0; i < kpts.size(); i++)
  {
    cv::KeyPoint& kpt = kpts[i];
    kpt.response = 0;
    cv::Rect r(kpt.pt.x - 4, kpt.pt.y - 4, 9, 9);
    //assure that the point is within the image
    if (!(imgroi.contains(r.tl()) && imgroi.contains(r.br())))
      continue;
    //calculate the response on the patch given by r
    kpt.response = harris(image(r), k);
  }
}


void SimpleFASTHarris::detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const
{
  //detect fast with a resonable threshhold, and nonmax
  cv::FastFeatureDetector fd(20, true);
  fd.detect(image, keypoints, mask);
  size_t n_features = desired_n_features_;
  //grab our harris response, operates in place on the keypoints
  HarrisResponse h(image);h(keypoints);
  //take only the top N if there are too many features
  if (keypoints.size() > n_features)
  {
    std::nth_element(keypoints.begin(), keypoints.begin() + n_features, keypoints.end(), keypointResponseGreater);
    keypoints.resize(n_features);
  }
}
}
#if 0
int main(int argc, char** argv){
  if(argc != 2)
  { 
    std::cerr << "usage:\n";
    std::cerr << argv[0] << " image_file" << std::endl;
    return -1;
  }
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  SimpleFASTHarris sfh(500);
  std::vector<cv::KeyPoint> kpts;
  sfh.detect(image,kpts);
  std::cout << "found " << kpts.size() << "kpts" << std::endl;
  cv::Mat draw_image;
  cv::drawKeypoints(image, kpts, draw_image, cv::Scalar::all(-1));
  cv::imshow("keypoints",draw_image);
  while((0xFF & cv::waitKey(0)) != 'q')
    std::cout << "press 'q' to quit" << std::endl;
}
#endif
