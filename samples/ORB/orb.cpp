#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <cmath>
#include "FASTHarris.h"

//disable show in here
#define DISABLE_SHOW 1
#if DISABLE_SHOW
#ifdef SHOW
#undef SHOW
#define SHOW() do{}while(false)
#endif
#endif

struct Pyramid : ecto::module
{

  void Config()
  {
    SHOW();
    levels_ = getParam<int> ("levels");
    scale_factor_ = getParam<float> ("scale_factor");
    magnification_ = getParam<int> ("magnification");
    for (int i = 0; i < levels_; i++)
    {
      setOut<cv::Mat> (str(boost::format("out_%d") % i));
      setOut<float> (str(boost::format("scale_%d") % i));
    }
    setIn<cv::Mat> ("in");
  }

  void Process()
  {
    SHOW();
    const cv::Mat& in = getIn<cv::Mat> ("in");
    for (int i = 0; i < levels_; i++)
    {
      cv::Mat& out = getOut<cv::Mat> (str(boost::format("out_%d") % i));
      float & scale = getOut<float> (str(boost::format("scale_%d") % i));
      scale = 1.0f / float(std::pow(scale_factor_, i - magnification_));
      //use liner interp if magnifying, area based if decimating
      cv::resize(in, out, cv::Size(), scale, scale, i - magnification_ < 0 ? CV_INTER_LINEAR : CV_INTER_AREA);
    }
  }

  static void Params(tendrils_t& p)
  {
    SHOW();
    p["levels"].set<int> ("Number of pyramid levels.", 3);
    p["scale_factor"].set<float> ("The scale factor between levels", 1.42);
    p["magnification"].set<int> (
                                 "The magnification, positive to start at a larger than real life. The scale at each pyramid level is 1.0/(scale_factor^{i - magnification}",
                                 0);
  }
  int levels_, magnification_;
  float scale_factor_;
};

struct PyramidRescale : ecto::module
{
  void Config()
  {
    SHOW();
    levels_ = getParam<int> ("levels");
    for (int i = 0; i < levels_; i++)
    {
      setIn<float> (str(boost::format("scale_%d") % i), "The scale of the level i");
      setIn<std::vector<cv::KeyPoint> > (str(boost::format("kpts_%d") % i), "The kpts at level i.");
    }
    setOut<std::vector<cv::KeyPoint> > ("out", "The rescaled kpts.");
  }
  void Process()
  {
    SHOW();
    std::vector<cv::KeyPoint>& kpts = getOut<std::vector<cv::KeyPoint> > ("out");
    kpts.clear();
    for (int i = 0; i < levels_; i++)
    {
      float scale = getIn<float> (str(boost::format("scale_%d") % i));
      const std::vector<cv::KeyPoint>& kpts_in = getIn<std::vector<cv::KeyPoint> > (str(boost::format("kpts_%d") % i));
      kpts.reserve(kpts.size()+kpts_in.size());
      for(size_t j = 0; j < kpts_in.size(); j++)
      {
        kpts.push_back(kpts_in[j]);
        cv::KeyPoint & x =  kpts.back();
        x.octave = i;
        x.pt *= 1/scale;
      }
    }

  }
  static void Params(tendrils_t& p)
  {
    SHOW();
    p["levels"].set<int> ("Number of pyramid levels.", 3);
  }
  int levels_;
};
struct FAST : ecto::module
{
  void Config()
  {
    SHOW();
    thresh_ = getParam<int> ("thresh");
    N_max_ = getParam<int> ("N_max");
    setOut<std::vector<cv::KeyPoint> > ("out", "Detected keypoints");
    setIn<cv::Mat> ("image", "The image to detect FAST on.");
    setIn<cv::Mat> ("mask", "optional mask");
  }
  void Process()
  {
    SHOW();
    const cv::Mat& in = getIn<cv::Mat> ("image");
    const cv::Mat& mask = getIn<cv::Mat> ("mask");
    std::vector<cv::KeyPoint>& kpts = getOut<std::vector<cv::KeyPoint> > ("out");
    cv::FastFeatureDetector fd(thresh_, true);
    fd.detect(in, kpts, mask);
    if (int(kpts.size()) > N_max_)
    {
      std::nth_element(kpts.begin(), kpts.end() + N_max_, kpts.end(), FASTHarris::keypointResponseGreater);
      kpts.resize(N_max_);
    }
  }
  static void Params(tendrils_t& p)
  {
    SHOW();
    p["thresh"].set<int> ("FAST threshhold.", 20);
    p["N_max"].set<int> ("The maximum number of keypoints", 2000);
  }
  int thresh_, N_max_;
};

struct Harris : ecto::module
{
  void Config()
  {
    SHOW();
    N_max_ = getParam<int> ("N_max");
    setOut<std::vector<cv::KeyPoint> > ("out", "Detected keypoints, with Harris");
    setIn<cv::Mat> ("image", "The image to calc harris response from.");
    setIn<std::vector<cv::KeyPoint> > ("kpts", "The keypoints to fill with Harris response.");
  }
  void Process()
  {
    SHOW();
    const cv::Mat& image = getIn<cv::Mat> ("image");
    const std::vector<cv::KeyPoint>& kpts_in = getIn<std::vector<cv::KeyPoint> > ("kpts");
    std::vector<cv::KeyPoint>& kpts = getOut<std::vector<cv::KeyPoint> > ("out");
    FASTHarris::HarrisResponse h(image);
    kpts = kpts_in;
    h(kpts);
    if (int(kpts.size()) > N_max_)
    {
      std::nth_element(kpts.begin(), kpts.end() + N_max_, kpts.end(), FASTHarris::keypointResponseGreater);
      kpts.resize(N_max_);
    }
  }
  static void Params(tendrils_t& p)
  {
    SHOW();
    p["N_max"].set<int> ("The maximum number of keypoints", 1000);
  }
  int N_max_;
};

struct DrawKeypoints : ecto::module
{
  void Config()
  {
    SHOW();
    setIn<cv::Mat> ("image", "The input image, to draw over.");
    setOut<cv::Mat> ("image", "The output image.");
    setIn<std::vector<cv::KeyPoint> > ("kpts", "The keypoints to draw.");
  }
  void Process()
  {
    SHOW();
    const cv::Mat& image = getIn<cv::Mat> ("image");
    const std::vector<cv::KeyPoint>& kpts_in = getIn<std::vector<cv::KeyPoint> > ("kpts");
    cv::Mat& out_image = getOut<cv::Mat> ("image");
    cv::drawKeypoints(image, kpts_in, out_image);
  }
  static void Params(tendrils_t& p)
  {
    SHOW();
  }
};
ECTO_MODULE(orb)
{
  ecto::wrap<Pyramid>("Pyramid");
  ecto::wrap<FAST>("FAST");
  ecto::wrap<Harris>("Harris");
  ecto::wrap<DrawKeypoints>("DrawKeypoints");
  ecto::wrap<PyramidRescale>("PyramidRescale");
}
