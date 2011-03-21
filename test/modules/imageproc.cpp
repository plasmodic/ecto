#include <ecto/ecto.hpp>

#include <iostream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//disable show in here
#if 1
#ifdef SHOW
#undef SHOW
#define SHOW() do{}while(false)
#endif
#endif

struct VideoCapture : ecto::module
{
  VideoCapture()
  {
    setOut<cv::Mat> ("out", "A video frame.", cv::Mat());
    setOut<int> ("frame_number", "The number of frames captured.", 0);
  }

  void Config()
  {
    int video_device = getParam<int> ("video_device");
    capture = cv::VideoCapture();
    capture.open(video_device);
    if (!capture.isOpened())
      throw std::runtime_error("Could not open video device " + video_device);
  }

  static void Params(tendrils_t& c)
  {
    c["video_device"].set<int> ("The device ID to open", 0);
  }

  void Process()
  {
    //getOut is a reference;
    capture >> getOut<cv::Mat> ("out");
    ++(getOut<int> ("frame_number"));
  }
  cv::VideoCapture capture;

};

struct imshow : ecto::module
{
  imshow() :
    window_name_("window"), waitkey_(10), auto_size_(true)
  {
    setIn<cv::Mat> ("in", "The image to show");
    setOut<int> ("out", "Character pressed.");
  }

  static void Params(tendrils_t& c)
  {
    c["name"].set<std::string> ("The window name", "image");
    c["waitKey"].set<int> ("Number of millis to wait, -1 for not at all, 0 for infinity.", -1);
    c["autoSize"].set<bool> ("Autosize the window.", true);
  }

  void Config()
  {
    window_name_ = getParam<std::string> ("name");
    waitkey_ = getParam<int> ("waitKey");
    auto_size_ = getParam<bool> ("autoSize");
  }
  void Process()
  {
    const cv::Mat& image = getIn<cv::Mat> ("in");
    if (image.empty())
    {
      getOut<int> ("out") = 0;
      throw std::logic_error("empty image!");
      return;
    }

    if (auto_size_)
    {
      cv::namedWindow(window_name_, CV_WINDOW_KEEPRATIO);
    }
    cv::imshow(window_name_, image);
    if (waitkey_ >= 0)
    {
      getOut<int> ("out") = int(0xff & cv::waitKey(waitkey_));
    }
    else
    {
      getOut<int> ("out") = 0;
    }
  }
  std::string window_name_;
  int waitkey_;
  bool auto_size_;
};

struct cvtColor : ecto::module
{
  cvtColor()
  {
  }
  void Config()
  {
    flag_ = getParam<int> ("flag");
    setIn<cv::Mat> ("in", "Color image.");
    setOut<cv::Mat> ("out", "input as a Gray image.");
  }
  void Process()
  {
    cv::cvtColor(getIn<cv::Mat> ("in"), getOut<cv::Mat> ("out"), flag_);
  }
  static void Params(tendrils_t& p)
  {
    std::stringstream ss;
    ss << "Convert an image's color using opencv, possible flags are:\n" << "RGB2GRAY = " << CV_RGB2GRAY << std::endl
        << "RGB2BGR = " << CV_RGB2BGR;
    p["flag"].set<int> (ss.str(), CV_RGB2BGR);
  }
  int flag_;
};

struct ChannelSplitter : ecto::module
{
    enum Channel
    {
     CB=0,CG = 1, CR=2, BLUE = 'b', GREEN = 'g', RED = 'r'
    };

    void pickColorIdx()
    {
       idxs_[CB] = bgr_ ? 0 : 2; //if its bgr then blue is first channel

       idxs_[CR]  = bgr_ ? 2 : 0; //if its bgr, then the red is the last channel

       idxs_[CG] = 1;

    }
    void Config()
    {
      SHOW();
      //store the color and the index per channel
      bgr_ = getParam<bool> ("bgr");
      pickColorIdx();

      setIn<cv::Mat> ("in", "The color image to split.");
      setOut<cv::Mat> ("red", "Red channel.");
      setOut<cv::Mat> ("green", "Green channel.");
      setOut<cv::Mat> ("blue", "Blue channel.");
    }
    void Process()
    {
      SHOW();
      const cv::Mat& in = getIn<cv::Mat> ("in");
      if (in.channels() == 3)
        cv::split(in, channels_);
      else if (in.channels() == 1)
      {
        for(int i = 0; i < 3; i++)
        {
          channels_[i] = in;
        }
      }else
      {
        throw std::runtime_error("unsupported number of channels! must be 1 or 3");
      }
      getOut<cv::Mat>("blue") = channels_[CB];
      getOut<cv::Mat>("green") = channels_[CG];
      getOut<cv::Mat>("red") = channels_[CR];
    }
    static void Params(tendrils_t& p)
    {
      SHOW();
      p["bgr"].set<bool> ("color format", true);
    }
    bool bgr_;
    int idxs_[3];
    cv::Mat channels_[3];
  };

struct Sobel : ecto::module
{
  Sobel() :
    x_(1), y_(1)
  {
    setIn<cv::Mat> ("in", "image.");
    setOut<cv::Mat> ("out", "sobel image");
  }

  static void Params(tendrils_t& p)
  {
    p["x"].set<int> ("The derivative order in the x direction", 0);
    p["y"].set<int> ("The derivative order in the y direction", 0);
  }

  void Config()
  {
    x_ = getParam<int> ("x");
    y_ = getParam<int> ("y");
  }
  void Process()
  {
    cv::Sobel(getIn<cv::Mat> ("in"), getOut<cv::Mat> ("out"), CV_32F, x_, y_);
  }
  int x_, y_;
};

template<typename T>
  struct Adder : ecto::module
  {
    Adder()
    {
      setIn<T> ("a", "to add to b");
      setIn<T> ("b", "to add to a");
      setOut<T> ("out", "a + b");
    }
    void Process()
    {
      getOut<T> ("out") = getIn<T> ("a") + getIn<T> ("b");
    }
    static void Params(tendrils_t& p)
    {
    }
  };

struct AbsNormalized : ecto::module
{
  AbsNormalized()
  {
    setIn<cv::Mat> ("in", "image.");
    setOut<cv::Mat> ("out", "absolute and normalized");
  }
  static void Params(tendrils_t& p)
  {
  }
  void Process()
  {
    const cv::Mat& m = getIn<cv::Mat> ("in");
    cv::Mat& out = getOut<cv::Mat> ("out");
    out = cv::abs(m) / (cv::norm(m, cv::NORM_INF) * 0.5);
  }
};

ECTO_MODULE(imageproc)
{
  ecto::wrap<VideoCapture>("VideoCapture");
  ecto::wrap<imshow>("imshow");
  ecto::wrap<AbsNormalized>("AbsNormalized");
  ecto::wrap<Sobel>("Sobel");
  ecto::wrap<cvtColor>("cvtColor");
  ecto::wrap<Adder<cv::Mat> >("ImageAdder");
  ecto::wrap<ChannelSplitter> ("ChannelSplitter");
}
