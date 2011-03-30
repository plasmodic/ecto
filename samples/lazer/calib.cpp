#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//disable show in here
#define DISABLE_SHOW 1
#if DISABLE_SHOW
#ifdef SHOW
#undef SHOW
#define SHOW() do{}while(false)
#endif
#endif

using ecto::tendrils;

enum Pattern
{
  CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
};

struct Camera
{
  cv::Mat K, D;
  cv::Size image_size;
};
static std::vector<cv::Point3f> calcChessboardCorners(cv::Size boardSize, float squareSize,
                                                      Pattern patternType = CHESSBOARD)
{
  std::vector<cv::Point3f> corners;
  switch (patternType)
  {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
      break;

    case ASYMMETRIC_CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(cv::Point3f(float((2 * j + i % 2) * squareSize), float(i * squareSize), 0));
      break;

    default:
      std::logic_error("Unknown pattern type.");
  }
  return corners;
}

struct PatternDetector : ecto::module
{
  void Config()
  {
    SHOW();
    setIn<cv::Mat> ("in", "The grayscale image to search for a calibration pattern in.");
    setOut<std::vector<cv::Point2f> > ("out", "The observed pattern points.");
    setOut<bool> ("found", "Whether or not a pattern was found...");
    grid_size_ = cv::Size(getParam<int> ("cols"), getParam<int> ("rows"));
    choosePattern();
  }
  void Process()
  {
    SHOW();
    const cv::Mat& in = getIn<cv::Mat> ("in");
    std::vector<cv::Point2f>& out = getOut<std::vector<cv::Point2f> > ("out");
    switch (pattern_)
    {
      case ASYMMETRIC_CIRCLES_GRID:
        getOut<bool> ("found") = cv::findCirclesGrid(in, grid_size_, out, cv::CALIB_CB_ASYMMETRIC_GRID);
        break;
      case CHESSBOARD:
        getOut<bool> ("found") = cv::findChessboardCorners(in, grid_size_, out);

        break;
      case CIRCLES_GRID:
        getOut<bool> ("found") = cv::findCirclesGrid(in, grid_size_, out, cv::CALIB_CB_SYMMETRIC_GRID);
        break;
    }
  }
  static void Params(tendrils& p)
  {
    SHOW();
    p.set<int> ("rows", "Number of dots in row direction", 4);
    p.set<int> ("cols", "Number of dots in col direction", 11);
    p.set<std::string> ("pattern_type", "The pattern type, possible values are: [chessboard|circles|acircles]",
                        "acircles");
  }
  void choosePattern()
  {
    std::string pattern = getParam<std::string> ("pattern_type");
    if (pattern == "chessboard")
    {
      pattern_ = CHESSBOARD;
    }
    else if (pattern == "circles")
    {
      pattern_ = CIRCLES_GRID;
    }
    else if (pattern == "acircles")
    {
      pattern_ = ASYMMETRIC_CIRCLES_GRID;
    }
    else
      throw std::runtime_error("Unknown pattern type : " + pattern + " Please use: [chessboard|circles|acircles]");
  }
  cv::Size grid_size_;
  Pattern pattern_;
};

struct PatternDrawer : ecto::module
{
  void Config()
  {
    SHOW();
    setIn<cv::Mat> ("in", "The image to to find a vertical lazer line in.");
    setIn<std::vector<cv::Point2f> > ("points", "Circle pattern points.");
    setIn<bool> ("found", "Found the pattern");
    setOut<cv::Mat> ("out", "Pattern Image");
    grid_size_ = cv::Size(getParam<int> ("cols"), getParam<int> ("rows"));
  }
  void Process()
  {
    SHOW();
    const cv::Mat& in = getIn<cv::Mat> ("in");
    const std::vector<cv::Point2f>& points = getIn<std::vector<cv::Point2f> > ("points");
    bool found = getIn<bool> ("found");
    cv::Mat& out = getOut<cv::Mat> ("out");
    in.copyTo(out);
    cv::drawChessboardCorners(out, grid_size_, points, found);
  }
  static void Params(tendrils& p)
  {
    SHOW();
    p.set<int> ("rows", "Number of dots in row direction", 4);
    p.set<int> ("cols", "Number of dots in col direction", 11);
  }
  cv::Size grid_size_;
};

struct CameraCalibrator : ecto::module
{
  typedef std::vector<cv::Point3f> object_pts_t;
  typedef std::vector<cv::Point2f> observation_pts_t;
  void Config()
  {
    SHOW();
    setIn<observation_pts_t> ("points", "Circle pattern points.");
    setIn<cv::Mat> ("image", "Image that is used for calibration");
    setIn<bool> ("found", "Found the pattern");
    setOut<float> ("norm", "Norm of the input points to all previous points observed.");
    grid_size_ = cv::Size(getParam<int> ("cols"), getParam<int> ("rows"));
    board_pts_ = calcChessboardCorners(grid_size_, 1.0, ASYMMETRIC_CIRCLES_GRID);
    n_obs_ = getParam<int> ("n_obs");
    object_pts_.clear();
    norm_thresh_ = 150; //pixel values;
    calibrated_ = false;
  }
  double calcDistance(const observation_pts_t& in) const
  {
    cv::Mat p_in(in);
    double norm = 10e6;
    for (size_t i = 0; i < observation_pts_.size(); i++)
    {
      cv::Mat p_o(observation_pts_[i]);
      cv::Mat diff = p_in - p_o;
      norm = std::min(cv::norm(diff), norm);
    }
    return norm;
  }
  void Process()
  {
    SHOW();
    const observation_pts_t& in = getIn<observation_pts_t> ("points");
    bool found = getIn<bool> ("found");
    float norm = 0;
    if (found)
    {
      norm = calcDistance(in);

      if (norm > norm_thresh_ || observation_pts_.empty())
      {
        std::cout << "distance: " << norm << std::endl << "capturing ..." << std::endl;
        object_pts_.push_back(board_pts_);
        observation_pts_.push_back(in);
      }

    }
    if (int(observation_pts_.size()) > n_obs_ && !calibrated_)
    {
      std::vector<cv::Mat> rvecs, tvecs;
      int flags = CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_PRINCIPAL_POINT;
      double rms = cv::calibrateCamera(object_pts_, observation_pts_, getIn<cv::Mat> ("image").size(), camera_.K,
                                       camera_.D, rvecs, tvecs, flags);
      std::cout << "K = " << camera_.K << std::endl;
      std::cout << "D = " << camera_.D << std::endl;

      printf("RMS error reported by calibrateCamera: %g\n", rms);
      calibrated_ = true;
    }

    getOut<float> ("norm") = norm;
  }
  static void Params(tendrils& p)
  {
    SHOW();
    p.set<int> ("rows", "Number of dots in row direction", 4);
    p.set<int> ("cols", "Number of dots in col direction", 11);
    p.set<int> ("n_obs", "Number of observations", 50);
    p.set<float> ("square_size", "Number of observations", 25);
  }
  cv::Size grid_size_;
  int n_obs_;
  float norm_thresh_;
  bool calibrated_;
  object_pts_t board_pts_;
  std::vector<object_pts_t> object_pts_;
  std::vector<observation_pts_t> observation_pts_;
  Camera camera_;
};

ECTO_MODULE(calib)
{
  ecto::wrap<PatternDetector>("PatternDetector");
  ecto::wrap<PatternDrawer>("PatternDrawer");
  ecto::wrap<CameraCalibrator>("CameraCalibrator");
}
