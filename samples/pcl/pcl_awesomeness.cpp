#include <ecto/ecto.hpp>

#include <iostream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

#include <pcl/io/kinect_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <boost/thread.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> cloud_t;
class SimpleKinectGrabber
{
public:
  SimpleKinectGrabber() :
    thread_(boost::ref(*this))
  {
  }
  ~SimpleKinectGrabber()
  {
    std::cout << "Attempting to stop" << std::endl;
    thread_.interrupt();
    thread_.join();
  }

  void operator ()()
  {
    boost::scoped_ptr<pcl::Grabber> interface(new pcl::OpenNIGrabber());

    boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
        boost::bind(&SimpleKinectGrabber::cloud_cb_, this, _1);

    boost::signals2::connection c = interface->registerCallback(f);

    interface->start();

    while (!thread_.interruption_requested())
    {
      boost::thread::yield();

    }

    c.disconnect();
    std::cerr << "Stopping" << std::endl;

    interface->stop();
  }
  /**
   * \brief don't hang on to this cloud!! or it won't get updated.
   */
  cloud_t::ConstPtr getLatestCloud()
  {
    boost::mutex::scoped_lock lock(mutex_);
    return cloud_;
  }
  void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cloud_ = cloud;
  }

  boost::mutex mutex_;
  cloud_t::ConstPtr cloud_;
  boost::thread thread_;

};

struct KinectGrabber : ecto::module
{
  void Config()
  {
    setOut<cloud_t::ConstPtr> ("out", "An rgb xyz point cloud from the kinect");
  }

  static void Params(tendrils_t& params)
  {
  }

  void Process()
  {
    getOut<cloud_t::ConstPtr> ("out") = grabber_.getLatestCloud();
  }
  SimpleKinectGrabber grabber_;
};

struct CloudViewer : ecto::module
{
  void Config()
  {
    setIn<cloud_t::ConstPtr> ("input", "The cloud to view");
    setOut<bool> ("stop", "True if stop requested", false);
    viewer_.reset(new pcl_visualization::CloudViewer(getParam<std::string> ("window_name")));
  }

  static void Params(tendrils_t& params)
  {
    params.set<std::string> ("window_name", "The window name", "cloud viewer");
  }

  void Process()
  {
    if (!viewer_)
      return;
    cloud_t::ConstPtr cloud = getIn<cloud_t::ConstPtr> ("input");
    if (cloud)
      viewer_->showCloud(*cloud);
    if (viewer_->wasStopped(10))
      getOut<bool> ("stop") = true;
  }
  boost::shared_ptr<pcl_visualization::CloudViewer> viewer_;
};

ECTO_MODULE(pcl)
{
  // ecto::wrap<VoxelGrid>("VoxelGrid");
  ecto::wrap<KinectGrabber>("KinectGrabber", "This grabs frames from the kinect!!!");
  ecto::wrap<CloudViewer>("CloudViewer");
}
