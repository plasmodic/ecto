/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Nico Blodow (blodow@cs.tum.edu)
 */

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
    thread_(this), cloud_(new cloud_t)
  {
  }
  ~SimpleKinectGrabber()
  {
    thread_.join();
  }

  void operator ()()
  {
    boost::scoped_ptr<pcl::Grabber> interface = new pcl::OpenNIGrabber();

    boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
        boost::bind(&SimpleKinectViewer::cloud_cb_, this, _1);

    boost::signals2::connection c = interface->registerCallback(f);

    interface->start();

    while (!thread_.interruption_requested())
    {
      thread_.sleep(10);
    }

    interface->stop();
  }
  /**
   * \brief don't hang on to this cloud!! or it won't get updated.
   */
  boost::weak_ptr<cloud_t> getLatestCloud()
  {
    boost::mutex::scoped_lock lock(mutex_);
    return cloud_;
  }
  void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (cloud_.use_count() > 1)
    {
      std::cout << "Someone is using the cloud, quick return, use count: " << cloud_.use_count() << std::endl;
      return;
    }
    *cloud_ = cloud;
  }

  boost::mutex mutex_;
  cloud_t::Ptr cloud_;
  boost::thread thread_;

};
