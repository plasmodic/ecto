// 
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// 
#include <ecto/all.hpp>

#include <boost/thread.hpp>

namespace ecto {
  namespace graph {

    struct edge::impl {
      std::string from_port, to_port;
      boost::mutex mtx;
      std::deque<ecto::tendril> deque;
    };

    edge::edge(const std::string& fp, const std::string& tp) 
      : impl_(new impl)
    { 
      impl_->from_port = fp;
      impl_->to_port = tp;
    }

    const std::string& edge::from_port() {
      return impl_->from_port;
    }

    const std::string& edge::to_port() {
      return impl_->to_port;
    }

    tendril& edge::front() 
    { 
      boost::unique_lock<boost::mutex> lock(impl_->mtx);
      return impl_->deque.front();
    }

    void edge::pop_front() 
    { 
      boost::unique_lock<boost::mutex> lock(impl_->mtx);
      impl_->deque.pop_front(); 
    }
    void edge::push_back(const ecto::tendril& t) 
    {
      boost::unique_lock<boost::mutex> lock(impl_->mtx);
      impl_->deque.push_back(t);
    }
    std::size_t edge::size() 
    {
      boost::unique_lock<boost::mutex> lock(impl_->mtx);
      return impl_->deque.size(); 
    }

  }
}

