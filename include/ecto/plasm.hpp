/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/tuple/tuple.hpp>
#include <string>
#include <map>
#include <list>
#include <ecto/tendril.hpp>

namespace ecto
{
  //forward declare module so we don't get affected by its header
  class module;
  typedef boost::shared_ptr<module> module_ptr;

  /**
   * \brief The plasm is the graph structure of ecto, responsible for keeping track
   * of the connectivity between module and the execution of modules in the context of this
   * graph.
   *
   * The plasm is mean to be tool interacted with from python, but may be useful
   * from c++ in a dynamicly loaded environment.
   */
  class plasm: boost::noncopyable
  {
  public:
    plasm();

    /**
     * \brief connect one module to another, and populate the plasms graph accordingly.
     * This will throw on a type mismatch.
     * @param from  The from module
     * @param output The output key of the from module
     * @param to The to module
     * @param input The input key from the to module.
     */
    void connect(module_ptr from, const std::string& output, module_ptr to,
                 const std::string& input);

    /**
     * Disconnect a tendril from another tendril.
     *
     * @param from
     * @param output
     * @param to
     * @param input
     */
    void disconnect(module_ptr from, const std::string& output, 
                    module_ptr to, const std::string& input);

    /**
     * \brief This executes the graph, by executing all nodes in dependency order.
     */
    int execute();
    void spin();

    /**
     * \brief output graphviz to a stream.
     * @param out the output stream. Graphviz will be in plain text format.
     */
    void viz(std::ostream& out) const;
    /**
     * \brief Get a std::string graphiz of the module.
     * @return
     */
    std::string viz() const;

  private:
    class impl;
    boost::shared_ptr<impl> impl_;
    friend class plasm_wrapper;
  };
}
