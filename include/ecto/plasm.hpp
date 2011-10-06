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

#include <string>
#include <map>
#include <list>

#include <ecto/tendril.hpp>
#include <ecto/graph_types.hpp>

namespace ecto
{
  //forward declare cell so we don't get affected by its header
  class cell;
  typedef boost::shared_ptr<cell> cell_ptr;

  //forward declare schedulers for friendliness.
  namespace schedulers
  {
    class singlethreaded;
  }

  /**
   * \brief The plasm helps construct the graph structure in ecto.
   * It enforces several invariants that are necessary for scheduling DAGs and
   * is used by all the ecto::schedulers to enable exectution of modules that are connected in the graph.
   */
  class ECTO_EXPORT plasm: 
    boost::noncopyable, public boost::enable_shared_from_this<plasm>
  {
  public:
    plasm();
    ~plasm();

    /**
     * \brief insert the cell into the graph so that it may be executed by a scheduler.
     *
     * @param mod The cell to insert into the graph.
     */
    void
    insert(cell_ptr mod);

    /**
     * \brief connect one cell to another, and populate the plasms graph accordingly.
     * This will throw on a type mismatch.
     * @param from  The from cell
     * @param output The output key of the from cell
     * @param to The to cell
     * @param input The input key from the to cell.
     */
    void
    connect(cell_ptr from, const std::string& output, cell_ptr to, const std::string& input);

    /**
     * Disconnect a tendril from another tendril.
     *
     * @param from
     * @param output
     * @param to
     * @param input
     */
    void
    disconnect(cell_ptr from, const std::string& output, cell_ptr to, const std::string& input);
    /**
     * \brief output graphviz to a stream.
     * @param out the output stream. Graphviz will be in plain text format.
     */
    void
    viz(std::ostream& out) const;
    /**
     * \brief Get a std::string graphiz of the cell.
     * @return
     */
    std::string
    viz() const;

    /**
     * \brief check that all tags on the graph are satisified.
     * This will throw on errors in the graph, including, if required inputs are not connected
     * if required outputs are not connected, if there are cycles, etc...
     */
    void
    check() const;

    /**
     * \brief Get the underlying boost graph that this plasm has constructed.
     * @return
     */
    graph::graph_t&
    graph();

    const graph::graph_t&
    graph() const;


    /**
     * \brief Return the number of cells in the plasm (vertices in the graph)
     * 
     */
    std::size_t size() const;

    /**
     * \brief Grab a set of all the cells from the plasm.
     * @return a set of cells.
     */
    std::vector<cell::ptr> cells() const;

    /**
     * \brief Calls configure on all modules, if configure has not already been called.
     */
    void configure_all();

    /**
     * \brief Execute using a predefined scheduler.
     */
    int
    execute(unsigned niter = 1);

    typedef boost::shared_ptr<plasm> ptr;
    typedef boost::shared_ptr<const plasm> const_ptr;

    void save(std::ostream&) const;
    void load(std::istream&);


  private:
    class impl;
    boost::shared_ptr<impl> impl_;

    template<class Archive>
    void
    save(Archive & ar, const unsigned int version) const;

    template<class Archive>
    void
    load(Archive & ar,  const unsigned int version);
    friend class boost::serialization::access;
    BOOST_SERIALIZATION_SPLIT_MEMBER()
  };
}
