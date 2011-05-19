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
  /**
   * vertex type enum
   */
  enum vertex_t
  {
    root, //!< This is a module node, not a tendril
    input, //!< Input tendril
    output, //!< Output tendril
    param
  //!< Parameter node ? TODO is this unused?
  };

  /**
   * A vertex in the plasm consists of a pointer to a module, a vertex type, the name of the tendril and the tendril its self.
   */
  typedef std::map<int,
      boost::tuple<module_ptr, vertex_t, std::string, tendril> > vertex_map_t;
  /**
   * \brief The edges encode the vertex to vertex relationship.
   */
  typedef std::list<boost::tuple<size_t, size_t> > edge_list_t;

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
  void disconnect(module_ptr from, const std::string& output, module_ptr to,
      const std::string& input);

  /**
   * \brief This executes the graph, by executing all nodes in dependency order.
   */
  void execute();
  /**
   * \brief Mark the given module dirty. This will recurse through the graph, dirting all modules downstream.
   * @param m The module to mark dirty.
   */
  void mark_dirty(module_ptr m);
  /**
   * Execute the given module, recursing to all dependencies and executing them. If the module is dirty this is a NOP.
   * @param m the module to execute.
   */
  void go(module_ptr m);
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

  /**
   * Get a map of the vertices, useful for graph introspection
   * @return The map vertices.
   */
  vertex_map_t getVertices();
  /**
   * Get a list of the edges, useful for graph introspection
   * @return The edge list.
   */
  edge_list_t getEdges();

private:
  class impl;
  boost::shared_ptr<impl> impl_;
  friend class plasm_wrapper;
};
}
