#include <ecto/ecto.hpp>
#include <iostream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

namespace buster
{
struct FooPOD
{
  int x;
  float y;
};

struct FooPODModule: ecto::module
{
  static void Initialize(ecto::tendrils& p)
  {
    p.declare<std::string> ("str", "I print this:", "Hello World");
  }

  void configure()
  {
    inputs.declare<FooPOD> ("foo", "A string to print");
    outputs.declare<FooPOD> ("foo", "A string to print");
  }

  void process()
  {
    std::cout << inputs.get<FooPOD> ("foo").x << std::endl;
    outputs.get<FooPOD> ("foo").y = 3.14;
  }
};
struct Printer: ecto::module
{
  static void Initialize(ecto::tendrils& p)
  {
    p.declare<std::string> ("str", "I print this:", "Hello World");
  }

  void configure()
  {
    inputs.declare<std::string> ("str", "A string to print", "hello");
  }

  void process()
  {
    std::cout << inputs.get<std::string> ("str") << std::endl;
  }
};

struct Generate: ecto::module
{
  int step_;

  static void Initialize(ecto::tendrils& p)
  {
    p.declare<double> ("step", "The step with which i generate integers.", 2);
    p.declare<double> ("start", "My starting value", 0);
  }

  void configure()
  {
    step_ = params.get<double> ("step");
    outputs.declare<double> ("out", "output",
        params.get<double> ("start") - step_);
  }

  void process()
  {
    outputs.get<double> ("out") += step_;
  }
};

struct Multiply: ecto::module
{
  double factor_;

  static void Initialize(ecto::tendrils& p)
  {
    p.declare<double> ("factor", "A factor to multiply by.", 3.14);
  }

  void configure()
  {
    factor_ = params.get<double> ("factor");
    inputs.declare<double> ("in", "multly in by factor");
    outputs.declare<double> ("out", "the result of in * factor");
  }
  void process()
  {
    outputs.get<double> ("out") = inputs.get<double> ("in") * factor_;
  }
};
struct SharedPass: ecto::module
{

  typedef ecto::Handle<boost::shared_ptr<int> > handle_t;
  typedef ecto::ConstHandle<boost::shared_ptr<int> > handle_const_t;

  static void Initialize(ecto::tendrils& p)
  {
    p.declare<int> ("x", "Default value", -1);
  }

  void configure()
  {
    input = inputs.declare<boost::shared_ptr<int> > ("input", "a pass through",
        boost::shared_ptr<int>(new int(params.get<int> ("x"))));
    output = outputs.declare<boost::shared_ptr<int> > ("output",
        "a pass through", *input);
    value = outputs.declare<int> ("value", "value", -1);
  }
  void process()
  {
    *output = *input;
    //std::cout << *output << std::endl;
    *value = **output;
  }
  handle_const_t input;
  handle_t output;
  ecto::Handle<int> value;
};
struct Scatter: ecto::module
{
  static void Initialize(ecto::tendrils& p)
  {
    p.declare<int> ("n", "Number to scatter...", 2);
    p.declare<int> ("x", "The value to scatter...", 13);
  }

  void configure()
  {
    n_ = params.get<int> ("n");
    x_ = params.get<int> ("x");
    for (int i = 0; i < n_; i++)
    {
      outputs.declare<int> (str(boost::format("out_%04d") % i),
          str(boost::format("The %dth scatter") % i));
    }
  }
  void process()
  {
    for (int i = 0; i < n_; i++)
    {
      outputs.get<int> (str(boost::format("out_%04d") % i)) = x_;
    }
  }
  int n_, x_;
};

template<typename ValueT>
struct Gather: public ecto::module
{
  typedef ValueT value_type;

  static void Initialize(ecto::tendrils& p)
  {
    p.declare<int> ("n", "N to gather", 2);
  }

  void configure()
  {
    n_ = params.get<int> ("n");
    for (int ii = 0; ii < n_; ii++)
    {
      inputs.declare<value_type> (str(boost::format("in_%04d") % ii),
          "An " + ecto::name_of<value_type>() + "input.");
    }
    outputs.declare<value_type> ("out", "The sum of all inputs.");
  }

  void process()
  {
    //SHOW();
    value_type& out = outputs.get<value_type> ("out");
    out = 0;
    typedef std::pair<std::string, ecto::tendril> pp;
    BOOST_FOREACH(const pp& in,inputs)
          {
            out += in.second.get<value_type> ();
          }
  }
  int n_;
};

boost::shared_ptr<ecto::tendril> makePodTendril()
{
  boost::shared_ptr<ecto::tendril> p;
  ecto::tendril* t = new ecto::tendril(0, "doc");
  p.reset(t);
  return p;
}

}

BOOST_PYTHON_MODULE(buster)
{
  using namespace buster;
  ecto::wrap<Printer>("Printer", "A printer...");
  ecto::wrap<Generate>("Generate", "A generator module.");
  ecto::wrap<SharedPass>("SharedPass", "A shared pointer pass through");
  ecto::wrap<Multiply>("Multiply", "Multiply an input with a constant");
  ecto::wrap<Scatter>("Scatter", "Scatter a value...");
  ecto::wrap<Gather<int> >("Gather", "Gather a scattered value...");
  ecto::wrap<Gather<double> >("Gather_double", "Gather a scattered value...");
  boost::python::def("make_pod_tendril", buster::makePodTendril);
}
