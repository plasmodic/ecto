#include <ecto/ecto.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/python/overloads.hpp>
namespace bp = boost::python;
namespace ecto
{
  struct Dealer
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<bp::object>("iterable", 
                            "iterable python object... values to be output")
        .required(true);
      p.declare<tendril_ptr>("tendril",
                              "Destination tendril...  used to set output type")
        .required(true);
    }

    static void
    declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      out.declare<tendril::none>("out", "Any type");
    }

    void
    configure(const tendrils& p, const tendrils& in, const tendrils& out)
    {
      bp::object iterable = p["iterable"]->get<bp::object>();

      size_t end = bp::len(iterable);

      tendril_ptr typer = p["tendril"]->get<tendril_ptr>();

      for (size_t j = 0; j < end; ++j)
        {
          bp::object value = iterable[j];
          tendril x;
          x << *typer; // set the type.
          x << value;  // extract from python
          values_.push_back(x);
        }

      out_ = out["out"];
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      if (values_.empty())
        return ecto::QUIT;
      *out_ << values_.front();
      values_.pop_front();
      return ecto::OK;
    }
    std::list<tendril> values_;
    tendril_ptr out_;
  };
}
ECTO_CELL(ecto, ecto::Dealer, "Dealer", "Emit values of python iterable");
