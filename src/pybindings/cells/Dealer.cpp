#include <ecto/ecto.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/python/overloads.hpp>
namespace bp = boost::python;
namespace ecto
{
  struct Dealer
  {
    static void
    declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      out.declare<tendril::none>("out", "Any type");
    }
    void
    configure(const tendrils& parms, const tendrils& in, const tendrils& out)
    {
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
    tendril::ptr out_;
  };

  cell::ptr
  createDealer(tendril::ptr typer, bp::object iterable)
  {
    cell_<Dealer>::ptr dealer = ecto::create_cell<Dealer>();
    cell::ptr base(dealer);
    base->configure();
    if (!iterable || iterable == bp::object())
      return dealer;
    size_t end = bp::len(iterable);
    for (size_t j = 0; j < end; ++j)
    {
      bp::object value = iterable[j];
      tendril x;
      x << *typer; //set the type.
      x << value;
      dealer->impl->values_.push_back(x);
    }
    return dealer;
  }
  namespace py
  {
    using bp::arg;
    void
    wrap_dealer()
    {
      bp::def("Dealer", createDealer, (arg("typer"), arg("iterable") = bp::object()), //args
              "Constructs a Dealer with the type determined by `typer` and the the python iterable." //doc str
              );
    }
  }
}

