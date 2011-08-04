//start
#include <ecto/ecto.hpp>
using ecto::tendrils;
namespace introduction
{
  struct InterfaceCell
  {
    static void
    declare_params(tendrils& params);

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out);

    void
    configure(tendrils& params, tendrils& in, tendrils& out);

    int
    process(tendrils& in, tendrils& out);
  };

}

ECTO_CELL(introduction, introduction::InterfaceCell, "InterfaceCell",
          "A cell cell implementing the entire ecto interface");
//end

//impl_start
namespace introduction
{
  void
  InterfaceCell::declare_params(tendrils & p)
  {
    //...
  }

  void
  InterfaceCell::declare_io(const tendrils & p, tendrils & i, tendrils & o)
  {
    //...
  }

  void
  InterfaceCell::configure(tendrils & p, tendrils & i, tendrils & o)
  {
    //....
  }

  int
  InterfaceCell::process(tendrils & i, tendrils & o)
  {
    return ecto::OK;
  }
}
//impl_end
