#include <ecto/ecto.hpp>
//start
namespace introduction
{
  struct NopCell
  {
  };
}
//end

ECTO_CELL(introduction, introduction::NopCell, "NopCell",
          "A cell that can't do anything.");
