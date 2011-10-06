#include <ecto/ecto.hpp>
#include <ecto/serialization/registry.hpp>
#include <ecto/serialization/cell.hpp>
#include <ecto/serialization/plasm.hpp>
#include <cstring>
int
main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cout << argv[0] << " plasm.ecto NITER\n";
    return 1;
  }
  ecto::plasm::ptr p(new ecto::plasm());
  std::ifstream in(argv[1]);
  boost::archive::text_iarchive ia(in);
  ia & *p;

  std::cout << "** graphviz" << std::endl;
  p->viz(std::cout);
  std::cout << std::endl;
  std::cout << "** cell listing" << std::endl;
  std::vector<ecto::cell::ptr> cells = p->cells();
  for (size_t i = 0; i < cells.size(); i++)
  {
    std::cout << cells[i]->name() << std::endl;
  }
  return p->execute(std::atoi(argv[2]));
}
