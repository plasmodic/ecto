#include "push_ups.cpp"
#include <ecto/plasm.hpp>
#include <iomanip>
#include <boost/progress.hpp>
#include <boost/timer.hpp>
int main()
{
  ecto::plasm p;
  ecto::module::ptr m(new ecto_push_ups::Add2()), m2(new ecto_push_ups::Add2());;
  m->Config();m2->Config();

  ecto::module::ptr b(new ecto_push_ups::BigData());
  b->Config();

  p.connect(m,"out",m2,"x");
  p.connect(m,"out",m2,"y");

  p.markDirty(m);

  p.go(m2);
  p.viz(std::cout);

}
