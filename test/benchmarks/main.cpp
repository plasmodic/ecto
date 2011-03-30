#include "push_ups.cpp"

#include <iomanip>
#include <boost/progress.hpp>
#include <boost/timer.hpp>

int main()
{
  ecto::plasm p;
  ecto::module::ptr m(new ecto_push_ups::Add2());
  m->Config();

  ecto::module::ptr b(new ecto_push_ups::BigData());
  b->Config();

  std::cout << "adding" << std::endl;
  std::cout << "ecto" << std::endl;
  {
    boost::progress_timer t; // start timing
    for (size_t i = 0; i < 10e6; i++)
    {
      p.markDirty(m);
      p.go(m);
    }
  }
  std::cout << "raw" << std::endl;
  {
    boost::progress_timer t; // start timing
    for (size_t i = 0; i < 10e6; i++)
    {
      ecto_push_ups::add2(std::rand(), std::rand());
    }
  }

  std::cout << "big data (10e3 times)\n"
      "size_t big_data();\n"
      "{   std::vector<int> data;\n"
      "    data.resize(10e4);\n"
      "    for (size_t i = 0; i < 10e4; i++);\n"
      "      data[i] = std::rand();\n"
      "    return std::accumulate(data.begin(), data.end(), 0);\n"
      "}" << std::endl;
  std::cout << "ecto" << std::endl;
  {
    boost::progress_timer t; // start timing
    for (size_t i = 0; i < 10e3; i++)
    {
      p.markDirty(b);
      p.go(b);
    }
  }
  std::cout << "raw" << std::endl;
  {
    boost::progress_timer t; // start timing
    for (size_t i = 0; i < 10e3; i++)
    {
      ecto_push_ups::big_data();
    }
  }

}
