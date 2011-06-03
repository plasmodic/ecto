#include "push_ups.cpp"
#include <ecto/plasm.hpp>

#include <iomanip>
#include <boost/progress.hpp>
#include <boost/timer.hpp>

#define STRINGYFY(stuffs) #stuffs

int main()
{
  ecto::plasm p;
  ecto::module::ptr m = ecto::create_module<ecto_push_ups::Add2>();
  ecto::module::ptr b = ecto::create_module<ecto_push_ups::BigData>();

  std::cout << "adding" << std::endl;
  std::cout << "ecto" << std::endl;
    {
      boost::progress_timer t; // start timing
      for (size_t i = 0; i < 10e6; i++)
        {
          m->process();
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
  std::string program = STRINGYFY(
      size_t big_data()
        {
          std::vector<int> data;
          data.resize(10e4);
          for (size_t i = 0; i < 10e4; i++)
          data[i] = std::rand();
          return std::accumulate(data.begin(), data.end(), 0);
        }
  );

  std::cout << program << std::endl;

  std::cout << "ecto" << std::endl;
    {
      boost::progress_timer t; // start timing
      for (size_t i = 0; i < 10e3; i++)
        {
          b->process();
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
