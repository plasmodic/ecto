#pragma once
//this file is for allowing efficient python binding compile
//see the following web page for details:
//http://www.boost.org/doc/libs/1_46_1/libs/python/doc/tutorial/doc/html/python/techniques.html#python.reducing_compiling_time
namespace ecto
{
namespace py
{
  void wrapConnection();
  void wrapPlasm();
  void wrapModule();
}

}
