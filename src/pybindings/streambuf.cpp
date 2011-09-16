#include <boost/python.hpp>
#include <ecto/python/streambuf.hpp>
namespace ecto
{
  namespace py
  {

    void
    wrap_python_streambuf()
    {
      using ecto::py::streambuf;
      using namespace boost::python;
      class_<streambuf, boost::shared_ptr<streambuf>, boost::noncopyable> sb("streambuf", no_init);
      sb.def(init<object&, std::size_t>((arg("file"), arg("buffer_size") = 0)));
      sb.def_readwrite("default_buffer_size", streambuf::default_buffer_size, "The default size of the buffer sitting "
                       "between a Python file object and a C++ stream.");
      using ecto::py::ostream;
      class_<std::ostream, boost::shared_ptr<std::ostream>, boost::noncopyable>("std_ostream", no_init);
      class_<ostream, boost::noncopyable, bases<std::ostream> > os("ostream", no_init);
      os.def(init<object&, std::size_t>((arg("python_file_obj"), arg("buffer_size") = 0)));

      using ecto::py::istream;
      class_<std::istream, boost::shared_ptr<std::istream>, boost::noncopyable>("std_istream", no_init);
      class_<istream, boost::noncopyable, bases<std::istream> > is("istream", no_init);
      is.def(init<object&, std::size_t>((arg("python_file_obj"), arg("buffer_size") = 0)));
    }

  }
}
