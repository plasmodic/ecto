// HACK!  We need access to the internal map so we can print Sensible
//        error messages; the map-of-typeid scheme that boost exception
//        uses internally is a PITA when dealing with cross-shared-library 
//        exceptions.
#include <ecto/util.hpp>
#define private public // hooah!
#include <boost/exception/exception.hpp>
#include <boost/exception/info.hpp>
#undef private
#include <boost/exception/all.hpp> 

#include <ecto/except.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/format.hpp>


using boost::format;

namespace ecto
{
  namespace except
  {

    namespace ed = ::boost::exception_detail;

#define MAKEWHAT(r, data, NAME)                 \
    const char* NAME::what() const throw() { return BOOST_PP_STRINGIZE(NAME); }

    BOOST_PP_SEQ_FOR_EACH(MAKEWHAT, ~, ECTO_EXCEPTIONS);

    const char* EctoException::what() const throw() { return "EctoException"; }

    std::string
    diagnostic_string(const EctoException& e)
    {
      format fmt("%25s  %s\n");
      std::ostringstream tmp;
#ifdef ECTO_TRACE_EXCEPTIONS
      char const * const * f=::boost::get_error_info< ::boost::throw_file>(e);
      tmp << str(fmt % "File" % (f ? *f : "(unknown)"));
      int const * l=::boost::get_error_info< ::boost::throw_line>(e);
      tmp << str(fmt % "Line" % (l ? *l : -1));
      
      char const * const * fn=::boost::get_error_info< ::boost::throw_function>(e);
      tmp << str(fmt % "Throw in function" % (fn ? *fn : "(unknown)"));
      tmp << '\n';
#endif
      // tmp << str(fmt % "Exception type" % name_of(typeid(e)));

      const std::exception& se = dynamic_cast<const std::exception&>(e);
      tmp << str(fmt % "exception_type" % se.what());

      if( char const * s=ed::get_diagnostic_information(e
#if BOOST_VERSION > 104000
                                                        , "MEH"
#endif
                                                        ))
        if( *s )
          tmp << s;
      return tmp.str();
    }

    boost::optional<std::string>
    diagnostic_string(const EctoException& e, const std::string& name)
    {
      bool some_found = false;
#define MAYBE_RETURN(r, data, NAME)                                     \
      if (name == BOOST_PP_STRINGIZE(NAME)) {                           \
        some_found = true;                                              \
        const std::string* the_s = boost::get_error_info<NAME>(e);      \
        if (the_s)                                                      \
          return *the_s;                                                \
        else                                                            \
          return boost::optional<std::string>();                        \
      }

      BOOST_PP_SEQ_FOR_EACH(MAYBE_RETURN, ~, ECTO_EXCEPTION_TAG_NAMES);
      if (! some_found)
        return name + " is not a valid exception info tag";
      else
        return boost::optional<std::string>();
    }


  }
}
