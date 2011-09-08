#include <ecto/except.hpp>
#include <boost/format.hpp>

using boost::format;

namespace ecto
{
  namespace except
  {

#define MAKEWHAT(r, data, NAME)                 \
    const char* NAME::what() const throw() { return BOOST_PP_STRINGIZE(NAME); }

    BOOST_PP_SEQ_FOR_EACH(MAKEWHAT, ~, ECTO_EXCEPTIONS);

    const char* EctoException::what() const throw() { return "EctoException"; }

    std::string diagnostic_string(const EctoException& e)
    {
      std::string s;
      format fmt("%25s  %s\n");

      s += str(fmt % "Exception Type" % e.what());

#define APPEND_ERRINFO(r, data, TAG)                                    \
      {                                                                 \
        const std::string* the_s =                                      \
          boost::get_error_info<TAG>(e);                                \
        if (the_s)                                                      \
          s += str(fmt % BOOST_PP_STRINGIZE(TAG) % the_s->c_str());     \
      }                                                                 \

      BOOST_PP_SEQ_FOR_EACH(APPEND_ERRINFO, ~, ECTO_EXCEPTION_TAG_NAMES);
      return s;
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
