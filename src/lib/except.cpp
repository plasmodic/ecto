#include <ecto/util.hpp>
#include <boost/exception/all.hpp> 

#include <ecto/except.hpp>
#include <boost/format.hpp>
#include <boost/preprocessor/seq/seq.hpp>
#include <boost/preprocessor/seq/for_each_product.hpp>
#include <boost/preprocessor/seq/to_tuple.hpp>
#include <boost/preprocessor/facilities/expand.hpp>


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

    
    error_info_container_impl::error_info_container_impl(): count_(0) {}
    
    error_info_container_impl::~error_info_container_impl() throw() {}

    using boost::exception_detail::error_info_base;

    void
    error_info_container_impl::set(boost::shared_ptr<error_info_base> const & x, 
                                   type_info_ const & typeid_)
    {
      BOOST_ASSERT(x);
      info_[typeid_.type_.name()] = x;
      diagnostic_info_str_.clear();
    }

    boost::shared_ptr<error_info_base>
    error_info_container_impl::get( type_info_ const & ti ) const
    {
      error_info_map::const_iterator i=info_.find(ti.type_.name());
      if( info_.end()!=i )
        {
          boost::shared_ptr<error_info_base> const & p = i->second;
#ifndef BOOST_NO_RTTI
          BOOST_ASSERT( BOOST_EXCEPTION_DYNAMIC_TYPEID(*p).type_==ti.type_ );
#endif
          return p;
        }
      return boost::shared_ptr<error_info_base>();
    }

    char const * 
    error_info_container_impl::diagnostic_information(char const*) const
    {
      boost::format fmt("%25s  %s\n");
      if( diagnostic_info_str_.empty() )
        {
          std::ostringstream tmp;
          for( error_info_map::const_iterator i=info_.begin(),end=info_.end(); i!=end; ++i )
            {
              boost::shared_ptr<error_info_base const> const & x = i->second;
              tmp << str(fmt % /*name_of(*/x->tag_typeid_name()/*) */
                         % x->value_as_string());
            }
          tmp.str().swap(diagnostic_info_str_);
        }
      return diagnostic_info_str_.c_str();
    }

    void error_info_container_impl::add_ref() const { ++count_; }
    void error_info_container_impl::release() const { if( !--count_ ) delete this; }

    struct HACK_HACK_HACK;
  }
}

namespace boost {
  namespace exception_detail {

    // this break in to a boost::exception and gets the
    // error_info_container out.  
    template <> struct get_info< ::ecto::except::HACK_HACK_HACK>
    {
      template <typename E>
      static
      refcount_ptr<exception_detail::error_info_container>&
      get(E const& e)
      {
        return e.data_;
      }
    };
  }

  template <class E,class Tag,class T>
  E const &
  operator<<( E const & x, 
              error_info< ::ecto::except::detail::wrap<Tag>, T> const & v )
  {
    typedef error_info< ::ecto::except::detail::wrap<Tag>, T> error_info_tag_t;
    ::boost::shared_ptr<error_info_tag_t> p( new error_info_tag_t(v) );
    exception_detail::refcount_ptr<exception_detail::error_info_container>& c 
      = exception_detail::get_info< ::ecto::except::HACK_HACK_HACK>::get(x);
    if( !(c.get()) )
      c.adopt(new ::ecto::except::error_info_container_impl);
    c.get()->set(p,BOOST_EXCEPTION_STATIC_TYPEID(error_info_tag_t));
    return x;
  }

#define INSTANTIATE_INSERT(ETYPE, TAGTYPE)                              \
  template ::ecto::except::ETYPE const&                                 \
  operator<<(::ecto::except::ETYPE const&,                              \
             error_info< ::ecto::except::detail::wrap<BOOST_PP_CAT(::ecto::except::tag_,TAGTYPE)>, std::string> const &);

#define INSTANTIATE_INSERT_THUNK(r, SEQ)        \
  BOOST_PP_EXPAND(INSTANTIATE_INSERT BOOST_PP_SEQ_TO_TUPLE(SEQ))

  BOOST_PP_SEQ_FOR_EACH_PRODUCT(INSTANTIATE_INSERT_THUNK, ((EctoException)ECTO_EXCEPTIONS)(ECTO_EXCEPTION_TAG_NAMES));
  

}
