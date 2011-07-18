#include <vector>
#include <iostream>
#include <ecto/util.hpp>
#if !defined(_WIN32)
#include <cxxabi.h>
#endif
#include <string>
#include <stdlib.h>
#include <cstring>
#include <map>
#include <boost/thread/mutex.hpp>
#include <boost/tuple/tuple.hpp>

#if defined(_WIN32)
// #define UNDNAME_COMPLETE                 (0x0000)
// #define UNDNAME_NO_LEADING_UNDERSCORES   (0x0001) /* Don't show __ in calling convention */
// #define UNDNAME_NO_MS_KEYWORDS           (0x0002) /* Don't show calling convention at all */
// #define UNDNAME_NO_FUNCTION_RETURNS      (0x0004) /* Don't show function/method return value */
// #define UNDNAME_NO_ALLOCATION_MODEL      (0x0008)
// #define UNDNAME_NO_ALLOCATION_LANGUAGE   (0x0010)
// #define UNDNAME_NO_MS_THISTYPE           (0x0020)
// #define UNDNAME_NO_CV_THISTYPE           (0x0040)
// #define UNDNAME_NO_THISTYPE              (0x0060)
// #define UNDNAME_NO_ACCESS_SPECIFIERS     (0x0080) /* Don't show access specifier (public/protected/private) */
// #define UNDNAME_NO_THROW_SIGNATURES      (0x0100)
// #define UNDNAME_NO_MEMBER_TYPE           (0x0200) /* Don't show static/virtual specifier */
// #define UNDNAME_NO_RETURN_UDT_MODEL      (0x0400)
// #define UNDNAME_32_BIT_DECODE            (0x0800)
// #define UNDNAME_NAME_ONLY                (0x1000) /* Only report the variable/method name */
// #define UNDNAME_NO_ARGUMENTS             (0x2000) /* Don't show method arguments */
// #define UNDNAME_NO_SPECIAL_SYMS          (0x4000)
// #define UNDNAME_NO_COMPLEX_TYPE          (0x8000)

// extern "C"
// char * _unDName(
    // char * outputString,
    // const char * name,
    // int maxStringLength,
    // void * (* pAlloc )(size_t),
    // void (* pFree )(void *),
    // unsigned short disableFlags);
	// _unDName(0, pName, 0, malloc, free, UNDNAME_NO_ARGUMENTS | UNDNAME_32_BIT_DECODE);

namespace abi
{
	char* __cxa_demangle(const char * const pName,...)
	{
		char * name = new char[std::strlen(pName)];
		std::strcpy(name,pName);
		return name;
	}
}
#endif

namespace ecto
{
  struct type_mapping
  {
    type_mapping()
    {
    }
    const std::string&
    lookup(const std::type_info& ti)
    {
      const char* mangled = ti.name();
      if (!mangled)
      {
        throw std::runtime_error("Could get a type name for your type! The world must be ending.");
      }
      boost::mutex::scoped_lock l(mtx);
      std::string rv;
      bool inserted;
      Hashes::iterator it;
      boost::tie(it, inserted) = hashes.insert(std::make_pair<std::string, std::string>(mangled, rv));
      if (inserted)
      {
        {
          int status=0;
          char* demangled = abi::__cxa_demangle(mangled, 0, 0, &status);
          if (status != 0)
            rv = mangled;
          else
            rv = demangled;
          free(demangled);
        }
        it->second = rv;
        //std::cout << "looking up id for : " << rv << " inserted " << size_t(it->second.c_str()) << std::endl;
      }
      return it->second;
    }
    typedef std::map<std::string, std::string> Hashes;
    boost::mutex mtx;
    Hashes hashes;
  };
  const std::string& name_of(const std::type_info &ti)
  {
    static type_mapping TypeMasterIndex;
    return TypeMasterIndex.lookup(ti);
  }
}

