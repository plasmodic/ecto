// as detailed on https://svn.boost.org/trac/boost/ticket/4816
/// 1st issue
#include <boost/asio/detail/pipe_select_interrupter.hpp>

/// 2nd issue
#ifdef __CYGWIN__
#include <termios.h>
#ifdef cfgetospeed
#define __cfgetospeed__impl(tp) cfgetospeed(tp)
#undef cfgetospeed
inline speed_t cfgetospeed(const struct termios *tp)
{
	return __cfgetospeed__impl(tp);
}
#undef __cfgetospeed__impl
#endif /// cfgetospeed is a macro

/// 3rd issue
#undef __CYGWIN__ 
#include <boost/asio/detail/buffer_sequence_adapter.hpp>
#define __CYGWIN__
#endif
