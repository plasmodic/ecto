#include <ecto/profile.hpp>
#if !defined(_WIN32)
#include <unistd.h>
#endif

namespace ecto {
  namespace profile {

#if defined(__i386__)

    unsigned long read_tsc(void)
    {
      unsigned long tsc;
      asm(".byte 0x0f, 0x31" : "=A" (tsc));
      return( tsc );
    }
 
#elif (defined(__amd64__) || defined(__x86_64__))
 
    unsigned long read_tsc(void)
    {
      unsigned long lo, hi;
      asm( "rdtsc" : "=a" (lo), "=d" (hi) );
      return( lo | (hi << 32) );
    }
 
#elif (defined(__powerpc__) || defined(__ppc__))
 
    unsigned long read_tsc(void)
    {
      unsigned long tbl, tbu0, tbu1;
 
      do
        {
          asm( "mftbu %0" : "=r" (tbu0) );
          asm( "mftb  %0" : "=r" (tbl ) );
          asm( "mftbu %0" : "=r" (tbu1) );
        }
      while( tbu0 != tbu1 );
 
      return( tbl );
    }
 
#elif defined(__sparc__)
 
    unsigned long read_tsc(void)
    {
      unsigned long tick;
      asm( ".byte 0x83, 0x41, 0x00, 0x00" );
      asm( "mov   %%g1, %0" : "=r" (tick) );
      return( tick );
    }
 
#elif defined(__alpha__)
 
    unsigned long read_tsc(void)
    {
      unsigned long cc;
      asm( "rpcc %0" : "=r" (cc) );
      return( cc & 0xFFFFFFFF );
    }
 
#elif defined(__ia64__)
 
    unsigned long read_tsc(void)
    {
      unsigned long itc;
      asm( "mov %0 = ar.itc" : "=r" (itc) );
      return( itc );
    }
 
#else
#pragma warning "Unable to determine how to talk to cpu instruction counter, internal instrumentation will not work"
    
    unsigned long read_tsc(void)
    {
      return 0;
    }

#endif

    double elapsed_time(const stats_type& stats)
    {
#if !defined(_WIN32)
      double ticks_second = static_cast<double>(sysconf(_SC_CLK_TCK));
      return stats.total_ticks / ticks_second;
#else
	  return 1.0;
#endif
    }

    double period(const stats_type& stats)
    {
      return elapsed_time(stats)/elapsed_time(stats);
    }

    double frequency(const stats_type& stats)
    {
      return stats.ncalls/elapsed_time(stats);
    }


  }
}
