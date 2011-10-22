#include <ecto/except.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>

using namespace boost;

#define ECTO_STRESS_TEST
#ifdef ECTO_STRESS_TEST
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/lexical_cast.hpp>

namespace ecto {
  namespace test {

    template <typename T>
    T get_from_env_with_default(const char* name, T defval)
    {
      const char* envval = getenv(name);
      if (!envval)
        return defval;

      T casted = boost::lexical_cast<T>(envval);
      return casted;
    }

    template unsigned get_from_env_with_default(const char*, unsigned);
    template int get_from_env_with_default(const char*, int);

    const unsigned max_delay = get_from_env_with_default("ECTO_MAX_DELAY", 10000); // 10 mst
    const unsigned min_delay = get_from_env_with_default("ECTO_MIN_DELAY", 10); // 10 mst
    const unsigned delay_seed = get_from_env_with_default("ECTO_DELAY_SEED", time(0));

    struct tls {
  
      boost::mt19937 gen;
      boost::uniform_int<unsigned> dist;
  
      tls() : gen(delay_seed), dist(0, max_delay) { }
  
      inline void rndsleep() {
        unsigned dur = dist(gen);
        if (max_delay > 0 && dur >= min_delay)
          {
            std::cout << "dur=" << dur << "\n";
            usleep(dur);
          }
      }
    };

    void random_delay() 
    {
      static boost::thread_specific_ptr<ecto::test::tls> bp;
      if (__builtin_expect(!bp.get(), 0))
        bp.reset(new ecto::test::tls);
      bp->rndsleep();
    }

  }
}

#endif
