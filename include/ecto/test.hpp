#pragma once

namespace ecto {
  namespace test {
    void random_delay();
    template <typename T>
    T get_from_env_with_default(const char* name, T defval);

    extern const unsigned max_delay;
  }
}

#ifdef ECTO_STRESS_TEST
#define ECTO_RANDOM_DELAY() ::ecto::test::random_delay()
#else
#define ECTO_RANDOM_DELAY()
#endif

