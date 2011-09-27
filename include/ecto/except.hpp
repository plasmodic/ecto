#pragma once
#include <boost/version.hpp>
#if BOOST_VERSION <= 104000
#include "except-1.40.hpp"
#else
#include "except-1.42.hpp"
#endif
