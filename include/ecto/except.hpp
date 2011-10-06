#pragma once


#define ECTO_EXCEPTION_TAG_NAMES                                        \
  (from_typename)(to_typename)(from_key)(to_key)(from_cell)             \
  (to_cell)(cpp_typename)(pyobject_repr)(actualtype_hint)(spore_typename) \
  (diag_msg)(actualkeys_hint)(tendril_key)(cell_name)(function_name)    \
  (hint)(which_tendrils)(prev_typename)(cur_typename)(type)             \
  (what)(when)

#define ECTO_EXCEPTIONS                                                 \
    (TypeMismatch)(ValueNone)(ValueRequired)(NonExistant)               \
    (FailedFromPythonConversion)(TendrilRedeclaration)(CellException)   \
    (NotConnected)(AlreadyConnected)(NullTendril)

// these are to speed up the preprocessor metaprogramming
# ifndef BOOST_PREPROCESSOR_CONFIG_LIMITS_HPP
# define BOOST_PREPROCESSOR_CONFIG_LIMITS_HPP
#
# define ECTO_PP_ITERLIMIT 22
# define BOOST_PP_LIMIT_MAG ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_TUPLE 25
# define BOOST_PP_LIMIT_DIM 3
# define BOOST_PP_LIMIT_REPEAT ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_WHILE ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_FOR ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_ITERATION ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_ITERATION_DIM 2
# define BOOST_PP_LIMIT_SEQ ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_SLOT_SIG 10
# define BOOST_PP_LIMIT_SLOT_COUNT 5
#
# endif

#include <boost/version.hpp>
#if BOOST_VERSION <= 104000
#include "except-1.40.hpp"
#else
#include "except-1.42.hpp"
#endif
