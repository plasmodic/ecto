#pragma once

// This header should get you what you need for ECTO_DEFINE_MODULE

#include <ecto/python.hpp>
#include <ecto/registry.hpp>

#define ECTO_ASSERT_MODULE_NAME(MODULE)                                 \
  template <unsigned T>  void module_must_be_named_##MODULE();          \
  extern template void module_must_be_named_##MODULE<MODULE##_ectomodule_EXPORTS>();

#define ECTO_DEFINE_MODULE(modname)                                     \
  ECTO_INSTANTIATE_REGISTRY(modname)                                    \
  ECTO_ASSERT_MODULE_NAME(modname)                                      \
  void init_module_##modname##_rest() ;                                 \
  BOOST_PYTHON_MODULE(modname) {                                        \
    ECTO_REGISTER(modname);                                             \
    init_module_##modname##_rest();                                     \
  }                                                                     \
  void init_module_##modname##_rest()


