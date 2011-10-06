/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
//do not include this in ecto lib files, only in client modules

#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>

//ecto includes
#include <ecto/version.hpp>
#include <ecto/abi.hpp>
#include <ecto/cell.hpp>
#include <ecto/util.hpp>
#include <ecto/traits.hpp>
#include <iostream>

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


