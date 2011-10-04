#pragma once
#include <ecto/cell.hpp>
#include <ecto/registry.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost
{
  namespace serialization
  {
    template<class Archive>
    inline void
    save(Archive & ar, const boost::shared_ptr<ecto::cell> &cell_, const unsigned int file_version)
    {
      std::string type_str = cell_->type();
      ar << type_str;
      std::string instance_name = cell_->name();
      ar << instance_name;
      ar << cell_->parameters << cell_->inputs << cell_->outputs;
    }

    template<class Archive>
    inline void
    load(Archive & ar, boost::shared_ptr<ecto::cell> &cell_, const unsigned int file_version)
    {
      std::string cell_type;
      ar >> cell_type;
      ecto::registry::entry_t e = ecto::registry::lookup(cell_type);
      ecto::cell::ptr p = e.construct();
      cell_.swap(p);
      std::string instance_name;
      ar >> instance_name;
      cell_->name(instance_name);
      ar >> cell_->parameters >> cell_->inputs >> cell_->outputs;
    }
  }
}

