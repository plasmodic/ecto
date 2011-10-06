#include <ecto/ecto.hpp>
#include <ecto/serialization/registry.hpp>
#include <ecto/serialization/cell.hpp>

ECTO_REGISTER_SERIALIZERS(std::string);

ECTO_REGISTER_SERIALIZERS(float);
ECTO_REGISTER_SERIALIZERS(double);

ECTO_REGISTER_SERIALIZERS(char);
ECTO_REGISTER_SERIALIZERS(short);
ECTO_REGISTER_SERIALIZERS(int);
ECTO_REGISTER_SERIALIZERS(long);

ECTO_REGISTER_SERIALIZERS(unsigned char);
ECTO_REGISTER_SERIALIZERS(unsigned short);
ECTO_REGISTER_SERIALIZERS(unsigned int);
ECTO_REGISTER_SERIALIZERS(unsigned long);

ECTO_REGISTER_SERIALIZERS(bool);

namespace boost
{
  namespace serialization
  {
    template<class Archive>
    void
    serialize(Archive & ar, ecto::tendril::none, const unsigned int version)
    {
    }
  } // namespace serialization
} // namespace boost

ECTO_REGISTER_SERIALIZERS(ecto::tendril::none);

ECTO_REGISTER_SERIALIZERS(ecto::cell::ptr);

namespace ecto
{
  template<class Archive>
  void
  tendril::save(Archive & ar, const unsigned int version) const
  {
    std::string type_name(type_ID_);
    ar << type_name;
    ar << doc_;
    ecto::serialization::registry<Archive>::instance().serialize(type_name, ar, const_cast<tendril&>(*this));
  }

  template<class Archive>
  void
  tendril::load(Archive & ar, const unsigned int version)
  {
    std::string type_name;
    ar >> type_name;
    ar >> doc_;
    ecto::serialization::registry<Archive>::instance().serialize(type_name, ar, *this);
  }

  //explicit instantiations
  template void tendril::load<boost::archive::text_iarchive> (boost::archive::text_iarchive & ar, const unsigned int version) ;
  template void tendril::save<boost::archive::text_oarchive> (boost::archive::text_oarchive & ar, const unsigned int version) const ;
  template void tendril::load<boost::archive::binary_iarchive> (boost::archive::binary_iarchive & ar, const unsigned int version) ;
  template void tendril::save<boost::archive::binary_oarchive> (boost::archive::binary_oarchive & ar, const unsigned int version) const;
    
  namespace serialization
  {
    template<typename Archive>
    void
    registry<Archive>::serialize(const std::string& key, Archive& ar, tendril& t) const
    {
      typename serial_map_t::const_iterator it = serial_map.find(key);
      if (it == serial_map.end())
      {
        throw std::logic_error("Could not find a serializer registered for the type: " + key);
      }
      it->second(ar, t);
    }

    template<typename Archive>
    void
    registry<Archive>::add(const std::string& name, serial_fn_t fnc)
    {
      typename serial_map_t::iterator it;
      bool inserted;
      boost::tie(it, inserted) = serial_map.insert(std::make_pair(name, fnc));
      if (!inserted)
      {
        std::cerr << "Warning: ignoring non novel serialization for " << name << "." << std::endl;
      }
    }

    template<typename Archive>
    registry<Archive>&
    registry<Archive>::instance()
    {
      static registry<Archive> instance_;
      return instance_;
    }

    template<typename Archive>
    registry<Archive>::registry()
    {
    }

    template class registry<boost::archive::binary_oarchive> ;
    template class registry<boost::archive::binary_iarchive> ;
    template class registry<boost::archive::text_oarchive> ;
    template class registry<boost::archive::text_iarchive> ;
  }
}
