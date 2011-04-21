#include <ecto/ecto.hpp>
#include <iostream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>


//struct PointCloud
//{
//  template<typename PointType>
//    const std::string& type_name() const
//    {
//      static const std::string _typename_ = typeid(PointType).name();
//      return _typename_;
//    }
//
//  template<typename PointType>
//  std::vector<PointType>& get()
//  {
//    add();
//    boost::shared_ptr<std::vector<PointType> > v =  boost::any_cast<boost::shared_ptr<std::vector<PointType> > >(fields_[type_name<PointType>()]);
//    return *v;
//  }
//
//  template<typename PointType>
//  void add()
//  {
//    if(fields_.count(type_name<PointType>()) == 0)
//      fields_[type_name<PointType>()] = boost::shared_ptr<std::vector<PointType> >(new std::vector<PointType>);
//  }
//
//  std::map<std::string, boost::any> fields_;
//
//  // When the class Archive corresponds to an output archive, the
//  // & operator is defined similar to <<.  Likewise, when the class Archive
//  // is a type of input archive the & operator is defined similar to >>.
//  template<class Archive>
//  void serialize(Archive & ar, const unsigned int version)
//  {
//      BOOST_FOREACH(boost::any field, fields_)
//      {
//
//      }
//  }
//
//};

struct Image
{
  std::vector<int> image_data;
  size_t width, height;
  Image(size_t width = 0, size_t height = 0):image_data(width*height),width(width),height(height)
  {
  }
  inline int & at(int u, int v)
  {
    return image_data[v * width + u];
  }
  inline const int & at(int u, int v) const
  {
    return image_data[v * width + u];
  }
  template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      ar & width;
      ar & height;
      ar & image_data;
    }
};

int main()
{


    Image im(20,40);

  // save data to archive
      {
      std::ofstream ofs("image.ppm");
          boost::archive::text_oarchive oa(ofs);
          // write class instance to archive
          oa << im;
          // archive and stream closed when destructors are called
      }

      // ... some time later restore the class instance to its orginal state
      Image newim;
      {
          // create and open an archive for input
          std::ifstream ifs("image.ppm");
          boost::archive::text_iarchive ia(ifs);
          // read class state from archive
          ia >> newim;
          std::cout << newim.width << " "<< newim.height << std::endl;
          // archive and stream closed when destructors are called
      }

  return 0;
}
