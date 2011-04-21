#include <ecto/ecto.hpp>
#include <iostream>
#include <fstream>

#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

struct Image
{
  std::vector<int> image_data;
  size_t width, height;
  Image(size_t width = 0, size_t height = 0) :
    image_data(width * height), width(width), height(height)
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

  Image im(20, 40);

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
    std::cout << newim.width << " " << newim.height << std::endl;
    // archive and stream closed when destructors are called
  }

  return 0;
}
