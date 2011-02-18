#include <vector>
#include <iostream>

#include <boost/random/mersenne_twister.hpp>
#include <boost/crc.hpp>

const std::vector<uint32_t>& gen_data(unsigned len)
{
  //std::cout << __PRETTY_FUNCTION__ << " " << len << "\n";
  
  static std::vector<uint32_t> v;
  v.resize(len);
  boost::mt19937 rng;

  for (unsigned j = 0; j<len-1; ++j)
    {
      v[j] = rng();
    }

  boost::crc_32_type crc;
  crc.process_block(v.data(), v.data() + v.size());

  v[len-1] = crc.checksum();
  return v;
}
