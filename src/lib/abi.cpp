#include <ecto/abi.hpp>
#include <iostream>

namespace ecto {
  namespace abi {
    namespace {
      bool flagged = false;
    }
    verifier::verifier(unsigned header_version)
    {
      if (!flagged && header_version != ECTO_ABI_VERSION)
        {
          std::cerr << "***************** WARNING *****************\n"
                    << "The ABI version of the ecto that you compiled against (" << header_version << ")\n"
                    << "does not match the version you are running with (" << ECTO_ABI_VERSION << "):\n"
                    << "typically this is the result of sloppy LD_LIBRARY_PATH or PYTHONPATH handling.\n"
                    << "Such version mismatches can result in very, very strange bugs.\n"
                    << "You should make the ecto you run with match the one that you compile against.\n"
                    << std::endl
            ;
          flagged = true;
        }
    }
  }
}


