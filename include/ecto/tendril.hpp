#pragma once
#include <ecto/util.hpp> //name_of

#include <boost/shared_ptr.hpp>

#include <stdexcept>
#include <string>
#include <sstream>
#include <cstring>

namespace ecto
{
  class connection
    {
    public:
      connection();

      template <typename T>
      static
      connection make(const T& t = T(), const std::string& name = std::string(), const std::string& doc = std::string());

      std::string type_name() const;
      std::string value() const;
      std::string name() const;
      std::string doc() const;

      template <typename T>
      const T& get() const;

      template <typename T>
      T& get();
      void connect(connection& rhs);

      inline bool dirty(bool b) { dirty_ = b; return dirty_; }
      inline bool dirty() const { return dirty_; }

    private:
  // ############################### NVI ####################################
      struct impl_base
      {
        typedef boost::shared_ptr<impl_base> ptr;
        virtual ~impl_base();
        virtual std::string type_name() const = 0;
        virtual void* get() = 0;
        virtual std::string value() const = 0;
        virtual const std::type_info & type_info() const = 0;

        //convience functions for checking types
        template<typename T>
        static bool inline check(impl_base& i);
        template<typename T>
        static inline void checkThrow(impl_base& i) throw(std::logic_error);
        template<typename T>
        static inline T* get(impl_base& i);

        std::string name,doc;
      };

      template <typename T>
      struct impl : impl_base
      {
        impl(const T& t);
        std::string type_name() const;
        const std::type_info & type_info() const;
        void* get();
        std::string value() const;
        T t;
      };
      connection(impl_base::ptr impl);
      boost::shared_ptr<impl_base> impl_;
      bool dirty_;
    };
#include "ecto/impl/connection_impl.hpp"
}
