#pragma once
#include <boost/python.hpp>
#include <ecto/util.hpp>
#include <boost/shared_ptr.hpp>
#include <numeric>
#include <map>
#include <string>
namespace ecto
{
  /**
   * \brief tags are useful for attaching meta data to tendril objects, for runtime
   * introspection purposes (i.e. constructing a gui to manipulate ecto cells).
   */
  namespace tags
  {
    /**
     * \brief All tags have type unique keys, a clone method, and a boost::python
     * extraction function.
     */
    struct tags_base
    {
      virtual
      ~tags_base()
      {
      }
      const char*
      key() const
      {
        return typeid(*this).name();
      }

      virtual
      boost::python::object
      extract() const = 0;

      virtual
      boost::shared_ptr<tags_base>
      clone() const = 0;
    };

    typedef boost::shared_ptr<tags_base> ptr;

    /**
     * \brief A typed tag, useful for storing tags of particular types in the
     * tendril.
     *
     * The value function may be used to get the value that this tag holds. The python
     * extraction is also generated for the type.
     *
     * \tparam T The type to hold.
     */
    template<typename T>
    struct tag_: tags_base
    {
      virtual
      ~tag_()
      {
      }

      const T& value() const{
        return val_;
      }

      boost::python::object
      extract() const
      {
        return boost::python::object(val_);
      }

      T val_;
      typedef boost::shared_ptr<tag_<T> > ptr;
    };

    /**
     * \brief Using CRTP, this allows easily clonable tags to be constructed.
     */
    template<typename T, typename U>
    struct tag_CRTP_: tag_<U>
    {
      boost::shared_ptr<tags_base>
      clone() const
      {
        boost::shared_ptr<tags_base> p(new T(*thiz()));
        return p;
      }
      const T* thiz() const
      {
        return static_cast<const T*>(this);
      }
    };

    /**
     * \brief tags is a collection for tag types. It supports typeless and typed
     * operations that are safe, and should be the prefered method of holding onto
     * tags.
     */
    struct tags
    {
      /**
       * Stores a tag that has been dynamically allocated.
       * @param c The tag to store.
       */
      void tag(ptr c);
      /**
       * Stores a tag from a reference. This will clone the tag.
       * @param c Tag to be cloned and stored.
       */
      void tag(const tags_base& c);

      /**
       * Retrieve a tag based on the key.  One should use the tag_base::key() method
       * to get this key.
       * @param key A type encoding unique key.
       * @return A shared pointer to the tag. This will be null if there is no tag stored by the given key.
       */
      ptr get_tag(const char * key) const;
      /**
       * Retrieve a key, given a canonical instance of the type of tag to retrieve. This should
       * @param c
       * @return key A type encoding unique key.
       * @return A shared pointer to the tag. This will be null if there is no tag stored by the given key.

       */
      ptr get_tag(const tags_base& c) const;

      /**
       * Given a fully typed tag_<T> this will grab a const ref to the value held by the tag.
       * @tparam T the type that the tag_ is based on.
       * @param _c a fully typed tag_<T> which will be used to deduce the type of T.
       * @return A const ref to the value held by the tag.
       */
      template <typename T>
      const T& tagged(const tag_<T>& _c) const
      {
        ptr cp = get_tag(_c);
        if(!cp)
          return _c.value();
        return static_cast<tag_<T>*>(cp.get())->value();
      }

      /**
       * Syntactic sugar to add a tag to the collection.
       * @param c the tag to add, will be cloned, same as calling tags::tag(c)
       * @return *this
       */
      tags& operator%(const tags_base& c);
      /**
       * Syntactic sugar to append a tags collection onto this tags collection.
       * @param c the tags to add, they will over write any existing same typed tags.
       * @return *this
       */
      tags& operator%(const tags& c);

      typedef std::map<std::string, ptr> tag_map;
      tag_map tags_;
    };

  }
}


