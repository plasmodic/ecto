#include <ecto/tags.hpp>
#include <algorithm>
namespace ecto
{
  namespace tags
  {
    void
    tags::tag(ptr c)
    {
      tags_[c->key()] = c;
    }
    void
    tags::tag(const tags_base& c)
    {
      tag(c.clone());
    }
    ptr
    tags::get_tag(const char * key) const
    {
      tag_map::const_iterator it = tags_.find(key);
      if (it != tags_.end())
        return it->second;
      return ptr();
    }
    ptr
    tags::get_tag(const tags_base& c) const
    {
      return get_tag(c.key());
    }
    tags&
    tags::operator%(const tags_base& c)
    {
      tag(c);
      return *this;
    }

    struct inserter
    {
      typedef std::map<std::string, ptr> tag_map;
      inserter(tags tm)
          :
            tm_(tm)
      {
      }
      void
      operator()(const tag_map::value_type& v)
      {
        tm_.tag(v.second);
      }
      tags& tm_;
    };

    tags&
    tags::operator%(const tags& c)
    {
      std::for_each(c.tags_.begin(), c.tags_.end(), inserter(*this));
      return *this;
    }
  }
}
