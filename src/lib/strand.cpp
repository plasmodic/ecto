#include <ecto/log.hpp>
#include <ecto/strand.hpp>
#include <ecto/atomic.hpp>
#include <ecto/cell.hpp>
#include <boost/unordered_map.hpp>
#include <boost/asio.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/function.hpp>

namespace ecto {

  struct strand::impl 
  {
    boost::scoped_ptr<boost::asio::io_service::strand> s;
  };

  strand::strand() : impl_(new impl) 
  { 
    ECTO_LOG_DEBUG("Created strand with id %p", id());
  }

  boost::asio::io_service::strand* 
  strand::get() const {
    return impl_->s.get();
  }

  void
  strand::set(boost::asio::io_service& serv) 
  {
    impl_->s.reset(new boost::asio::io_service::strand(serv));
  }

  std::size_t strand::id() const 
  { 
    return reinterpret_cast<std::size_t>(impl_.get()); 
  }

  bool operator==(const strand& lhs, const strand& rhs) 
  {
    return lhs.id() == rhs.id();
  }

  std::size_t strand_hash::operator()(const strand& s) const
  {
    return s.id();
  }

  // FIXME encapsulate
  /*
  typedef ecto::atomic<boost::unordered_map<ecto::strand,
                                            boost::shared_ptr<boost::asio::io_service::strand>,
                                            ecto::strand_hash> > strands_t;
  strands_t& strands();
  */
  void on_strand(cell_ptr c, boost::asio::io_service& s, boost::function<void()> h)
  {
    if (c->strand_) {
      ECTO_LOG_DEBUG("Yup %s should have a strand", c->name());
      //strands_t::scoped_lock l(strands());

      //      const ecto::strand& skey = *(c->strand_);
      //      ECTO_LOG_DEBUG("skey @ %p", &skey);
      //      boost::shared_ptr<boost::asio::io_service::strand>& strand_p = l.value[skey];
      boost::scoped_ptr<boost::asio::io_service::strand>& thestrand = c->strand_->impl_->s;
      if (!thestrand)
        {
          thestrand.reset(new boost::asio::io_service::strand(s));
          ECTO_LOG_DEBUG("Allocated new strand %p for %s", &thestrand->get_io_service() % c->name());
        }
      else
        {
          ECTO_LOG_DEBUG("strand matches, %p ??? %p", &thestrand->get_io_service() % &s);
          ECTO_ASSERT(&thestrand->get_io_service() == &s,
                      "Hmm, this strand thinks it should be on a different io_service");
        }
      ECTO_LOG_DEBUG("Cell %s posting via strand %p to %p", c->name() % c->strand_->id() % thestrand.get());
      thestrand->post(h);
    } else {
      s.post(h);
    }
  }
}
