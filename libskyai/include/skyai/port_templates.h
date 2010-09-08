//-------------------------------------------------------------------------------------------
/*! \file    port_templates.h
    \brief   libskyai - port templates
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.24, 2009-

    Copyright (C) 2009, 2010  Akihiko Yamaguchi

    This file is part of SkyAI.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//-------------------------------------------------------------------------------------------
#ifndef skyai_port_templates
#define skyai_port_templates
//-------------------------------------------------------------------------------------------
#include <skyai/base.h>
#include <boost/preprocessor.hpp>
//-------------------------------------------------------------------------------------------
#define SKYAI_DUMMY_INCLUDE
// these includings are for dependency analysis of a compiler
#  include <skyai/port_templates/out_port_tmpl.h>
#  include <skyai/port_templates/in_port_tmpl.h>
#  include <skyai/port_templates/signal_port_tmpl.h>
#  include <skyai/port_templates/slot_port_tmpl.h>
#undef SKYAI_DUMMY_INCLUDE
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

#ifndef SKYAI_FUNCTION_MAX_ARGS
#  define SKYAI_FUNCTION_MAX_ARGS 10
#endif


//-------------------------------------------------------------------------------------------
namespace skyai_detail
{
//-------------------------------------------------------------------------------------------

/*!\brief list that has a robust iterator for erasing and pushing elements;
          defined for TConnectedPortSet, whose elements may be erased during scanning its elements
    \note push-backed elements cannot be accessed unless execute Unlock */
template<typename T>
class TErasableList
{
public:
  class TElem
    {
    public:
      TElem(const T &x) : entity_(x), erased_(false) {}
      bool operator==(const TElem &rhs)
        {
          if(erased_||rhs.erased_)  return false;
          return entity_==rhs.entity_;
        }
      bool operator==(const T &rhs)
        {
          if(erased_)  return false;
          return entity_==rhs;
        }
      operator       T&()       {LASSERT(!erased_); return entity_;}
      operator const T&() const {LASSERT(!erased_); return entity_;}
            T& Entity()       {LASSERT(!erased_); return entity_;}
      const T& Entity() const {LASSERT(!erased_); return entity_;}
      bool Erased() const {return erased_;}
    private:
      T entity_;
      bool erased_;
    friend class TErasableList<T>;
    };
  typedef typename std::list<TElem>::iterator        iterator;
  typedef typename std::list<TElem>::const_iterator  const_iterator;

  TErasableList(void) : locked_(false), erased_size_(0) {}

  void PushBack(const T &x)
    {
      if(locked_ && locked_end_==entity_.end())
      {
        LASSERT(locked_c_end_==entity_.end());
        entity_.push_back(TElem(x));
        locked_end_=entity_.end();
        --locked_end_;
        locked_c_end_=entity_.end();
        --locked_c_end_;
      }
      else
        entity_.push_back(TElem(x));
    }

  iterator        Begin()  {return entity_.begin();}
  const_iterator  Begin() const {return entity_.begin();}
  iterator        End()  {return locked_ ? locked_end_ : entity_.end();}
  const_iterator  End() const {return locked_ ? locked_c_end_ : entity_.end();}

  iterator Erase(iterator i)
    {
      if(locked_)  {i->erased_=true; ++erased_size_; ++i; return i;}
      else         {return entity_.erase(i);}
    }
  void Lock()
    {
      locked_=true;
      locked_end_=entity_.end();
      locked_c_end_=entity_.end();
    }
  void Unlock()
    {
      cleanup();
      locked_=false;
    }
  iterator Find(const T &x) {return std::find(Begin(),End(),x);}
  size_t Size() const {return entity_.size()-erased_size_;}

private:
  std::list<TElem>  entity_;
  bool locked_;
  int  erased_size_;
  iterator        locked_end_;
  const_iterator  locked_c_end_;

  void cleanup()
    {
      iterator itr(Begin());
      while(itr!=End())
      {
        if(itr->erased_)  {itr=entity_.erase(itr); --erased_size_;}
        else  ++itr;
      }
      LASSERT1op1(erased_size_,==,0);
    }
};
//-------------------------------------------------------------------------------------------

/*!\brief defined for the dynamic_caster argument of Connect function */
static TPortInterface* through (TPortInterface &v_port, const TPortInterface &)
{
  return &v_port;
}
//-------------------------------------------------------------------------------------------

/*!\brief Management class of port connections
    \note If a port using this class is out-port or slot-port, let t_connectable_port==TPortInterface.
        Because, their role is just providing Get, Exec to in-port, out-port, respectively.
        That is, they do not call the functions. */
template <typename t_connectable_port>
struct TPortConnector
{
  typedef TErasableList<t_connectable_port*>  TConnectedPortSet;
  TConnectedPortSet  ConnectedPorts;

  /*!\brief Connect v_port to v_this_port (return true if successful) */
  bool Connect (TPortInterface &v_port,
                const TPortInterface &v_this_port,
                t_connectable_port* (*dynamic_caster)(TPortInterface&, const TPortInterface &))
    {
      if (static_cast<int>(ConnectedPorts.Size()) < v_this_port.MaxConnectionSize())
      {
        if (t_connectable_port *p= dynamic_caster (v_port, v_this_port))
        {
          ConnectedPorts.PushBack (p);
          return true;
        }
      }
      else
        {LERROR("too many ports are connected to the port "<<&v_this_port);}

      /*!\note if *this is already connected to the v_port, then disconnect it
          to prevent 'dead' connections  */
      Disconnect (&v_port);             // Disconnect v_this_port ---> v_port
      v_port.Disconnect (&v_this_port); // Disconnect v_port ---> v_this_port

      //*DEBUG*/lexit(df);
      return false;
    }

  /*!\brief Disconnect the connection with the port specified by port_ptr (disconnected:true) */
  bool Disconnect (const TPortInterface *port_ptr)
    {
      bool res(false);
      for (typename TConnectedPortSet::iterator itr(ConnectedPorts.Begin()); itr!=ConnectedPorts.End(); /*do nothing*/)
        if (!itr->Erased() && *itr==port_ptr)  {itr= ConnectedPorts.Erase(itr); res=true;}
        else                                   {++itr;}
      return res;
    }

  /*!\brief for each connected port, apply the function f
      \note if the return of the function f is false, the iteration is finished immidiately,
            else the iteration is continued.  */
  void ForEachConnectedPort (boost::function<bool(TPortInterface*)> f)
    {
      for (typename TConnectedPortSet::iterator itr(ConnectedPorts.Begin()); itr!=ConnectedPorts.End(); ++itr)
        {if (!itr->Erased())  {if (!f(*itr))  break;}}
    }

  /*!\brief for each connected port, apply the function f
      \note if the return of the function f is false, the iteration is finished immidiately,
            else the iteration is continued.  */
  void ForEachConnectedPort (boost::function<bool(const TPortInterface*)> f) const
    {
      for (typename TConnectedPortSet::const_iterator itr(ConnectedPorts.Begin()); itr!=ConnectedPorts.End(); ++itr)
        {if (!itr->Erased())  {if (!f(*itr))  break;}}
    }

  /*!\brief find a connected port by a pointer */
  typename TConnectedPortSet::const_iterator FindByPtr (const TPortInterface *ptr) const
    {
      for (typename TConnectedPortSet::const_iterator itr(ConnectedPorts.Begin()); itr!=ConnectedPorts.End(); ++itr)
        if (!itr->Erased() && *itr==ptr)  return itr;
      return ConnectedPorts.End();
    }
};
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace skyai_detail
//-------------------------------------------------------------------------------------------


template <typename t_signature>
class TOutPortTemplate : public TOutPortInterface
{
protected:
  //! this function is protected to prevent an instance of non-specialized object is generated
  TOutPortTemplate (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=SKYAI_CONNECTION_SIZE_MAX) :
      TOutPortInterface (v_outer_base, v_name, v_max_connection_size)
    {}
};
// specialize for TOutPortTemplate <TReturn(TArgument1,TArgument2,...)>
// NOTE: to execute the following iteration, `-I.' option for gcc is required
#define BOOST_PP_ITERATION_LIMITS (0,SKYAI_FUNCTION_MAX_ARGS)
#define BOOST_PP_FILENAME_1 <skyai/port_templates/out_port_tmpl.h>
#include BOOST_PP_ITERATE()
//-------------------------------------------------------------------------------------------

template <typename t_signature>
class TInPortTemplate : public TInPortInterface
{
protected:
  //! this function is protected to prevent an instance of non-specialized object is generated
  TInPortTemplate (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=1) :
      TInPortInterface (v_outer_base, v_name, v_max_connection_size)
    {}
};
// specialize for TInPortTemplate <TReturn(TArgument1,TArgument2,...)>
// NOTE: to execute the following iteration, `-I.' option for gcc is required
#define BOOST_PP_ITERATION_LIMITS (0,SKYAI_FUNCTION_MAX_ARGS)
#define BOOST_PP_FILENAME_1 <skyai/port_templates/in_port_tmpl.h>
#include BOOST_PP_ITERATE()
//-------------------------------------------------------------------------------------------


template <typename t_signature>
class TSlotPortTemplate : public TSlotPortInterface
{
protected:
  //! this function is protected to prevent an instance of non-specialized object is generated
  TSlotPortTemplate (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=SKYAI_CONNECTION_SIZE_MAX) :
      TSlotPortInterface (v_outer_base, v_name, v_max_connection_size)
    {}
};

template <typename t_signature>
class TSignalPortTemplate : public TSignalPortInterface
{
protected:
  //! this function is protected to prevent an instance of non-specialized object is generated
  TSignalPortTemplate (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=SKYAI_CONNECTION_SIZE_MAX) :
      TSignalPortInterface (v_outer_base, v_name, v_max_connection_size)
    {}
};

// specialize for TSignalPortTemplate <TReturn(TArgument1,TArgument2,...)>
// NOTE: to execute the following iteration, `-I.' option for gcc is required
#define BOOST_PP_ITERATION_LIMITS (0,SKYAI_FUNCTION_MAX_ARGS)
#define BOOST_PP_FILENAME_1 <skyai/port_templates/signal_port_tmpl.h>
#include BOOST_PP_ITERATE()

// specialize for TSlotPortTemplate <TReturn(TArgument1,TArgument2,...)>
// NOTE: to execute the following iteration, `-I.' option for gcc is required
#define BOOST_PP_ITERATION_LIMITS (0,SKYAI_FUNCTION_MAX_ARGS)
#define BOOST_PP_FILENAME_1 <skyai/port_templates/slot_port_tmpl.h>
#include BOOST_PP_ITERATE()
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_port_templates
//-------------------------------------------------------------------------------------------
