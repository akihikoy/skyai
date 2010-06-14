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

static TPortInterface* through (TPortInterface &v_port, const TPortInterface &)
{
  return &v_port;
}
//-------------------------------------------------------------------------------------------

/*! Management class of port connections
    \note If a port using this class is out-port or slot-port, let t_connectable_port==TPortInterface.
        Because, their role is just providing Get, Exec to in-port, out-port, respectively.
        That is, they do not call the functions. */
template <typename t_connectable_port>
struct TPortConnector
{
  typedef std::list<t_connectable_port*>  TConnectedPortSet;
  TConnectedPortSet  ConnectedPorts;

  /*!\brief Connect v_port to v_this_port (return true if successful) */
  bool Connect (TPortInterface &v_port,
                const TPortInterface &v_this_port,
                t_connectable_port* (*dynamic_caster)(TPortInterface&, const TPortInterface &))
    {
      if (static_cast<int>(ConnectedPorts.size()) < v_this_port.MaxConnectionSize())
      {
        if (t_connectable_port *p= dynamic_caster (v_port, v_this_port))
        {
          ConnectedPorts.push_back (p);
          return true;
        }
      }
      else
        {LERROR("too many ports are connected to the port "<<v_this_port.UniqueCode());}

      /*!\note if *this is already connected to the v_port, then disconnect it
          to prevent 'dead' connections  */
      Disconnect (&v_port);             // Disconnect v_this_port ---> v_port
      v_port.Disconnect (&v_this_port); // Disconnect v_port ---> v_this_port

      /*DEBUG*/lexit(df);
      return false;
    }

  /*!\brief Disconnect the connection with the port specified by port_ptr (disconnected:true) */
  bool Disconnect (const TPortInterface *port_ptr)
    {
      bool res(false);
      for (typename TConnectedPortSet::iterator itr(ConnectedPorts.begin()); itr!=ConnectedPorts.end(); /*do nothing*/)
        if (*itr==port_ptr)  {itr= ConnectedPorts.erase(itr); res=true;}
        else                 {++itr;}
      return res;
    }

  /*!\brief for each connected port, apply the function f
      \note if the return of the function f is false, the iteration is finished immidiately,
            else the iteration is continued.  */
  void ForEachConnectedPort (boost::function<bool(TPortInterface*)> f)
    {
      for (typename TConnectedPortSet::iterator itr(ConnectedPorts.begin()); itr!=ConnectedPorts.end(); ++itr)
        if (!f(*itr))  break;
    }

  /*!\brief for each connected port, apply the function f
      \note if the return of the function f is false, the iteration is finished immidiately,
            else the iteration is continued.  */
  void ForEachConnectedPort (boost::function<bool(const TPortInterface*)> f) const
    {
      for (typename TConnectedPortSet::const_iterator itr(ConnectedPorts.begin()); itr!=ConnectedPorts.end(); ++itr)
        if (!f(*itr))  break;
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
