//-------------------------------------------------------------------------------------------
/*! \file    out_port_tmpl.h
    \brief   libskyai - specialize for TOutPortTemplate <TReturn(TArgument1,TArgument2,...)>
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.18, 2009-

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
// DO NOT USE INCLUDE GUARD
//-------------------------------------------------------------------------------------------
#ifndef SKYAI_DUMMY_INCLUDE
//-------------------------------------------------------------------------------------------
#include "function_obj_head.h"
//-------------------------------------------------------------------------------------------
#define FUNC_OBJ_N_CLASS_NAME  \
      BOOST_PP_CAT(TOutPortTemplate,FUNC_OBJ_NUM_ARGS)
      // TOutPortTemplate2


template <typename t_return  FUNC_OBJ_COMMA  FUNC_OBJ_TEMPLATE_PARAMS >
class  FUNC_OBJ_N_CLASS_NAME : public TOutPortInterface
{
public:
  typedef TOutPortInterface  TParent;

  typedef typename skyai_detail::TPortConnector<TPortInterface>
                        ::TConnectedPortSet
                        ::const_iterator          TConnectedPortIterator;

  typedef t_return     TReturn;
  FUNC_OBJ_ARGMENTS_TYPEDEFS

  FUNC_OBJ_N_CLASS_NAME (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=SKYAI_CONNECTION_SIZE_MAX) :
      TParent (v_outer_base, v_name, v_max_connection_size)
    {}

  t_return Get (FUNC_OBJ_FUNC_PARAMS)
    {
      TActivate aco(*this);
      if (outer_base_.ModuleMode()==TModuleInterface::mmDebug)
        {outer_base_.DebugStream()<<"OUT-PORT: "<<this<<" ))))"<<std::endl;}
      first_call_check();
      return get_ (FUNC_OBJ_FUNC_ARGS);
    }


  /*!\brief Connect v_port to this port (return true if successful) */
  override bool Connect (TPortInterface &v_port)
    {
      return port_connector_.Connect (v_port, *this, &skyai_detail::through);
    }

  /*!\brief Disconnect the connection with the port specified by port_ptr (disconnected:true) */
  override bool Disconnect (const TPortInterface *port_ptr)
    {
      return port_connector_.Disconnect (port_ptr);
    }

  override int ConnectionSize() const  {return port_connector_.ConnectedPorts.Size();}

  /*!\brief for each connected port, apply the function f
      \note if the return of the function f is false, the iteration is finished immidiately,
            else the iteration is continued.  */
  override void ForEachConnectedPort (boost::function<bool(TPortInterface*)> f)
    {
      TActivate aco(*this);
      port_connector_.ForEachConnectedPort (f);
    }

  /*!\brief for each connected port, apply the function f
      \note if the return of the function f is false, the iteration is finished immidiately,
            else the iteration is continued.  */
  override void ForEachConnectedPort (boost::function<bool(const TPortInterface*)> f) const
    {
      TActivate aco(*this);
      port_connector_.ForEachConnectedPort (f);
    }

  TConnectedPortIterator  ConnectedPortBegin () const  {return port_connector_.ConnectedPorts.Begin();}
  TConnectedPortIterator  ConnectedPortEnd () const  {return port_connector_.ConnectedPorts.End();}
  TConnectedPortIterator  ConnectedPortFind (const TPortInterface *ptr) const  {return port_connector_.FindByPtr(ptr);}
  void  ConnectedPortLock ()   {port_connector_.ConnectedPorts.Lock();}
  void  ConnectedPortUnlock ()   {port_connector_.ConnectedPorts.Unlock();}

protected:

  virtual t_return get_ (FUNC_OBJ_TEMPLATE_ARGS) = 0;

  skyai_detail::TPortConnector<TPortInterface>  port_connector_;

private:
};
//-------------------------------------------------------------------------------------------


//! partial specialization of TOutPortTemplate
template <typename t_return  FUNC_OBJ_COMMA  FUNC_OBJ_TEMPLATE_PARAMS >
class TOutPortTemplate < t_return (FUNC_OBJ_TEMPLATE_ARGS) >
  : public FUNC_OBJ_N_CLASS_NAME < t_return  FUNC_OBJ_COMMA  FUNC_OBJ_TEMPLATE_ARGS >
{
public:
  typedef FUNC_OBJ_N_CLASS_NAME < t_return
                          FUNC_OBJ_COMMA
                          FUNC_OBJ_TEMPLATE_ARGS >  TParent;
  // typedef t_signature  TSignature;

  TOutPortTemplate (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=SKYAI_CONNECTION_SIZE_MAX)
    : TParent (v_outer_base, v_name, v_max_connection_size)
    {}

protected:
private:
};
//-------------------------------------------------------------------------------------------


#undef FUNC_OBJ_N_CLASS_NAME
//-------------------------------------------------------------------------------------------
#include "function_obj_tail.h"
//-------------------------------------------------------------------------------------------
#endif  // SKYAI_DUMMY_INCLUDE
//-------------------------------------------------------------------------------------------
