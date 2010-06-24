//-------------------------------------------------------------------------------------------
/*! \file    in_port_tmpl.h
    \brief   libskyai - specialize for TInPortTemplate <TReturn(TArgument1,TArgument2,...)>
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
      BOOST_PP_CAT(TInPortTemplate,FUNC_OBJ_NUM_ARGS)
      // TInPortTemplate2


template <typename t_return  FUNC_OBJ_COMMA  FUNC_OBJ_TEMPLATE_PARAMS >
class  FUNC_OBJ_N_CLASS_NAME : public TInPortInterface
{
public:
  typedef TInPortInterface  TParent;
  typedef TOutPortTemplate<
            t_return (FUNC_OBJ_TEMPLATE_ARGS) >  TConnectableOut;

  typedef typename skyai_detail::TPortConnector<TConnectableOut>
                        ::TConnectedPortSet
                        ::const_iterator          TConnectedPortIterator;

  typedef t_return     TReturn;
  FUNC_OBJ_ARGMENTS_TYPEDEFS

  FUNC_OBJ_N_CLASS_NAME (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=1) :
      TParent (v_outer_base, v_name, v_max_connection_size)
    {}

  /*!\brief Connect v_port to this port (return true if successful) */
  override bool Connect (TPortInterface &v_port)
    {
      return port_connector_.Connect (v_port, *this, &dynamic_caster);
    }

  /*!\brief Disconnect the connection with the port specified by port_ptr (disconnected:true) */
  override bool Disconnect (const TPortInterface *port_ptr)
    {
      return port_connector_.Disconnect (port_ptr);
    }

  override int ConnectionSize() const  {return port_connector_.ConnectedPorts.size();}

  /*!\brief for each connected port, apply the function f
      \note if the return of the function f is false, the iteration is finished immidiately,
            else the iteration is continued.  */
  override void ForEachConnectedPort (boost::function<bool(TPortInterface*)> f)
    {
      port_connector_.ForEachConnectedPort (f);
    }

  /*!\brief for each connected port, apply the function f
      \note if the return of the function f is false, the iteration is finished immidiately,
            else the iteration is continued.  */
  override void ForEachConnectedPort (boost::function<bool(const TPortInterface*)> f) const
    {
      port_connector_.ForEachConnectedPort (f);
    }

  TConnectedPortIterator  ConnectedPortBegin () const  {return port_connector_.ConnectedPorts.begin();}
  TConnectedPortIterator  ConnectedPortEnd () const  {return port_connector_.ConnectedPorts.end();}
  TConnectedPortIterator  ConnectedPortFind (const std::string &unique_code) const  {return port_connector_.FindByUniqueCode(unique_code);}

  t_return GetFirst (FUNC_OBJ_FUNC_PARAMS) const
    {
      return GetCurrent (ConnectedPortBegin()  FUNC_OBJ_COMMA  FUNC_OBJ_FUNC_ARGS);
    }

  // virtual t_return GetCurrent (FUNC_OBJ_FUNC_PARAMS) const
  t_return GetCurrent (TConnectedPortIterator current_itr
                            FUNC_OBJ_COMMA FUNC_OBJ_FUNC_PARAMS) const
    {
      if (current_itr==ConnectedPortEnd())
      {
        LERROR("invalid port iterator");
        lexit(df); return dummy_return<t_return>::value();
      }
      if (outer_base_.ModuleMode()==TModuleInterface::mmDebug)
        {outer_base_.DebugStream()<<"IN-PORT: "<<UniqueCode()<<" (((( "<<(*current_itr)->UniqueCode()<<std::endl;}
      return (*current_itr)->Get(FUNC_OBJ_FUNC_ARGS);
    }

protected:

  skyai_detail::TPortConnector<TConnectableOut>  port_connector_;

  //! dynamic_cast from TPortInterface v_port to TConnectableOut
  static TConnectableOut* dynamic_caster (TPortInterface &v_port, const TPortInterface &v_this_port)
    {
      if (TConnectableOut *p=
            dynamic_cast< TConnectableOut* >(&v_port))
      {
        return p;
      }
      else
        {LERROR("the port "<<v_port.UniqueCode()<<" cannot be connected to the port "<<v_this_port.UniqueCode());}
      return NULL;
    }

};
//-------------------------------------------------------------------------------------------


//! partial specialization of TInPortTemplate
template <typename t_return  FUNC_OBJ_COMMA  FUNC_OBJ_TEMPLATE_PARAMS >
class TInPortTemplate < t_return (FUNC_OBJ_TEMPLATE_ARGS) >
  : public FUNC_OBJ_N_CLASS_NAME < t_return  FUNC_OBJ_COMMA  FUNC_OBJ_TEMPLATE_ARGS >
{
public:
  typedef TInPortInterface  TParent;
  // typedef t_signature  TSignature;

  TInPortTemplate (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=1) :
      FUNC_OBJ_N_CLASS_NAME < t_return
              FUNC_OBJ_COMMA
              FUNC_OBJ_TEMPLATE_ARGS >         (v_outer_base, v_name, v_max_connection_size)
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
