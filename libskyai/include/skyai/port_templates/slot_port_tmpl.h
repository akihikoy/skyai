//-------------------------------------------------------------------------------------------
/*! \file    slot_port_tmpl.h
    \brief   libskyai - specialize for TSlotPortTemplate <TReturn(TArgument1,TArgument2,...)>
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.20, 2009-

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
      BOOST_PP_CAT(TSlotPortTemplate,FUNC_OBJ_NUM_ARGS)
      // TSlotPortTemplate2


template <typename t_return  FUNC_OBJ_COMMA  FUNC_OBJ_TEMPLATE_PARAMS >
class  FUNC_OBJ_N_CLASS_NAME : public TSlotPortInterface
{
public:
  typedef TSlotPortInterface  TParent;

  typedef typename skyai_detail::TPortConnector<TPortInterface>
                        ::TConnectedPortSet
                        ::const_iterator          TConnectedPortIterator;

  typedef t_return     TReturn;
  FUNC_OBJ_ARGMENTS_TYPEDEFS

  FUNC_OBJ_N_CLASS_NAME (const TModuleInterface &v_outer_base, const std::string v_name,
                          int v_max_connection_size=SKYAI_CONNECTION_SIZE_MAX,
                          const std::string v_forwarding_sinal_port_name="")
    : TParent (v_outer_base, v_name, v_max_connection_size),
      forwarding_sinal_port_ (v_outer_base, (v_forwarding_sinal_port_name=="" ?
                                    get_forwarding_sinal_port_name(v_name) :
                                    v_forwarding_sinal_port_name),   v_max_connection_size)
    {}

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


  t_return Exec (FUNC_OBJ_FUNC_PARAMS)
    {
      if (outer_base_.ModuleMode()==TModuleInterface::mmDebug)
        {outer_base_.DebugStream()<<"SLOT-PORT: "<<UniqueCode()<<" <<<<"<<std::endl;}
      return exec_switcher<t_return>(*this) (FUNC_OBJ_FUNC_ARGS);
    }

  /*!\brief return the reference to the forwarding-signal-port
      that is emitted after the slot (exec_) is executed */
  override TSignalPortInterface&  ForwardingSinalPortBase()
    {
      return forwarding_sinal_port_;
    }

  const TSignalPortTemplate< void (FUNC_OBJ_TEMPLATE_ARGS) >&  forwardingSinalPort() const
    {
      return forwarding_sinal_port_;
    }

protected:

  skyai_detail::TPortConnector<TPortInterface>  port_connector_;

  //! the forwarding-signal-port that is emitted after the slot (exec_) is executed
  TSignalPortTemplate< void (FUNC_OBJ_TEMPLATE_ARGS) >  forwarding_sinal_port_;

  virtual t_return exec_ (FUNC_OBJ_TEMPLATE_ARGS) = 0;

  //! get the name of the forwarding-signal-port
  static std::string get_forwarding_sinal_port_name (const std::string v_slot_port_name)
    {
      return v_slot_port_name+std::string("_finished");
    }

  /*!\par Purpose of exec_switcher
      This is needed to define "signal forwarding".
      The execution process is as follows:
        - 1. exec_ is called when this slot port catches a signal
        - 2. after executing exec_, forwarding_sinal_port_.ExecAll is called
        - 3. finally, return the result of exec_.
      <b>Thus, we need to keep the returned value of exec_</b> during 2.
      However, if the type of the return value of exec_ (==t_return) is void,
      we cannot keep a value of void.
      Hence, we need to change the code according to t_return (void or the others).
      exec_switcher is one which fulfills that.

      By the way, a template argument t_dummy of exec_switcher is a technique to
      specialize a member function.
      (see http://d.hatena.ne.jp/aki-yam/20090827/1251368274)
  */

  //! function object to execute the slot
  template <typename t_return2, typename t_dummy=void>
  struct exec_switcher
    {
      typedef FUNC_OBJ_N_CLASS_NAME < t_return  FUNC_OBJ_COMMA  FUNC_OBJ_TEMPLATE_ARGS >
          TOuter;
      TOuter &outer;
      exec_switcher(TOuter &v_outer) : outer(v_outer) {};
      t_return2 operator() (FUNC_OBJ_FUNC_PARAMS)
        {
          t_return2 tmp (outer.exec_(FUNC_OBJ_FUNC_ARGS));
          outer.forwarding_sinal_port_.ExecAll (FUNC_OBJ_FUNC_ARGS);
          return tmp;
        }
    };
  //! function object to execute the slot, specialized to the t_return==void
  template <typename t_return2>
  struct exec_switcher <void, t_return2>
    {
      typedef FUNC_OBJ_N_CLASS_NAME < t_return  FUNC_OBJ_COMMA  FUNC_OBJ_TEMPLATE_ARGS >
          TOuter;
      TOuter &outer;
      exec_switcher(TOuter &v_outer) : outer(v_outer) {};
      void operator() (FUNC_OBJ_FUNC_PARAMS)
        {
          outer.exec_(FUNC_OBJ_FUNC_ARGS);
          outer.forwarding_sinal_port_.ExecAll (FUNC_OBJ_FUNC_ARGS);
        }
    };

private:
};
//-------------------------------------------------------------------------------------------


//! partial specialization of TSlotPortTemplate
template <typename t_return  FUNC_OBJ_COMMA  FUNC_OBJ_TEMPLATE_PARAMS >
class TSlotPortTemplate < t_return (FUNC_OBJ_TEMPLATE_ARGS) >
  : public FUNC_OBJ_N_CLASS_NAME < t_return  FUNC_OBJ_COMMA  FUNC_OBJ_TEMPLATE_ARGS >
{
public:
  typedef FUNC_OBJ_N_CLASS_NAME < t_return
                          FUNC_OBJ_COMMA
                          FUNC_OBJ_TEMPLATE_ARGS >  TParent;
  // typedef t_signature  TSignature;

  TSlotPortTemplate (const TModuleInterface &v_outer_base, const std::string v_name,
                      int v_max_connection_size=SKYAI_CONNECTION_SIZE_MAX,
                      const std::string v_forwarding_sinal_port_name="")
    : TParent  (v_outer_base, v_name, v_max_connection_size,
                  (v_forwarding_sinal_port_name=="" ?
                    TParent::get_forwarding_sinal_port_name(v_name) :
                    v_forwarding_sinal_port_name) )
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
