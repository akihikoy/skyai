//-------------------------------------------------------------------------------------------
/*! \file    port_generators.h
    \brief   libskyai - macros to generate ports
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

    -----------------------------------------------------------------------------------------

    \note use these macros within protected section in a subclass of TModuleInterface
*/
//-------------------------------------------------------------------------------------------
#ifndef skyai_port_generators_h
#define skyai_port_generators_h
//-------------------------------------------------------------------------------------------

//! obtain a port type from its instance generated by following macros
#define GET_PORT_TYPE(x_port_name)  type_port_##x_port_name

#define MAKE_IN_PORT_SPECIFIC(x_port_name,x_signature,x_type_module,x_max_connection_size)  \
  class type_port_##x_port_name                                         \
        : public TInPortTemplate<x_signature>                           \
    {                                                                   \
    public:                                                             \
      typedef TInPortTemplate<x_signature> TParent;                     \
      const x_type_module   &outer;                                     \
      type_port_##x_port_name (x_type_module &v_outer) :                \
          TParent    (v_outer, #x_port_name, x_max_connection_size),    \
          outer      (v_outer)                                          \
        {}                                                              \
    } x_port_name
#define MAKE_IN_PORT(x_port_name,x_signature,x_type_module)  \
  MAKE_IN_PORT_SPECIFIC(x_port_name,x_signature,x_type_module,1)

#define MAKE_OUT_PORT_SPECIFIC(x_port_name,x_return_type,x_arg_list,x_param_list,x_type_module,x_max_connection_size)  \
  class type_port_##x_port_name                                         \
        : public TOutPortTemplate < x_return_type x_arg_list >          \
    {                                                                   \
    public:                                                             \
      typedef TOutPortTemplate < x_return_type x_arg_list > TParent;    \
      const x_type_module   &outer;                                     \
      type_port_##x_port_name (x_type_module &v_outer) :                \
          TParent    (v_outer, #x_port_name, x_max_connection_size),    \
          outer      (v_outer)                                          \
        {}                                                              \
    protected:                                                          \
      override x_return_type get_ x_arg_list const                      \
        {                                                               \
          return outer.x_port_name##_get x_param_list;                  \
        }                                                               \
    } x_port_name
#define MAKE_OUT_PORT(x_port_name,x_return_type,x_arg_list,x_param_list,x_type_module)  \
  MAKE_OUT_PORT_SPECIFIC(x_port_name,x_return_type,x_arg_list,x_param_list,x_type_module,SKYAI_CONNECTION_SIZE_MAX)


#define MAKE_SLOT_PORT_SPECIFIC(x_port_name,x_return_type,x_arg_list,x_param_list,x_type_module,x_max_connection_size,x_forwarding_sinal_port_name)  \
  class type_port_##x_port_name                                                                     \
        : public TSlotPortTemplate < x_return_type x_arg_list >                                     \
    {                                                                                               \
    public:                                                                                         \
      typedef TSlotPortTemplate < x_return_type x_arg_list > TParent;                               \
      x_type_module  &outer;                                                                        \
      type_port_##x_port_name (x_type_module &v_outer) :                                            \
          TParent (v_outer, #x_port_name, x_max_connection_size, x_forwarding_sinal_port_name),     \
          outer (v_outer)                                                                           \
        {}                                                                                          \
    protected:                                                                                      \
      override x_return_type  exec_  x_arg_list                                                     \
        {                                                                                           \
          return outer.x_port_name##_exec x_param_list;                                             \
        }                                                                                           \
    } x_port_name
#define MAKE_SLOT_PORT(x_port_name,x_return_type,x_arg_list,x_param_list,x_type_module) \
  MAKE_SLOT_PORT_SPECIFIC(x_port_name,x_return_type,x_arg_list,x_param_list,x_type_module,SKYAI_CONNECTION_SIZE_MAX,"")


#define MAKE_SIGNAL_PORT_SPECIFIC(x_port_name,x_signature,x_type_module,x_max_connection_size)  \
  class type_port_##x_port_name                                      \
        : public TSignalPortTemplate < x_signature >                 \
    {                                                                \
    public:                                                          \
      typedef TSignalPortTemplate < x_signature > TParent;           \
      const x_type_module  &outer;                                   \
      type_port_##x_port_name (x_type_module &v_outer) :             \
          TParent (v_outer, #x_port_name, x_max_connection_size),    \
          outer (v_outer)                                            \
        {}                                                           \
    } x_port_name
#define MAKE_SIGNAL_PORT(x_port_name,x_signature,x_type_module)  \
  MAKE_SIGNAL_PORT_SPECIFIC(x_port_name,x_signature,x_type_module,SKYAI_CONNECTION_SIZE_MAX)


//-------------------------------------------------------------------------------------------
#endif // skyai_port_generators_h
//-------------------------------------------------------------------------------------------
