//-------------------------------------------------------------------------------------------
/*! \file    function_obj_head.h
    \brief   libskyai - define some macros to generate function objects
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

#define FUNC_OBJ_NUM_ARGS BOOST_PP_ITERATION()


#if FUNC_OBJ_NUM_ARGS==0
#  define FUNC_OBJ_COMMA
#else
#  define FUNC_OBJ_COMMA ,
#endif

#define FUNC_OBJ_TEMPLATE_PARAMS  \
      BOOST_PP_ENUM_PARAMS(FUNC_OBJ_NUM_ARGS, typename t_argument)
      // typename t_argument0, typename t_argument1

#define FUNC_OBJ_ARGMENT_TYPEDEF(Z,I,D)  \
      typedef BOOST_PP_CAT(t_argument,I)  BOOST_PP_CAT(TArgument,I);
#define FUNC_OBJ_ARGMENTS_TYPEDEFS  \
      BOOST_PP_REPEAT_FROM_TO(1, FUNC_OBJ_NUM_ARGS, FUNC_OBJ_ARGMENT_TYPEDEF, BOOST_PP_EMPTY)
      // typedef t_argument0  TArgument0;
      // typedef t_argument1  TArgument1;

#define FUNC_OBJ_TEMPLATE_ARGS  \
      BOOST_PP_ENUM_PARAMS(FUNC_OBJ_NUM_ARGS, t_argument)
      // t_argument0, t_argument1

#define FUNC_OBJ_FUNC_PARAMS  \
      BOOST_PP_ENUM_BINARY_PARAMS(FUNC_OBJ_NUM_ARGS, t_argument, v_arg)
      // t_argument0 v_arg0, t_argument1 v_arg1
#define FUNC_OBJ_FUNC_ARGS  \
      BOOST_PP_ENUM_PARAMS(FUNC_OBJ_NUM_ARGS, v_arg)
      // v_arg0, v_arg1

//-------------------------------------------------------------------------------------------
