//-------------------------------------------------------------------------------------------
/*! \file    variable_any.h
    \brief   liblora - variable-space : "any primitive" that can hold any type of a primitive type (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.10, 2010

    Copyright (C) 2010  Akihiko Yamaguchi

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
#ifndef loco_rabbits_variable_any_h
#define loco_rabbits_variable_any_h
//-------------------------------------------------------------------------------------------
#include <lora/variable_space_fwd.h>
#include <lora/string.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
namespace var_space
{
//-------------------------------------------------------------------------------------------

extern pt_string AnyPrimitiveIdentifier;

//===========================================================================================
struct TAnyPrimitive
//===========================================================================================
{
  enum TType {ptInt=0, ptReal, ptBool, ptString};
  pt_int Type;
  union
  {
    pt_int       EInt;
    pt_real      EReal;
    pt_bool      EBool;
  };
  pt_string      EString;

  TAnyPrimitive() : Type(static_cast<int>(ptInt)), EInt(0) {}
  TAnyPrimitive(const pt_int    &x) : Type(static_cast<int>(ptInt   )), EInt   (x) {}
  TAnyPrimitive(const pt_real   &x) : Type(static_cast<int>(ptReal  )), EReal  (x) {}
  TAnyPrimitive(const pt_bool   &x) : Type(static_cast<int>(ptBool  )), EBool  (x) {}
  TAnyPrimitive(const pt_string &x) : Type(static_cast<int>(ptString)), EString(x) {}
  void Set(const pt_int    &x)  {Type=static_cast<int>(ptInt   ); EInt   = x;}
  void Set(const pt_real   &x)  {Type=static_cast<int>(ptReal  ); EReal  = x;}
  void Set(const pt_bool   &x)  {Type=static_cast<int>(ptBool  ); EBool  = x;}
  void Set(const pt_string &x)  {Type=static_cast<int>(ptString); EString= x;}
};
//-------------------------------------------------------------------------------------------

std::ostream& operator<< (std::ostream &lhs, const TAnyPrimitive &rhs);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
//-------------------------------------------------------------------------------------------

template <> inline const std::string ConvertToStr (const var_space::TAnyPrimitive &val)
{
  using namespace var_space;
  switch(static_cast<TAnyPrimitive::TType>(val.Type))
  {
  case TAnyPrimitive::ptInt    : return ConvertToStr(val.EInt   ); break;
  case TAnyPrimitive::ptReal   : return ConvertToStr(val.EReal  ); break;
  case TAnyPrimitive::ptBool   : return ConvertToStr(val.EBool  ); break;
  case TAnyPrimitive::ptString : return ConvertToStr(val.EString); break;
  default : LERROR("fatal!"); lexit(df);
  }
  return "";
}
template <> inline const var_space::TAnyPrimitive ConvertFromStr (const std::string &str)
{
  using namespace var_space;
  FIXME("not implemented yet");
  return TAnyPrimitive();
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_any_h
//-------------------------------------------------------------------------------------------
