//-------------------------------------------------------------------------------------------
/*! \file    variable_any.cpp
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
#include <lora/variable_any.h>
#include <lora/variable_space_impl.h>
#include <lora/string_impl.h>  // NumericalContainerToString
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{
using namespace std;
// using namespace boost;


pt_string AnyPrimitiveIdentifier("@");

//-------------------------------------------------------------------------------------------

template <typename t_to>
void any_converter_generator_pr(const TAnyPrimitive *from, t_to &to);
template <typename t_from>
void any_converter_generator_rp(const t_from &from, TAnyPrimitive *to);

#define ANY_CONVERTER_GEN(x_type)  \
  template <>                                                                    \
  void any_converter_generator_pr(const TAnyPrimitive *from, x_type &to)         \
  {                                                                              \
    switch(static_cast<TAnyPrimitive::TType>(from->Type()))                      \
    {                                                                            \
    case TAnyPrimitive::ptInt    : to= lora_cast<x_type>(from->Int()   ); break; \
    case TAnyPrimitive::ptReal   : to= lora_cast<x_type>(from->Real()  ); break; \
    case TAnyPrimitive::ptBool   : to= lora_cast<x_type>(from->Bool()  ); break; \
    case TAnyPrimitive::ptString : to= lora_cast<x_type>(from->String()); break; \
    default : LERROR("fatal!"); lexit(df);                                       \
    }                                                                            \
  }                                                                              \
  template <>                                                                    \
  void any_converter_generator_rp(const x_type &from, TAnyPrimitive *to)         \
  {                                                                              \
    to->Set(lora_cast<x_type>(from));                                            \
  }
ANY_CONVERTER_GEN(pt_int   )
ANY_CONVERTER_GEN(pt_real  )
ANY_CONVERTER_GEN(pt_bool  )
ANY_CONVERTER_GEN(pt_string)
#undef ANY_CONVERTER_GEN
//-------------------------------------------------------------------------------------------

TVariable any_get_member_generator (const TVariableMap &members, const TVariable &id)
{
  TIdentifier sid;
  id.PrimitiveGetAs<TIdentifier>(sid);
  TVariableMap::const_iterator itr (members.find(sid));
  if (itr==members.end())
    {LERROR("does not have a member: "<<sid); return TVariable();}
  return itr->second;
}
//-------------------------------------------------------------------------------------------

void any_direct_assign_generator(TAnyPrimitive *x, TVariableMap &members, const TVariable &var)
{
  TIdentifier sid("-");
  TVariable id(sid);
  if(!var.IsPrimitive() || var.NoMember() || var.GetMember(id).PrimitiveGetAs<pt_string>()!=AnyPrimitiveIdentifier)
  {
    x->Set(var.PrimitiveGetAs<pt_string>());
    return;
  }
  sid="t";
  x->SetType(var.GetMember(id).PrimitiveGetAs<pt_int>());
  switch(static_cast<TAnyPrimitive::TType>(x->Type()))
  {
  case TAnyPrimitive::ptInt    : x->Set(var.PrimitiveGetAs<pt_int   >()); break;
  case TAnyPrimitive::ptReal   : x->Set(var.PrimitiveGetAs<pt_real  >()); break;
  case TAnyPrimitive::ptBool   : x->Set(var.PrimitiveGetAs<pt_bool  >()); break;
  case TAnyPrimitive::ptString : x->Set(var.PrimitiveGetAs<pt_string>()); break;
  default : LERROR("fatal!"); lexit(df);
  }
}
//-------------------------------------------------------------------------------------------

void any_write_to_stream_generator (TAnyPrimitive *x, const TVariableMap &, std::ostream &os, bool, const pt_string&)
{
  os<<*x;
}
void any_write_to_binary_generator (TAnyPrimitive *x, const TVariableMap &, TBinaryStack &bstack)
{
  switch(static_cast<TAnyPrimitive::TType>(x->Type()))
  {
  case TAnyPrimitive::ptInt    : AddPushLiteral(bstack,x->Int()   ); break;
  case TAnyPrimitive::ptReal   : AddPushLiteral(bstack,x->Real()  ); break;
  case TAnyPrimitive::ptBool   : AddPushLiteral(bstack,x->Bool()  ); break;
  case TAnyPrimitive::ptString : AddPushLiteral(bstack,x->String()); break;
  default : LERROR("fatal!"); lexit(df);
  }
}
//-------------------------------------------------------------------------------------------


#define GET_AS(x_var)  boost::bind(any_converter_generator_pr<x_var>,&x,_1)
#define SET_BY(x_var)  boost::bind(any_converter_generator_rp<x_var>,_1,&x)
void TVariable::generator<TAnyPrimitive>::operator() (TAnyPrimitive &x)
{
  o.is_primitive_ = true;
  o.f_primitive_get_as_int_     = GET_AS(pt_int    );
  o.f_primitive_get_as_real_    = GET_AS(pt_real   );
  o.f_primitive_get_as_bool_    = GET_AS(pt_bool   );
  o.f_primitive_get_as_string_  = GET_AS(pt_string );

  o.f_primitive_set_by_int_     = SET_BY(pt_int    );
  o.f_primitive_set_by_real_    = SET_BY(pt_real   );
  o.f_primitive_set_by_bool_    = SET_BY(pt_bool   );
  o.f_primitive_set_by_string_  = SET_BY(pt_string );

  o.AddMemberVariable<pt_string>("-",AnyPrimitiveIdentifier);
  o.AddMemberVariable<pt_int>("t",x.SetType());

  o.f_get_member_ =  any_get_member_generator;

  o.f_direct_assign_ = boost::bind(any_direct_assign_generator,&x,_1,_2);
  o.f_write_to_stream_ = boost::bind(any_write_to_stream_generator,&x,_1,_2,_3,_4);
  o.f_write_to_binary_ = boost::bind(any_write_to_binary_generator,&x,_1,_2);
}
//-------------------------------------------------------------------------------------------
#undef GET_AS
#undef SET_BY

//===========================================================================================

#define SPECIALIZER(x_type)  \
  template<> void list_write_to_stream_generator (std::list<x_type> *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent) \
    {os<<"("<<NumericalContainerToString(*x, ", ")<<")";}
#define SPECIALIZERI(x_type)  \
  SPECIALIZER(x_type) \
  template struct TVariable::generator<std::list<x_type> >;
SPECIALIZERI(TAnyPrimitive   )
#undef SPECIALIZERI
#undef SPECIALIZER

//===========================================================================================

std::ostream& operator<< (std::ostream &lhs, const TAnyPrimitive &rhs)
{
  lhs<<ConvertToStr(rhs);
  return lhs;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

