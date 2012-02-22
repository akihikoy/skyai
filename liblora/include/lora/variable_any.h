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
#include <lora/cast.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
namespace var_space
{
//-------------------------------------------------------------------------------------------

extern pt_string AnyPrimitiveIdentifier;

//===========================================================================================
class TAnyPrimitive
//===========================================================================================
{
public:
  enum TType {ptInt=0, ptReal, ptBool, ptString};

  TAnyPrimitive() : type_(static_cast<int>(ptInt)), e_int_(0) {}
  TAnyPrimitive(const pt_int    &x) : type_(static_cast<int>(ptInt   )), e_int_   (x) {}
  TAnyPrimitive(const pt_real   &x) : type_(static_cast<int>(ptReal  )), e_real_  (x) {}
  TAnyPrimitive(const pt_bool   &x) : type_(static_cast<int>(ptBool  )), e_bool_  (x) {}
  TAnyPrimitive(const pt_string &x) : type_(static_cast<int>(ptString)), e_string_(x) {}

  const pt_int& Type() const {return type_;}
  void SetType(pt_int t)  {type_= t;}
  pt_int& SetType()  {return type_;}

  void Set(const pt_int    &x)  {type_=static_cast<int>(ptInt   ); e_int_   = x;}
  void Set(const pt_real   &x)  {type_=static_cast<int>(ptReal  ); e_real_  = x;}
  void Set(const pt_bool   &x)  {type_=static_cast<int>(ptBool  ); e_bool_  = x;}
  void Set(const pt_string &x)  {type_=static_cast<int>(ptString); e_string_= x;}

  const pt_int&    Int    () const {return e_int_   ;}
  const pt_real&   Real   () const {return e_real_  ;}
  const pt_bool&   Bool   () const {return e_bool_  ;}
  const pt_string& String () const {return e_string_;}

private:
  pt_int type_;
  union
  {
    pt_int       e_int_;
    pt_real      e_real_;
    pt_bool      e_bool_;
  };
  pt_string      e_string_;

};
//-------------------------------------------------------------------------------------------

std::ostream& operator<< (std::ostream &lhs, const TAnyPrimitive &rhs);
//-------------------------------------------------------------------------------------------

inline void CastToBool(TAnyPrimitive &value)
{
  switch(value.Type())
  {
  case TAnyPrimitive::ptInt :     value.Set(lora_cast<pt_bool>(value.Int())); break;
  case TAnyPrimitive::ptReal :    value.Set(lora_cast<pt_bool>(value.Real())); break;
  case TAnyPrimitive::ptBool :    break;
  case TAnyPrimitive::ptString :  value.Set(lora_cast<pt_bool>(value.String())); break;
  default : FIXME("fatal value type: "<<static_cast<int>(value.Type()));
  }
}

inline void CastToBool(TAnyPrimitive &lhs, TAnyPrimitive &rhs)
{
  CastToBool(lhs);
  CastToBool(rhs);
}

inline void Cast2(TAnyPrimitive &lhs, TAnyPrimitive &rhs)
{
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :
    switch(rhs.Type())
    {
    case TAnyPrimitive::ptInt :     break;
    case TAnyPrimitive::ptReal :    lhs.Set(lora_cast<pt_real>(lhs.Int())); break;
    case TAnyPrimitive::ptBool :    rhs.Set(lora_cast<pt_int>(rhs.Bool())); break;
    case TAnyPrimitive::ptString :  lhs.Set(lora_cast<pt_string>(lhs.Int())); break;
    default : FIXME("fatal value type: "<<static_cast<int>(rhs.Type()));
    }
    break;
  case TAnyPrimitive::ptReal :
    switch(rhs.Type())
    {
    case TAnyPrimitive::ptInt :     rhs.Set(lora_cast<pt_real>(rhs.Int())); break;
    case TAnyPrimitive::ptReal :    break;
    case TAnyPrimitive::ptBool :    rhs.Set(lora_cast<pt_real>(rhs.Bool())); break;
    case TAnyPrimitive::ptString :  lhs.Set(lora_cast<pt_string>(lhs.Real())); break;
    default : FIXME("fatal value type: "<<static_cast<int>(rhs.Type()));
    }
    break;
  case TAnyPrimitive::ptBool :
    switch(rhs.Type())
    {
    case TAnyPrimitive::ptInt :     lhs.Set(lora_cast<pt_int>(lhs.Bool())); break;
    case TAnyPrimitive::ptReal :    lhs.Set(lora_cast<pt_real>(lhs.Bool())); break;
    case TAnyPrimitive::ptBool :    break;
    case TAnyPrimitive::ptString :  lhs.Set(lora_cast<pt_string>(lhs.Bool())); break;
    default : FIXME("fatal value type: "<<static_cast<int>(rhs.Type()));
    }
    break;
  case TAnyPrimitive::ptString :
    switch(rhs.Type())
    {
    case TAnyPrimitive::ptInt :     rhs.Set(lora_cast<pt_string>(rhs.Int())); break;
    case TAnyPrimitive::ptReal :    rhs.Set(lora_cast<pt_string>(rhs.Real())); break;
    case TAnyPrimitive::ptBool :    rhs.Set(lora_cast<pt_string>(rhs.Bool())); break;
    case TAnyPrimitive::ptString :  break;
    default : FIXME("fatal value type: "<<static_cast<int>(rhs.Type()));
    }
    break;
  default : FIXME("fatal value type: "<<static_cast<int>(lhs.Type()));
  }
}

inline TAnyPrimitive operator+(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  Cast2(lhs,rhs);
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :     return TAnyPrimitive(lhs.Int()    + rhs.Int());
  case TAnyPrimitive::ptReal :    return TAnyPrimitive(lhs.Real()   + rhs.Real());
  case TAnyPrimitive::ptBool :    LERROR("operator+ is not available for bool"); break;
  case TAnyPrimitive::ptString :  return TAnyPrimitive(lhs.String() + rhs.String());
  }
  return TAnyPrimitive();
}
inline TAnyPrimitive operator-(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  Cast2(lhs,rhs);
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :     return TAnyPrimitive(lhs.Int()    - rhs.Int());
  case TAnyPrimitive::ptReal :    return TAnyPrimitive(lhs.Real()   - rhs.Real());
  case TAnyPrimitive::ptBool :    LERROR("operator- is not available for bool"); break;
  case TAnyPrimitive::ptString :  return TAnyPrimitive(lhs.String() - rhs.String());
  }
  return TAnyPrimitive();
}
inline TAnyPrimitive operator*(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  if(lhs.Type()==TAnyPrimitive::ptString && rhs.Type()==TAnyPrimitive::ptInt)
    return TAnyPrimitive(loco_rabbits::operator*(lhs.String(),rhs.Int()));
  if(lhs.Type()==TAnyPrimitive::ptInt && rhs.Type()==TAnyPrimitive::ptString)
    return TAnyPrimitive(loco_rabbits::operator*(lhs.Int(),rhs.String()));
  Cast2(lhs,rhs);
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :     return TAnyPrimitive(lhs.Int()    * rhs.Int());
  case TAnyPrimitive::ptReal :    return TAnyPrimitive(lhs.Real()   * rhs.Real());
  case TAnyPrimitive::ptBool :    LERROR("operator* is not available for bool"); break;
  case TAnyPrimitive::ptString :  LERROR("operator* is not available for string"); break;
  }
  return TAnyPrimitive();
}
inline TAnyPrimitive operator/(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  Cast2(lhs,rhs);
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :     return TAnyPrimitive(lhs.Int()    / rhs.Int());
  case TAnyPrimitive::ptReal :    return TAnyPrimitive(lhs.Real()   / rhs.Real());
  case TAnyPrimitive::ptBool :    LERROR("operator/ is not available for bool"); break;
  case TAnyPrimitive::ptString :  LERROR("operator/ is not available for string"); break;
  }
  return TAnyPrimitive();
}
inline TAnyPrimitive operator%(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  Cast2(lhs,rhs);
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :     return TAnyPrimitive(lhs.Int()    % rhs.Int());
  case TAnyPrimitive::ptReal :    return TAnyPrimitive(real_fmod(lhs.Real(),rhs.Real()));
  case TAnyPrimitive::ptBool :    LERROR("operator% is not available for bool"); break;
  case TAnyPrimitive::ptString :  LERROR("operator% is not available for string"); break;
  }
  return TAnyPrimitive();
}
inline TAnyPrimitive operator&&(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  CastToBool(lhs,rhs);
  return TAnyPrimitive(lhs.Bool() && rhs.Bool());
}
inline TAnyPrimitive operator||(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  CastToBool(lhs,rhs);
  return TAnyPrimitive(lhs.Bool() || rhs.Bool());
}
inline TAnyPrimitive operator!(TAnyPrimitive rhs)
{
  CastToBool(rhs);
  return TAnyPrimitive(!rhs.Bool());
}
inline TAnyPrimitive operator==(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  Cast2(lhs,rhs);
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :     return TAnyPrimitive(lhs.Int()    == rhs.Int());
  case TAnyPrimitive::ptReal :    return TAnyPrimitive(lhs.Real()   == rhs.Real());
  case TAnyPrimitive::ptBool :    return TAnyPrimitive(lhs.Bool()   == rhs.Bool());
  case TAnyPrimitive::ptString :  return TAnyPrimitive(lhs.String() == rhs.String());
  }
  return TAnyPrimitive();
}
inline TAnyPrimitive operator!=(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  Cast2(lhs,rhs);
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :     return TAnyPrimitive(lhs.Int()    != rhs.Int());
  case TAnyPrimitive::ptReal :    return TAnyPrimitive(lhs.Real()   != rhs.Real());
  case TAnyPrimitive::ptBool :    return TAnyPrimitive(lhs.Bool()   != rhs.Bool());
  case TAnyPrimitive::ptString :  return TAnyPrimitive(lhs.String() != rhs.String());
  }
  return TAnyPrimitive();
}
inline TAnyPrimitive operator<=(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  Cast2(lhs,rhs);
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :     return TAnyPrimitive(lhs.Int()    <= rhs.Int());
  case TAnyPrimitive::ptReal :    return TAnyPrimitive(lhs.Real()   <= rhs.Real());
  case TAnyPrimitive::ptBool :    LERROR("operator<= is not available for bool"); break;
  case TAnyPrimitive::ptString :  return TAnyPrimitive(lhs.String() <= rhs.String());
  }
  return TAnyPrimitive();
}
inline TAnyPrimitive operator>=(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  Cast2(lhs,rhs);
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :     return TAnyPrimitive(lhs.Int()    >= rhs.Int());
  case TAnyPrimitive::ptReal :    return TAnyPrimitive(lhs.Real()   >= rhs.Real());
  case TAnyPrimitive::ptBool :    LERROR("operator>= is not available for bool"); break;
  case TAnyPrimitive::ptString :  return TAnyPrimitive(lhs.String() >= rhs.String());
  }
  return TAnyPrimitive();
}
inline TAnyPrimitive operator<(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  Cast2(lhs,rhs);
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :     return TAnyPrimitive(lhs.Int()    < rhs.Int());
  case TAnyPrimitive::ptReal :    return TAnyPrimitive(lhs.Real()   < rhs.Real());
  case TAnyPrimitive::ptBool :    LERROR("operator< is not available for bool"); break;
  case TAnyPrimitive::ptString :  return TAnyPrimitive(lhs.String() < rhs.String());
  }
  return TAnyPrimitive();
}
inline TAnyPrimitive operator>(TAnyPrimitive lhs, TAnyPrimitive rhs)
{
  Cast2(lhs,rhs);
  switch(lhs.Type())
  {
  case TAnyPrimitive::ptInt :     return TAnyPrimitive(lhs.Int()    > rhs.Int());
  case TAnyPrimitive::ptReal :    return TAnyPrimitive(lhs.Real()   > rhs.Real());
  case TAnyPrimitive::ptBool :    LERROR("operator> is not available for bool"); break;
  case TAnyPrimitive::ptString :  return TAnyPrimitive(lhs.String() > rhs.String());
  }
  return TAnyPrimitive();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
//-------------------------------------------------------------------------------------------

template <> inline const std::string ConvertToStr (const var_space::TAnyPrimitive &val)
{
  using namespace var_space;
  switch(static_cast<TAnyPrimitive::TType>(val.Type()))
  {
  case TAnyPrimitive::ptInt    : return ConvertToStr(val.Int()   ); break;
  case TAnyPrimitive::ptReal   : return ConvertToStr(val.Real()  ); break;
  case TAnyPrimitive::ptBool   : return ConvertToStr(val.Bool()  ); break;
  case TAnyPrimitive::ptString : return ConvertToStr(val.String()); break;
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
