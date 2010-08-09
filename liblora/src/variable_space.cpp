//-------------------------------------------------------------------------------------------
/*! \file    variable_space.cpp
    \brief   liblora - variable-space : generic manipulators of variables  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    May.17, 2010-

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
#include <lora/variable_space.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
#include <lora/string.h>
#include <lora/stl_ext.h>
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{
using namespace std;
// using namespace boost;


//===========================================================================================
// function object generators
//===========================================================================================

//-------------------------------------------------------------------------------------------
// generators generators for struct types (i.e. variable space)
//-------------------------------------------------------------------------------------------

void struct_direct_assign_generator (TVariableMap &members, const TVariable &value)
{
  for (TVariableMap::iterator itr(members.begin()),last(members.end()); itr!=last; ++itr)
  {
    TIdentifier sid(itr->first);
    TVariable id(sid);
    itr->second.SetMember(id,value.GetMember(id));
  }
}
//-------------------------------------------------------------------------------------------

TVariable struct_get_member_generator (const TVariableMap &members, const TVariable &id)
{
  TIdentifier sid;
  id.PrimitiveGetAs<TIdentifier>(sid);
  TVariableMap::const_iterator itr (members.find(sid));
  if (itr==members.end())
    {VAR_SPACE_ERR_EXIT("does not have a member: "<<sid);}
  return itr->second;
}
//-------------------------------------------------------------------------------------------

void struct_set_member_generator (const TVariableMap &members, const TVariable &id, const TVariable &value)
{
  struct_get_member_generator(members,id).DirectAssign(value);
}
//-------------------------------------------------------------------------------------------

void struct_write_to_stream_generator (const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent)
{
  pt_string new_indent;
  if (!bare)
  {
    os<< "{" << std::endl;
    new_indent= indent+VAR_SPACE_INDENT_STEP;
  }
  else
    new_indent= indent;
  for (TVariableMap::const_iterator itr(members.begin()),last(members.end()); itr!=last; ++itr)
  {
    os<< new_indent << itr->first << " = ";
    itr->second.WriteToStream(os, false, new_indent+VAR_SPACE_INDENT_STEP);
    os<< std::endl;
  }
  if (!bare)
    os<< indent << "}";
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// generators for generic types
//-------------------------------------------------------------------------------------------

void generic_function_call_generator (const TVariableMap &members, const TIdentifier &name, TVariableList &argv)
{
  TVariableMap::const_iterator itr (members.find(name));
  if (itr==members.end())
    {VAR_SPACE_ERR_EXIT("does not have a function: "<<name);}
  return itr->second.DirectCall(argv);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// generators for arrays (vector and list) of primitive types
//-------------------------------------------------------------------------------------------

template <typename t_array>
void array_of_primitive_get_as_string_generator (t_array *x, pt_string &out)
{
  out= NumericalContainerToString(*x, ", ");
}
template <typename t_array>
void array_of_primitive_set_by_string_generator (t_array *x, const pt_string &in)
{
  (*x)= ConvertFromStr<t_array>(in);
}
//-------------------------------------------------------------------------------------------

//!\brief set f_primitive_get_as_string_ and f_primitive_set_by_string_  (for arrays of primitive types)
template <typename t_array>
void  array_of_primitive_get_set_string_generator_generic (t_array &x, boost::function<void(pt_string&)> &get_as, boost::function<void(const pt_string&)> &set_by)
{
  get_as= boost::bind(array_of_primitive_get_as_string_generator<t_array>,&x,_1);
  set_by= boost::bind(array_of_primitive_set_by_string_generator<t_array>,&x,_1);
}
//-------------------------------------------------------------------------------------------

#define SPECIALIZER(x_type)  \
  template<> void  array_of_primitive_get_set_string_generator (std::vector<x_type> &x, boost::function<void(pt_string&)> &get_as, boost::function<void(const pt_string&)> &set_by)  \
      {array_of_primitive_get_set_string_generator_generic(x,get_as,set_by);}                                                                                                         \
  template<> void  array_of_primitive_get_set_string_generator (std::list<x_type> &x, boost::function<void(pt_string&)> &get_as, boost::function<void(const pt_string&)> &set_by)    \
      {array_of_primitive_get_set_string_generator_generic(x,get_as,set_by);}
SPECIALIZER(unsigned short  )
SPECIALIZER(unsigned int    )
SPECIALIZER(unsigned long   )
SPECIALIZER(signed short    )
SPECIALIZER(signed int      )
SPECIALIZER(signed long     )
SPECIALIZER(pt_bool         )

SPECIALIZER(float           )
SPECIALIZER(double          )
SPECIALIZER(long double     )
#undef SPECIALIZER
//-------------------------------------------------------------------------------------------



//===========================================================================================
// class TVariable
//===========================================================================================

void TVariable::VariableSpaceMode (void)
{
  is_primitive_ = false;
  f_direct_assign_   =  struct_direct_assign_generator;
  f_set_member_      =  struct_set_member_generator;
  f_get_member_      =  struct_get_member_generator;
  f_write_to_stream_ =  struct_write_to_stream_generator;
}
//-------------------------------------------------------------------------------------------

std::size_t TVariable::Size() const
{
  if (!is_primitive_ && !f_get_begin_)  return 0;
  if (is_primitive_ && !f_get_begin_)  return 1;
  TConstForwardIterator  id_itr, id_last;
  GetEnd (id_last);
  std::size_t size(0);
  for (GetBegin(id_itr); id_itr!=id_last; ++id_itr)  ++size;
  return size;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// specialize TVariable::Generate for primitive types
//===========================================================================================

#define GET_AS(x_gen, x_var)  boost::bind(converter_generator_pr<x_gen, x_var>,&x,_1)
#define SET_BY(x_gen, x_var)  boost::bind(converter_generator_rp<x_var, x_gen>,_1,&x)

#if 0
template<> struct TVariable::generator<pt_SAMPLE>
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (pt_SAMPLE &x);
};
void TVariable::generator<pt_SAMPLE>::operator() (pt_SAMPLE &x)
{
  o.is_primitive_ = true;
  o.f_primitive_get_as_int_      = GET_AS(pt_SAMPLE, pt_int      );
  o.f_primitive_get_as_real_     = GET_AS(pt_SAMPLE, pt_real     );
  o.f_primitive_get_as_bool_     = GET_AS(pt_SAMPLE, pt_bool     );
  // o.f_primitive_get_as_string_   = GET_AS(pt_SAMPLE, pt_string   );

  o.f_primitive_set_by_int_      = SET_BY(pt_SAMPLE, pt_int      );
  o.f_primitive_set_by_real_     = SET_BY(pt_SAMPLE, pt_real     );
  o.f_primitive_set_by_bool_     = SET_BY(pt_SAMPLE, pt_bool     );
  // o.f_primitive_set_by_string_   = SET_BY(pt_SAMPLE, pt_string   );

  o.f_direct_assign_ = boost::bind(primitive_direct_assign_generator<pt_SAMPLE,pt_AS>,&x,_1,_2);
  o.f_write_to_stream_ = boost::bind(primitive_write_to_stream_generator<pt_SAMPLE>,&x,_1,_2,_3,_4);
}
#endif

#define SPECIALIZE_GENERATE_INT(x_type)  \
  void TVariable::generator<x_type>::operator() (x_type &x)                    \
  {                                                                            \
    o.is_primitive_ = true;                                                    \
    o.f_primitive_get_as_int_      = GET_AS(x_type, pt_int      );             \
    o.f_primitive_get_as_real_     = GET_AS(x_type, pt_real     );             \
    o.f_primitive_get_as_bool_     = GET_AS(x_type, pt_bool     );             \
    o.f_primitive_get_as_string_   = GET_AS(x_type, pt_string   );             \
                                                                               \
    o.f_primitive_set_by_int_      = SET_BY(x_type, pt_int      );             \
    o.f_primitive_set_by_real_     = SET_BY(x_type, pt_real     );             \
    o.f_primitive_set_by_bool_     = SET_BY(x_type, pt_bool     );             \
    o.f_primitive_set_by_string_   = SET_BY(x_type, pt_string   );             \
                                                                               \
    o.f_direct_assign_ = boost::bind(primitive_direct_assign_generator<x_type, pt_int>,&x,_1,_2);    \
    o.f_write_to_stream_ = boost::bind(primitive_write_to_stream_generator<x_type>,&x,_1,_2,_3,_4);  \
  }
SPECIALIZE_GENERATE_INT(unsigned short)
SPECIALIZE_GENERATE_INT(unsigned int  )
SPECIALIZE_GENERATE_INT(unsigned long )
SPECIALIZE_GENERATE_INT(signed short  )
SPECIALIZE_GENERATE_INT(signed int    )
SPECIALIZE_GENERATE_INT(signed long   )
#undef SPECIALIZE_GENERATE_INT
//-------------------------------------------------------------------------------------------

void TVariable::generator<pt_bool>::operator() (pt_bool &x)
{
  o.is_primitive_ = true;
  o.f_primitive_get_as_int_      = GET_AS(pt_bool, pt_int      );
  o.f_primitive_get_as_real_     = GET_AS(pt_bool, pt_real     );
  o.f_primitive_get_as_bool_     = GET_AS(pt_bool, pt_bool     );
  o.f_primitive_get_as_string_   = GET_AS(pt_bool, pt_string   );

  o.f_primitive_set_by_int_      = SET_BY(pt_bool, pt_int      );
  o.f_primitive_set_by_real_     = SET_BY(pt_bool, pt_real     );
  o.f_primitive_set_by_bool_     = SET_BY(pt_bool, pt_bool     );
  o.f_primitive_set_by_string_   = SET_BY(pt_bool, pt_string   );

  o.f_direct_assign_ = boost::bind(primitive_direct_assign_generator<pt_bool, pt_bool>,&x,_1,_2);
  o.f_write_to_stream_ = boost::bind(primitive_write_to_stream_generator<pt_bool>,&x,_1,_2,_3,_4);
}
//-------------------------------------------------------------------------------------------

#define SPECIALIZE_GENERATE_REAL(x_type)  \
  void TVariable::generator<x_type>::operator() (x_type &x)                    \
  {                                                                            \
    o.is_primitive_ = true;                                                    \
    o.f_primitive_get_as_int_      = GET_AS(x_type, pt_int      );             \
    o.f_primitive_get_as_real_     = GET_AS(x_type, pt_real     );             \
    o.f_primitive_get_as_bool_     = GET_AS(x_type, pt_bool     );             \
    o.f_primitive_get_as_string_   = GET_AS(x_type, pt_string   );             \
                                                                               \
    o.f_primitive_set_by_int_      = SET_BY(x_type, pt_int      );             \
    o.f_primitive_set_by_real_     = SET_BY(x_type, pt_real     );             \
    o.f_primitive_set_by_bool_     = SET_BY(x_type, pt_bool     );             \
    o.f_primitive_set_by_string_   = SET_BY(x_type, pt_string   );             \
                                                                               \
    o.f_direct_assign_ = boost::bind(primitive_direct_assign_generator<x_type, pt_real>,&x,_1,_2);    \
    o.f_write_to_stream_ = boost::bind(primitive_write_to_stream_generator<x_type>,&x,_1,_2,_3,_4);   \
  }
SPECIALIZE_GENERATE_REAL(float       )
SPECIALIZE_GENERATE_REAL(double      )
SPECIALIZE_GENERATE_REAL(long double )
#undef SPECIALIZE_GENERATE_REAL
//-------------------------------------------------------------------------------------------

template <>
void primitive_write_to_stream_generator<pt_string> (pt_string *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent)
{
  os<<ConvertToStr(*x);
}

void TVariable::generator<pt_string>::operator() (pt_string &x)
{
  o.is_primitive_ = true;
  o.f_primitive_get_as_int_      = GET_AS(pt_string, pt_int      );
  o.f_primitive_get_as_real_     = GET_AS(pt_string, pt_real     );
  o.f_primitive_get_as_bool_     = GET_AS(pt_string, pt_bool     );
  o.f_primitive_get_as_string_   = GET_AS(pt_string, pt_string   );

  o.f_primitive_set_by_int_      = SET_BY(pt_string, pt_int      );
  o.f_primitive_set_by_real_     = SET_BY(pt_string, pt_real     );
  o.f_primitive_set_by_bool_     = SET_BY(pt_string, pt_bool     );
  o.f_primitive_set_by_string_   = SET_BY(pt_string, pt_string   );

  o.f_direct_assign_ = boost::bind(primitive_direct_assign_generator<pt_string, pt_string>,&x,_1,_2);
  o.f_write_to_stream_ = boost::bind(primitive_write_to_stream_generator<pt_string>,&x,_1,_2,_3,_4);
}
//-------------------------------------------------------------------------------------------


#undef GET_AS
#undef SET_BY
//-------------------------------------------------------------------------------------------


//===========================================================================================
/* specialization of TVariable::generator of std::vector for basic types
    NOTE: the following includes instantiation (SPECIALIZERI)
      (you need not to include variable_space_impl.h for these types) */
//===========================================================================================

#define SPECIALIZER(x_type)  \
  template<> void vector_write_to_stream_generator (std::vector<x_type> *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent) \
    {os<<"("<<NumericalContainerToString(*x, ", ")<<")";}
#define SPECIALIZERI(x_type)  \
  SPECIALIZER(x_type) \
  template struct TVariable::generator<std::vector<x_type> >;
SPECIALIZER (unsigned short  )
SPECIALIZER (unsigned int    )
SPECIALIZER (unsigned long   )
SPECIALIZER (signed short    )
SPECIALIZERI(signed int      )
SPECIALIZER (signed long     )
SPECIALIZER (bool            )

SPECIALIZER (float           )
SPECIALIZERI(double          )
SPECIALIZERI(long double     )
#undef SPECIALIZERI
#undef SPECIALIZER

//-------------------------------------------------------------------------------------------


//===========================================================================================
/* specialization of TVariable::generator of std::list for basic types
    NOTE: the following includes instantiation (SPECIALIZERI)
      (you need not to include variable_space_impl.h for these types) */
//===========================================================================================

#define SPECIALIZER(x_type)  \
  template<> void list_write_to_stream_generator (std::list<x_type> *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent) \
    {os<<"("<<NumericalContainerToString(*x, ", ")<<")";}
#define SPECIALIZERI(x_type)  \
  SPECIALIZER(x_type) \
  template struct TVariable::generator<std::list<x_type> >;
SPECIALIZER (unsigned short  )
SPECIALIZER (unsigned int    )
SPECIALIZER (unsigned long   )
SPECIALIZER (signed short    )
SPECIALIZERI(signed int      )
SPECIALIZER (signed long     )
SPECIALIZER (bool            )

SPECIALIZER (float           )
SPECIALIZERI(double          )
SPECIALIZERI(long double     )
#undef SPECIALIZER

//-------------------------------------------------------------------------------------------


//===========================================================================================
// supplementary functions
//===========================================================================================

//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

