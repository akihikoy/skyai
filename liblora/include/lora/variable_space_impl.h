//-------------------------------------------------------------------------------------------
/*! \file    variable_space_impl.h
    \brief   liblora - variable-space : generic manipulators of variables  (implement header)
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
#ifndef loco_rabbits_variable_space_impl_h
#define loco_rabbits_variable_space_impl_h
//-------------------------------------------------------------------------------------------
#include <lora/variable_space.h>
//-------------------------------------------------------------------------------------------
#include <lora/cast.h>
#include <lora/string.h>
#include <lora/stl_ext.h>  // for list_itr_at
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
namespace var_space
{
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
// macros
//-------------------------------------------------------------------------------------------

/*!\brief specialize TVariable::generator for enum type \p x_type
    \note use this macro inside namespace loco_rabbits.
        e.g. in global namespace,
        namespace loco_rabbits {SPECIALIZE_TVARIABLE_TO_ENUM(XXX_ENUM)}  */
#define SPECIALIZE_TVARIABLE_TO_ENUM(x_type)                                                             \
  namespace var_space{                                                                                   \
    template<> struct TVariable::generator<x_type>                                                       \
    {                                                                                                    \
      TVariable &o;                                                                                      \
      generator(TVariable &outer) : o(outer) {}                                                          \
      void operator() (x_type &x)                                                                        \
      {                                                                                                  \
        o.f_primitive_get_as_int_      = boost::bind(converter_generator_pr<x_type, pt_int    >,&x,_1);  \
        o.f_primitive_get_as_bool_     = boost::bind(converter_generator_pr<x_type, pt_bool   >,&x,_1);  \
        o.f_primitive_get_as_string_   = boost::bind(converter_generator_pr<x_type, pt_string >,&x,_1);  \
                                                                                                         \
        o.f_primitive_set_by_int_      = boost::bind(converter_generator_rp<pt_int    , x_type>,_1,&x);  \
        o.f_primitive_set_by_bool_     = boost::bind(converter_generator_rp<pt_bool   , x_type>,_1,&x);  \
        o.f_primitive_set_by_string_   = boost::bind(converter_generator_rp<pt_string , x_type>,_1,&x);  \
                                                                                                         \
        o.f_direct_assign_ = boost::bind(primitive_direct_assign_generator<x_type, pt_string>,&x,_1,_2); \
        o.f_write_to_stream_ = boost::bind(enum_write_to_stream_generator<x_type>,&x,_1,_2,_3,_4);       \
      }                                                                                                  \
    };                                                                                                   \
  }

//-------------------------------------------------------------------------------------------



//===========================================================================================
// function object generators
//===========================================================================================

//-------------------------------------------------------------------------------------------
// generators for primitive types
//-------------------------------------------------------------------------------------------

//!\brief generators of f_primitive_get_as_*
template <typename t_from, typename t_to>
void converter_generator_pr(const t_from *from, t_to &to)
{
  to= lora_cast<t_to>(*from);
}
//!\brief generators of f_primitive_set_by_*
template <typename t_from, typename t_to>
void converter_generator_rp(const t_from &from, t_to *to)
{
  *to= lora_cast<t_to>(from);
}
//-------------------------------------------------------------------------------------------

//!\brief generators of f_direct_assign_ for primitive types
template <typename t_var, typename t_as>
void primitive_direct_assign_generator(t_var *x, TVariableMap &, const TVariable &var)
{
  t_as tmp;
  var.PrimitiveGetAs<t_as>(tmp);
  *x= lora_cast<t_var>(tmp);
}
//-------------------------------------------------------------------------------------------

template <typename t_var>
void primitive_write_to_stream_generator (t_var *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent)
{
  os<<ConvertToStr(*x);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// generators for enum types
//-------------------------------------------------------------------------------------------

template <typename t_var>
void enum_write_to_stream_generator (t_var *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent)
{
  os<<ConvertToStr(ConvertToStr(*x));
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// partial specialization of TVariable::generator for std::vector
//===========================================================================================

template <typename t_elem>
void vector_direct_assign_generator (std::vector<t_elem> *x, TVariableMap &, const TVariable &value)
{
  TConstForwardIterator  value_itr, value_last;
  value.GetEnd (value_last);
  int size(0);
  for (value.GetBegin (value_itr); value_itr!=value_last; ++value_itr)
    ++size;
  x->resize(size);
  value.GetBegin (value_itr);
  for (typename std::vector<t_elem>::iterator itr(x->begin()),last(x->end()); itr!=last; ++itr,++value_itr)
  {
    TVariable var(*itr);
    var.DirectAssign(*value_itr);
  }
}

template <typename t_elem>
void vector_set_member_generator (std::vector<t_elem> *x, const TVariableMap &members, const TVariable &id, const TVariable &value)
{
  vector_get_member_generator(x,members,id).DirectAssign(value);
}

template <typename t_elem>
TVariable vector_get_member_generator (std::vector<t_elem> *x, const TVariableMap &, const TVariable &id)
{
  return TVariable((*x)[id.PrimitiveGetAs<pt_int>()]);
}

template <typename t_elem>
struct vector_clear_function
{
  std::vector<t_elem> *PtrEntity;
  vector_clear_function(std::vector<t_elem> &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "clear";}
  static void f (std::vector<t_elem> *x, TVariableList &argv)
    {
      if (argv.size()!=1)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(void)");}
      TVariableList::const_iterator itr(argv.begin());
      ++itr; // skip void
      x->clear ();
    }
};
template <typename t_elem>
struct TVariable::generator<vector_clear_function<t_elem> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (vector_clear_function<t_elem> &x)
  {
    o.f_direct_call_ = boost::bind(vector_clear_function<t_elem>::f,x.PtrEntity,_1);
  }
};

template <typename t_elem>
struct vector_pop_back_function
{
  std::vector<t_elem> *PtrEntity;
  vector_pop_back_function(std::vector<t_elem> &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "pop_back";}
  static void f (std::vector<t_elem> *x, TVariableList &argv)
    {
      if (argv.size()!=1)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(void)");}
      TVariableList::const_iterator itr(argv.begin());
      ++itr; // skip void
      x->pop_back ();
    }
};
template <typename t_elem>
struct TVariable::generator<vector_pop_back_function<t_elem> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (vector_pop_back_function<t_elem> &x)
  {
    o.f_direct_call_ = boost::bind(vector_pop_back_function<t_elem>::f,x.PtrEntity,_1);
  }
};

template <typename t_elem>
struct vector_resize_function
{
  std::vector<t_elem> *PtrEntity;
  vector_resize_function(std::vector<t_elem> &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "resize";}
  static void f (std::vector<t_elem> *x, TVariableList &argv)
    {
      if (argv.size()!=2)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(int)");}
      TVariableList::const_iterator itr(argv.begin());
      ++itr; // skip void
      x->resize (itr->PrimitiveGetAs<pt_int>());
    }
};
template <typename t_elem>
struct TVariable::generator<vector_resize_function<t_elem> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (vector_resize_function<t_elem> &x)
  {
    o.f_direct_call_ = boost::bind(vector_resize_function<t_elem>::f,x.PtrEntity,_1);
  }
};

template <typename t_elem>
void vector_register_functions (std::vector<t_elem> &entity, TVariableMap &mmap)
{
  #define ADD(x_func)  \
    vector_##x_func##_function<t_elem>  x_func##_func(entity);  \
    mmap[x_func##_func.Name()]= TVariable(x_func##_func);
  ADD(clear)
  ADD(pop_back)
  ADD(resize)
  #undef ADD
}

template <typename t_elem>
TVariable vector_push_generator (std::vector<t_elem> *x)
{
  x->push_back(t_elem());
  return TVariable(x->back());
}

#define TO_PITR(x_entity)  (static_cast<typename std::vector<t_elem>::iterator*>(x_entity))
template <typename t_elem>
void vector_iterator_entity_destructor_generator (void *entity)
{
  delete TO_PITR(entity);
}
template <typename t_elem>
void vector_iterator_increment_generator (void *entity)
{
  ++(*TO_PITR(entity));
}
template <typename t_elem>
void vector_iterator_decrement_generator (void *entity)
{
  --(*TO_PITR(entity));
}
template <typename t_elem>
TVariable vector_iterator_dereference_generator (void *entity)
{
  return TVariable(*(*TO_PITR(entity)));
}
template <typename t_elem>
bool vector_iterator_is_equal_to_generator (void *entity, void *rhs_entity)
{
  return (*TO_PITR(entity))==(*TO_PITR(rhs_entity));
}
#undef TO_PITR


/*!\brief wrapper of vector<>::iterator
  \note This structure seems useless. But, if we try to specialize the generator by
    'template \<typename t_elem\>
    struct TForwardIterator::generator \<typename std::vector\<t_elem\>::iterator\>',
    a compiler(g++ ver.4.4.3) says an error: template parameters not used in
    partial specialization: 't_elem'.
    To avoid this error, this structure is defined.
*/
template <typename t_elem>
struct TVectorIteratorWrapper
{
  typename std::vector<t_elem>::iterator  Entity;
  TVectorIteratorWrapper(typename std::vector<t_elem>::iterator itr) : Entity(itr) {}
};

template <typename t_elem>
struct TForwardIterator::generator <TVectorIteratorWrapper<t_elem> >
{
  TForwardIterator &o;
  generator(TForwardIterator &outer) : o(outer) {}
  void operator() (const TVectorIteratorWrapper<t_elem> &init)
  {
    typename std::vector<t_elem>::iterator *entity = new typename std::vector<t_elem>::iterator;
    *entity = init.Entity;
    o.entity_ = entity;
    entity=NULL;
    o.f_entity_destructor_ = vector_iterator_entity_destructor_generator<t_elem>;
    o.f_increment_    = vector_iterator_increment_generator<t_elem>;
    o.f_decrement_    = vector_iterator_decrement_generator<t_elem>;
    o.f_dereference_  = vector_iterator_dereference_generator<t_elem>;
    o.f_is_equal_to_  = vector_iterator_is_equal_to_generator<t_elem>;
  }
};

template <typename t_elem>
void vector_get_begin_generator (std::vector<t_elem> *x, const TVariableMap &, TForwardIterator &res)
{
  res.Generate<TVectorIteratorWrapper<t_elem> >(TVectorIteratorWrapper<t_elem>(x->begin()));
}
template <typename t_elem>
void vector_get_end_generator (std::vector<t_elem> *x, const TVariableMap &, TForwardIterator &res)
{
  res.Generate<TVectorIteratorWrapper<t_elem> >(TVectorIteratorWrapper<t_elem>(x->end()));
}

template <typename t_elem>
void vector_write_to_stream_generator (std::vector<t_elem> *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent)
{
  pt_string new_indent;
  if (!bare)
  {
    os<< "{" << std::endl;
    new_indent= indent+VAR_SPACE_INDENT_STEP;
  }
  else
    new_indent= indent;
  os<< new_indent << "clear()" << std::endl;
  if (x->size()>0)
  {
    os<< new_indent << "resize(" << x->size() << ")" << std::endl;
    int idx(0);
    for (typename std::vector<t_elem>::iterator itr(x->begin()),last(x->end()); itr!=last; ++itr,++idx)
    {
      os<< new_indent <<"["<< idx <<"] = ";
      TVariable(*itr).WriteToStream(os, false, new_indent+VAR_SPACE_INDENT_STEP);
      os<< std::endl;
    }
  }
  if (!bare)
    os<< indent << "}";
}
#define SPECIALIZER(x_type)  \
  template<> void vector_write_to_stream_generator (std::vector<x_type> *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent);
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

//!\brief partial specialization for std::vector

template <typename t_elem>
void TVariable::generator<std::vector<t_elem> >::operator() (std::vector<t_elem> &x)
{
  o.f_direct_assign_ = boost::bind(vector_direct_assign_generator<t_elem>,&x,_1,_2);

  o.f_set_member_ = boost::bind(vector_set_member_generator<t_elem>,&x,_1,_2,_3);
  o.f_get_member_ = boost::bind(vector_get_member_generator<t_elem>,&x,_1,_2);

  vector_register_functions(x,o.SetMemberMap());

  o.f_function_call_ = boost::bind(generic_function_call_generator,_1,_2,_3);

  o.f_push_      = boost::bind(vector_push_generator<t_elem>,&x);
  o.f_get_begin_ = boost::bind(vector_get_begin_generator<t_elem>,&x,_1,_2);
  o.f_get_end_   = boost::bind(vector_get_end_generator<t_elem>,&x,_1,_2);

  o.f_write_to_stream_ = boost::bind(vector_write_to_stream_generator<t_elem>,&x,_1,_2,_3,_4);
}
//-------------------------------------------------------------------------------------------



//===========================================================================================
// partial specialization of TVariable::generator for std::list
//===========================================================================================

template <typename t_elem>
void list_direct_assign_generator (std::list<t_elem> *x, TVariableMap &, const TVariable &value)
{
  TConstForwardIterator  value_itr, value_last;
  value.GetEnd (value_last);
  int size(0);
  for (value.GetBegin (value_itr); value_itr!=value_last; ++value_itr)
    ++size;
  x->resize(size);
  value.GetBegin (value_itr);
  for (typename std::list<t_elem>::iterator itr(x->begin()),last(x->end()); itr!=last; ++itr,++value_itr)
  {
    TVariable var(*itr);
    var.DirectAssign(*value_itr);
  }
}

template <typename t_elem>
void list_set_member_generator (std::list<t_elem> *x, const TVariableMap &members, const TVariable &id, const TVariable &value)
{
  list_get_member_generator(x,members,id).DirectAssign(value);
}

template <typename t_elem>
TVariable list_get_member_generator (std::list<t_elem> *x, const TVariableMap &, const TVariable &id)
{
  return TVariable(*list_itr_at(*x,id.PrimitiveGetAs<pt_int>()));
}

template <typename t_elem>
struct list_clear_function
{
  std::list<t_elem> *PtrEntity;
  list_clear_function(std::list<t_elem> &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "clear";}
  static void f (std::list<t_elem> *x, TVariableList &argv)
    {
      if (argv.size()!=1)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(void)");}
      TVariableList::const_iterator itr(argv.begin());
      ++itr; // skip void
      x->clear ();
    }
};
template <typename t_elem>
struct TVariable::generator<list_clear_function<t_elem> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (list_clear_function<t_elem> &x)
  {
    o.f_direct_call_ = boost::bind(list_clear_function<t_elem>::f,x.PtrEntity,_1);
  }
};

template <typename t_elem>
struct list_pop_back_function
{
  std::list<t_elem> *PtrEntity;
  list_pop_back_function(std::list<t_elem> &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "pop_back";}
  static void f (std::list<t_elem> *x, TVariableList &argv)
    {
      if (argv.size()!=1)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(void)");}
      TVariableList::const_iterator itr(argv.begin());
      ++itr; // skip void
      x->pop_back ();
    }
};
template <typename t_elem>
struct TVariable::generator<list_pop_back_function<t_elem> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (list_pop_back_function<t_elem> &x)
  {
    o.f_direct_call_ = boost::bind(list_pop_back_function<t_elem>::f,x.PtrEntity,_1);
  }
};

template <typename t_elem>
struct list_pop_front_function
{
  std::list<t_elem> *PtrEntity;
  list_pop_front_function(std::list<t_elem> &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "pop_front";}
  static void f (std::list<t_elem> *x, TVariableList &argv)
    {
      if (argv.size()!=1)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(void)");}
      TVariableList::const_iterator itr(argv.begin());
      ++itr; // skip void
      x->pop_front ();
    }
};
template <typename t_elem>
struct TVariable::generator<list_pop_front_function<t_elem> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (list_pop_front_function<t_elem> &x)
  {
    o.f_direct_call_ = boost::bind(list_pop_front_function<t_elem>::f,x.PtrEntity,_1);
  }
};

template <typename t_elem>
struct list_resize_function
{
  std::list<t_elem> *PtrEntity;
  list_resize_function(std::list<t_elem> &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "resize";}
  static void f (std::list<t_elem> *x, TVariableList &argv)
    {
      if (argv.size()!=2)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(int)");}
      TVariableList::const_iterator itr(argv.begin());
      ++itr; // skip void
      x->resize (itr->PrimitiveGetAs<pt_int>());
    }
};
template <typename t_elem>
struct TVariable::generator<list_resize_function<t_elem> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (list_resize_function<t_elem> &x)
  {
    o.f_direct_call_ = boost::bind(list_resize_function<t_elem>::f,x.PtrEntity,_1);
  }
};

template <typename t_elem>
void list_register_functions (std::list<t_elem> &entity, TVariableMap &mmap)
{
  #define ADD(x_func)  \
    list_##x_func##_function<t_elem>  x_func##_func(entity);  \
    mmap[x_func##_func.Name()]= TVariable(x_func##_func);
  ADD(clear)
  ADD(pop_back)
  ADD(pop_front)
  ADD(resize)
  #undef ADD
}

template <typename t_elem>
TVariable list_push_generator (std::list<t_elem> *x)
{
  x->push_back(t_elem());
  return TVariable(x->back());
}

#define TO_PITR(x_entity)  (static_cast<typename std::list<t_elem>::iterator*>(x_entity))
template <typename t_elem>
void list_iterator_entity_destructor_generator (void *entity)
{
  delete TO_PITR(entity);
}
template <typename t_elem>
void list_iterator_increment_generator (void *entity)
{
  ++(*TO_PITR(entity));
}
template <typename t_elem>
void list_iterator_decrement_generator (void *entity)
{
  --(*TO_PITR(entity));
}
template <typename t_elem>
TVariable list_iterator_dereference_generator (void *entity)
{
  return TVariable(*(*TO_PITR(entity)));
}
template <typename t_elem>
bool list_iterator_is_equal_to_generator (void *entity, void *rhs_entity)
{
  return (*TO_PITR(entity))==(*TO_PITR(rhs_entity));
}
#undef TO_PITR


/*!\brief wrapper of list<>::iterator
  \note This structure seems useless. But, if we try to specialize the generator by
    'template \<typename t_elem\>
    struct TForwardIterator::generator \<typename std::list\<t_elem\>::iterator\>',
    a compiler(g++ ver.4.4.3) says an error: template parameters not used in
    partial specialization: 't_elem'.
    To avoid this error, this structure is defined.
*/
template <typename t_elem>
struct TListIteratorWrapper
{
  typename std::list<t_elem>::iterator  Entity;
  TListIteratorWrapper(typename std::list<t_elem>::iterator itr) : Entity(itr) {}
};

template <typename t_elem>
struct TForwardIterator::generator <TListIteratorWrapper<t_elem> >
{
  TForwardIterator &o;
  generator(TForwardIterator &outer) : o(outer) {}
  void operator() (const TListIteratorWrapper<t_elem> &init)
  {
    typename std::list<t_elem>::iterator *entity = new typename std::list<t_elem>::iterator;
    *entity = init.Entity;
    o.entity_ = entity;
    entity=NULL;
    o.f_entity_destructor_ = list_iterator_entity_destructor_generator<t_elem>;
    o.f_increment_    = list_iterator_increment_generator<t_elem>;
    o.f_decrement_    = list_iterator_decrement_generator<t_elem>;
    o.f_dereference_  = list_iterator_dereference_generator<t_elem>;
    o.f_is_equal_to_  = list_iterator_is_equal_to_generator<t_elem>;
  }
};

template <typename t_elem>
void list_get_begin_generator (std::list<t_elem> *x, const TVariableMap &, TForwardIterator &res)
{
  res.Generate<TListIteratorWrapper<t_elem> >(TListIteratorWrapper<t_elem>(x->begin()));
}
template <typename t_elem>
void list_get_end_generator (std::list<t_elem> *x, const TVariableMap &, TForwardIterator &res)
{
  res.Generate<TListIteratorWrapper<t_elem> >(TListIteratorWrapper<t_elem>(x->end()));
}

template <typename t_elem>
void list_write_to_stream_generator (std::list<t_elem> *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent)
{
  pt_string new_indent;
  if (!bare)
  {
    os<< "{" << std::endl;
    new_indent= indent+VAR_SPACE_INDENT_STEP;
  }
  else
    new_indent= indent;
  os<< new_indent << "clear()" << std::endl;
  for (typename std::list<t_elem>::iterator itr(x->begin()),last(x->end()); itr!=last; ++itr)
  {
    os<< new_indent <<"[] = ";
    TVariable(*itr).WriteToStream(os, false, new_indent+VAR_SPACE_INDENT_STEP);
    os<< std::endl;
  }
  if (!bare)
    os<< indent << "}";
}

#define SPECIALIZER(x_type)  \
  template<> void list_write_to_stream_generator (std::list<x_type> *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent);
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

//!\brief partial specialization for std::list

template <typename t_elem>
void TVariable::generator<std::list<t_elem> >::operator() (std::list<t_elem> &x)
{
  o.f_direct_assign_ = boost::bind(list_direct_assign_generator<t_elem>,&x,_1,_2);

  o.f_set_member_ = boost::bind(list_set_member_generator<t_elem>,&x,_1,_2,_3);
  o.f_get_member_ = boost::bind(list_get_member_generator<t_elem>,&x,_1,_2);

  list_register_functions(x,o.SetMemberMap());

  o.f_function_call_ = boost::bind(generic_function_call_generator,_1,_2,_3);

  o.f_push_      = boost::bind(list_push_generator<t_elem>,&x);
  o.f_get_begin_ = boost::bind(list_get_begin_generator<t_elem>,&x,_1,_2);
  o.f_get_end_   = boost::bind(list_get_end_generator<t_elem>,&x,_1,_2);

  o.f_write_to_stream_ = boost::bind(list_write_to_stream_generator<t_elem>,&x,_1,_2,_3,_4);
}
//-------------------------------------------------------------------------------------------



//===========================================================================================
// partial specialization of TVariable::generator for std::map
//===========================================================================================

template <typename t_key, typename t_elem>
void map_direct_assign_generator (std::map<t_key,t_elem> *x, TVariableMap &, const TVariable &value)
{
  TConstForwardIterator  value_itr, value_last;
  value.GetEnd (value_last);
  x->clear();
  t_key key;
  TVariable var_key(key);
  for (value.GetBegin (value_itr); value_itr!=value_last; ++value_itr)
  {
    var_key.DirectAssign(value_itr.Key());
    TVariable var((*x)[key]);
    var.DirectAssign(*value_itr);
  }
}

template <typename t_key, typename t_elem>
void map_set_member_generator (std::map<t_key,t_elem> *x, const TVariableMap &members, const TVariable &id, const TVariable &value)
{
  map_get_member_generator(x,members,id).DirectAssign(value);
}

template <typename t_key, typename t_elem>
TVariable map_get_member_generator (std::map<t_key,t_elem> *x, const TVariableMap &, const TVariable &id)
{
  t_key key;
  TVariable(key).DirectAssign(id);
  return TVariable((*x)[key]);
}

template <typename t_key, typename t_elem>
struct map_clear_function
{
  std::map<t_key,t_elem> *PtrEntity;
  map_clear_function(std::map<t_key,t_elem> &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "clear";}
  static void f (std::map<t_key,t_elem> *x, TVariableList &argv)
    {
      if (argv.size()!=1)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(void)");}
      TVariableList::const_iterator itr(argv.begin());
      ++itr; // skip void
      x->clear ();
    }
};
template <typename t_key, typename t_elem>
struct TVariable::generator<map_clear_function<t_key,t_elem> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (map_clear_function<t_key,t_elem> &x)
  {
    o.f_direct_call_ = boost::bind(map_clear_function<t_key,t_elem>::f,x.PtrEntity,_1);
  }
};

template <typename t_key, typename t_elem>
void map_register_functions (std::map<t_key,t_elem> &entity, TVariableMap &mmap)
{
  #define ADD(x_func)  \
    map_##x_func##_function<t_key,t_elem>  x_func##_func(entity);  \
    mmap[x_func##_func.Name()]= TVariable(x_func##_func);
  ADD(clear)
  #undef ADD
}

#define TO_PITR(x_entity)  (static_cast<typename std::map<t_key,t_elem>::iterator*>(x_entity))
template <typename t_key, typename t_elem>
void map_iterator_entity_destructor_generator (void *entity)
{
  delete TO_PITR(entity);
}
template <typename t_key, typename t_elem>
void map_iterator_increment_generator (void *entity)
{
  ++(*TO_PITR(entity));
}
template <typename t_key, typename t_elem>
void map_iterator_decrement_generator (void *entity)
{
  --(*TO_PITR(entity));
}
template <typename t_key, typename t_elem>
TVariable map_iterator_dereference_generator (void *entity)
{
  return TVariable((*TO_PITR(entity))->second);
}
template <typename t_key, typename t_elem>
bool map_iterator_is_equal_to_generator (void *entity, void *rhs_entity)
{
  return (*TO_PITR(entity))==(*TO_PITR(rhs_entity));
}
#undef TO_PITR


/*!\brief wrapper of map<>::iterator
  \note This structure seems useless. But, if we try to specialize the generator by
    'template \<typename t_elem\>
    struct TForwardIterator::generator \<typename std::map\<t_elem\>::iterator\>',
    a compiler(g++ ver.4.4.3) says an error: template parameters not used in
    partial specialization: 't_elem'.
    To avoid this error, this structure is defined.
*/
template <typename t_key, typename t_elem>
struct TMapIteratorWrapper
{
  typename std::map<t_key,t_elem>::iterator  Entity;
  TMapIteratorWrapper(typename std::map<t_key,t_elem>::iterator itr) : Entity(itr) {}
};

template <typename t_key, typename t_elem>
struct TForwardIterator::generator <TMapIteratorWrapper<t_key,t_elem> >
{
  TForwardIterator &o;
  generator(TForwardIterator &outer) : o(outer) {}
  void operator() (const TMapIteratorWrapper<t_key,t_elem> &init)
  {
    typename std::map<t_key,t_elem>::iterator *entity = new typename std::map<t_key,t_elem>::iterator;
    *entity = init.Entity;
    o.entity_ = entity;
    entity=NULL;
    o.f_entity_destructor_ = map_iterator_entity_destructor_generator<t_key,t_elem>;
    o.f_increment_    = map_iterator_increment_generator<t_key,t_elem>;
    o.f_decrement_    = map_iterator_decrement_generator<t_key,t_elem>;
    o.f_dereference_  = map_iterator_dereference_generator<t_key,t_elem>;
    o.f_is_equal_to_  = map_iterator_is_equal_to_generator<t_key,t_elem>;
  }
};

template <typename t_key, typename t_elem>
void map_get_begin_generator (std::map<t_key,t_elem> *x, const TVariableMap &, TForwardIterator &res)
{
  res.Generate<TMapIteratorWrapper<t_key,t_elem> >(TMapIteratorWrapper<t_key,t_elem>(x->begin()));
}
template <typename t_key, typename t_elem>
void map_get_end_generator (std::map<t_key,t_elem> *x, const TVariableMap &, TForwardIterator &res)
{
  res.Generate<TMapIteratorWrapper<t_key,t_elem> >(TMapIteratorWrapper<t_key,t_elem>(x->end()));
}

template <typename t_key, typename t_elem>
void map_write_to_stream_generator (std::map<t_key,t_elem> *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent)
{
  pt_string new_indent;
  if (!bare)
  {
    os<< "{" << std::endl;
    new_indent= indent+VAR_SPACE_INDENT_STEP;
  }
  else
    new_indent= indent;
  os<< new_indent << "clear()" << std::endl;
  for (typename std::map<t_key,t_elem>::iterator itr(x->begin()),last(x->end()); itr!=last; ++itr)
  {
    t_key key= itr->first;
    os<< new_indent <<"[";
    TVariable(key).WriteToStream(os, false, new_indent+VAR_SPACE_INDENT_STEP);
    os<<"] = ";
    TVariable(itr->second).WriteToStream(os, false, new_indent+VAR_SPACE_INDENT_STEP);
    os<< std::endl;
  }
  if (!bare)
    os<< indent << "}";
}

//!\brief partial specialization for std::map

template <typename t_key, typename t_elem>
void TVariable::generator<std::map<t_key,t_elem> >::operator() (std::map<t_key,t_elem> &x)
{
  o.f_direct_assign_ = boost::bind(map_direct_assign_generator<t_key,t_elem>,&x,_1,_2);

  o.f_set_member_ = boost::bind(map_set_member_generator<t_key,t_elem>,&x,_1,_2,_3);
  o.f_get_member_ = boost::bind(map_get_member_generator<t_key,t_elem>,&x,_1,_2);

  map_register_functions(x,o.SetMemberMap());

  o.f_function_call_ = boost::bind(generic_function_call_generator,_1,_2,_3);

  o.f_get_begin_ = boost::bind(map_get_begin_generator<t_key,t_elem>,&x,_1,_2);
  o.f_get_end_   = boost::bind(map_get_end_generator<t_key,t_elem>,&x,_1,_2);

  o.f_write_to_stream_ = boost::bind(map_write_to_stream_generator<t_key,t_elem>,&x,_1,_2,_3,_4);
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_space_impl_h
//-------------------------------------------------------------------------------------------
