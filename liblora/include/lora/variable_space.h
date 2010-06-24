//-------------------------------------------------------------------------------------------
/*! \file    variable_space.h
    \brief   liblora - variable-space : generic manipulators of variables  (header)
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
#ifndef loco_rabbits_variable_space_h
#define loco_rabbits_variable_space_h
//-------------------------------------------------------------------------------------------
#include <lora/variable_space_fwd.h>
#include <vector>
#include <boost/function.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
namespace var_space
{
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// generators for generic types
//-------------------------------------------------------------------------------------------

void generic_function_call_generator (const TVariableMap &members, const TIdentifier &name, TVariableList &argv);
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TVariable
//===========================================================================================
{
public:

  template <typename t_var>
  struct TFuncPrimitiveGetAs
    {
      typedef boost::function<void(t_var&)> F;
    };

  template <typename t_var>
  struct TFuncPrimitiveSetBy
    {
      typedef boost::function<void(const t_var&)> F;
    };


  TVariable () : is_primitive_(false), is_const_(false)  {}

  #define INIT_LIST  \
      is_primitive_                 (x.is_primitive_                 ),  \
      is_const_                     (x.is_const_                     ),  \
                                                                         \
      members_                      (x.members_                      ),  \
                                                                         \
      f_primitive_get_as_int_       (x.f_primitive_get_as_int_       ),  \
      f_primitive_get_as_real_      (x.f_primitive_get_as_real_      ),  \
      f_primitive_get_as_bool_      (x.f_primitive_get_as_bool_      ),  \
      f_primitive_get_as_string_    (x.f_primitive_get_as_string_    ),  \
                                                                         \
      f_primitive_set_by_int_       (x.f_primitive_set_by_int_       ),  \
      f_primitive_set_by_real_      (x.f_primitive_set_by_real_      ),  \
      f_primitive_set_by_bool_      (x.f_primitive_set_by_bool_      ),  \
      f_primitive_set_by_string_    (x.f_primitive_set_by_string_    ),  \
                                                                         \
      f_direct_assign_              (x.f_direct_assign_              ),  \
      f_set_member_                 (x.f_set_member_                 ),  \
      f_get_member_                 (x.f_get_member_                 ),  \
      f_direct_call_                (x.f_direct_call_                ),  \
      f_function_call_              (x.f_function_call_              ),  \
      f_push_                       (x.f_push_                       ),  \
      f_get_begin_                  (x.f_get_begin_                  ),  \
      f_get_end_                    (x.f_get_end_                    ),  \
      f_write_to_stream_            (x.f_write_to_stream_            )
  TVariable (TVariable &x) : INIT_LIST {}
  TVariable (const TVariable &x) : INIT_LIST {}
  #undef INIT_LIST

  TVariable (TVariableSpace) : is_primitive_(false), is_const_(false)  {VariableSpaceMode();}

  template <typename t_var>
  TVariable (t_var &x) : is_primitive_(false), is_const_(false)
    {
      generator<t_var>(*this)(x);
    }

  template <typename t_var>
  void Generate (t_var &x)
    {
      Clear();
      generator<t_var>(*this)(x);
    }

  //!\brief use this variable as a struct or a variable space
  void VariableSpaceMode (void);

  bool IsPrimitive() const {return is_primitive_;}
  bool IsConst() const {return is_const_;}

  std::size_t Size() const;

  void Clear ()
    {
      is_primitive_                 = false;
      is_const_                     = false;

      members_                      .clear();

      f_primitive_get_as_int_       .clear();
      f_primitive_get_as_real_      .clear();
      f_primitive_get_as_bool_      .clear();
      f_primitive_get_as_string_    .clear();

      f_primitive_set_by_int_       .clear();
      f_primitive_set_by_real_      .clear();
      f_primitive_set_by_bool_      .clear();
      f_primitive_set_by_string_    .clear();

      f_direct_assign_              .clear();
      f_set_member_                 .clear();
      f_get_member_                 .clear();
      f_direct_call_                .clear();
      f_function_call_              .clear();
      f_push_                       .clear();
      f_get_begin_                  .clear();
      f_get_end_                    .clear();
      f_write_to_stream_            .clear();
    }

  void AddMemberVariable(const TIdentifier &id, TVariable var)  {members_[id]= var;}

  const TVariableMap& MemberMap() const {return members_;}
  TVariableMap& SetMemberMap()  {return members_;}
  TVariableMap::iterator MemberMapBegin()  {return members_.begin();}
  TVariableMap::iterator MemberMapEnd()  {return members_.end();}
  TVariableMap::iterator MemberMapFind(TVariableMap::key_type &key)  {return members_.find(key);}

  template <typename t_var>
  inline void PrimitiveGetAs (t_var &x) const;
  template <typename t_var>
  inline void PrimitiveSetBy (const t_var &x);

  template <typename t_var>
  inline t_var PrimitiveGetAs () const
    {
      t_var x;
      PrimitiveGetAs<t_var>(x);
      return x;
    }

  inline void DirectAssign (const TVariable &value);
  inline void SetMember (const TVariable &id, const TVariable &value);
  inline TVariable GetMember (const TVariable &id);
  inline TVariable GetMember (const TVariable &id) const;
  inline void FunctionCall (const TIdentifier &id, TVariableList &argv);
  inline void FunctionCall (const TIdentifier &id, TVariableList &argv) const;
    //!< \note this const attribute is nonsense (since this method is potentially modifiable the contents of the variable)
  inline void DirectCall (TVariableList &argv);
  inline void DirectCall (TVariableList &argv) const;
    //!< \note this const attribute is nonsense (since this method is potentially modifiable the contents of the variable)
  inline TVariable Push (void);
  inline void GetBegin (TForwardIterator &res);
  inline void GetBegin (TConstForwardIterator &res) const;
  inline void GetEnd (TForwardIterator &res);
  inline void GetEnd (TConstForwardIterator &res) const;
  inline void WriteToStream (std::ostream &os, bool bare=false, const pt_string &indent="") const;

  //!\todo FIXME: add remove (erase) method

protected:

  /*!\brief specialize this object for a type which you want to treat as a TVariable
    \note in default, this generator is specialized for a struct type
    \note this generator implementation for a struct type (T) assumes that
          a function Register(T&,TVariableMap&) is overloaded for T
          which registers the member variables to a map of type TVariableMap
    \note in a specialized operator(), some function objects of \p o should be assigned
    \warning each function object must not use the outer \p o because such object
          is not safe for copying the outer */
  template <typename t_var>
  struct generator
    {
      TVariable &o;
      generator(TVariable &outer) : o(outer) {}
      void operator() (t_var&x)
        {
          o.VariableSpaceMode();
          Register(x, o.SetMemberMap());
        }
    };


  bool is_primitive_;  //!< \todo FIXME: implement this feature (used in save to file)
    //! \todo FIXME: kind_ in {primitive,composite,function} is better
  bool is_const_;  //!< \todo FIXME: implement this feature

  TVariableMap  members_;  //!< map identifier --\> member

  // function objects:
  TFuncPrimitiveGetAs<pt_int      >::F  f_primitive_get_as_int_      ;
  TFuncPrimitiveGetAs<pt_real     >::F  f_primitive_get_as_real_     ;
  TFuncPrimitiveGetAs<pt_bool     >::F  f_primitive_get_as_bool_     ;
  TFuncPrimitiveGetAs<pt_string   >::F  f_primitive_get_as_string_   ;

  TFuncPrimitiveSetBy<pt_int      >::F  f_primitive_set_by_int_      ;
  TFuncPrimitiveSetBy<pt_real     >::F  f_primitive_set_by_real_     ;
  TFuncPrimitiveSetBy<pt_bool     >::F  f_primitive_set_by_bool_     ;
  TFuncPrimitiveSetBy<pt_string   >::F  f_primitive_set_by_string_   ;

  boost::function<void(TVariableMap &members, const TVariable&)>  f_direct_assign_;

  boost::function<void(const TVariableMap &members, const TVariable &id, const TVariable &value)> f_set_member_;

  boost::function<TVariable(const TVariableMap &members, const TVariable &id)> f_get_member_;

  boost::function<void(TVariableList&)> f_direct_call_;

  /*!\brief call function identified by id with argv
      \note first element of argv should be a return */
  boost::function<void(const TVariableMap &members, const TIdentifier &id, TVariableList &argv)> f_function_call_;

  /*!\brief push (back) a new element and return its TVariable object
      \note the lifetime of the returned object is until another element is pushed / removed */
  boost::function<TVariable(void)> f_push_;

  /*!\brief return the first iterator
      \note the lifetime of the iterator is until an element is pushed / removed */
  boost::function<void(const TVariableMap &members, TForwardIterator&)> f_get_begin_;

  /*!\brief return the last iterator
      \note the lifetime of the iterator is until an element is pushed / removed */
  boost::function<void(const TVariableMap &members, TForwardIterator&)> f_get_end_;

  /*!\brief write the data into a stream */
  boost::function<void(const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent)> f_write_to_stream_;


  friend class TForwardIterator;
  friend class TConstForwardIterator;

};  // TVariable
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TForwardIterator
//===========================================================================================
{
public:

  TForwardIterator() : entity_(NULL) {}

  /*!\note do not define a complete copy-constructor because this class includes
      a memory allocated dynamically.
      the following one gives a copy-constructor which works only when entity_ is NULL.
      this is defined so that we can std::list< TForwardIterator >, or something like that. */
  TForwardIterator(const TForwardIterator &rhs) : entity_(NULL)
    {
      if (rhs.entity_!=NULL)
        {VAR_SPACE_ERR_EXIT("copy constructor of TForwardIterator works only when entity_ is NULL");}
    }

  ~TForwardIterator()  {free_entity();}

  template <typename t_var>
  TForwardIterator (const t_var &init) : entity_(NULL)  {Generate<t_var>(init);}

  template <typename t_var>
  void Generate (const t_var &init)
    {
      free_entity();
      generator<t_var>(*this)(init);
    }

  TVariable& operator*(void)  {dereferenced_=f_dereference_(entity_); return dereferenced_;}
  TVariable* operator->(void)  {dereferenced_=f_dereference_(entity_); return &dereferenced_;}
  const TForwardIterator& operator++(void)  {f_increment_(entity_); return *this;}
  const TForwardIterator& operator--(void)  {f_decrement_(entity_); return *this;}
  bool operator==(const TForwardIterator &rhs) const {return f_is_equal_to_(entity_,rhs.entity_);}
  bool operator!=(const TForwardIterator &rhs) const {return !f_is_equal_to_(entity_,rhs.entity_);}

  TVariable& Key(void)
    {
      if(!f_key_) {VAR_SPACE_ERR_EXIT("does not have Key() operation");}
      key_= f_key_(entity_);
      return key_;
    }

protected:

  /*!\note do not define an operator= because this class includes
      a memory allocated dynamically. */
  const TForwardIterator& operator= (const TForwardIterator&);

  /*!\brief specialize this object for a type which you want to treat as a TForwardIterator
    \note in a specialized operator(), entity_ and all function objects of \p o should be assigned */
  template <typename t_var>
  struct generator
    {
      TForwardIterator &o;
      generator(TForwardIterator &outer) : o(outer) {}
      void operator() (const t_var &init);
    };

  void *entity_;
  TVariable dereferenced_;
  TVariable key_;

  //! free the entity_
  boost::function<void(void *entity)> f_entity_destructor_;

  boost::function<void(void *entity)> f_increment_;
  boost::function<void(void *entity)> f_decrement_;
  boost::function<TVariable(void *entity)> f_dereference_;
  boost::function<TVariable(void *entity)> f_key_;
  boost::function<bool(void *entity, void *rhs_entity)> f_is_equal_to_;

  void free_entity()
    {
      if (entity_)
      {
        LASSERT(f_entity_destructor_);
        f_entity_destructor_(entity_);
      }
      entity_= NULL;
    }

};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TConstForwardIterator : public TForwardIterator
//===========================================================================================
{
public:

  TConstForwardIterator() : TForwardIterator() {}

  template <typename t_var>
  TConstForwardIterator (const t_var &init) : TForwardIterator(init)  {}

  TVariable& operator*(void)
    {
      dereferenced_=f_dereference_(entity_);
      dereferenced_.is_const_=true;
      return dereferenced_;
    }
  TVariable* operator->(void)
    {
      dereferenced_=f_dereference_(entity_);
      dereferenced_.is_const_=true;
      return &dereferenced_;
    }

};
//-------------------------------------------------------------------------------------------

//===========================================================================================
// implementations / specializations of inline methods
//===========================================================================================

inline void  TVariable::DirectAssign (const TVariable &value)
{
  if (!f_direct_assign_)
    {VAR_SPACE_ERR_EXIT("DirectAssign is not defined");}
  f_direct_assign_ (members_,value);
}
inline void  TVariable::SetMember (const TVariable &id, const TVariable &value)
{
  if (!f_set_member_)
    {VAR_SPACE_ERR_EXIT("SetMember is not defined");}
  f_set_member_ (members_,id,value);
}
inline TVariable  TVariable::GetMember (const TVariable &id)
{
  if (!f_get_member_)
    {VAR_SPACE_ERR_EXIT("GetMember is not defined");}
  return f_get_member_ (members_,id);
}
inline TVariable  TVariable::GetMember (const TVariable &id) const
{
  if (!f_get_member_)
    {VAR_SPACE_ERR_EXIT("GetMember is not defined");}
  TVariable res= f_get_member_ (members_,id);
  res.is_const_= true;
  return res;
}
inline void  TVariable::FunctionCall (const TIdentifier &id, TVariableList &argv)
{
  if (!f_function_call_)
    {VAR_SPACE_ERR_EXIT("FunctionCall is not defined");}
  f_function_call_ (members_,id,argv);
}
inline void  TVariable::FunctionCall (const TIdentifier &id, TVariableList &argv) const
{
  if (!f_function_call_)
    {VAR_SPACE_ERR_EXIT("FunctionCall is not defined");}
  f_function_call_ (members_,id,argv);
}
inline void  TVariable::DirectCall (TVariableList &argv)
{
  if (!f_direct_call_)
    {VAR_SPACE_ERR_EXIT("DirectCall is not defined");}
  f_direct_call_ (argv);
}
inline void  TVariable::DirectCall (TVariableList &argv) const
{
  if (!f_direct_call_)
    {VAR_SPACE_ERR_EXIT("DirectCall is not defined");}
  f_direct_call_ (argv);
}
inline TVariable  TVariable::Push (void)
{
  if (!f_push_)
    {VAR_SPACE_ERR_EXIT("Push is not defined");}
  return f_push_ ();
}
inline void  TVariable::GetBegin (TForwardIterator &res)
{
  if (!f_get_begin_)
    {VAR_SPACE_ERR_EXIT("GetBegin is not defined");}
  f_get_begin_ (members_,res);
}
inline void  TVariable::GetBegin (TConstForwardIterator &res) const
{
  if (!f_get_begin_)
    {VAR_SPACE_ERR_EXIT("GetBegin is not defined");}
  f_get_begin_ (members_,res);
}
inline void  TVariable::GetEnd (TForwardIterator &res)
{
  if (!f_get_end_)
    {VAR_SPACE_ERR_EXIT("GetEnd is not defined");}
  f_get_end_ (members_,res);
}
inline void  TVariable::GetEnd (TConstForwardIterator &res) const
{
  if (!f_get_end_)
    {VAR_SPACE_ERR_EXIT("GetEnd is not defined");}
  f_get_end_ (members_,res);
}
inline void TVariable::WriteToStream (std::ostream &os, bool bare, const pt_string &indent) const
{
  if(!f_write_to_stream_)
    {VAR_SPACE_ERR_EXIT("WriteToStream is not defined");}
  f_write_to_stream_ (members_,os,bare,indent);
}
//-------------------------------------------------------------------------------------------

//!\brief specialization of PrimitiveGetAs and PrimitiveSetBy
//!\todo FIXME: error-message is unclear
#define SPECIALIZER(x_ptype)                                      \
  template<> inline void TVariable::PrimitiveGetAs (pt_##x_ptype &x) const   \
    {                                                             \
      if(!f_primitive_get_as_##x_ptype##_)                        \
        {VAR_SPACE_ERR_EXIT("cannot convert to pt_"#x_ptype);}    \
      f_primitive_get_as_##x_ptype##_(x);                         \
    }                                                             \
  template<> inline void TVariable::PrimitiveSetBy (const pt_##x_ptype &x)   \
    {                                                             \
      if(!f_primitive_set_by_##x_ptype##_)                        \
        {VAR_SPACE_ERR_EXIT("cannot convert from pt_"#x_ptype);}  \
      f_primitive_set_by_##x_ptype##_(x);                         \
    }
SPECIALIZER(int      )
SPECIALIZER(real     )
SPECIALIZER(bool     )
SPECIALIZER(string   )
#undef SPECIALIZER


//===========================================================================================
// specializations of TVariable::generator
//===========================================================================================

#define SPECIALIZER(x_type)  \
  template<> struct TVariable::generator<x_type>                                 \
  {                                                                              \
    TVariable &o;                                                                \
    generator(TVariable &outer) : o(outer) {}                                    \
    void operator() (x_type &x);                                                 \
  };
SPECIALIZER(unsigned short  )
SPECIALIZER(unsigned int    )
SPECIALIZER(unsigned long   )
SPECIALIZER(signed short    )
SPECIALIZER(signed int      )
SPECIALIZER(signed long     )
SPECIALIZER(bool            )

SPECIALIZER(float           )
SPECIALIZER(double          )
SPECIALIZER(long double     )

SPECIALIZER(pt_string       )
#undef SPECIALIZER

//-------------------------------------------------------------------------------------------

/*!\brief partial specialization for std::vector
    \note include variable_space_impl.h to use this partial specialization
    \note already instantiated for signed int, double, and long double
        (you need not to include variable_space_impl.h for these types)
    \warning you cannot instantiate TVariable::generator of std::vector for bool
        because std::vector\<bool\> is not a standard STL container.
*/
template <typename t_elem>
struct TVariable::generator<std::vector<t_elem> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (std::vector<t_elem> &x);
};
//-------------------------------------------------------------------------------------------

/*!\brief partial specialization for std::list
    \note include variable_space_impl.h to use this partial specialization
    \note already instantiated for signed int, double, and long double
        (you need not to include variable_space_impl.h for these types) */
template <typename t_elem>
struct TVariable::generator<std::list<t_elem> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (std::list<t_elem> &x);
};
//-------------------------------------------------------------------------------------------

/*!\brief partial specialization for std::map
    \note include variable_space_impl.h to use this partial specialization */
template <typename t_key, typename t_elem>
struct TVariable::generator<std::map<t_key,t_elem> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (std::map<t_key,t_elem> &x);
};
//-------------------------------------------------------------------------------------------



//===========================================================================================
// supplementary functions
//===========================================================================================

template <typename T>
inline bool AddToVarMap (TVariableMap &mmap, const std::string &identifier, T &x)
{
  TVariableMap::const_iterator itr=mmap.find(identifier);
  if (itr!=mmap.end())
    {LWARNING(identifier<<" was already registered to the variable map."); return false;}
  mmap[identifier]= TVariable(x);
  return true;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_space_h
//-------------------------------------------------------------------------------------------
