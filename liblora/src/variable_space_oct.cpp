//-------------------------------------------------------------------------------------------
/*! \file    variable_space_oct.cpp
    \brief   liblora - liboctave extension of variable-space  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    May.21, 2010-

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
#include <lora/variable_space_oct.h>
#include <lora/variable_bindef.h>
//-------------------------------------------------------------------------------------------
#include <lora/octave.h>
#include <lora/string.h>
#include <lora/type_gen_oct.h>
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{
using namespace std;
// using namespace boost;


//===========================================================================================
// function object generators for an iterator (double*)
// note: common in ColumnVector, RowVector, Matrix
//===========================================================================================

#define TO_PITR(x_entity)  (static_cast<double**>(x_entity))
void oct_iterator_entity_destructor_generator (void *entity)
{
  delete TO_PITR(entity);
}
void oct_iterator_increment_generator (void *entity)
{
  ++(*TO_PITR(entity));
}
void oct_iterator_decrement_generator (void *entity)
{
  --(*TO_PITR(entity));
}
TVariable oct_iterator_dereference_generator (void *entity)
{
  return TVariable(*(*TO_PITR(entity)));
}
bool oct_iterator_is_equal_to_generator (void *entity, void *rhs_entity)
{
  return (*TO_PITR(entity))==(*TO_PITR(rhs_entity));
}
#undef TO_PITR


struct TOctIteratorWrapper
{
  double *Entity;
  TOctIteratorWrapper(double *itr) : Entity(itr) {}
};

template<>
struct TForwardIterator::generator <TOctIteratorWrapper >
{
  TForwardIterator &o;
  generator(TForwardIterator &outer) : o(outer) {}
  void operator() (const TOctIteratorWrapper &init)
  {
    double **entity = new double*;
    *entity = init.Entity;
    o.entity_ = entity;
    entity=NULL;
    o.f_entity_destructor_ = oct_iterator_entity_destructor_generator;
    o.f_increment_    = oct_iterator_increment_generator;
    o.f_decrement_    = oct_iterator_decrement_generator;
    o.f_dereference_  = oct_iterator_dereference_generator;
    o.f_is_equal_to_  = oct_iterator_is_equal_to_generator;
  }
};

template <typename t_oct>
void oct_get_begin_generator (t_oct *x, const TVariableMap &, TForwardIterator &res)
{
  res.Generate<TOctIteratorWrapper>(TOctIteratorWrapper(OctBegin(*x)));
}
template <typename t_oct>
void oct_get_end_generator (t_oct *x, const TVariableMap &, TForwardIterator &res)
{
  res.Generate<TOctIteratorWrapper>(TOctIteratorWrapper(OctEnd(*x)));
}


//===========================================================================================
// function object generators for ColumnVector and RowVector
//===========================================================================================

template <typename t_oct_vec>
void oct_vec_direct_assign_generator (t_oct_vec *x, TVariableMap &, const TVariable &value)
{
  if (value.IsPrimitive())  // if value is primitive:
  {
    x->resize(1);
    (*x)(0)=value.PrimitiveGetAs<typename TypeExt<t_oct_vec>::value_type>();
    return;
  }
  // otherwise, try to "scan" (GetBegin..GetEnd)
  TConstForwardIterator  value_itr, value_last;
  value.GetEnd (value_last);
  int size(0);
  for (value.GetBegin (value_itr); value_itr!=value_last; ++value_itr)
    ++size;
  x->resize(size);
  value.GetBegin (value_itr);
  for (double *itr(OctBegin(*x)),*last(OctEnd(*x)); itr!=last; ++itr,++value_itr)
  {
    TVariable var(*itr);
    var.DirectAssign(*value_itr);
  }
}

template <typename t_oct_vec>
void oct_vec_set_member_generator (t_oct_vec *x, const TVariableMap &members, const TVariable &id, const TVariable &value)
{
  oct_vec_get_member_generator(x,members,id).DirectAssign(value);
}

template <typename t_oct_vec>
TVariable oct_vec_get_member_generator (t_oct_vec *x, const TVariableMap &, const TVariable &id)
{
  return TVariable((*x)(id.PrimitiveGetAs<pt_int>()));
}

template <typename t_oct_vec>
struct oct_vec_clear_function
{
  t_oct_vec *PtrEntity;
  oct_vec_clear_function(t_oct_vec &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "clear";}
  static void f (t_oct_vec *x, TVariableList &argv)
    {
      if (argv.size()!=1)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(void)");}
      TVariableList::iterator itr(argv.begin());
      ++itr; // skip void
      (*x)= t_oct_vec();
    }
};
template <typename t_oct_vec>
struct TVariable::generator<oct_vec_clear_function<t_oct_vec> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (oct_vec_clear_function<t_oct_vec> &x)
  {
    o.f_direct_call_ = boost::bind(oct_vec_clear_function<t_oct_vec>::f,x.PtrEntity,_1);
  }
};

template <typename t_oct_vec>
struct oct_vec_resize_function
{
  t_oct_vec *PtrEntity;
  oct_vec_resize_function(t_oct_vec &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "resize";}
  static void f (t_oct_vec *x, TVariableList &argv)
    {
      if (argv.size()!=2)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(int)");}
      TVariableList::iterator itr(argv.begin());
      ++itr; // skip void
      x->resize (itr->PrimitiveGetAs<pt_int>(),0.0);
    }
};
template <typename t_oct_vec>
struct TVariable::generator<oct_vec_resize_function<t_oct_vec> >
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (oct_vec_resize_function<t_oct_vec> &x)
  {
    o.f_direct_call_ = boost::bind(oct_vec_resize_function<t_oct_vec>::f,x.PtrEntity,_1);
  }
};

template <typename t_oct_vec>
void oct_vec_register_functions (t_oct_vec &entity, TVariableMap &mmap)
{
  #define ADD(x_func)  \
    oct_vec_##x_func##_function<t_oct_vec>  x_func##_func(entity);  \
    mmap[x_func##_func.Name()]= TVariable(x_func##_func);
  ADD(clear)
  ADD(resize)
  #undef ADD
}

template <typename t_oct_vec>
TVariable oct_vec_push_generator (t_oct_vec *x);

template <>
TVariable oct_vec_push_generator (ColumnVector *x)
{
  *x= x->stack(ColumnVector(1,0.0));
  return TVariable((*x)(x->length()-1));
}
template <>
TVariable oct_vec_push_generator (RowVector *x)
{
  *x= x->append(RowVector(1,0.0));
  return TVariable((*x)(x->length()-1));
}

template <typename t_oct_vec>
void oct_vec_write_to_stream_generator (t_oct_vec *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent)
{
  os<< "(";
  double *itr(OctBegin(*x)), *last(OctEnd(*x));
  if(itr==last) {os<<")"; return;}
  os<<ConvertToStr(*itr);
  for(++itr; itr!=last; ++itr)
    os<<", "<<ConvertToStr(*itr);
  os<<")";
}

template <typename t_oct_vec>
void oct_vec_write_to_binary_generator (t_oct_vec *x, const TVariableMap &members, TBinaryStack &bstack)
{
  AddPushID(bstack,"clear");
  AddCommand(bstack,bin::cmd::LLISTS);
  AddCommand(bstack,bin::cmd::FUNC_CALL);
  AddCommand(bstack,bin::cmd::POP);

  if (x->length()>0)
  {
    AddPushID(bstack,"resize");
    AddCommand(bstack,bin::cmd::LLISTS);
    AddPushLiteral(bstack,x->length());
    AddCommand(bstack,bin::cmd::FUNC_CALL);
    AddCommand(bstack,bin::cmd::POP);
    int idx(0);
    for (double *itr(OctBegin(*x)), *last(OctEnd(*x)); itr!=last; ++itr,++idx)
    {
      AddPushLiteral(bstack,idx);
      AddPushLiteral(bstack,*itr);
      AddCommand(bstack,bin::cmd::E_ASGN_P);
    }
  }
}

#define SET_FUNC_OBJECTS(x_oct_vec)  \
  o.f_direct_assign_ = boost::bind(oct_vec_direct_assign_generator<x_oct_vec>,&x,_1,_2);          \
                                                                                                  \
  o.f_set_member_ = boost::bind(oct_vec_set_member_generator<x_oct_vec>,&x,_1,_2,_3);             \
  o.f_get_member_ = boost::bind(oct_vec_get_member_generator<x_oct_vec>,&x,_1,_2);                \
                                                                                                  \
  oct_vec_register_functions(x,o.SetMemberMap());                                                 \
                                                                                                  \
  o.f_function_call_ = boost::bind(generic_function_call_generator,_1,_2,_3);                     \
                                                                                                  \
  o.f_push_      = boost::bind(oct_vec_push_generator<x_oct_vec>,&x);                             \
  o.f_get_begin_ = boost::bind(oct_get_begin_generator<x_oct_vec>,&x,_1,_2);                      \
  o.f_get_end_   = boost::bind(oct_get_end_generator<x_oct_vec>,&x,_1,_2);                        \
                                                                                                  \
  o.f_write_to_stream_ = boost::bind(oct_vec_write_to_stream_generator<x_oct_vec>,&x,_1,_2,_3,_4);\
  o.f_write_to_binary_ = boost::bind(oct_vec_write_to_binary_generator<x_oct_vec>,&x,_1,_2);

void TVariable::generator<ColumnVector>::operator() (ColumnVector &x)
{
  SET_FUNC_OBJECTS(ColumnVector);
}
void TVariable::generator<RowVector>::operator() (RowVector &x)
{
  SET_FUNC_OBJECTS(RowVector);
}

#undef SET_FUNC_OBJECTS

//-------------------------------------------------------------------------------------------



//===========================================================================================
// function object generators for a row vector of a Matrix
//===========================================================================================

class TRowOfMatrixIterator
{
public:
  TRowOfMatrixIterator(double *pos, int rows) : pos_(pos), rows_(rows)  {}
  TRowOfMatrixIterator operator++()
    {
      pos_+=rows_;
      return *this;
    }
  TRowOfMatrixIterator operator--()
    {
      pos_-=rows_;
      return *this;
    }
  bool operator==(const TRowOfMatrixIterator &rhs)
    {return pos_==rhs.pos_ && rows_==rhs.rows_;}
  bool operator!=(const TRowOfMatrixIterator &rhs)
    {return !operator==(rhs);}
  double& operator*()  {return *pos_;}
  double* operator->()  {return pos_;}
private:
  double* pos_;
  int rows_;  // step of Pos
};
class TRowOfMatrix
{
public:
  TRowOfMatrix(Matrix &v_m, int v_row) : m_(v_m), row_(v_row)  {}
  TRowOfMatrix(const TRowOfMatrix &rhs) : m_(rhs.m_), row_(rhs.row_)  {}
  TRowOfMatrixIterator Begin()
    {return TRowOfMatrixIterator(OctBegin(m_)+row_, m_.rows());}
  TRowOfMatrixIterator End()
    {return TRowOfMatrixIterator(OctBegin(m_)+(row_+m_.rows()*m_.cols()), m_.rows());}
  int Size() const {return m_.cols();}
  double& operator()(int idx)  {return m_(row_,idx);}
private:
  Matrix &m_;
  int row_;
};

void oct_rom_direct_assign_generator (TRowOfMatrix x, TVariableMap &, const TVariable &value)
{
  if (value.IsPrimitive())  // if value is primitive:
  {
    if(x.Size()!=1)
      {LERROR("in assignment of row vector of matrix: size mismatch: "
          "lhs.size()="<<x.Size()<<", rhs.size()="<<1); lexit(df);}
    x(0)=value.PrimitiveGetAs<TypeExt<Matrix>::value_type>();
    return;
  }
  // otherwise, try to "scan" (GetBegin..GetEnd)
  TConstForwardIterator  value_itr, value_last;
  value.GetEnd (value_last);
  int size(0);
  for (value.GetBegin (value_itr); value_itr!=value_last; ++value_itr)
    ++size;
  if(x.Size()!=size)
    {LERROR("in assignment of row vector of matrix: size mismatch: "
        "lhs.size()="<<x.Size()<<", rhs.size()="<<size); lexit(df);}
  value.GetBegin (value_itr);
  for (TRowOfMatrixIterator itr(x.Begin()),last(x.End()); itr!=last; ++itr,++value_itr)
  {
    TVariable var(*itr);
    var.DirectAssign(*value_itr);
  }
}

TVariable oct_rom_get_member_generator (TRowOfMatrix x, const TVariableMap &, const TVariable &id)
{
  return TVariable(x(id.PrimitiveGetAs<pt_int>()));
}

void oct_rom_set_member_generator (TRowOfMatrix x, const TVariableMap &members, const TVariable &id, const TVariable &value)
{
  oct_rom_get_member_generator(x,members,id).DirectAssign(value);
}

#define TO_PITR(x_entity)  (static_cast<TRowOfMatrixIterator*>(x_entity))
void oct_rom_iterator_entity_destructor_generator (void *entity)
{
  delete TO_PITR(entity);
}
void oct_rom_iterator_increment_generator (void *entity)
{
  ++(*TO_PITR(entity));
}
void oct_rom_iterator_decrement_generator (void *entity)
{
  --(*TO_PITR(entity));
}
TVariable oct_rom_iterator_dereference_generator (void *entity)
{
  return TVariable(*(*TO_PITR(entity)));
}
bool oct_rom_iterator_is_equal_to_generator (void *entity, void *rhs_entity)
{
  return (*TO_PITR(entity))==(*TO_PITR(rhs_entity));
}
#undef TO_PITR

template<>
struct TForwardIterator::generator <TRowOfMatrixIterator>
{
  TForwardIterator &o;
  generator(TForwardIterator &outer) : o(outer) {}
  void operator() (const TRowOfMatrixIterator &init)
  {
    TRowOfMatrixIterator *entity = new TRowOfMatrixIterator(init);
    o.entity_ = entity;
    entity=NULL;
    o.f_entity_destructor_ = oct_rom_iterator_entity_destructor_generator;
    o.f_increment_    = oct_rom_iterator_increment_generator;
    o.f_decrement_    = oct_rom_iterator_decrement_generator;
    o.f_dereference_  = oct_rom_iterator_dereference_generator;
    o.f_is_equal_to_  = oct_rom_iterator_is_equal_to_generator;
  }
};

void oct_rom_get_begin_generator (TRowOfMatrix x, const TVariableMap &, TForwardIterator &res)
{
  res.Generate<TRowOfMatrixIterator>(x.Begin());
}
void oct_rom_get_end_generator (TRowOfMatrix x, const TVariableMap &, TForwardIterator &res)
{
  res.Generate<TRowOfMatrixIterator>(x.End());
}

void oct_rom_write_to_stream_generator (TRowOfMatrix x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent)
{
  os<< "(";
  TRowOfMatrixIterator itr(x.Begin()), last(x.End());
  if(itr==last) {os<<")"; return;}
  os<<ConvertToStr(*itr);
  for(++itr; itr!=last; ++itr)
    os<<", "<<ConvertToStr(*itr);
  os<<")";
}

void oct_rom_write_to_binary_generator (TRowOfMatrix x, const TVariableMap &members, TBinaryStack &bstack)
{
  int idx(0);
  for (TRowOfMatrixIterator itr(x.Begin()), last(x.End()); itr!=last; ++itr,++idx)
  {
    AddPushLiteral(bstack,idx);
    AddPushLiteral(bstack,*itr);
    AddCommand(bstack,bin::cmd::E_ASGN_P);
  }
}

void TVariable::generator<TRowOfMatrix>::operator() (TRowOfMatrix &x)
{
  o.f_direct_assign_ = boost::bind(oct_rom_direct_assign_generator,x,_1,_2);

  o.f_set_member_ = boost::bind(oct_rom_set_member_generator,x,_1,_2,_3);
  o.f_get_member_ = boost::bind(oct_rom_get_member_generator,x,_1,_2);

  o.f_get_begin_ = boost::bind(oct_rom_get_begin_generator,x,_1,_2);
  o.f_get_end_   = boost::bind(oct_rom_get_end_generator,x,_1,_2);

  o.f_write_to_stream_ = boost::bind(oct_rom_write_to_stream_generator,x,_1,_2,_3,_4);
  o.f_write_to_binary_ = boost::bind(oct_rom_write_to_binary_generator,x,_1,_2);
}

//-------------------------------------------------------------------------------------------


//===========================================================================================
// function object generators for Matrix
//===========================================================================================

void oct_mat_direct_assign_generator (Matrix *x, TVariableMap &, const TVariable &value)
{
  if (value.IsPrimitive())  // if value is primitive:
  {
    x->resize(1,1);
    (*x)(0,0)=value.PrimitiveGetAs<TypeExt<Matrix>::value_type>();
    return;
  }
  // otherwise, try to "scan" (GetBegin..GetEnd)
  pt_int rows,cols;
  TVariableList argv;
  argv.push_back(TVariable(rows));
  value.FunctionCall("rows",argv);
  argv.clear();
  argv.push_back(TVariable(cols));
  value.FunctionCall("cols",argv);

  x->resize(rows,cols);

  TConstForwardIterator  value_itr, value_last;
  value.GetBegin (value_itr);
  value.GetEnd (value_last);
  double *itr(OctBegin(*x)),*last(OctEnd(*x));
  for (; itr!=last && value_itr!=value_last; ++itr,++value_itr)
  {
    TVariable var(*itr);
    var.DirectAssign(*value_itr);
  }
  if (itr!=last || value_itr!=value_last)
    {LWARNING("UNEXPECTED ERROR");}
}

TVariable oct_mat_get_member_generator (Matrix *x, const TVariableMap &, const TVariable &id)
{
  if(id.IsPrimitive())  // if id is int, then return row(int)
  {
    TRowOfMatrix rom(*x,id.PrimitiveGetAs<pt_int>());
    return TVariable(rom);
  }
  else  // if id is (int,int), then return element(int,int)
  {
    int id_size(id.Size());
    if(id_size==1)
    {
      TConstForwardIterator  id_itr;
      id.GetBegin (id_itr);
      TRowOfMatrix rom(*x,id_itr->PrimitiveGetAs<pt_int>());
      return TVariable(rom);
    }
    else if(id_size==2)
    {
      TConstForwardIterator  id_itr;
      id.GetBegin (id_itr);
      int row= id_itr->PrimitiveGetAs<pt_int>();
      ++id_itr;
      int col= id_itr->PrimitiveGetAs<pt_int>();
      return TVariable((*x)(row,col));
    }
    else
      {VAR_SPACE_ERR_EXIT("invalid matrix member identifier (size of id: "<<id_size<<")");}
  }
  return TVariable();
}

void oct_mat_set_member_generator (Matrix *x, const TVariableMap &members, const TVariable &id, const TVariable &value)
{
  oct_mat_get_member_generator(x,members,id).DirectAssign(value);
}

struct oct_mat_clear_function
{
  Matrix *PtrEntity;
  oct_mat_clear_function(Matrix &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "clear";}
  static void f (Matrix *x, TVariableList &argv)
    {
      if (argv.size()!=1)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(void)");}
      TVariableList::iterator itr(argv.begin());
      ++itr; // skip void
      (*x)= Matrix();
    }
};
template<> struct TVariable::generator<oct_mat_clear_function>
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (oct_mat_clear_function &x)
  {
    o.f_direct_call_ = boost::bind(oct_mat_clear_function::f,x.PtrEntity,_1);
  }
};

struct oct_mat_resize_function
{
  Matrix *PtrEntity;
  oct_mat_resize_function(Matrix &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "resize";}
  static void f (Matrix *x, TVariableList &argv)
    {
      if (argv.size()!=3)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be void(int,int)");}
      TVariableList::iterator itr(argv.begin());
      ++itr; // skip void
      pt_int rows(itr->PrimitiveGetAs<pt_int>());
      ++itr;
      pt_int cols(itr->PrimitiveGetAs<pt_int>());
      x->resize (rows,cols,0.0);
    }
};
template<> struct TVariable::generator<oct_mat_resize_function>
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (oct_mat_resize_function &x)
  {
    o.f_direct_call_ = boost::bind(oct_mat_resize_function::f,x.PtrEntity,_1);
  }
};

struct oct_mat_rows_function
{
  Matrix *PtrEntity;
  oct_mat_rows_function(Matrix &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "rows";}
  static void f (Matrix *x, TVariableList &argv)
    {
      if (argv.size()!=1)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be int(void)");}
      TVariableList::iterator itr(argv.begin());
      itr->PrimitiveSetBy<pt_int>(x->rows());
    }
};
template<> struct TVariable::generator<oct_mat_rows_function>
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (oct_mat_rows_function &x)
  {
    o.f_direct_call_ = boost::bind(oct_mat_rows_function::f,x.PtrEntity,_1);
  }
};

struct oct_mat_cols_function
{
  Matrix *PtrEntity;
  oct_mat_cols_function(Matrix &entity) : PtrEntity(&entity) {}
  static TIdentifier Name() {return "cols";}
  static void f (Matrix *x, TVariableList &argv)
    {
      if (argv.size()!=1)
        {VAR_SPACE_ERR_EXIT("syntax of "<<Name()<<" should be int(void)");}
      TVariableList::iterator itr(argv.begin());
      itr->PrimitiveSetBy<pt_int>(x->cols());
    }
};
template<> struct TVariable::generator<oct_mat_cols_function>
{
  TVariable &o;
  generator(TVariable &outer) : o(outer) {}
  void operator() (oct_mat_cols_function &x)
  {
    o.f_direct_call_ = boost::bind(oct_mat_cols_function::f,x.PtrEntity,_1);
  }
};

void oct_mat_register_functions (Matrix &entity, TVariableMap &mmap)
{
  #define ADD(x_func)  \
    oct_mat_##x_func##_function  x_func##_func(entity);  \
    mmap[x_func##_func.Name()]= TVariable(x_func##_func);
  ADD(clear)
  ADD(resize)
  ADD(rows)
  ADD(cols)
  #undef ADD
}

//!\note Push operator for Matrix requires large computational cost
TVariable oct_mat_push_generator (Matrix *x)
{
  *x= x->stack(RowVector(x->cols(),0.0));
  TRowOfMatrix rom(*x,x->rows()-1);
  return TVariable(rom);
}

void oct_mat_write_to_stream_generator (Matrix *x, const TVariableMap &members, std::ostream &os, bool bare, const pt_string &indent)
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
  if (x->rows()>0)
  {
    os<< new_indent << "resize("<<x->rows()<<", "<<x->cols()<<")" << std::endl;
    for (int row(0); row<x->rows(); ++row)
    {
      os<< new_indent <<"["<<row<<"] = ";
      oct_rom_write_to_stream_generator (TRowOfMatrix(*x,row), members, os, false, new_indent+VAR_SPACE_INDENT_STEP);
      os<< std::endl;
    }
  }
  if (!bare)
    os<< indent << "}";
}

void oct_mat_write_to_binary_generator (Matrix *x, const TVariableMap &members, TBinaryStack &bstack)
{
  AddPushID(bstack,"clear");
  AddCommand(bstack,bin::cmd::LLISTS);
  AddCommand(bstack,bin::cmd::FUNC_CALL);
  AddCommand(bstack,bin::cmd::POP);

  if (x->rows()>0)
  {
    AddPushID(bstack,"resize");
    AddCommand(bstack,bin::cmd::LLISTS);
    AddPushLiteral(bstack,x->rows());
    AddPushLiteral(bstack,x->cols());
    AddCommand(bstack,bin::cmd::FUNC_CALL);
    AddCommand(bstack,bin::cmd::POP);

    for (int row(0); row<x->rows(); ++row)
    {
      AddPushLiteral(bstack,row);
      AddCommand(bstack,bin::cmd::E_ASGN_CS);
      oct_rom_write_to_binary_generator (TRowOfMatrix(*x,row), members, bstack);
      AddCommand(bstack,bin::cmd::CASGN_END);
    }
  }
}

void TVariable::generator<Matrix>::operator() (Matrix &x)
{
  o.f_direct_assign_ = boost::bind(oct_mat_direct_assign_generator,&x,_1,_2);

  o.f_set_member_ = boost::bind(oct_mat_set_member_generator,&x,_1,_2,_3);
  o.f_get_member_ = boost::bind(oct_mat_get_member_generator,&x,_1,_2);

  oct_mat_register_functions(x,o.SetMemberMap());

  o.f_function_call_ = boost::bind(generic_function_call_generator,_1,_2,_3);

  o.f_push_      = boost::bind(oct_mat_push_generator,&x);
  o.f_get_begin_ = boost::bind(oct_get_begin_generator<Matrix>,&x,_1,_2);
  o.f_get_end_   = boost::bind(oct_get_end_generator<Matrix>,&x,_1,_2);

  o.f_write_to_stream_ = boost::bind(oct_mat_write_to_stream_generator,&x,_1,_2,_3,_4);
  o.f_write_to_binary_ = boost::bind(oct_mat_write_to_binary_generator,&x,_1,_2);
}
//-------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

