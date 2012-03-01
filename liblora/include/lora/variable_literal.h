//-------------------------------------------------------------------------------------------
/*! \file    variable_literal.h
    \brief   liblora - certain program (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.07, 2012

    Copyright (C) 2012  Akihiko Yamaguchi

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
#ifndef loco_rabbits_variable_literal_h
#define loco_rabbits_variable_literal_h
//-------------------------------------------------------------------------------------------
#include <lora/variable_space.h>
#include <lora/variable_any.h>
//-------------------------------------------------------------------------------------------
#include <map>
#include <set>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{
//-------------------------------------------------------------------------------------------


class TLiteral;
std::ostream& operator<<(std::ostream &lhs, const TLiteral &rhs);

//===========================================================================================
class TLiteral
//===========================================================================================
{
public:
  enum TDataKind {dkNull=-1, dkIdentifier=0, dkCommand, dkType, dkPrimitive, dkList, dkVariable};
  typedef std::list<TAnyPrimitive> TType;  // this may be replaced by a type descriptor

  TLiteral() : kind_(dkNull), type_(list_) {}
  TLiteral(TDataKind kind)  : kind_(kind), type_(list_)  {}

  TLiteral(TDataKind kind, const TIdentifier &value)
      : kind_(dkIdentifier), primitive_(value), type_(list_)  {LASSERT(kind==dkIdentifier);}
  TLiteral(TDataKind kind, const pt_int &value)
      : kind_(dkCommand), primitive_(value), type_(list_)  {LASSERT(kind==dkCommand);}

  template <typename t_primitive>
  TLiteral(const t_primitive &value) : kind_(dkPrimitive), primitive_(value), type_(list_)  {}
  TLiteral(const std::list<TAnyPrimitive> &value) : kind_(dkList), list_(value), type_(list_)  {}
  TLiteral(const TVariable &var) : kind_(dkVariable), variable_(var), type_(list_)  {}

  TLiteral(const TLiteral &rhs) : type_(list_)
    {
      operator=(rhs);
    }
  const TLiteral& operator=(const TLiteral &rhs)
    {
      kind_= rhs.kind_;
      switch(kind_)
      {
      case dkNull       : break;
      case dkIdentifier : primitive_.Set(rhs.AsIdentifier()); break;
      case dkCommand    : primitive_.Set(rhs.AsCommand()); break;
      case dkType       : type_= rhs.AsType(); break;
      case dkPrimitive  : primitive_= rhs.AsPrimitive(); break;
      case dkList       : list_= rhs.AsList(); break;
      case dkVariable   : variable_= rhs.AsVariable(); break;
      default : LERROR("fatal!"); lexit(df);
      }
      return *this;
    }

  bool IsNull() const {return kind_==dkNull;}
  bool IsIdentifier() const {return kind_==dkIdentifier;}
  bool IsCommand() const {return kind_==dkCommand;}
  bool IsCommand(const pt_int &cmd) const {return kind_==dkCommand && primitive_.Int()==cmd;}
  bool IsType() const {return kind_==dkType;}
  bool IsPrimitive() const {return kind_==dkPrimitive || (kind_==dkList && list_.size()==1);}
  bool IsList() const {return kind_==dkList;}
  bool IsVariable() const {return kind_==dkVariable;}

  pt_string& AsIdentifier()  {return primitive_.String();}
  TAnyPrimitive& AsPrimitive()
    {
      if(kind_==dkList && list_.size()==1)  return list_.front();
      return primitive_;
    }
  std::list<TAnyPrimitive>& AsList()  {return list_;}
  TVariable& AsVariable()  {return variable_;}

  const pt_string& AsIdentifier() const {return primitive_.String();}
  const pt_int& AsCommand() const {return primitive_.Int();}
  const TType& AsType() const {return type_;}
  const TAnyPrimitive& AsPrimitive() const
    {
      if(kind_==dkList && list_.size()==1)  return list_.front();
      return primitive_;
    }
  const std::list<TAnyPrimitive>& AsList() const {return list_;}
  const TVariable& AsVariable() const {return variable_;}

  void SetId(const TIdentifier &value)
    {
      kind_= dkIdentifier;
      primitive_.Set(value);
    }
  void SetCmd(const pt_int &value)
    {
      kind_= dkCommand;
      primitive_.Set(value);
    }
  void SetType(const pt_int &type)
    {
      kind_= dkType;
      type_.clear();
      type_.push_front(type);
    }
  void AppendToType(const pt_int &type)
    {
      type_.push_front(type);
    }
  template <typename t_primitive>
  void Set(const t_primitive &value)
    {
      kind_= dkPrimitive;
      primitive_.Set(value);
      list_.clear();
    }
  void Set(const std::list<TAnyPrimitive> &value)
    {
      kind_= dkList;
      list_= value;
    }
  void Set(const TVariable &var)
    {
      kind_= dkVariable;
      variable_= var;
    }
  void Unset()
    {
      kind_= dkNull;
      list_.clear();
      variable_.Clear();
    }

  void ToList()
    {
      if(kind_==dkList)  return;
      if(kind_==dkPrimitive)
      {
        kind_= dkList;
        list_.clear();
        list_.push_back(primitive_);
        return;
      }
      LERROR("TLiteral::invalid conversion to list: "<<*this);
      lexit(df);
    }
  void AppendToList(const TAnyPrimitive &value)
    {
      ToList();
      list_.push_back(value);
    }

private:
  TDataKind       kind_;
  TAnyPrimitive   primitive_;
  std::list<TAnyPrimitive>  list_;
  TVariable       variable_;
  TType           &type_;
};
//-------------------------------------------------------------------------------------------

inline TLiteral LiteralId(const TIdentifier &value)
{
  return TLiteral(TLiteral::dkIdentifier, value);
}
inline TLiteral LiteralCmd(const pt_int &value)
{
  return TLiteral(TLiteral::dkCommand, value);
}
inline TLiteral LiteralType(const pt_int &value)
{
  TLiteral res(TLiteral::dkType);
  res.AppendToType(value);
  return res;
}
inline TLiteral LiteralEmptyList(void)
{
  return TLiteral(TLiteral::dkList);
}
//-------------------------------------------------------------------------------------------

inline TVariable Variable(TLiteral &literal)
{
  TVariable var;
  if(literal.IsNull())            {LERROR("Variable::the given literal is null"); lexit(df);}
  // else if(literal.IsIdentifier()) {LERROR("Variable::cannot convert an identifier `"<<literal.AsIdentifier()<<"\' to a TVariable"); lexit(df);}
  else if(literal.IsIdentifier()) var.Generate(literal.AsIdentifier());
  else if(literal.IsCommand())    {LERROR("Variable::cannot convert a command `"<<literal.AsCommand()<<"\' to a TVariable"); lexit(df);}
  else if(literal.IsType())       {LERROR("Variable::cannot convert a type to a TVariable"); lexit(df);}
  else if(literal.IsPrimitive())  var.Generate(literal.AsPrimitive());
  else if(literal.IsList())       var.Generate(literal.AsList());
  else if(literal.IsVariable())   return literal.AsVariable();
  else  {LERROR("fatal!"); lexit(df);}
  return var;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// literal operators:
//-------------------------------------------------------------------------------------------

#define DEF_OP1(x_op)  \
  inline TLiteral operator x_op (const TLiteral &rhs)                           \
  {                                                                             \
    LASSERT(rhs.IsPrimitive() || rhs.IsList());                                 \
    if(rhs.IsPrimitive())                                                       \
      return TLiteral(x_op rhs.AsPrimitive());                                  \
    else                                                                        \
    {                                                                           \
      TLiteral res(TLiteral::dkList);                                           \
      for(std::list<TAnyPrimitive>::const_iterator ritr(rhs.AsList().begin()),  \
              rend(rhs.AsList().end());                                         \
          ritr!=rend; ++ritr)                                                   \
      {                                                                         \
        res.AsList().push_back(x_op (*ritr));                                   \
      }                                                                         \
      return res;                                                               \
    }                                                                           \
  }

#define DEF_OP2(x_op)  \
  inline TLiteral operator x_op (const TLiteral &lhs, const TLiteral &rhs)                \
  {                                                                                       \
    LASSERT((lhs.IsPrimitive() && rhs.IsPrimitive()) || (lhs.IsList() && rhs.IsList()));  \
    if(lhs.IsPrimitive())                                                                 \
      return TLiteral(lhs.AsPrimitive() x_op rhs.AsPrimitive());                          \
    else                                                                                  \
    {                                                                                     \
      LASSERT1op1(lhs.AsList().size(),==,rhs.AsList().size());                            \
      TLiteral res(TLiteral::dkList);                                                     \
      for(std::list<TAnyPrimitive>::const_iterator litr(lhs.AsList().begin()),            \
              lend(lhs.AsList().end()),ritr(rhs.AsList().begin());                        \
          litr!=lend; ++litr,++ritr)                                                      \
      {                                                                                   \
        res.AsList().push_back((*litr) x_op (*ritr));                                     \
      }                                                                                   \
      return res;                                                                         \
    }                                                                                     \
  }

DEF_OP2(+)
DEF_OP2(-)

//! \todo FIXME: implement the operation for scalar * list
DEF_OP2(*)
DEF_OP2(/)
DEF_OP2(%)

DEF_OP2(&&)
DEF_OP2(||)
DEF_OP1(!)

DEF_OP2(==)
DEF_OP2(!=)
DEF_OP2(<=)
DEF_OP2(>=)
DEF_OP2(<)
DEF_OP2(>)

#undef DEF_OP1
#undef DEF_OP2
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TLiteralTable
//===========================================================================================
{
public:

  enum TAddResult {arFailed=0, arInserted, arOverwritten};

  typedef std::map<TIdentifier,TLiteral> TTable;

  TLiteralTable(void);

  const TLiteral* Find(const TIdentifier &id) const;

  TAddResult AddIdentifier(const TIdentifier &id, const TIdentifier &value)
    {return add_to_table(id,LiteralId(value));}

  template <typename t_primitive>
  TAddResult AddLiteral(const TIdentifier &id, const t_primitive &value)
    {return add_to_table(id,TLiteral(value));}
  TAddResult AddLiteral(const TIdentifier &id, const std::list<TAnyPrimitive>  &value)
    {return add_to_table(id,TLiteral(value));}
  TAddResult AddLiteral(const TIdentifier &id, const TLiteral &value)
    {return add_to_table(id,value);}

private:

  TTable table_;
  std::set<pt_string>  keywords_;

  TAddResult add_to_table(const TIdentifier &id, const TLiteral &value);

};
//-------------------------------------------------------------------------------------------


struct TEvaluateLiteralConfig
{
  bool AllowId;
  bool ExitByError;
  bool Recursive;  //!< if true, an identifier is recursively evaluated
  TEvaluateLiteralConfig() : AllowId(false), ExitByError(true), Recursive(false) {}
};
TLiteral EvaluateLiteral (const TLiteral &src, const TLiteralTable *literal_table, const TEvaluateLiteralConfig &config, bool &error);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of var_space
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_literal_h
//-------------------------------------------------------------------------------------------
