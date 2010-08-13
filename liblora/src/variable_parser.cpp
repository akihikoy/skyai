//-------------------------------------------------------------------------------------------
/*! \file    variable_parser.cpp
    \brief   liblora - parser for variable-space  (source)
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
#include <lora/variable_parser_impl.h>
#include <lora/stl_ext.h>
#include <lora/string_impl.h>  // NumericalContainerToString
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{
using namespace std;
using namespace boost::spirit::classic;

// explicit instantiation
template class TParserAgent <file_iterator<char> >;
template class TCodeParser <file_iterator<char> >;

//-------------------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream &lhs, const TLiteral &rhs)
{
  switch(rhs.LType)
  {
  case TLiteral::ltIdentifier  :  lhs<<rhs.LPrimitive.EString<<"[identifier]";    break;
  case TLiteral::ltPrimitive   :  lhs<<rhs.LPrimitive<<"[primitive]";  break;
  case TLiteral::ltList        :  lhs<<"("<<NumericalContainerToString(rhs.LList, ", ")<<")[list]";  break;
  default : LERROR("fatal!"); LDBGVAR(int(rhs.LType)); lexit(df);
  }
  return lhs;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TLiteralTable
//===========================================================================================

TLiteralTable::TLiteralTable(void)
{
  AddToKeywordSet(keywords_);
}
//-------------------------------------------------------------------------------------------

const TLiteral* TLiteralTable::Find(const TIdentifier &id) const
{
  TTable::const_iterator itr(table_.find(id));
  if (itr==table_.end())  return NULL;
  return &(itr->second);
}
//-------------------------------------------------------------------------------------------

TLiteralTable::TAddResult TLiteralTable::add_to_table(const TIdentifier &id, const TLiteral &value)
{
  if (keywords_.find(id)!=keywords_.end())
  {
    LERROR("`"<<id<<"\' is a reserved keyword");
    return arFailed;
  }
  TTable::iterator itr(table_.find(id));
  if (itr!=table_.end())
  {
    itr->second= value;
    return arOverwritten;
  }
  table_.insert(TTable::value_type(id,value));
  return arInserted;
}
//-------------------------------------------------------------------------------------------

TLiteralTable::TAddResult TLiteralTable::AddIdentifier(const TIdentifier &id, const TIdentifier &value)
{
  TLiteral l;
  l.LType= TLiteral::ltIdentifier;
  l.LPrimitive= TAnyPrimitive(pt_string(value));
  return add_to_table(id,l);
}
TLiteralTable::TAddResult TLiteralTable::AddLiteral(const TIdentifier &id, const std::list<TAnyPrimitive>  &value)
{
  TLiteral l;
  l.LType= TLiteral::ltList;
  l.LList= value;
  return add_to_table(id,l);
}
//-------------------------------------------------------------------------------------------

TLiteral EvaluateLiteral (const TLiteral &src, const TLiteralTable *literal_table, const TEvaluateLiteralConfig &config, bool &error)
{
  if (src.LType==TLiteral::ltIdentifier)
  {
    const TIdentifier &src_id(src.LPrimitive.EString);
    const TLiteral *alt(NULL);
    if(literal_table && (alt=literal_table->Find(src_id))!=NULL)
    {
      TLiteral res;
      if(alt->LType==TLiteral::ltIdentifier)
      {
        TEvaluateLiteralConfig sub_config(config); sub_config.ExitByError= false;
        res= EvaluateLiteral(*alt,literal_table,sub_config,error);  // NOTE: comment out this line not to evaluate recursively
      }
      else
        res= *alt;
      if (res.LType==TLiteral::ltIdentifier)
      {
        LERROR("failed to evaluate `"<<src_id<<"\'");
        error= true;
        if(config.ExitByError) lexit(df);
      }
      return res;
    }
    else
    {
      if (config.AllowId)  return src;
      LERROR("identifier `"<<src_id<<"\' is not registered in the literal-table");
      error= true;
      if(config.ExitByError) lexit(df);
      TLiteral res;  res.LType=TLiteral::ltIdentifier;  return res;
    }
  }
  else
  {
    return src;
  }
}
//-------------------------------------------------------------------------------------------

std::string ExpandIdentifier (const std::string &id, const TLiteralTable *literal_table, bool &error)
{
  std::string identifier(id);
  if(literal_table)
  {
    const TLiteral *alt(NULL);
    if((alt=literal_table->Find(identifier))!=NULL)  // NOTE: replace "if" by "while" to search recursively
    {
      if(alt->LType==TLiteral::ltIdentifier)
        identifier= alt->LPrimitive.EString;
      else
      {
        LERROR("`"<<identifier<<"\' does not store an identifier");
        LERROR("but, `"<<identifier<<" stores: "<<*alt);
        error= true;
        return identifier;
      }
    }
  }
  return identifier;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================

bool ParseFile (TParserInfoIn &in, TParserInfoOut *out)
{
  typedef file_iterator<char> TIterator;
  TIterator  first(in.FileName);
  if (!first)
  {
    LERROR("failed to open file: "<<in.FileName);
    return false;
  }

  TParserAgent<TIterator> pagent;

  TIterator last= first.make_end();

  parse_info<TIterator> info= pagent.Parse(in, first, last, out);
  if (out)  out->IsLast= (info.stop==last);
  return !pagent.Error();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// class TExtVariable
//-------------------------------------------------------------------------------------------

TExtVariable::TExtVariable (TExtForwardIterator &first, TExtForwardIterator &last)
  :
    kind_(kArray)
{
  LASSERT1op1(first.kind_,==,last.kind_);
  switch(first.kind_)
  {
  case TExtForwardIterator::kSingle :
    for(; first.entity_!=last.entity_; ++first.entity_)
      array_.push_back(*(first.entity_));
    break;
  case TExtForwardIterator::kArray :
    for(std::list<TForwardIterator>::iterator first_itr(first.array_.begin()),first_last(first.array_.end()),last_itr(last.array_.begin());
        first_itr!=first_last; ++first_itr,++last_itr)
    {
      for(; (*first_itr)!=(*last_itr); ++(*first_itr))
        array_.push_back(*(*first_itr));
    }
    break;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}

void TExtVariable::DirectAssign (const TVariable &value)
{
  switch(kind_)
  {
  case kSingle :  return entity_.DirectAssign(value);
  case kArray :   for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    itr->DirectAssign(value);
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
void TExtVariable::SetMember (const TVariable &id, const TVariable &value)
{
  switch(kind_)
  {
  case kSingle :  return entity_.SetMember(id, value);
  case kArray :   for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    itr->SetMember(id, value);
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}

TExtVariable TExtVariable::GetMember (const TVariable &id)
{
  switch(kind_)
  {
  case kSingle :  return TExtVariable(entity_.GetMember(id));
  case kArray :   {TExtVariable res(kArray);
                  for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    res.array_.push_back(itr->GetMember(id));
                  return res;}
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
  return TExtVariable();
}
void TExtVariable::FunctionCall (const TIdentifier &id, TVariableList &argv)
{
  switch(kind_)
  {
  case kSingle :  return entity_.FunctionCall(id,argv);
  case kArray :   for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    itr->FunctionCall(id,argv);
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
void TExtVariable::DirectCall (TVariableList &argv)
{
  switch(kind_)
  {
  case kSingle :  return entity_.DirectCall(argv);
  case kArray :   for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    itr->DirectCall(argv);
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
TExtVariable TExtVariable::Push (void)
{
  switch(kind_)
  {
  case kSingle :  return entity_.Push();
  case kArray :   {TExtVariable res(kArray);
                  for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    res.array_.push_back(itr->Push());
                  return res;}
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
  return TExtVariable();
}
void TExtVariable::GetBegin (TExtForwardIterator &res)
{
  switch(kind_)
  {
  case kSingle :  res.kind_= TExtForwardIterator::kSingle;
                  entity_.GetBegin(res.entity_);
                  return;
  case kArray :   res.kind_= TExtForwardIterator::kArray;
                  res.array_.clear();
                  for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    {res.array_.push_back(TForwardIterator()); itr->GetBegin(res.array_.back());}
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
void TExtVariable::GetEnd (TExtForwardIterator &res)
{
  switch(kind_)
  {
  case kSingle :  res.kind_= TExtForwardIterator::kSingle;
                  entity_.GetEnd(res.entity_);
                  return;
  case kArray :   res.kind_= TExtForwardIterator::kArray;
                  res.array_.clear();
                  for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    {res.array_.push_back(TForwardIterator()); itr->GetEnd(res.array_.back());}
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// class TExtForwardIterator
//-------------------------------------------------------------------------------------------

const TExtForwardIterator& TExtForwardIterator::operator++(void)
{
  switch(kind_)
  {
  case kSingle:  ++entity_; break;
  case kArray :
    for (std::list<TForwardIterator>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
      ++(*itr);
    break;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
  return *this;
}
const TExtForwardIterator& TExtForwardIterator::operator--(void)
{
  switch(kind_)
  {
  case kSingle:  --entity_; break;
  case kArray :
    for (std::list<TForwardIterator>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
      --(*itr);
    break;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
  return *this;
}
bool TExtForwardIterator::operator==(const TExtForwardIterator &rhs) const
{
  switch(kind_)
  {
  case kSingle:  return entity_==rhs.entity_;
  case kArray :
    {std::list<TForwardIterator>::const_iterator rhs_itr(rhs.array_.begin());
    for (std::list<TForwardIterator>::const_iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr,++rhs_itr)
      if ((*itr)!=(*rhs_itr))  return false;
    return true;}
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
  return false;
}
//-------------------------------------------------------------------------------------------

void TExtForwardIterator::i_dereference_()
{
  switch(kind_)
  {
  case kSingle:
    dereferenced_.kind_= TExtVariable::kSingle;
    dereferenced_.entity_= *entity_;
    return;
  case kArray :
    dereferenced_.kind_= TExtVariable::kArray;
    dereferenced_.array_.clear();
    for (std::list<TForwardIterator>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
      dereferenced_.array_.push_back(*(*itr));
    return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

