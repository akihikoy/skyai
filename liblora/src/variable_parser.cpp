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


bool ParseFile (TVariable &var, const std::string &filename, bool *is_last)
{
  typedef file_iterator<char> TIterator;
  TIterator  first(filename);
  if (!first)
  {
    LERROR("failed to open file: "<<filename);
    return false;
  }

  TParserAgent<TIterator> pagent;

  TIterator last= first.make_end();

  parse_info<TIterator> info= pagent.Parse(var, first, last);
  if (is_last)  *is_last= (info.stop==last);
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

