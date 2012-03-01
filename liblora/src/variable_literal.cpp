//-------------------------------------------------------------------------------------------
/*! \file    variable_literal.cpp
    \brief   liblora - certain program (source)
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
#include <lora/variable_literal.h>
#include <lora/variable_bindef.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{


std::ostream& operator<<(std::ostream &lhs, const TLiteral &rhs)
{
  if(rhs.IsNull())            lhs<<"()[null]";
  else if(rhs.IsIdentifier()) lhs<<rhs.AsIdentifier()<<"[identifier]";
  else if(rhs.IsCommand())    lhs<<rhs.AsCommand()<<"[command]";
  else if(rhs.IsType())       {for(TLiteral::TType::const_iterator itr(rhs.AsType().begin()),last(rhs.AsType().end());itr!=last;++itr) lhs<<" "<<bin::TypeStr(itr->Int()); lhs<<"[type]";}
  else if(rhs.IsPrimitive())  lhs<<rhs.AsPrimitive()<<"[primitive]";
  else if(rhs.IsList())       lhs<<"("<<NumericalContainerToString(rhs.AsList(), ", ")<<")[list]";
  else if(rhs.IsVariable())   {rhs.AsVariable().WriteToStream(lhs); lhs<<"[variable]";}
  else  {LERROR("fatal!"); lexit(df);}
  return lhs;
}
//-------------------------------------------------------------------------------------------


//!\todo FIXME: where should we define this function???
template <typename t_container>
void AddToKeywordSet (t_container &container)
{
  container.insert("true");
  container.insert("false");
  container.insert("inf");
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


//-------------------------------------------------------------------------------------------
TLiteral EvaluateLiteral (const TLiteral &src, const TLiteralTable *literal_table, const TEvaluateLiteralConfig &config, bool &error)
//-------------------------------------------------------------------------------------------
{
  if (src.IsIdentifier())
  {
    const TIdentifier &src_id(src.AsIdentifier());
    const TLiteral *alt(NULL);
    if(literal_table && (alt=literal_table->Find(src_id))!=NULL)
    {
      TLiteral res;
      if(alt->IsIdentifier() && config.Recursive)
      {
        TEvaluateLiteralConfig sub_config(config); sub_config.ExitByError= false;
        res= EvaluateLiteral(*alt,literal_table,sub_config,error);
      }
      else
        res= *alt;
      if (!config.AllowId && res.IsIdentifier())
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
      LERROR("if `"<<src_id<<"\' is a variable, use cast<TYPE>("<<src_id<<")");
      error= true;
      if(config.ExitByError) lexit(df);
      return LiteralId("");
    }
  }
  else
  {
    return src;
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of var_space
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
