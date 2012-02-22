//-------------------------------------------------------------------------------------------
/*! \file    variable_binexec.cpp
    \brief   liblora - certain program (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.03, 2012

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
#include <lora/variable_binexec.h>
#include <lora/variable_parser.h>
#include <lora/stl_ext.h>
#include <lora/string_impl.h>  // NumericalContainerToString
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{



//===========================================================================================
// class TExtVariable
//===========================================================================================

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


//===========================================================================================
// class TBinExecutor
//===========================================================================================

//! call function of identifier func_id with arguments argv, store the return value into ret_val
/*virtual*/bool TBinExecutor::function_call(const std::string &func_id, std::list<TLiteral> &argv, TLiteral &ret_val)
{
  if(variable_stack_.empty())  return false;
  if(!variable_stack_.back().IfDefFunctionCall())  return false;

  TVariableList  argv_var;
  argv_var.push_back(Variable(ret_val));
  for(std::list<TLiteral>::iterator itr(argv.begin()),last(argv.end()); itr!=last; ++itr)
    argv_var.push_back(Variable(*itr));

  // CONV_ERR_CATCHER_S
  variable_stack_.back().FunctionCall(func_id, argv_var);
  // CONV_ERR_CATCHER_E
  return true;
}
//-------------------------------------------------------------------------------------------

//! access to the member of value
/*virtual*/TVariable TBinExecutor::member_access(const TLiteral &value, const TIdentifier &member)
{
  TIdentifier member_id(member);
  if(value.IsIdentifier())
  {
    TIdentifier id(value.AsIdentifier());
    LASSERT(!variable_stack_.empty());
    // VAR_ERR_CATCHER_S
    return variable_stack_.back().GetMember(TVariable(id)).ToVariable().GetMember(TVariable(member_id));
    // VAR_ERR_CATCHER_E
  }
  else if(value.IsVariable())
  {
    return value.AsVariable().GetMember(TVariable(member_id));
  }
  else
  {
    error_= true;
    LERROR("error: "<<member<<" is not a member of: "<<value);
    return TVariable();
  }
}
//-------------------------------------------------------------------------------------------

/*virtual*/void TBinExecutor::PartiallyExecute(const std::string& file_name, int line_num, bool error_stat)
{
  if(error_stat)  return;
  LASSERT(bin_stack_!=NULL);

  file_name_= file_name;
  line_num_= line_num;
  // std::cout<<"parsing:"<<file_name<<":"<<line_num<<std::endl;
  // PrintToStream(bin_stack);
  Execute(true);
}
//-------------------------------------------------------------------------------------------

/*virtual*/void TBinExecutor::Execute(bool from_current)
{
  LASSERT(bin_stack_!=NULL);

  if(!from_current)  bin_stack_->GoFirst();
  while(!bin_stack_->IsEOD())
  {
    exec_command(bin_stack_->ReadI(), *bin_stack_);
  }
}
//-------------------------------------------------------------------------------------------

/*virtual*/void TBinExecutor::exec_command(int command, const TBinaryStack &bstack)
{
  switch(command)
  {
  #define CALL_CMD_EXEC(x_cmd) case bin::cmd::x_cmd: cmd_##x_cmd (command, bstack); break;
  CALL_CMD_EXEC( PUSH      )
  CALL_CMD_EXEC( PUSHL     )
  CALL_CMD_EXEC( LAPPEND   )
  CALL_CMD_EXEC( PUSH_EMPL )
  CALL_CMD_EXEC( LLISTS    )
  CALL_CMD_EXEC( POP       )

  CALL_CMD_EXEC( M_ASGN_P  )
  CALL_CMD_EXEC( M_ASGN_CS )
  CALL_CMD_EXEC( E_ASGN_P  )
  CALL_CMD_EXEC( E_ASGN_CS )
  CALL_CMD_EXEC( P_ASGN_P  )
  CALL_CMD_EXEC( P_ASGN_CS )
  CALL_CMD_EXEC( F_ASGN_P  )
  CALL_CMD_EXEC( F_ASGN_CS )
  CALL_CMD_EXEC( CASGN_END )

  CALL_CMD_EXEC( FUNC_CALL )

  CALL_CMD_EXEC( CONCAT )
  CALL_CMD_EXEC( ADD    )
  CALL_CMD_EXEC( SUBT   )
  CALL_CMD_EXEC( MULT   )
  CALL_CMD_EXEC( DIV    )
  CALL_CMD_EXEC( MOD    )
  CALL_CMD_EXEC( AND    )
  CALL_CMD_EXEC( OR     )
  CALL_CMD_EXEC( NOT    )
  CALL_CMD_EXEC( EQ     )
  CALL_CMD_EXEC( NEQ    )
  CALL_CMD_EXEC( LTEQ   )
  CALL_CMD_EXEC( GTEQ   )
  CALL_CMD_EXEC( LT     )
  CALL_CMD_EXEC( GT     )
  CALL_CMD_EXEC( MEMBER )
  CALL_CMD_EXEC( ELEM   )

  CALL_CMD_EXEC( CAST   )

  CALL_CMD_EXEC( T_TO_LIST )
  #undef CALL_CMD_EXEC

  default:  FIXME("unknown command code:"<<command);
  }
}
//-------------------------------------------------------------------------------------------


#define CONV_ERR_CATCHER_S  \
  try {
#define CONV_ERR_CATCHER_E  \
  } catch (boost::bad_lexical_cast &) {       \
    print_error("bad_lexical_cast");          \
    lexit(df);                                \
  } catch (std::string &s) {                  \
    print_error(s);                           \
    lexit(df);                                \
  } catch (...) {                             \
    print_error("bad type conversion");       \
    lexit(df);                                \
  }

#define VAR_ERR_CATCHER_S  \
  try {
#define VAR_ERR_CATCHER_E  \
  } catch (std::string &s) {                  \
    print_error(s);                           \
    lexit(df);                                \
  } catch (...) {                             \
    print_error("fatal!");                    \
    lexit(df);                                \
  }

#define IMPL_CMD_EXEC(x_cmd)  void TBinExecutor::cmd_##x_cmd (int command, const TBinaryStack &bstack)

IMPL_CMD_EXEC( PUSH      )  // bin=[- vtype value]; push a value of vtype;
{
  using namespace bin;
  switch(bstack.ReadI())
  {
  case vtype::ID   :  literal_stack_.push_back(LiteralId(bstack.ReadS())); break;
  case vtype::INT  :  literal_stack_.push_back(TLiteral(bstack.ReadI())); break;
  case vtype::REAL :  literal_stack_.push_back(TLiteral(bstack.ReadR())); break;
  case vtype::BOOL :  literal_stack_.push_back(TLiteral(bstack.ReadB())); break;
  case vtype::STR  :  literal_stack_.push_back(TLiteral(bstack.ReadS())); break;
  case vtype::TYPE :  literal_stack_.push_back(LiteralType(bstack.ReadI())); break;
  default:  FIXME("unknown value type code");
  }
}
IMPL_CMD_EXEC( PUSHL     )  // bin=[- N vtype value1 .. valueN]; push a list (value1,..,valueN) of vtype;
{
  using namespace bin;
  int num= bstack.ReadI();
  literal_stack_.push_back(TLiteral(LiteralEmptyList()));
  TLiteral  &back(literal_stack_.back());
  switch(bstack.ReadI())
  {
  case vtype::INT  :  for(;num>0;--num) back.AppendToList(TAnyPrimitive(bstack.ReadI())); break;
  case vtype::REAL :  for(;num>0;--num) back.AppendToList(TAnyPrimitive(bstack.ReadR())); break;
  case vtype::BOOL :  for(;num>0;--num) back.AppendToList(TAnyPrimitive(bstack.ReadB())); break;
  case vtype::STR  :  for(;num>0;--num) back.AppendToList(TAnyPrimitive(bstack.ReadS())); break;
  default:  FIXME("unknown value type code");
  }
}
IMPL_CMD_EXEC( LAPPEND   )  // bin=[-]; pop two values(1,2), append 1 to 2:(2,1), push the result;
{
  TLiteral value= pop_literal();
  literal_stack_.back().AppendToList(value.AsPrimitive());
}
IMPL_CMD_EXEC( PUSH_EMPL )  // bin=[-]; push an empty list;
{
  literal_stack_.push_back(TLiteral(LiteralEmptyList()));
}
IMPL_CMD_EXEC( LLISTS    )  // bin=[-]; start list-of-literals (in l-o-l, PUSH, PUSHL, VLIST{S,E} are available);
{
  literal_stack_.push_back(LiteralCmd(bin::cmd::LLISTS));
}
IMPL_CMD_EXEC( POP       )  // bin=[-]; pop a value;
{
  literal_stack_.pop_back();
}

IMPL_CMD_EXEC( M_ASGN_P  )  // bin=[-]; pop two values(1,2;2 shoud be an identifier), assign:2=1;
{
  TLiteral value= pop_literal();
  std::string identifier(pop_id());

  LASSERT(!variable_stack_.empty());
  CONV_ERR_CATCHER_S
  variable_stack_.back().SetMember(TVariable(identifier), Variable(value));
  CONV_ERR_CATCHER_E
}
IMPL_CMD_EXEC( M_ASGN_CS )  // bin=[-]; pop an identifier, start composite assign:id={..};
{
  std::string identifier(pop_id());

  LASSERT(!variable_stack_.empty());
  TExtVariable  member;
  VAR_ERR_CATCHER_S
  member= variable_stack_.back().GetMember(TVariable(identifier));
  VAR_ERR_CATCHER_E
  variable_stack_.push_back(member);
}
IMPL_CMD_EXEC( E_ASGN_P  )  // bin=[-]; pop two values(1,2), elemental assign:[2]=1;
{
  TLiteral value= pop_literal();
  TLiteral key= pop_literal();

  LASSERT(!variable_stack_.empty());
  CONV_ERR_CATCHER_S
  variable_stack_.back().SetMember(Variable(key), Variable(value));
  CONV_ERR_CATCHER_E
}
IMPL_CMD_EXEC( E_ASGN_CS )  // bin=[-]; pop a value, start elemental composite assign:[val]={..};
{
  TLiteral key= pop_literal();

  LASSERT(!variable_stack_.empty());
  TExtVariable  member;
  VAR_ERR_CATCHER_S
  member= variable_stack_.back().GetMember(Variable(key));
  VAR_ERR_CATCHER_E
  variable_stack_.push_back(member);
}
IMPL_CMD_EXEC( P_ASGN_P  )  // bin=[-]; pop a value, push:[]=val;
{
  TLiteral value= pop_literal();

  LASSERT(!variable_stack_.empty());
  CONV_ERR_CATCHER_S
  variable_stack_.back().Push().DirectAssign(Variable(value));
  CONV_ERR_CATCHER_E
}
IMPL_CMD_EXEC( P_ASGN_CS )  // bin=[-]; start composite push:[]={..};
{
  LASSERT(!variable_stack_.empty());
  TExtVariable  new_var;
  VAR_ERR_CATCHER_S
  new_var= variable_stack_.back().Push();
  VAR_ERR_CATCHER_E
  variable_stack_.push_back(new_var);
}
IMPL_CMD_EXEC( F_ASGN_P  )  // bin=[-]; pop a value, fill:[@]=val;
{
  TLiteral value= pop_literal();
  TExtForwardIterator itr,ilast;

  LASSERT(!variable_stack_.empty());
  VAR_ERR_CATCHER_S
  variable_stack_.back().GetBegin(itr);
  variable_stack_.back().GetEnd(ilast);
  VAR_ERR_CATCHER_E

  TVariable  var_value (Variable(value));
  CONV_ERR_CATCHER_S
  for (; itr!=ilast; ++itr)
    itr->DirectAssign(var_value);
  CONV_ERR_CATCHER_E
}
IMPL_CMD_EXEC( F_ASGN_CS )  // bin=[-]; start composite fill:[@]={..};
{
  LASSERT(!variable_stack_.empty());
  TExtForwardIterator itr,ilast;
  VAR_ERR_CATCHER_S
  variable_stack_.back().GetBegin(itr);
  variable_stack_.back().GetEnd(ilast);
  VAR_ERR_CATCHER_E
  variable_stack_.push_back(TExtVariable(itr,ilast));
}
IMPL_CMD_EXEC( CASGN_END )  // finish M_ASGN_CS, E_ASGN_CS, P_ASGN_CS, F_ASGN_CS;
{
  LASSERT(!variable_stack_.empty());
  variable_stack_.pop_back();
}

IMPL_CMD_EXEC( FUNC_CALL )  // bin=[-]; pop list-of-literals, pop an identifier, call function:id(l-o-l);
{
  TEvaluateLiteralConfig config;  config.AllowId= true;
  std::list<TLiteral>  argv;
  pop_literal_list(argv,config);
  std::string identifier(pop_id());

  TLiteral ret_val(LiteralEmptyList());
  if(!function_call(identifier, argv, ret_val))
    print_error("failed to execute function: "+identifier);
  literal_stack_.push_back(ret_val);
}

IMPL_CMD_EXEC( CONCAT )  // bin=[-]; pop two values(1,2;2 shoud be an identifier), concatenate:2##1, push the result as an identifier;
{
  std::string id2(pop_id()), id1(pop_id());
  literal_stack_.push_back(LiteralId(id1+id2));
}
IMPL_CMD_EXEC( ADD    )  // bin=[-]; pop two values, add them(2+1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 + value1);
}
IMPL_CMD_EXEC( SUBT   )  // bin=[-]; pop two values, subtract them(2-1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 - value1);
}
IMPL_CMD_EXEC( MULT   )  // bin=[-]; pop two values, multiply them(2*1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 * value1);
}
IMPL_CMD_EXEC( DIV    )  // bin=[-]; pop two values, divide them(2/1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 / value1);
}
IMPL_CMD_EXEC( MOD    )  // bin=[-]; pop two values, compute mod(2%1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 % value1);
}
IMPL_CMD_EXEC( AND    )  // bin=[-]; pop two values, compute and(2&&1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 && value1);
}
IMPL_CMD_EXEC( OR     )  // bin=[-]; pop two values, compute or(2||1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 || value1);
}
IMPL_CMD_EXEC( NOT    )  // bin=[-]; pop a value, compute not(!1), push the result;
{
  TLiteral value1(pop_literal());
  literal_stack_.push_back( ! value1);
}
IMPL_CMD_EXEC( EQ     )  // bin=[-]; pop two values, compute equality(2==1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 == value1);
}
IMPL_CMD_EXEC( NEQ    )  // bin=[-]; pop two values, compute inequality(2!=1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 != value1);
}
IMPL_CMD_EXEC( LTEQ   )  // bin=[-]; pop two values, compute relation(2<=1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 <= value1);
}
IMPL_CMD_EXEC( GTEQ   )  // bin=[-]; pop two values, compute relation(2>=1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 >= value1);
}
IMPL_CMD_EXEC( LT     )  // bin=[-]; pop two values, compute relation(2<1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 < value1);
}
IMPL_CMD_EXEC( GT     )  // bin=[-]; pop two values, compute relation(2>1), push the result;
{
  TLiteral value1(pop_literal()),value2(pop_literal());
  literal_stack_.push_back(value2 > value1);
}
IMPL_CMD_EXEC( MEMBER )  // bin=[-]; pop two values(1,2;1 shoud be an identifier), get a member ref(2.1), push the result;
{
  TEvaluateLiteralConfig config;  config.AllowId= true;
  std::string member(pop_id());
  TLiteral value(pop_literal(config));
  TVariable var(member_access(value,member));
  literal_stack_.push_back(var);
}
IMPL_CMD_EXEC( ELEM   )  // bin=[-]; pop two values, get an elemental ref(2[1]), push the result;
{
}

IMPL_CMD_EXEC( CAST   )  // bin=[-]; pop two values(1,2;2 should be a type), cast 1 to 2(cast<2>(1), push the result;
{
  TEvaluateLiteralConfig config;  config.AllowId= true;
  TLiteral value(pop_literal(config)),type(pop_literal());
  if(!type.IsType())  {LERROR("type is required, but used: "<<type); error_= true; return;}

  if(value.IsVariable())
  {
    switch(type.AsType().front().Int())
    {
    case bin::vtype::INT  :
      {
        TLiteral  casted(value.AsVariable().PrimitiveGetAs<pt_int>());
        literal_stack_.push_back(casted);
      }
      break;
    case bin::vtype::REAL :
      {
        TLiteral  casted(value.AsVariable().PrimitiveGetAs<pt_real>());
        literal_stack_.push_back(casted);
      }
      break;
    case bin::vtype::BOOL :
      {
        TLiteral  casted(value.AsVariable().PrimitiveGetAs<pt_bool>());
        literal_stack_.push_back(casted);
      }
      break;
    case bin::vtype::STR  :
      {
        TLiteral  casted(value.AsVariable().PrimitiveGetAs<pt_string>());
        literal_stack_.push_back(casted);
      }
      break;
    case bin::vtype::LIST :
      {
        FIXME("...");
      }
      break;
    default:
      FIXME("fatal!");
      break;
    }
  }
  else
  {
    FIXME("fixme");
  }
}

IMPL_CMD_EXEC( T_TO_LIST )  // bin=[-]; pop a type(1), push the list of it (list<t>);
{
  TLiteral type(pop_literal());
  if(!type.IsType())  {LERROR("type is required, but used: "<<type); error_= true; return;}
  type.AppendToType(bin::vtype::LIST);
}

#undef IMPL_CMD_EXEC

#undef VAR_ERR_CATCHER_S
#undef VAR_ERR_CATCHER_E

#undef CONV_ERR_CATCHER_S
#undef CONV_ERR_CATCHER_E


//===========================================================================================

static void partially_execute(TBinExecutor *executor, TBinaryStack *bin_stack, const std::string& file_name, int line_num, bool error_stat)
{
  bin_stack->GoFirst();
  executor->PartiallyExecute(file_name,line_num,error_stat);
  bin_stack->Clear();
}
//-------------------------------------------------------------------------------------------

bool LoadFromFile (const std::string &file_name, TVariable &var, TLiteralTable &literal_table)
{
  TBinExecutor executor;

  TBinaryStack bin_stack;

  executor.PushVariable(var);
  executor.SetBinStack(&bin_stack);
  executor.SetLiteralTable(&literal_table);

  TParserCallbacks callbacks;
  // callbacks.OnCommandPushed= callback;
  callbacks.OnEndOfLine= boost::bind(&partially_execute,&executor,&bin_stack,_1,_2,_3);
  if(ParseFile(file_name,bin_stack,callbacks))
  {
    partially_execute(&executor,&bin_stack,file_name,-1,false);
      //! this code is needed if there is no newline at the end of file; \todo FIXME: the line number (-1)
    executor.PopVariable();
    LASSERT(executor.VariableStackSize()==0);
    return executor.Error();
  }
  return false;
}
//-------------------------------------------------------------------------------------------

bool ExecuteBinary (const TBinaryStack &bin_stack, TVariable &var, TLiteralTable &literal_table)
{
  TBinExecutor executor;
  executor.PushVariable(var);
  executor.SetBinStack(&bin_stack);
  executor.SetLiteralTable(&literal_table);
  executor.Execute(true);
  executor.PopVariable();
  LASSERT(executor.VariableStackSize()==0);
  return executor.Error();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of var_space
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
