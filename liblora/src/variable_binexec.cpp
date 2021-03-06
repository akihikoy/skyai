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
#include <lora/boost_filesystem_ext.h>
#include <boost/bind.hpp>
#include <boost/filesystem/operations.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{

inline boost::filesystem::path  operator+ (const boost::filesystem::path &file_path, const std::string &rhs)
{
  // return file_path.parent_path()/(file_path.filename()+rhs);
  return ConcatenatePath(file_path,rhs);
}


//===========================================================================================
// built-in functions
//===========================================================================================

// return void:

// FIXME: following code does not work.  fix me.
// void dcopy(src_var,dest_var): Copy `src_var' to `dest_var' by using DirectAssign
// void builtin_function_dcopy (TBinExecutor &context, var_space::TVariableList &argv)
// {
  // using namespace loco_rabbits::var_space;
  // if (argv.size()!=3)
    // {VAR_SPACE_ERR_EXIT("syntax of " "dcopy" " should be void(src_var,dest_var)");}
  // TVariableList::iterator itr(argv.begin());
  // ++itr;  // skip the return (void)
  // TVariable &arg1(*itr); ++itr;
  // TVariable &arg2(*itr); ++itr;

// LDBGVAR(arg1);
// LDBGVAR(arg2);
  // arg2.DirectAssign(arg1);
// }

// return real:

static void builtin_function_pi (TBinExecutor &context, TVariableList &argv)
{
  if (argv.size()!=1)
    {VAR_SPACE_ERR_EXIT("syntax of pi should be real()");}
  TVariableList::const_iterator itr(argv.begin());
  argv.front().PrimitiveSetBy<pt_real>(REAL_PI);
}

#define DEF_UNARY_FUNC(x_func)  \
  static void builtin_function_##x_func (TBinExecutor &context, TVariableList &argv) \
  {                                                                         \
    if (argv.size()!=2)                                                     \
      {VAR_SPACE_ERR_EXIT("syntax of " #x_func " should be real(real)");}   \
    TVariableList::const_iterator itr(argv.begin());                        \
    ++itr; /*skip return value*/                                            \
    pt_real arg1(itr->PrimitiveGetAs<pt_real>());                           \
    argv.front().PrimitiveSetBy<pt_real>(real_##x_func(arg1));              \
  }
#define DEF_BINARY_FUNC(x_func)  \
  static void builtin_function_##x_func (TBinExecutor &context, TVariableList &argv) \
  {                                                                         \
    if (argv.size()!=3)                                                     \
      {VAR_SPACE_ERR_EXIT("syntax of " #x_func " should be real(real,real)");} \
    TVariableList::const_iterator itr(argv.begin());                        \
    ++itr; /*skip return value*/                                            \
    pt_real arg1(itr->PrimitiveGetAs<pt_real>());                           \
    ++itr;                                                                  \
    pt_real arg2(itr->PrimitiveGetAs<pt_real>());                           \
    argv.front().PrimitiveSetBy<pt_real>(real_##x_func(arg1,arg2));         \
  }
DEF_UNARY_FUNC (acos  )
DEF_UNARY_FUNC (asin  )
DEF_UNARY_FUNC (atan  )
DEF_BINARY_FUNC(atan2 )
DEF_UNARY_FUNC (ceil  )
DEF_UNARY_FUNC (cos   )
DEF_UNARY_FUNC (cosh  )
DEF_UNARY_FUNC (exp   )
DEF_UNARY_FUNC (fabs  )
DEF_UNARY_FUNC (floor )
DEF_BINARY_FUNC(fmod  )
DEF_UNARY_FUNC (log   )
DEF_UNARY_FUNC (log10 )
DEF_BINARY_FUNC(pow   )
DEF_UNARY_FUNC (sin   )
DEF_UNARY_FUNC (sinh  )
DEF_UNARY_FUNC (sqrt  )
DEF_UNARY_FUNC (tan   )
DEF_UNARY_FUNC (tanh  )
DEF_UNARY_FUNC (round )
#undef DEF_UNARY_FUNC
#undef DEF_BINARY_FUNC

static void builtin_function_norm (TBinExecutor &context, TVariableList &argv)
{
  if (argv.size()!=2)
    {VAR_SPACE_ERR_EXIT("syntax of " "norm" " should be real(list<real>)");}
  TVariableList::iterator itr(argv.begin());
  TVariable &res(*itr); ++itr;
  TVariable &arg1(*itr);

  pt_real n(0.0l);
  TForwardIterator a1itr,a1last;
  arg1.GetBegin(a1itr); arg1.GetEnd(a1last);
  for(; a1itr!=a1last; ++a1itr)
    n+= Square(a1itr->PrimitiveGetAs<pt_real>());
  res.PrimitiveSetBy<pt_real>(real_sqrt(n));
}

// return bool:

static void builtin_function_and (TBinExecutor &context, TVariableList &argv)
{
  if (argv.size()!=2)
    {VAR_SPACE_ERR_EXIT("syntax of " "and" " should be bool(list<bool>)");}
  TVariableList::iterator itr(argv.begin());
  TVariable &res(*itr); ++itr;
  TVariable &arg1(*itr);

  TForwardIterator a1itr,a1last;
  arg1.GetBegin(a1itr); arg1.GetEnd(a1last);
  for(; a1itr!=a1last; ++a1itr)
    if(!a1itr->PrimitiveGetAs<pt_bool>())
    {
      res.PrimitiveSetBy<pt_bool>(false);
      return;
    }
  res.PrimitiveSetBy<pt_bool>(true);
}

static void builtin_function_or (TBinExecutor &context, TVariableList &argv)
{
  if (argv.size()!=2)
    {VAR_SPACE_ERR_EXIT("syntax of " "or" " should be bool(list<bool>)");}
  TVariableList::iterator itr(argv.begin());
  TVariable &res(*itr); ++itr;
  TVariable &arg1(*itr);

  TForwardIterator a1itr,a1last;
  arg1.GetBegin(a1itr); arg1.GetEnd(a1last);
  for(; a1itr!=a1last; ++a1itr)
    if(a1itr->PrimitiveGetAs<pt_bool>())
    {
      res.PrimitiveSetBy<pt_bool>(true);
      return;
    }
  res.PrimitiveSetBy<pt_bool>(false);
}

//! bool finclude(var,str): Functional include: load data to `var' from file `str'
void builtin_function_finclude (TBinExecutor &context, var_space::TVariableList &argv)
{
  using namespace loco_rabbits::var_space;
  if (argv.size()!=3)
    {VAR_SPACE_ERR_EXIT("syntax of " "finclude" " should be bool(var,str)");}
  TVariableList::iterator itr(argv.begin());
  TVariable &res(*itr); ++itr;
  TVariable &arg1(*itr); ++itr;
  TVariable &arg2(*itr); ++itr;

  boost::filesystem::path file_path;
  if(!context.SearchFile(arg2.PrimitiveGetAs<pt_string>(), file_path))
  {
    LERROR("File not found: "<<arg2.PrimitiveGetAs<pt_string>());
    res.PrimitiveSetBy<pt_bool>(false);
    return;
  }

  bool load_res;
  if(!(load_res= var_space::LoadFromFile(file_path.string(),arg1,context.LiteralTable(),&context.BuiltinFunctions())))
    LERROR("Failed to load: "<<file_path.string());
  res.PrimitiveSetBy<pt_bool>(load_res);
}

// return list:

static void builtin_function_shuffle (TBinExecutor &context, TVariableList &argv)
{
  if (argv.size()!=3)
    {VAR_SPACE_ERR_EXIT("syntax of " "shuffle" " should be list(list,list)");}
  TVariableList::iterator itr(argv.begin());
  TVariable &res(*itr); ++itr;
  TVariable &arg1(*itr); ++itr;
  TVariable &arg2(*itr);

  TForwardIterator a2itr,a2last;
  arg2.GetBegin(a2itr); arg2.GetEnd(a2last);
  for(; a2itr!=a2last; ++a2itr)
    res.Push().DirectAssign( arg1.GetMember(*a2itr) );
}



//===========================================================================================
// class TBuiltinFunctions
//===========================================================================================

void TBuiltinFunctions::AddDefaultFunctions()
{
  #define ADD(x_func)  Add(#x_func, TFunction(rtVoid, &builtin_function_##x_func));
  // ADD( dcopy  )
  #undef ADD

  #define ADD(x_func)  Add(#x_func, TFunction(rtReal, &builtin_function_##x_func));
  ADD( pi     )
  ADD( acos   )
  ADD( asin   )
  ADD( atan   )
  ADD( atan2  )
  ADD( ceil   )
  ADD( cos    )
  ADD( cosh   )
  ADD( exp    )
  ADD( fabs   )
  ADD( floor  )
  ADD( fmod   )
  ADD( log    )
  ADD( log10  )
  ADD( pow    )
  ADD( sin    )
  ADD( sinh   )
  ADD( sqrt   )
  ADD( tan    )
  ADD( tanh   )
  ADD( round  )

  ADD( norm   )
  #undef ADD

  #define ADD(x_func)  Add(#x_func, TFunction(rtBool, &builtin_function_##x_func));
  ADD( and      )
  ADD( or       )
  ADD( finclude )
  #undef ADD

  #define ADD(x_func)  Add(#x_func, TFunction(rtList, &builtin_function_##x_func));
  ADD( shuffle )
  #undef ADD
}
//-------------------------------------------------------------------------------------------



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

bool TBinExecutor::AddPath (const std::string &dir_name)
{
  using namespace boost::filesystem;
  if (path_list_==NULL)
  {
    print_error("add_path: failed because path-list is not assigned to the TBinExecutor's instance");
    return false;
  }
  boost::filesystem::path absolute_dir;
  if(!SearchFile(dir_name,absolute_dir))
    path_list_->push_back (absolute(path(dir_name/*,native*/)));
  else
    path_list_->push_back (absolute_dir);
  return true;
}
//-------------------------------------------------------------------------------------------

bool TBinExecutor::SearchFile (const boost::filesystem::path &file_path, boost::filesystem::path &absolute_path, const char *extension) const
{
  using namespace boost::filesystem;
  if (file_path.is_absolute())
  {
    if (exists((absolute_path= file_path)))  return true;
    if (extension && exists((absolute_path= file_path.parent_path()/(file_path.filename()+extension))))  return true;
    return false;
  }

  if (exists((absolute_path= current_dir_/file_path)))  return true;
  if (extension && exists((absolute_path= current_dir_/file_path+extension)))  return true;

  if (path_list_==NULL)  return false;

  path  tmp_path(file_path);
  for (std::list<path>::const_iterator ditr(path_list_->begin()), ditr_last(path_list_->end()); ditr!=ditr_last; ++ditr)
    if (exists((absolute_path= *ditr/tmp_path)))  return true;
  if (extension)
  {
    tmp_path= file_path+extension;
    for (std::list<path>::const_iterator ditr(path_list_->begin()), ditr_last(path_list_->end()); ditr!=ditr_last; ++ditr)
      if (exists((absolute_path= *ditr/tmp_path)))  return true;
  }
  return false;
}
//-------------------------------------------------------------------------------------------

//! call function of identifier func_id with arguments argv, store the return value into ret_val
/*virtual*/bool TBinExecutor::function_call(const std::string &func_id, std::list<TLiteral> &argv, TLiteral &ret_val)
{
  if(!variable_stack_.empty() && variable_stack_.back().FunctionExists(func_id))
  {
    TVariableList  argv_var;
    argv_var.push_back(Variable(ret_val));
    for(std::list<TLiteral>::iterator itr(argv.begin()),last(argv.end()); itr!=last; ++itr)
      argv_var.push_back(Variable(*itr));

    // CONV_ERR_CATCHER_S
    variable_stack_.back().FunctionCall(func_id, argv_var);
    // CONV_ERR_CATCHER_E
    return true;
  }
  else if(const TBuiltinFunctions::TFunction *builtin_f= builtin_functions_.Find(func_id))
  {
    TVariableList  argv_var;

    switch(builtin_f->ReturnType)
    {
    case TBuiltinFunctions::rtVoid:
      ret_val.Unset();
      argv_var.push_back(TVariable());
      break;
    case TBuiltinFunctions::rtInt:
      ret_val.Set(GetZero<pt_int>());
      break;
    case TBuiltinFunctions::rtReal:
      ret_val.Set(GetZero<pt_real>());
      break;
    case TBuiltinFunctions::rtBool:
      ret_val.Set(GetZero<pt_bool>());
      break;
    case TBuiltinFunctions::rtString:
      ret_val.Set(GetZero<pt_string>());
      break;
    case TBuiltinFunctions::rtList:
      ret_val= LiteralEmptyList();
      break;
    default:
      LERROR("Invalid return type: "<<static_cast<int>(builtin_f->ReturnType));
      lexit(df);
    }
    if(builtin_f->ReturnType != TBuiltinFunctions::rtVoid)
      argv_var.push_back(Variable(ret_val));

    for(std::list<TLiteral>::iterator itr(argv.begin()),last(argv.end()); itr!=last; ++itr)
      argv_var.push_back(Variable(*itr));

    builtin_f->F(*this,argv_var);
    return true;
  }
  else
  {
    return false;
  }
}
//-------------------------------------------------------------------------------------------

//! access to the member of value
/*virtual*/TVariable TBinExecutor::member_access(const TLiteral &value, const TLiteral &member_c)
{
  TLiteral member(member_c);
  if(value.IsIdentifier())
  {
    TIdentifier id(value.AsIdentifier());
    LASSERT(!variable_stack_.empty());
    // VAR_ERR_CATCHER_S
    return variable_stack_.back().GetMember(TVariable(id)).ToVariable().GetMember(Variable(member));
    // VAR_ERR_CATCHER_E
  }
  else if(value.IsVariable())
  {
    return value.AsVariable().GetMember(Variable(member));
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

  CALL_CMD_EXEC( PRINT     )

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
  int type_code= bstack.ReadI();
  switch(type_code)
  {
  case vtype::ID   :  literal_stack_.push_back(LiteralId(bstack.ReadS())); break;
  case vtype::INT  :  literal_stack_.push_back(TLiteral(bstack.ReadI())); break;
  case vtype::REAL :  literal_stack_.push_back(TLiteral(bstack.ReadR())); break;
  case vtype::BOOL :  literal_stack_.push_back(TLiteral(bstack.ReadB())); break;
  case vtype::STR  :  literal_stack_.push_back(TLiteral(bstack.ReadS())); break;
  case vtype::TYPE :  literal_stack_.push_back(LiteralType(bstack.ReadI())); break;
  default:  FIXME("unknown value type code: "<<type_code);
  }
}
IMPL_CMD_EXEC( PUSHL     )  // bin=[- N vtype value1 .. valueN]; push a list (value1,..,valueN) of vtype;
{
  using namespace bin;
  int num= bstack.ReadI();
  literal_stack_.push_back(LiteralEmptyList());
  TLiteral  &back(literal_stack_.back());
  int type_code= bstack.ReadI();
  switch(type_code)
  {
  case vtype::INT  :  for(;num>0;--num) back.AppendToList(TAnyPrimitive(bstack.ReadI())); break;
  case vtype::REAL :  for(;num>0;--num) back.AppendToList(TAnyPrimitive(bstack.ReadR())); break;
  case vtype::BOOL :  for(;num>0;--num) back.AppendToList(TAnyPrimitive(bstack.ReadB())); break;
  case vtype::STR  :  for(;num>0;--num) back.AppendToList(TAnyPrimitive(bstack.ReadS())); break;
  default:  FIXME("unknown value type code: "<<type_code);
  }
}
IMPL_CMD_EXEC( LAPPEND   )  // bin=[-]; pop two values(1,2), append 1 to 2:(2,1), push the result;
{
  TLiteral value= pop_literal();
  literal_stack_.back().AppendToList(value.AsPrimitive());
}
IMPL_CMD_EXEC( PUSH_EMPL )  // bin=[-]; push an empty list;
{
  literal_stack_.push_back(LiteralEmptyList());
}
IMPL_CMD_EXEC( LLISTS    )  // bin=[-]; start list-of-literals (in l-o-l, PUSH, PUSHL, VLIST{S,E} are available);
{
  literal_stack_.push_back(LiteralCmd(bin::cmd::LLISTS));
}
IMPL_CMD_EXEC( POP       )  // bin=[-]; pop a value;
{
  literal_stack_.pop_back();
}

IMPL_CMD_EXEC( PRINT     ) // bin=[-]; pop a value(1), print 1 into std-out;
{
  TLiteral value= pop_literal();
  std::cout << value << std::endl;
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
  PushVariable(member);
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
  PushVariable(member);
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
  PushVariable(new_var);
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
  TExtVariable array_all(itr,ilast);
  PushVariable(array_all);
}
IMPL_CMD_EXEC( CASGN_END )  // finish M_ASGN_CS, E_ASGN_CS, P_ASGN_CS, F_ASGN_CS;
{
  LASSERT(!variable_stack_.empty());
  PopVariable();
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
  TIdentifier member(pop_id());
  TLiteral value(pop_literal(config));
  literal_stack_.push_back( member_access(value,LiteralId(member)) );
}
IMPL_CMD_EXEC( ELEM   )  // bin=[-]; pop two values, get an elemental ref(2[1]), push the result;
{
  TEvaluateLiteralConfig config;  config.AllowId= true;
  TLiteral member(pop_literal());
  TLiteral value(pop_literal(config));
  literal_stack_.push_back( member_access(value,member) );
}

IMPL_CMD_EXEC( CAST   )  // bin=[-]; pop two values(1,2;2 should be a type), cast 1 to 2(cast<2>(1), push the result;
{
  TEvaluateLiteralConfig config;  config.AllowId= true;
  TLiteral value(pop_literal(config)),type(pop_literal());
  if(!type.IsType())  {LERROR("type is required, but used: "<<type); error_= true; return;}

  if(value.IsIdentifier() || value.IsVariable())
  {
    TVariable tmp_var;
    TVariable *pvalue(NULL);
    if(value.IsVariable())  pvalue= &value.AsVariable();
    else
    {
      pt_string id(value.AsIdentifier());
      LASSERT(!variable_stack_.empty());
      VAR_ERR_CATCHER_S
      tmp_var= variable_stack_.back().GetMember(TVariable(id)).ToVariable();
      VAR_ERR_CATCHER_E
      pvalue= &tmp_var;
    }

    switch(type.AsType().front().Int())
    {
    case bin::vtype::INT  :
      literal_stack_.push_back(TLiteral( pvalue->PrimitiveGetAs<pt_int>() ));
      break;
    case bin::vtype::REAL :
      literal_stack_.push_back(TLiteral( pvalue->PrimitiveGetAs<pt_real>() ));
      break;
    case bin::vtype::BOOL :
      literal_stack_.push_back(TLiteral( pvalue->PrimitiveGetAs<pt_bool>() ));
      break;
    case bin::vtype::STR  :
      literal_stack_.push_back(TLiteral( pvalue->PrimitiveGetAs<pt_string>() ));
      break;
    case bin::vtype::LIST :
      {
        TLiteral casted(LiteralEmptyList());
        TForwardIterator itr,ilast;
        pvalue->GetBegin(itr);
        pvalue->GetEnd(ilast);
        switch((++type.AsType().begin())->Int())
        {
        case bin::vtype::INT  :
          for (; itr!=ilast; ++itr)  casted.AppendToList(itr->PrimitiveGetAs<pt_int>());
          break;
        case bin::vtype::REAL :
          for (; itr!=ilast; ++itr)  casted.AppendToList(itr->PrimitiveGetAs<pt_real>());
          break;
        case bin::vtype::BOOL :
          for (; itr!=ilast; ++itr)  casted.AppendToList(itr->PrimitiveGetAs<pt_bool>());
          break;
        case bin::vtype::STR  :
          for (; itr!=ilast; ++itr)  casted.AppendToList(itr->PrimitiveGetAs<pt_string>());
          break;
        case bin::vtype::LIST :
          print_error("cannot cast to list<list<...> >");
          break;
        default:
          FIXME("fatal!");
          break;
        }
        literal_stack_.push_back(casted);
      }
      break;
    default:
      FIXME("fatal!");
      break;
    }
  }
  else if(value.IsPrimitive())
  {
    switch(type.AsType().front().Int())
    {
    case bin::vtype::INT  :
      literal_stack_.push_back(TLiteral( value.AsPrimitive().GetAs<pt_int>() ));
      break;
    case bin::vtype::REAL :
      literal_stack_.push_back(TLiteral( value.AsPrimitive().GetAs<pt_real>() ));
      break;
    case bin::vtype::BOOL :
      literal_stack_.push_back(TLiteral( value.AsPrimitive().GetAs<pt_bool>() ));
      break;
    case bin::vtype::STR  :
      literal_stack_.push_back(TLiteral( value.AsPrimitive().GetAs<pt_string>() ));
      break;
    case bin::vtype::LIST :
      {
        TLiteral casted(LiteralEmptyList());
        casted.AppendToList(value.AsPrimitive());
        literal_stack_.push_back(casted);
      }
      break;
    default:
      FIXME("fatal!");
      break;
    }
  }
  else if(value.IsList())
  {
    switch(type.AsType().front().Int())
    {
    case bin::vtype::INT  :
    case bin::vtype::REAL :
    case bin::vtype::BOOL :
    case bin::vtype::STR  :
      print_error("cannot cast a list to primitive other than a list of size 1");
      lexit(df);
      break;
    case bin::vtype::LIST :
      {
        TLiteral casted(LiteralEmptyList());
        std::list<TAnyPrimitive>::const_iterator itr(value.AsList().begin()), ilast(value.AsList().end());
        switch((++type.AsType().begin())->Int())
        {
        case bin::vtype::INT  :
          for (; itr!=ilast; ++itr)  casted.AppendToList(itr->GetAs<pt_int>());
          break;
        case bin::vtype::REAL :
          for (; itr!=ilast; ++itr)  casted.AppendToList(itr->GetAs<pt_real>());
          break;
        case bin::vtype::BOOL :
          for (; itr!=ilast; ++itr)  casted.AppendToList(itr->GetAs<pt_bool>());
          break;
        case bin::vtype::STR  :
          for (; itr!=ilast; ++itr)  casted.AppendToList(itr->GetAs<pt_string>());
          break;
        case bin::vtype::LIST :
          print_error("cannot cast to list<list<...> >");
          break;
        default:
          FIXME("fatal!");
          break;
        }
        literal_stack_.push_back(casted);
      }
      break;
    default:
      FIXME("fatal!");
      break;
    }
  }
  else
  {
    LERROR("cannot cast: "<<value);
    lexit(df);
  }
}

IMPL_CMD_EXEC( T_TO_LIST )  // bin=[-]; pop a type(1), push the list of it (list<t>);
{
  TLiteral type(pop_literal());
  if(!type.IsType())  {LERROR("type is required, but used: "<<type); error_= true; return;}
  type.AppendToType(bin::vtype::LIST);
  literal_stack_.push_back(type);
}

#undef IMPL_CMD_EXEC

#undef VAR_ERR_CATCHER_S
#undef VAR_ERR_CATCHER_E

#undef CONV_ERR_CATCHER_S
#undef CONV_ERR_CATCHER_E


//===========================================================================================
// class TBinWriter
//===========================================================================================

/*virtual*/void TBinWriter::PartiallyExecute(const std::string& file_name, int line_num, bool error_stat)
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

/*virtual*/void TBinWriter::Execute(bool from_current)
{
  LASSERT(bin_stack_!=NULL);

  if(!from_current)  bin_stack_->GoFirst();
  while(!bin_stack_->IsEOD())
  {
    exec_command(bin_stack_->ReadI(), *bin_stack_);
  }
}
//-------------------------------------------------------------------------------------------

/*virtual*/void TBinWriter::exec_command(int command, const TBinaryStack &bstack)
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

  CALL_CMD_EXEC( PRINT     )

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

#define IMPL_CMD_EXEC(x_cmd)  void TBinWriter::cmd_##x_cmd (int command, const TBinaryStack &bstack)

IMPL_CMD_EXEC( PUSH      )  // bin=[- vtype value]; push a value of vtype;
{
  using namespace bin;
  int type_code= bstack.ReadI();
  switch(type_code)
  {
  case vtype::ID   :  literal_stack_.push_back(TLiteral(bstack.ReadS())); break;
  case vtype::INT  :  literal_stack_.push_back(TLiteral(ConvertToStr(bstack.ReadI()))); break;
  case vtype::REAL :  literal_stack_.push_back(TLiteral(ConvertToStr(bstack.ReadR()))); break;
  case vtype::BOOL :  literal_stack_.push_back(TLiteral(ConvertToStr(bstack.ReadB()))); break;
  case vtype::STR  :  literal_stack_.push_back(TLiteral(ConvertToStr(bstack.ReadS()))); break;
  case vtype::TYPE :  literal_stack_.push_back(TLiteral(std::string(bin::TypeStr(bstack.ReadI())))); break;
  default:  FIXME("unknown value type code: "<<type_code);
  }
}
IMPL_CMD_EXEC( PUSHL     )  // bin=[- N vtype value1 .. valueN]; push a list (value1,..,valueN) of vtype;
{
  using namespace bin;
  int num= bstack.ReadI();
  int type_code= bstack.ReadI();
  std::string value, delim("");
  switch(type_code)
  {
  case vtype::INT  :  for(;num>0;delim=", ",--num) value+=delim+ConvertToStr(bstack.ReadI()); break;
  case vtype::REAL :  for(;num>0;delim=", ",--num) value+=delim+ConvertToStr(bstack.ReadR()); break;
  case vtype::BOOL :  for(;num>0;delim=", ",--num) value+=delim+ConvertToStr(bstack.ReadB()); break;
  case vtype::STR  :  for(;num>0;delim=", ",--num) value+=delim+ConvertToStr(bstack.ReadS()); break;
  default:  FIXME("unknown value type code: "<<type_code);
  }
  literal_stack_.push_back(TLiteral(value));
}
IMPL_CMD_EXEC( LAPPEND   )  // bin=[-]; pop two values(1,2), append 1 to 2:(2,1), push the result;
{
  std::string value= pop_literal();
  if(literal_stack_.back().AsPrimitive().String()=="") literal_stack_.back().AsPrimitive().String()= value;
  else  literal_stack_.back().AsPrimitive().String()+= ","+value;
}
IMPL_CMD_EXEC( PUSH_EMPL )  // bin=[-]; push an empty list;
{
  literal_stack_.push_back(TLiteral(std::string("")));
}
IMPL_CMD_EXEC( LLISTS    )  // bin=[-]; start list-of-literals (in l-o-l, PUSH, PUSHL, VLIST{S,E} are available);
{
  literal_stack_.push_back(LiteralCmd(bin::cmd::LLISTS));
}
IMPL_CMD_EXEC( POP       )  // bin=[-]; pop a value;
{
  out_to_stream()<<pop_literal()<<std::endl;
}

IMPL_CMD_EXEC( PRINT     ) // bin=[-]; pop a value(1), print 1 into std-out;
{
  std::string value(pop_literal());
  out_to_stream()<<"print "<<value<<std::endl;
}

IMPL_CMD_EXEC( M_ASGN_P  )  // bin=[-]; pop two values(1,2;2 shoud be an identifier), assign:2=1;
{
  std::string value= pop_paren_value();
  std::string identifier(pop_id());

  out_to_stream()<<identifier<<" = "<<value<<std::endl;
}
IMPL_CMD_EXEC( M_ASGN_CS )  // bin=[-]; pop an identifier, start composite assign:id={..};
{
  std::string identifier(pop_id());

  out_to_stream()<<identifier<<" ={"<<std::endl;
  indent_+=2;
}
IMPL_CMD_EXEC( E_ASGN_P  )  // bin=[-]; pop two values(1,2), elemental assign:[2]=1;
{
  std::string value= pop_paren_value();
  std::string key= pop_paren_value();

  out_to_stream()<<"["<<key<<"]"<<" = "<<value<<std::endl;
}
IMPL_CMD_EXEC( E_ASGN_CS )  // bin=[-]; pop a value, start elemental composite assign:[val]={..};
{
  std::string key= pop_paren_value();

  out_to_stream()<<"["<<key<<"]"<<" ={"<<std::endl;
  indent_+=2;
}
IMPL_CMD_EXEC( P_ASGN_P  )  // bin=[-]; pop a value, push:[]=val;
{
  std::string value= pop_paren_value();

  out_to_stream()<<"[]"<<" = "<<value<<std::endl;
}
IMPL_CMD_EXEC( P_ASGN_CS )  // bin=[-]; start composite push:[]={..};
{
  out_to_stream()<<"[]"<<" ={"<<std::endl;
  indent_+=2;
}
IMPL_CMD_EXEC( F_ASGN_P  )  // bin=[-]; pop a value, fill:[@]=val;
{
  std::string value= pop_paren_value();

  out_to_stream()<<"[@]"<<" = "<<value<<std::endl;
}
IMPL_CMD_EXEC( F_ASGN_CS )  // bin=[-]; start composite fill:[@]={..};
{
  out_to_stream()<<"[@]"<<" ={"<<std::endl;
  indent_+=2;
}
IMPL_CMD_EXEC( CASGN_END )  // finish M_ASGN_CS, E_ASGN_CS, P_ASGN_CS, F_ASGN_CS;
{
  --indent_;
  out_to_stream()<<"}"<<std::endl;
  --indent_;
}

IMPL_CMD_EXEC( FUNC_CALL )  // bin=[-]; pop list-of-literals, pop an identifier, call function:id(l-o-l);
{
  std::list<std::string>  argv;
  pop_literal_list(argv);
  std::string identifier(pop_id());

  std::string delim("");
  std::stringstream fcall;
  fcall<<identifier<<"(";
  for(std::list<std::string>::const_iterator itr(argv.begin()),last(argv.end());itr!=last;delim=", ",++itr)
    fcall<<delim<<*itr;
  fcall<<")";
  literal_stack_.push_back(fcall.str());
}

IMPL_CMD_EXEC( CONCAT )  // bin=[-]; pop two values(1,2;2 shoud be an identifier), concatenate:2##1, push the result as an identifier;
{
  std::string id2(pop_id()), id1(pop_id());
  literal_stack_.push_back(id1+"##"+id2);
}
IMPL_CMD_EXEC( ADD    )  // bin=[-]; pop two values, add them(2+1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +"+"+ value1);
}
IMPL_CMD_EXEC( SUBT   )  // bin=[-]; pop two values, subtract them(2-1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +"-"+ value1);
}
IMPL_CMD_EXEC( MULT   )  // bin=[-]; pop two values, multiply them(2*1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +"*"+ value1);
}
IMPL_CMD_EXEC( DIV    )  // bin=[-]; pop two values, divide them(2/1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +"/"+ value1);
}
IMPL_CMD_EXEC( MOD    )  // bin=[-]; pop two values, compute mod(2%1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +"%"+ value1);
}
IMPL_CMD_EXEC( AND    )  // bin=[-]; pop two values, compute and(2&&1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +"&&"+ value1);
}
IMPL_CMD_EXEC( OR     )  // bin=[-]; pop two values, compute or(2||1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +"||"+ value1);
}
IMPL_CMD_EXEC( NOT    )  // bin=[-]; pop a value, compute not(!1), push the result;
{
  std::string value1(pop_paren_value());
  literal_stack_.push_back( "!"+ value1);
}
IMPL_CMD_EXEC( EQ     )  // bin=[-]; pop two values, compute equality(2==1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +"=="+ value1);
}
IMPL_CMD_EXEC( NEQ    )  // bin=[-]; pop two values, compute inequality(2!=1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +"!="+ value1);
}
IMPL_CMD_EXEC( LTEQ   )  // bin=[-]; pop two values, compute relation(2<=1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +"<="+ value1);
}
IMPL_CMD_EXEC( GTEQ   )  // bin=[-]; pop two values, compute relation(2>=1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +">="+ value1);
}
IMPL_CMD_EXEC( LT     )  // bin=[-]; pop two values, compute relation(2<1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +"<"+ value1);
}
IMPL_CMD_EXEC( GT     )  // bin=[-]; pop two values, compute relation(2>1), push the result;
{
  std::string value1(pop_paren_value()),value2(pop_paren_value());
  literal_stack_.push_back(value2 +">"+ value1);
}
IMPL_CMD_EXEC( MEMBER )  // bin=[-]; pop two values(1,2;1 shoud be an identifier), get a member ref(2.1), push the result;
{
  TIdentifier member(pop_id());
  std::string value(pop_paren_value());
  literal_stack_.push_back(value +"."+member);
}
IMPL_CMD_EXEC( ELEM   )  // bin=[-]; pop two values, get an elemental ref(2[1]), push the result;
{
  std::string member(pop_paren_value());
  std::string value(pop_paren_value());
  literal_stack_.push_back(value +"["+member+"]");
}

IMPL_CMD_EXEC( CAST   )  // bin=[-]; pop two values(1,2;2 should be a type), cast 1 to 2(cast<2>(1), push the result;
{
  std::string value(pop_paren_value()),type(pop_literal());
  literal_stack_.push_back("cast<"+type+" >("+value+")");
}

IMPL_CMD_EXEC( T_TO_LIST )  // bin=[-]; pop a type(1), push the list of it (list<t>);
{
  std::string type(pop_literal());
  literal_stack_.push_back("list<"+type+">");
}

#undef IMPL_CMD_EXEC

// end of TBinWriter

//===========================================================================================

static void partially_execute(TBinExecutor *executor, TBinaryStack *bin_stack, const std::string& file_name, int line_num, bool error_stat)
{
  bin_stack->GoFirst();
  executor->PartiallyExecute(file_name,line_num,error_stat);
  bin_stack->Clear();
}
//-------------------------------------------------------------------------------------------

bool LoadFromFile (const std::string &file_name, TVariable &var, TLiteralTable &literal_table, const TBuiltinFunctions *additional_funcs)
{
  boost::filesystem::path file_path
    = boost::filesystem::absolute(boost::filesystem::path(file_name/*,boost::filesystem::native*/));

  if (boost::filesystem::exists(file_path) && boost::filesystem::is_empty(file_path))  // if file is empty...
    return true;

  TBinExecutor executor;
  TBinaryStack bin_stack;

  executor.PushVariable(var);
  executor.SetBinStack(&bin_stack);
  executor.SetLiteralTable(&literal_table);
  executor.SetCurrentDir(file_path.parent_path());
  if(additional_funcs)  executor.BuiltinFunctions().Merge(*additional_funcs);

  TParserCallbacks callbacks;
  // callbacks.OnCommandPushed= callback;
  callbacks.OnEndOfLine= boost::bind(&partially_execute,&executor,&bin_stack,_1,_2,_3);
  if(ParseFile(file_path.string(),bin_stack,callbacks))
  {
    partially_execute(&executor,&bin_stack,file_path.string(),-1,false);
      //! this code is needed if there is no newline at the end of file; \todo FIXME: the line number (-1)
    executor.PopVariable();
    LASSERT(executor.VariableStackSize()==0);
    return !executor.Error();
  }
  return false;
}
//-------------------------------------------------------------------------------------------

bool ExecuteBinary (const TBinaryStack &bin_stack, TVariable &var, TLiteralTable &literal_table, const TBuiltinFunctions *additional_funcs)
{
  TBinExecutor executor;
  executor.PushVariable(var);
  executor.SetBinStack(&bin_stack);
  executor.SetLiteralTable(&literal_table);
  if(additional_funcs)  executor.BuiltinFunctions().Merge(*additional_funcs);
  executor.Execute(true);
  executor.PopVariable();
  LASSERT(executor.VariableStackSize()==0);
  return !executor.Error();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of var_space
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
