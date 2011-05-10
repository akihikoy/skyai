//-------------------------------------------------------------------------------------------
/*! \file    variable_parser_impl.h
    \brief   liblora - parser for variable-space  (implement header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    May.17, 2010-
    \date    Jun.24, 2010    Bug fixed for FillAllComposite ([@]={...})

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
#ifndef loco_rabbits_variable_parser_impl_h
#define loco_rabbits_variable_parser_impl_h
//-------------------------------------------------------------------------------------------
#include <lora/variable_parser.h>
#include <lora/variable_space.h>
#include <lora/variable_any.h>
//-------------------------------------------------------------------------------------------
#include <lora/string.h>
#include <lora/stl_ext.h>
#include <boost/spirit/include/classic.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
namespace var_space
{
//-------------------------------------------------------------------------------------------

struct TVarFormat
{
  TLiteral &Literal;
  TVarFormat(TLiteral &l) : Literal(l) {}
};
inline std::ostream& operator<< (std::ostream &lhs, const TVarFormat &rhs)
{
  switch(rhs.Literal.LType)
  {
  case TLiteral::ltIdentifier  : lhs<<rhs.Literal.LPrimitive.EString; break;
  case TLiteral::ltPrimitive   : lhs<<TVariable(rhs.Literal.LPrimitive);      break;
  case TLiteral::ltList        : lhs<<TVariable(rhs.Literal.LList); break;
  default                      : LERROR("fatal!"); lexit(df);
  }
  return lhs;
}
//-------------------------------------------------------------------------------------------

struct TVarListFormat
{
  typedef std::list<TLiteral>::iterator iterator;
  std::string Delim;
  iterator First,Last;
  TVarListFormat(iterator fst, iterator lst) : Delim(","), First(fst), Last(lst)  {}
};
inline std::ostream& operator<< (std::ostream &lhs, const TVarListFormat &rhs)
{
  TVarListFormat::iterator itr(rhs.First);
  if(itr==rhs.Last)  return lhs;
  lhs<<TVarFormat(*itr);
  for(++itr; itr!=rhs.Last; ++itr)
    lhs<<rhs.Delim<< TVarFormat(*itr);
  return lhs;
}
//-------------------------------------------------------------------------------------------

class TExtForwardIterator;

//! extended TVariable class to treat fill-all
class TExtVariable
{
public:
  enum TKind {kUnknown=0,kSingle,kArray};
  TExtVariable () : kind_(kUnknown)  {}
  TExtVariable (TKind k) : kind_(k)  {}
  TExtVariable (TVariable v_var) : kind_(kSingle), entity_(v_var)  {}
  TExtVariable (TExtForwardIterator &first, TExtForwardIterator &last);

  void DirectAssign (const TVariable &value);
  void SetMember (const TVariable &id, const TVariable &value);
  TExtVariable GetMember (const TVariable &id);
  void FunctionCall (const TIdentifier &id, TVariableList &argv);
  void DirectCall (TVariableList &argv);
  TExtVariable Push (void);
  void GetBegin (TExtForwardIterator &res);
  void GetEnd (TExtForwardIterator &res);

private:

  TKind                  kind_;
  TVariable              entity_;
  std::list<TVariable>   array_;

  friend class TExtForwardIterator;

};
//-------------------------------------------------------------------------------------------

//! extended TVariable class to treat fill-all
class TExtForwardIterator
{
public:
  enum TKind {kUnknown=0,kSingle,kArray};

  TExtForwardIterator() : kind_(kUnknown) {}

  TExtVariable& operator*(void)  {i_dereference_(); return dereferenced_;}
  TExtVariable* operator->(void)  {i_dereference_(); return &dereferenced_;}
  const TExtForwardIterator& operator++(void);
  const TExtForwardIterator& operator--(void);
  bool operator==(const TExtForwardIterator &rhs) const;
  bool operator!=(const TExtForwardIterator &rhs) const {return !operator==(rhs);}

private:
  TKind                          kind_;
  TForwardIterator               entity_;
  std::list<TForwardIterator>    array_;

  TExtVariable                   dereferenced_;
  void i_dereference_();

  friend class TExtVariable;

};
//-------------------------------------------------------------------------------------------

template <typename t_container>
void AddToKeywordSet (t_container &container)
{
  container.insert("true");
  container.insert("false");
  container.insert("inf");
}
//-------------------------------------------------------------------------------------------


class TBasicParserAgent
{
public:

  TBasicParserAgent() : expand_id_(true), error_(false), literal_table_(NULL) {}

  #define DECL_ACTION(x_func)  template <typename t_iterator> void x_func (t_iterator first, t_iterator last)
  DECL_ACTION(EndOfLine);
  DECL_ACTION(PushLiteralIdentifier);
  DECL_ACTION(PushLiteralInt);
  DECL_ACTION(PushLiteralReal);
  DECL_ACTION(PushLiteralBool);
  DECL_ACTION(PushLiteralString);
  DECL_ACTION(BeginLiteralListAny);
  DECL_ACTION(BeginLiteralListPrimitive);
  DECL_ACTION(EndLiteralListPrimitive);
  DECL_ACTION(ConcatenateIdentifiers);
  #undef DECL_ACTION

  bool Error() const {return error_;}
  int LineNum() const {return line_num_;}

protected:

  enum TLiteralCellType {ltBeginListAny=0, ltValue};
  typedef std::pair<TLiteralCellType,TLiteral> TLiteralCell;
  typedef std::list<TLiteralCell> TLiteralList;

  bool expand_id_;
  int  line_num_;
  std::string  file_name_;
  bool error_;
  bool in_literal_list_;

  const TLiteralTable  *literal_table_;
  TLiteralList literal_stack_;

  TVariable ToVariable(TLiteral &literal)
    {
      TVariable var;
      switch(literal.LType)
      {
      case TLiteral::ltIdentifier  : LERROR("cannot convert an identifier `"<<literal.LPrimitive.EString<<"\' to a TVariable"); lexit(df);
      case TLiteral::ltPrimitive   : var.Generate(literal.LPrimitive);      break;
      case TLiteral::ltList        : var.Generate(literal.LList); break;
      default                      : LERROR("fatal!"); lexit(df);
      }
      return var;
    }
  TVariable ToVariable(TLiteralCell &literal_cell)
    {
      LASSERT1op1(literal_cell.first,==,ltValue);
      return ToVariable(literal_cell.second);
    }

  TLiteral evaluate_literal (const TLiteral &src, const TEvaluateLiteralConfig &config=TEvaluateLiteralConfig())
    {
      return EvaluateLiteral(src, literal_table_, config, error_);
    }

  TLiteral pop_literal_x (const TEvaluateLiteralConfig &config=TEvaluateLiteralConfig())
    {
      LASSERT(!literal_stack_.empty());
      TLiteralCell value= literal_stack_.back();  literal_stack_.pop_back();
      LASSERT1op1(value.first,==,ltValue);
      return expand_id_ ? evaluate_literal(value.second, config) : value.second;
    }

  void pop_literal_list_x(std::list<TLiteral> &literal_list, const TEvaluateLiteralConfig &config=TEvaluateLiteralConfig())
    {
      TLiteralList::iterator  ilast(literal_stack_.end()),ifirst(literal_stack_.begin());
      TLiteralList::iterator  itr(ilast);
      for(--itr; itr!=ifirst && itr->first!=ltBeginListAny; --itr) {}
      if(itr->first!=ltBeginListAny)  {LERROR("fatal!"); lexit(df);}
      for(++itr; itr!=ilast; ++itr)
        literal_list.push_back(expand_id_ ? evaluate_literal(itr->second, config) : itr->second);
      while(literal_stack_.back().first!=ltBeginListAny)
        literal_stack_.pop_back();
      literal_stack_.pop_back();
    }

  virtual void on_push_literal_identifier (const std::string &id)
    {
    }

  virtual std::string pop_id_x (void)
    {
      TEvaluateLiteralConfig config;  config.AllowId= true;
      TLiteral id(pop_literal_x(config));
      if(id.LType!=TLiteral::ltIdentifier)  {LERROR("identifier is required, but used: "<<id); lexit(df);}
      return id.LPrimitive.EString;
    }

  template <typename t_iterator>
  static std::string join_iterators (t_iterator first, t_iterator last)
    {
      std::stringstream ss;
      for(;first!=last;++first)
        ss<<*first;
      return ss.str();
    }

  void print_error (const std::string &str)
    {
      error_= true;
      std::cerr<<"("<<file_name_<<":"<<line_num_<<") "<<str<<std::endl;
    }
  void lora_error (message_system::TMessageType type, int linenum, const char *filename, const char *functionname, std::stringstream &ss)
    {
      if(type==message_system::mtError)
        print_error(ss.str());
      else
        DefaultFormat(type, linenum, filename, functionname, ss);
    }
};
//-------------------------------------------------------------------------------------------

#define PRINT_ERROR(x_msg)  do{std::stringstream ss; ss<<x_msg; print_error(ss.str());}while(0)
//-------------------------------------------------------------------------------------------

#define TEMPLATE_DEC  template <typename t_iterator>
#define XCLASS        TBasicParserAgent

TEMPLATE_DEC
void XCLASS::EndOfLine (t_iterator first, t_iterator last)
{
  ++line_num_;
}

TEMPLATE_DEC
void XCLASS::PushLiteralIdentifier (t_iterator first, t_iterator last)
{
  TLiteral literal;
  literal.LType= TLiteral::ltIdentifier;
  literal.LPrimitive= TAnyPrimitive(pt_string(join_iterators(first,last)));
  on_push_literal_identifier(literal.LPrimitive.EString);
  if(!in_literal_list_)
  {
    literal_stack_.push_back(TLiteralCell(ltValue,literal));
  }
  else
  {
    LASSERT(!literal_stack_.empty());
    TLiteral  evaluated_l(expand_id_ ? evaluate_literal(literal) : literal);
    if (evaluated_l.LType!=TLiteral::ltPrimitive)
    {
      PRINT_ERROR("non-primitive-type cannot be an element of a list");
      PRINT_ERROR("note: "<<literal<<" is evaluated as "<<evaluated_l);
    }
    literal_stack_.back().second.LList.push_back(evaluated_l.LPrimitive);
  }
}
TEMPLATE_DEC
void XCLASS::PushLiteralInt (t_iterator first, t_iterator last)
{
  TLiteral literal;
  literal.LType= TLiteral::ltPrimitive;
  literal.LPrimitive= TAnyPrimitive(ConvertFromStr<pt_int>(join_iterators(first,last)));
  if(!in_literal_list_)
  {
    literal_stack_.push_back(TLiteralCell(ltValue,literal));
  }
  else
  {
    LASSERT(!literal_stack_.empty());
    literal_stack_.back().second.LList.push_back(literal.LPrimitive);
  }
}
TEMPLATE_DEC
void XCLASS::PushLiteralReal (t_iterator first, t_iterator last)
{
  TLiteral literal;
  literal.LType= TLiteral::ltPrimitive;
  literal.LPrimitive= TAnyPrimitive(ConvertFromStr<pt_real>(join_iterators(first,last)));
 if(!in_literal_list_)
  {
    literal_stack_.push_back(TLiteralCell(ltValue,literal));
  }
  else
  {
    LASSERT(!literal_stack_.empty());
    literal_stack_.back().second.LList.push_back(literal.LPrimitive);
  }
}
TEMPLATE_DEC
void XCLASS::PushLiteralBool (t_iterator first, t_iterator last)
{
  TLiteral literal;
  literal.LType= TLiteral::ltPrimitive;
  literal.LPrimitive= TAnyPrimitive(ConvertFromStr<pt_bool>(join_iterators(first,last)));
  if(!in_literal_list_)
  {
    literal_stack_.push_back(TLiteralCell(ltValue,literal));
  }
  else
  {
    LASSERT(!literal_stack_.empty());
    literal_stack_.back().second.LList.push_back(literal.LPrimitive);
  }
}
TEMPLATE_DEC
void XCLASS::PushLiteralString (t_iterator first, t_iterator last)
{
  TLiteral literal;
  literal.LType= TLiteral::ltPrimitive;
  literal.LPrimitive= TAnyPrimitive(pt_string(DecodeString(join_iterators(first,last))));
  if(!in_literal_list_)
  {
    literal_stack_.push_back(TLiteralCell(ltValue,literal));
  }
  else
  {
    LASSERT(!literal_stack_.empty());
    literal_stack_.back().second.LList.push_back(literal.LPrimitive);
  }
}
TEMPLATE_DEC
void XCLASS::BeginLiteralListAny (t_iterator first, t_iterator last)
{
  literal_stack_.push_back(TLiteralCell(ltBeginListAny,TLiteral()));
}
TEMPLATE_DEC
void XCLASS::BeginLiteralListPrimitive (t_iterator first, t_iterator last)
{
  TLiteral literal;
  literal.LType= TLiteral::ltList;
  literal_stack_.push_back(TLiteralCell(ltValue,literal));
  in_literal_list_= true;
}
TEMPLATE_DEC
void XCLASS::EndLiteralListPrimitive (t_iterator first, t_iterator last)
{
  in_literal_list_= false;
}
TEMPLATE_DEC
void XCLASS::ConcatenateIdentifiers (t_iterator first, t_iterator last)
{
  std::string id2(pop_id_x()), id1(pop_id_x());
  TLiteral literal;
  literal.LType= TLiteral::ltIdentifier;
  if(expand_id_)
  {
    literal.LPrimitive= TAnyPrimitive(pt_string(id1+id2));
    on_push_literal_identifier(literal.LPrimitive.EString);
  }
  else
  {
    literal.LPrimitive= TAnyPrimitive(pt_string(id1+"##"+id2));
  }
  literal_stack_.push_back(TLiteralCell(ltValue,literal));
}

#undef TEMPLATE_DEC
#undef XCLASS
//-------------------------------------------------------------------------------------------


template <typename t_iterator>
class TParserAgent : public TBasicParserAgent
{
public:

  TParserAgent () : TBasicParserAgent(), equivalent_code_(NULL) {}

  boost::spirit::classic::parse_info<t_iterator> Parse (TParserInfoIn &in, t_iterator first, t_iterator last, TParserInfoOut *out);
  void StartSubParse (TParserInfoIn &in);
  bool EndSubParse (TParserInfoOut *out);

  #define DECL_ACTION(x_func)  void x_func (t_iterator first, t_iterator last)
  DECL_ACTION(CloseByBrace);
  DECL_ACTION(DebugMessage);
  DECL_ACTION(SyntaxError);
  DECL_ACTION(PrimitiveAssignToMember);
  DECL_ACTION(CompositeAssignToMemberS);
  DECL_ACTION(ElementalPrimitiveAssignToMember);
  DECL_ACTION(ElementalCompositeAssignToMemberS);
  DECL_ACTION(PushPrimitive);
  DECL_ACTION(PushComposite);
  DECL_ACTION(FillAllPrimitive);
  DECL_ACTION(FillAllComposite);
  DECL_ACTION(FunctionCall);
  #undef DECL_ACTION

private:

  std::list<TExtVariable>  variable_stack_;
  enum TContext {cGlobal=0, cCompositeMemberAssign, cCompositeFillAll};
  std::list<TContext>  context_stack_;
  TParseMode  parse_mode_;
  std::stringstream  *equivalent_code_;

  LORA_MESSAGE_FORMAT_FUNCTION tmp_lora_msg_format_;

};
//-------------------------------------------------------------------------------------------

template <typename t_iterator, typename ScannerT>
struct basic_parser_definition
{
  typedef boost::spirit::classic::rule<ScannerT> rule_t;
  rule_t  identifier_p;
  rule_t  parenthesized_list_expr_any, list_expr_any, list_expr_primitive;
  rule_t  expr_any, expr_primitive;
  rule_t  expr_parenthesized_list;
  rule_t  expr_identifier;
  rule_t  parenthesized_list_literal_any, list_literal_any, list_literal_primitive;
  rule_t  literal_any, literal_primitive;
  rule_t  literal_parenthesized_list;
  rule_t  literal_identifier, int_p_i, literal_int, literal_real, literal_boolean, literal_string;
  rule_t  lcomment;
  rule_t  end_of_line, blank_eol_p;
  rule_t  op_semicolon, op_comma, op_dot, op_eq, op_at;
  rule_t  op_brace_l, op_brace_r, op_parenthesis_l, op_parenthesis_r;
  rule_t  op_bracket_l, op_bracket_r;
  rule_t  op_cat;

  basic_parser_definition (TBasicParserAgent &pagent);
};
//-------------------------------------------------------------------------------------------

template <typename t_iterator>
class TCodeParser : public boost::spirit::classic::grammar<TCodeParser<t_iterator> >
{
public:
  typedef TParserAgent<t_iterator> TPAgent;

  TCodeParser (TPAgent &a) : pvar_(a) {}

  template <typename ScannerT>
  struct definition : basic_parser_definition<t_iterator,ScannerT>
    {
      typedef basic_parser_definition<t_iterator,ScannerT> TParent;
      using TParent::identifier_p                   ;
      using TParent::expr_identifier                ;
      using TParent::parenthesized_list_expr_any    ;
      using TParent::list_expr_any                  ;
      using TParent::list_expr_primitive            ;
      using TParent::expr_any                       ;
      using TParent::expr_primitive                 ;
      using TParent::expr_parenthesized_list        ;
      using TParent::expr_identifier                ;
      using TParent::parenthesized_list_literal_any ;
      using TParent::list_literal_any               ;
      using TParent::list_literal_primitive         ;
      using TParent::literal_any                    ;
      using TParent::literal_primitive              ;
      using TParent::literal_parenthesized_list     ;
      using TParent::literal_identifier             ;
      using TParent::literal_int                    ;
      using TParent::literal_real                   ;
      using TParent::literal_boolean                ;
      using TParent::literal_string                 ;
      using TParent::lcomment                       ;
      using TParent::end_of_line                    ;
      using TParent::blank_eol_p                    ;
      using TParent::op_semicolon                   ;
      using TParent::op_comma                       ;
      using TParent::op_dot                         ;
      using TParent::op_eq                          ;
      using TParent::op_at                          ;
      using TParent::op_brace_l                     ;
      using TParent::op_brace_r                     ;
      using TParent::op_parenthesis_l               ;
      using TParent::op_parenthesis_r               ;
      using TParent::op_bracket_l                   ;
      using TParent::op_bracket_r                   ;
      using TParent::op_cat                         ;

      typedef boost::spirit::classic::rule<ScannerT> rule_t;
      rule_t  statements, statements_b, statement, end_of_statement;
      rule_t  statement_std;
      rule_t  statement_starting_with_identifier;
      rule_t  statement_primitive_assign;
      rule_t  statement_composite_assign;
      rule_t  statement_elemental_assign;
      rule_t  statement_elemental_primitive_assign;
      rule_t  statement_elemental_composite_assign;
      rule_t  statement_push;
      rule_t  statement_push_primitive;
      rule_t  statement_push_composite;
      rule_t  statement_fill_all;
      rule_t  statement_fill_all_primitive;
      rule_t  statement_fill_all_composite;
      rule_t  statement_function_call;
      rule_t  statement_unexpected;

      definition (const TCodeParser &self);
      const rule_t& start() const {return statements;}
    };

private:
  TPAgent &pvar_;

};
//-------------------------------------------------------------------------------------------


#define TEMPLATE_DEC  template <typename t_iterator>
#define XCLASS        TParserAgent <t_iterator>

#define CONV_ERR_CATCHER_S  \
  try {
#define CONV_ERR_CATCHER_E  \
  } catch (boost::bad_lexical_cast &) {       \
    PRINT_ERROR("bad_lexical_cast");          \
    lexit(df);                                \
  } catch (std::string &s) {                  \
    PRINT_ERROR(s);                           \
    lexit(df);                                \
  } catch (...) {                             \
    PRINT_ERROR("bad type conversion");       \
    lexit(df);                                \
  }

#define VAR_ERR_CATCHER_S  \
  try {
#define VAR_ERR_CATCHER_E  \
  } catch (std::string &s) {                  \
    PRINT_ERROR(s);                           \
    lexit(df);                                \
  } catch (...) {                             \
    PRINT_ERROR("fatal!");                    \
    lexit(df);                                \
  }

TEMPLATE_DEC
boost::spirit::classic::parse_info<t_iterator> XCLASS::Parse (TParserInfoIn &in, t_iterator first, t_iterator last, TParserInfoOut *out)
{
  using namespace boost::spirit::classic;
  error_= false;
  in_literal_list_= false;
  line_num_= in.StartLineNum;
  file_name_= in.FileName;
  literal_table_= in.LiteralTable;
  parse_mode_= in.ParseMode;
  equivalent_code_= in.EquivalentCode;
  TCodeParser<t_iterator> parser(*this);
  variable_stack_.push_back(TExtVariable(in.Var));

  tmp_lora_msg_format_= message_system::GetFormat<LORA_MESSAGE_FORMAT_FUNCTION>();
  message_system::SetFormat(LORA_MESSAGE_FORMAT_FUNCTION(boost::bind(&XCLASS::lora_error,this,_1,_2,_3,_4,_5)) );

  switch(parse_mode_)
  {
  case pmNormal   :  expand_id_=true;   break;
  case pmPhantom  :  expand_id_=false;  break;
  default:  PRINT_ERROR("invalid parse mode: "<<int(parse_mode_)); lexit(df);
  }

  parse_info<t_iterator> res= parse(first, last, parser);

  message_system::SetFormat(tmp_lora_msg_format_);

  LASSERT(!variable_stack_.empty());
  variable_stack_.pop_back();

  #define STACK_CHECK(x_stack)  do{if(!x_stack.empty()) {LERROR(#x_stack " is not empty."); error_=true;}} while(0)
  STACK_CHECK(variable_stack_);
  STACK_CHECK(context_stack_);
  STACK_CHECK(literal_stack_);
  #undef STACK_CHECK

  if(out)
  {
    out->LastLineNum= line_num_;
  }

  return res;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
void XCLASS::StartSubParse (TParserInfoIn &in)
{
  using namespace boost::spirit::classic;
  error_= false;
  in_literal_list_= false;
  line_num_= in.StartLineNum;
  file_name_= in.FileName;
  literal_table_= in.LiteralTable;
  parse_mode_= in.ParseMode;
  equivalent_code_= in.EquivalentCode;
  variable_stack_.push_back(TExtVariable(in.Var));
  tmp_lora_msg_format_= message_system::GetFormat<LORA_MESSAGE_FORMAT_FUNCTION>();
  message_system::SetFormat(LORA_MESSAGE_FORMAT_FUNCTION(boost::bind(&XCLASS::lora_error,this,_1,_2,_3,_4,_5)) );

  switch(parse_mode_)
  {
  case pmNormal   :  expand_id_=true;   break;
  case pmPhantom  :  expand_id_=false;  break;
  default:  PRINT_ERROR("invalid parse mode: "<<int(parse_mode_)); lexit(df);
  }
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
bool XCLASS::EndSubParse (TParserInfoOut *out)
{
  message_system::SetFormat(tmp_lora_msg_format_);
  LASSERT(!variable_stack_.empty());
  variable_stack_.pop_back();

  literal_table_= NULL;

  #define STACK_CHECK(x_stack)  do{if(!x_stack.empty()) {LERROR(#x_stack " is not empty:"); error_=true;}} while(0)
  if(!error_)
  {
    STACK_CHECK(variable_stack_);
    STACK_CHECK(context_stack_);
    STACK_CHECK(literal_stack_);
  }
  #undef STACK_CHECK

  if(out)
  {
    out->LastLineNum= line_num_;
  }

  return !error_;
}
//-------------------------------------------------------------------------------------------


TEMPLATE_DEC
void XCLASS::CloseByBrace (t_iterator first, t_iterator last)
{
  LASSERT(!context_stack_.empty());
  switch(context_stack_.back())
  {
  case cCompositeMemberAssign:
  case cCompositeFillAll:
    switch(parse_mode_)
    {
    case pmNormal   :
      LASSERT(!variable_stack_.empty());
      variable_stack_.pop_back();
      break;
    case pmPhantom  :
      LASSERT(equivalent_code_);
      *equivalent_code_<<"}"<<std::endl;
      break;
    }
    context_stack_.pop_back();
    break;
  default:
    PRINT_ERROR("invalid `}'");
    lexit(qfail);
    break;
  }
}

TEMPLATE_DEC
void XCLASS::DebugMessage (t_iterator first, t_iterator last)
{
  LDEBUG(join_iterators(first,last));
}

TEMPLATE_DEC
void XCLASS::SyntaxError (t_iterator first, t_iterator last)
{
  PRINT_ERROR("syntax error:");
  std::cerr<<"  > "<<join_iterators(first,last)<<std::endl;
}

TEMPLATE_DEC
void XCLASS::PrimitiveAssignToMember (t_iterator first, t_iterator last)
{
  TLiteral value= pop_literal_x();
  std::string identifier(pop_id_x());

  switch(parse_mode_)
  {
  case pmNormal   :
    LASSERT(!variable_stack_.empty());
    CONV_ERR_CATCHER_S
    variable_stack_.back().SetMember(TVariable(identifier), ToVariable(value));
    CONV_ERR_CATCHER_E
    break;
  case pmPhantom  :
    LASSERT(equivalent_code_);
    *equivalent_code_<<identifier<<"="<<TVarFormat(value)<<std::endl;
    break;
  }
}

TEMPLATE_DEC
void XCLASS::CompositeAssignToMemberS (t_iterator first, t_iterator last)
{
  std::string identifier(pop_id_x());

  switch(parse_mode_)
  {
  case pmNormal   :
    {
      LASSERT(!variable_stack_.empty());
      TExtVariable  member;
      VAR_ERR_CATCHER_S
      member= variable_stack_.back().GetMember(TVariable(identifier));
      VAR_ERR_CATCHER_E
      variable_stack_.push_back(member);
    }
    break;
  case pmPhantom  :
    LASSERT(equivalent_code_);
    *equivalent_code_<<identifier<<"={"<<std::endl;
    break;
  }

  context_stack_.push_back(cCompositeMemberAssign);
}

TEMPLATE_DEC
void XCLASS::ElementalPrimitiveAssignToMember (t_iterator first, t_iterator last)
{
  TLiteral value= pop_literal_x();
  TLiteral key= pop_literal_x();

  switch(parse_mode_)
  {
  case pmNormal   :
    LASSERT(!variable_stack_.empty());
    CONV_ERR_CATCHER_S
    variable_stack_.back().SetMember(ToVariable(key), ToVariable(value));
    CONV_ERR_CATCHER_E
    break;
  case pmPhantom  :
    LASSERT(equivalent_code_);
    *equivalent_code_<<"["<<TVarFormat(key)<<"]="<<TVarFormat(value)<<std::endl;
    break;
  }
}

TEMPLATE_DEC
void XCLASS::ElementalCompositeAssignToMemberS (t_iterator first, t_iterator last)
{
  TLiteral key= pop_literal_x();

  switch(parse_mode_)
  {
  case pmNormal   :
    {
      LASSERT(!variable_stack_.empty());
      TExtVariable  member;
      VAR_ERR_CATCHER_S
      member= variable_stack_.back().GetMember(ToVariable(key));
      VAR_ERR_CATCHER_E
      variable_stack_.push_back(member);
    }
    break;
  case pmPhantom  :
    LASSERT(equivalent_code_);
    *equivalent_code_<<"["<<TVarFormat(key)<<"]={"<<std::endl;
    break;
  }

  context_stack_.push_back(cCompositeMemberAssign);
}

TEMPLATE_DEC
void XCLASS::PushPrimitive (t_iterator first, t_iterator last)
{
  TLiteral value= pop_literal_x();

  switch(parse_mode_)
  {
  case pmNormal   :
    LASSERT(!variable_stack_.empty());
    CONV_ERR_CATCHER_S
    variable_stack_.back().Push().DirectAssign(ToVariable(value));
    CONV_ERR_CATCHER_E
    break;
  case pmPhantom  :
    LASSERT(equivalent_code_);
    *equivalent_code_<<"[]="<<TVarFormat(value)<<std::endl;
    break;
  }
}

TEMPLATE_DEC
void XCLASS::PushComposite (t_iterator first, t_iterator last)
{
  switch(parse_mode_)
  {
  case pmNormal   :
    {
      LASSERT(!variable_stack_.empty());
      TExtVariable  new_var;
      VAR_ERR_CATCHER_S
      new_var= variable_stack_.back().Push();
      VAR_ERR_CATCHER_E
      variable_stack_.push_back(new_var);
    }
    break;
  case pmPhantom  :
    LASSERT(equivalent_code_);
    *equivalent_code_<<"[]={"<<std::endl;
    break;
  }

  context_stack_.push_back(cCompositeMemberAssign);
}

TEMPLATE_DEC
void XCLASS::FillAllPrimitive (t_iterator first, t_iterator last)
{
  TLiteral value= pop_literal_x();
  TExtForwardIterator itr,ilast;

  switch(parse_mode_)
  {
  case pmNormal   :
    {
      LASSERT(!variable_stack_.empty());
      VAR_ERR_CATCHER_S
      variable_stack_.back().GetBegin(itr);
      variable_stack_.back().GetEnd(ilast);
      VAR_ERR_CATCHER_E

      TVariable  var_value (ToVariable(value));
      CONV_ERR_CATCHER_S
      for (; itr!=ilast; ++itr)
        itr->DirectAssign(var_value);
      CONV_ERR_CATCHER_E
    }
    break;
  case pmPhantom  :
    LASSERT(equivalent_code_);
    *equivalent_code_<<"[@]="<<TVarFormat(value)<<std::endl;
    break;
  }
}

TEMPLATE_DEC
void XCLASS::FillAllComposite (t_iterator first, t_iterator last)
{
  switch(parse_mode_)
  {
  case pmNormal   :
    {
      LASSERT(!variable_stack_.empty());
      TExtForwardIterator itr,ilast;
      VAR_ERR_CATCHER_S
      variable_stack_.back().GetBegin(itr);
      variable_stack_.back().GetEnd(ilast);
      VAR_ERR_CATCHER_E
      variable_stack_.push_back(TExtVariable(itr,ilast));
    }
    break;
  case pmPhantom  :
    LASSERT(equivalent_code_);
    *equivalent_code_<<"[@]={"<<std::endl;
    break;
  }

  context_stack_.push_back(cCompositeFillAll);
}

TEMPLATE_DEC
void XCLASS::FunctionCall (t_iterator first, t_iterator last)
{
  std::list<TLiteral>  argv_entity;
  pop_literal_list_x(argv_entity);
  std::string identifier(pop_id_x());

  switch(parse_mode_)
  {
  case pmNormal   :
    {
      TVariableList        argv;
      argv.push_back(TVariable()); // means TVariable of void
      for(std::list<TLiteral>::iterator itr(argv_entity.begin()),last(argv_entity.end()); itr!=last; ++itr)
        argv.push_back(ToVariable(*itr));
      LASSERT(!variable_stack_.empty());
      CONV_ERR_CATCHER_S
      variable_stack_.back().FunctionCall(identifier, argv);
      CONV_ERR_CATCHER_E
    }
    break;
  case pmPhantom  :
    LASSERT(equivalent_code_);
    *equivalent_code_<<identifier<<"("<<TVarListFormat(argv_entity.begin(),argv_entity.end())<<")"<<std::endl;
    break;
  }
}

#undef VAR_ERR_CATCHER_S
#undef VAR_ERR_CATCHER_E

#undef CONV_ERR_CATCHER_S
#undef CONV_ERR_CATCHER_E

#undef TEMPLATE_DEC
#undef XCLASS
//-------------------------------------------------------------------------------------------

template <typename t_iterator, typename ScannerT>
basic_parser_definition<t_iterator,ScannerT>::basic_parser_definition (TBasicParserAgent &pagent)
{
  using namespace boost::spirit::classic;

  #define ALIAS_ACTION(x_action,x_as)  \
    boost::function<void(t_iterator,t_iterator)>  \
      x_as= boost::bind(&TBasicParserAgent::x_action<t_iterator>,&pagent,_1,_2)
  ALIAS_ACTION(EndOfLine                   , f_end_of_line                    );
  ALIAS_ACTION(PushLiteralIdentifier       , f_push_literal_identifier        );
  ALIAS_ACTION(PushLiteralInt              , f_push_literal_int               );
  ALIAS_ACTION(PushLiteralReal             , f_push_literal_real              );
  ALIAS_ACTION(PushLiteralBool             , f_push_literal_bool              );
  ALIAS_ACTION(PushLiteralString           , f_push_literal_string            );
  ALIAS_ACTION(BeginLiteralListAny         , f_begin_list_literal_any         );
  ALIAS_ACTION(BeginLiteralListPrimitive   , f_begin_list_literal_primitive   );
  ALIAS_ACTION(EndLiteralListPrimitive     , f_end_list_literal_primitive     );
  ALIAS_ACTION(ConcatenateIdentifiers      , f_concatenate_identifiers        );
  #undef ALIAS_ACTION

  identifier_p
    = ((alpha_p | '_') >> *(alnum_p | '_'));

  parenthesized_list_expr_any  // FIXME
    = op_parenthesis_l [f_begin_list_literal_any]
      >> list_expr_any
        >> op_parenthesis_r;

  list_expr_any  // FIXME
    = !(expr_any >> *(op_comma >> expr_any));

  list_expr_primitive  // FIXME
    = !(expr_primitive >> *(op_comma >> expr_primitive));

  expr_any  // FIXME
    = ( expr_primitive | expr_parenthesized_list );

  expr_primitive  // FIXME
    = (
        literal_real
        | literal_int
        | literal_boolean
        | literal_string
        | expr_identifier
      ) >> *blank_p;

  expr_parenthesized_list  // FIXME
    = op_parenthesis_l  [f_begin_list_literal_primitive]
        >> list_expr_primitive
          >> op_parenthesis_r  [f_end_list_literal_primitive];

  expr_identifier
    = literal_identifier
      >> (
        (op_cat >> expr_identifier)  [f_concatenate_identifiers]
        | eps_p
        );

  parenthesized_list_literal_any
    = op_parenthesis_l [f_begin_list_literal_any]
      >> list_literal_any
        >> op_parenthesis_r;

  list_literal_any
    = !(literal_any >> *(op_comma >> literal_any));

  list_literal_primitive
    = !(literal_primitive >> *(op_comma >> literal_primitive));

  literal_any
    = ( literal_primitive | literal_parenthesized_list );

  literal_primitive
    = (
        literal_real
        | literal_int
        | literal_boolean
        | literal_string
        | literal_identifier
      ) >> *blank_p;

  // NOTE: we do not use a bracketed list as a vector
  // because bracket is already used as a member indicator
  // literal_bracketed_list
    // = op_bracket_l  [f_begin_list_literal_primitive]
        // >> list_literal_primitive
          // >> op_bracket_r  [f_end_list_literal_primitive];

  literal_parenthesized_list
    = op_parenthesis_l  [f_begin_list_literal_primitive]
        >> list_literal_primitive
          >> op_parenthesis_r  [f_end_list_literal_primitive];

  literal_identifier
    = identifier_p  [f_push_literal_identifier];

  int_p_i
    = int_parser<pt_int, 10, 1, -1>();
  literal_int
    = int_p_i  [f_push_literal_int];

  literal_real
    = (real_parser<pt_real, strict_real_parser_policies<pt_real> >()
      | (!(ch_p('+') | ch_p('-')) >> str_p("inf")))  [f_push_literal_real];
    // = real_parser<pt_real, real_parser_policies<pt_real> >();

  literal_boolean
    = (str_p("false") | str_p("true") | int_p_i)  [f_push_literal_bool];

  literal_string
    = (confix_p('"', *c_escape_ch_p, '"'))  [f_push_literal_string];

  lcomment
    = str_p("//")>>*(anychar_p - eol_p)
        >> *blank_p ;

  end_of_line
    = eol_p [f_end_of_line];
  blank_eol_p
    = blank_p | end_of_line | (lcomment>>end_of_line);

  op_semicolon
    = *blank_p >> ch_p(';') >> *blank_p ;
  op_comma
    = *blank_p >> ch_p(',') >> *blank_p ;
  op_dot
    = *blank_p >> ch_p('.') >> *blank_p ;
  op_eq
    = *blank_p >> ch_p('=') >> *blank_p ;
  op_at
    = *blank_p >> ch_p('@') >> *blank_p ;
  op_brace_l
    = *blank_p >> ch_p('{') >> *blank_p ;
  op_brace_r
    = *blank_p >> ch_p('}') >> *blank_p ;
  op_parenthesis_l
    = *blank_p >> ch_p('(') >> *blank_p ;
  op_parenthesis_r
    = *blank_p >> ch_p(')') >> *blank_p ;
  op_bracket_l
    = *blank_p >> ch_p('[') >> *blank_p ;
  op_bracket_r
    = *blank_p >> ch_p(']') >> *blank_p ;
  op_cat
    = *blank_p >> str_p("##") >> *blank_p ;
}
//-------------------------------------------------------------------------------------------

template <typename t_iterator>
template <typename ScannerT>
TCodeParser<t_iterator>::definition<ScannerT>::definition (const TCodeParser &self)
  :
    basic_parser_definition<t_iterator,ScannerT>(self.pvar_)
{
  using namespace boost::spirit::classic;

  #define ALIAS_ACTION(x_action,x_as)  \
    boost::function<void(t_iterator,t_iterator)>  \
      x_as= boost::bind(&TPAgent::x_action,&self.pvar_,_1,_2)
  ALIAS_ACTION(PrimitiveAssignToMember            , f_primitive_assign               );
  ALIAS_ACTION(CompositeAssignToMemberS           , f_composite_assign_s             );
  ALIAS_ACTION(ElementalPrimitiveAssignToMember   , f_elemental_primitive_assign     );
  ALIAS_ACTION(ElementalCompositeAssignToMemberS  , f_elemental_composite_assign_s   );
  ALIAS_ACTION(PushPrimitive                      , f_push_primitive                 );
  ALIAS_ACTION(PushComposite                      , f_push_composite                 );
  ALIAS_ACTION(FillAllPrimitive                   , f_fill_all_primitive             );
  ALIAS_ACTION(FillAllComposite                   , f_fill_all_composite             );
  ALIAS_ACTION(FunctionCall                       , f_function_call                  );
  ALIAS_ACTION(SyntaxError                        , f_syntax_error                   );
  ALIAS_ACTION(CloseByBrace                       , f_close_by_brace                 );
  ALIAS_ACTION(DebugMessage                       , f_debug                          );
  #undef ALIAS_ACTION

  statements
    = *(*blank_eol_p >> statement);
  statements_b
    = *(*blank_eol_p >> statement)
      >> *blank_eol_p
        >> op_brace_r [f_close_by_brace];
  statement
    = (
      (lcomment >> (end_of_line | end_p))
      | (statement_std >> end_of_statement)
      | op_semicolon
      | (statement_unexpected [f_syntax_error] >> end_of_statement)
      );
  end_of_statement
    = *blank_p
      >> (
        op_semicolon
        | end_of_line
        | end_p
        | (lcomment>>end_of_line)
        );

  statement_unexpected
    = (anychar_p - eol_p - op_brace_r) >> *(anychar_p - eol_p);

  statement_std
    = (
      statement_starting_with_identifier
      | statement_fill_all
      | statement_elemental_assign
      | statement_push
      );

  statement_starting_with_identifier
    = expr_identifier
      >> (
        statement_composite_assign
        | statement_function_call
        | statement_primitive_assign
        );

  statement_elemental_assign
    = op_bracket_l >> expr_any >> op_bracket_r
      >> (
        statement_elemental_primitive_assign
        | statement_elemental_composite_assign
        );

  statement_push
    = op_bracket_l >> op_bracket_r
      >> (
        statement_push_primitive
        | statement_push_composite
        );

  statement_fill_all
    = op_bracket_l >> op_at >> op_bracket_r
      >> (
        statement_fill_all_primitive
        | statement_fill_all_composite
        );

  statement_primitive_assign
    = (op_eq >> expr_any) [f_primitive_assign];
  statement_composite_assign
    = op_eq >> *blank_eol_p
      >> op_brace_l [f_composite_assign_s]
        >> statements_b;

  statement_elemental_primitive_assign
    = (op_eq >> expr_any) [f_elemental_primitive_assign];
  statement_elemental_composite_assign
    = op_eq >> *blank_eol_p
      >> op_brace_l [f_elemental_composite_assign_s]
        >> statements_b;

  statement_push_primitive
    = (op_eq >> expr_any) [f_push_primitive];
  statement_push_composite
    = op_eq >> *blank_eol_p
      >> op_brace_l [f_push_composite]
        >> statements_b;

  statement_fill_all_primitive
    = (op_eq >> expr_any) [f_fill_all_primitive];
  statement_fill_all_composite
    = op_eq >> *blank_eol_p
      >> op_brace_l [f_fill_all_composite]
        >> statements_b;

  statement_function_call
    = parenthesized_list_expr_any [f_function_call];

}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_parser_impl_h
//-------------------------------------------------------------------------------------------
