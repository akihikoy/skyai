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
#include <sstream>
#ifndef VAR_SPACE_ERR_EXIT
#define VAR_SPACE_ERR_EXIT(x_err_message)  \
  do{std::stringstream ss; ss<<x_err_message; throw ss.str();}while(0)
#endif
//-------------------------------------------------------------------------------------------
#include <lora/variable_parser.h>
#include <lora/variable_space.h>
// #include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
#include <lora/string.h>
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


template <typename t_iterator>
class TParserAgent
{
public:

  TParserAgent () : error_(false) {}

  boost::spirit::classic::parse_info<t_iterator> Parse (TVariable &var, t_iterator first, t_iterator last);
  void StartSubParse (TVariable &var, int line_num);
  bool EndSubParse (int *line_num);

  bool Error() const {return error_;}

  void EndOfLine (t_iterator first, t_iterator last);
  void CloseByBrace (t_iterator first, t_iterator last);
  void DebugMessage (t_iterator first, t_iterator last);
  void SyntaxError (t_iterator first, t_iterator last);
  void PushIdentifier (t_iterator first, t_iterator last);
  // void PushLiteral (t_iterator first, t_iterator last);
  void PushLiteralInt (t_iterator first, t_iterator last);
  void PushLiteralReal (t_iterator first, t_iterator last);
  void PushLiteralBool (t_iterator first, t_iterator last);
  void PushLiteralString (t_iterator first, t_iterator last);
  void BeginLiteralListAny (t_iterator first, t_iterator last);
  void BeginLiteralListPrimitive (t_iterator first, t_iterator last);
  void EndLiteralListPrimitive (t_iterator first, t_iterator last);
  void PrimitiveAssignToMember (t_iterator first, t_iterator last);
  void CompositeAssignToMemberS (t_iterator first, t_iterator last);
  void ElementalPrimitiveAssignToMember (t_iterator first, t_iterator last);
  void ElementalCompositeAssignToMemberS (t_iterator first, t_iterator last);
  void PushPrimitive (t_iterator first, t_iterator last);
  void PushComposite (t_iterator first, t_iterator last);
  void FillAllPrimitive (t_iterator first, t_iterator last);
  void FillAllComposite (t_iterator first, t_iterator last);
  void FunctionCall (t_iterator first, t_iterator last);

private:


  //! \todo FIXME: very inefficient code
  struct TLiteral
    {
      enum TType {ltBeginListAny=0, ltInt, ltReal, ltBool, ltString, ltRealList};
      TType        LType;
      pt_int       LInt;
      pt_real      LReal;
      pt_bool      LBool;
      pt_string    LString;
      std::list<pt_real>  LRealList;

      //!\todo implement a copy operator
      TVariable ToVariable ()
        {
          TVariable var;
          switch(LType)
          {
          case ltInt      :  var.Generate(LInt);      break;
          case ltReal     :  var.Generate(LReal);     break;
          case ltBool     :  var.Generate(LBool);     break;
          case ltString   :  var.Generate(LString);   break;
          case ltRealList :  var.Generate(LRealList); break;
          default         :  LERROR("fatal!"); lexit(df);
          }
          return var;
        }
    };
  typedef std::list<TLiteral> TLiteralList;


  std::list<TExtVariable>  variable_stack_;
  enum TContext {cGlobal=0, cCompositeMemberAssign, cCompositeFillAll};
  std::list<TContext>  context_stack_;
  std::list<std::string>  id_stack_;
  bool error_;
  int  line_num_;

  TLiteralList literal_stack_;
  bool in_literal_list_;

  // TLiteral pop_literal_stack ()
    // {
      // res= literal_stack_.back();
      // literal_stack_.pop_back();
      // return res;
    // }

  static std::string join_iterators (t_iterator first, t_iterator last)
    {
      std::stringstream ss;
      for(;first!=last;++first)
        ss<<*first;
      return ss.str();
    }

};
//-------------------------------------------------------------------------------------------


template <typename t_iterator>
class TCodeParser : public boost::spirit::classic::grammar<TCodeParser<t_iterator> >
{
public:
  typedef TParserAgent<t_iterator> TPAgent;

  TCodeParser (TPAgent &a) : pvar_(a) {}

  template <typename ScannerT>
  struct definition
    {
      typedef boost::spirit::classic::rule<ScannerT> rule_t;
      rule_t  identifier_p;
      rule_t  list_literal_any, list_literal_primitive;
      rule_t  literal_any, literal_primitive;
      // rule_t  literal_bracketed_list;
      rule_t  literal_parenthesized_list;
      rule_t  literal_int, literal_real, literal_boolean, literal_string;
      rule_t  lcomment;
      rule_t  end_of_line, blank_eol_p;
      rule_t  op_semicolon, op_comma, op_dot, op_eq, op_at;
      rule_t  op_brace_l, op_brace_r, op_parenthesis_l, op_parenthesis_r;
      rule_t  op_bracket_l, op_bracket_r;
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
  } catch (boost::bad_lexical_cast &) {                                                                \
    std::cout<<"(l."<<line_num_<<") bad_lexical_cast: "<<join_iterators(first,last)<<std::endl;        \
    lexit(df);                                                                                         \
  } catch (std::string &s) {                                                                           \
    std::cout<<"(l."<<line_num_<<") "<<s<<": "<<join_iterators(first,last)<<std::endl;                 \
    lexit(df);                                                                                         \
  } catch (...) {                                                                                      \
    std::cout<<"(l."<<line_num_<<") bad type conversion: "<<join_iterators(first,last)<<std::endl;     \
    lexit(df);                                                                                         \
  }

#define VAR_ERR_CATCHER_S  \
  try {
#define VAR_ERR_CATCHER_E  \
  } catch (std::string &s) {                                                             \
    std::cout<<"(l."<<line_num_<<") "<<s<<": "<<join_iterators(first,last)<<std::endl;   \
    lexit(df);                                                                           \
  } catch (...) {                                                                        \
    std::cout<<"(l."<<line_num_<<") fatal!: "<<join_iterators(first,last)<<std::endl;    \
    lexit(df);                                                                           \
  }

TEMPLATE_DEC
boost::spirit::classic::parse_info<t_iterator> XCLASS::Parse (TVariable &var, t_iterator first, t_iterator last)
{
  using namespace boost::spirit::classic;
  error_= false;
  line_num_= 1;
  in_literal_list_= false;
  TCodeParser<t_iterator> parser(*this);
  variable_stack_.push_back(TExtVariable(var));
  parse_info<t_iterator> res= parse(first, last, parser);
  //*dbg*/std::cerr<<"id-stack: ";while(!id_stack_.empty()) {std::cerr<<" "<<id_stack_.back();id_stack_.pop_back();} std::cerr<<std::endl;

  LASSERT(!variable_stack_.empty());
  variable_stack_.pop_back();

  #define STACK_CHECH(x_stack)  do{if(!x_stack.empty()) {LERROR(#x_stack " is not empty."); error_=true;}} while(0)
  STACK_CHECH(variable_stack_);
  STACK_CHECH(context_stack_);
  STACK_CHECH(id_stack_);
  STACK_CHECH(literal_stack_);
  #undef STACK_CHECH

  return res;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
void XCLASS::StartSubParse (TVariable &var, int line_num)
{
  using namespace boost::spirit::classic;
  error_= false;
  line_num_= line_num;
  in_literal_list_= false;
  variable_stack_.push_back(TExtVariable(var));
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
bool XCLASS::EndSubParse (int *line_num)
{
  LASSERT(!variable_stack_.empty());
  variable_stack_.pop_back();

  #define STACK_CHECH(x_stack)  do{if(!x_stack.empty()) {LERROR(#x_stack " is not empty."); error_=true;}} while(0)
  STACK_CHECH(variable_stack_);
  STACK_CHECH(context_stack_);
  STACK_CHECH(id_stack_);
  STACK_CHECH(literal_stack_);
  #undef STACK_CHECH

  if(line_num) *line_num= line_num_;

  return !error_;
}
//-------------------------------------------------------------------------------------------


TEMPLATE_DEC
void XCLASS::EndOfLine (t_iterator first, t_iterator last)
{
  ++line_num_;
}

TEMPLATE_DEC
void XCLASS::CloseByBrace (t_iterator first, t_iterator last)
{
  LASSERT(!variable_stack_.empty());
  LASSERT(!context_stack_.empty());
  switch(context_stack_.back())
  {
  case cCompositeMemberAssign:
    variable_stack_.pop_back();
    context_stack_.pop_back();
    break;
  case cCompositeFillAll:
    variable_stack_.pop_back();
    context_stack_.pop_back();
    break;
  default:
    LERROR("(l."<<line_num_<<") invalid `}'");
    lexit(df);
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
  error_= true;
  std::cerr<<ioscc::red<<"Syntax error (l."<<line_num_<<"):"<<std::endl<<"  > ";
  for(;first!=last;++first)
    std::cerr<<*first;
  std::cout<<std::endl;
}

TEMPLATE_DEC
void XCLASS::PushIdentifier (t_iterator first, t_iterator last)
{
  id_stack_.push_back(join_iterators(first,last));
}
// TEMPLATE_DEC
// void XCLASS::PushLiteral (t_iterator first, t_iterator last)
// {
  // TLiteral literal;
  // literal.LString= join_iterators(first,last);
  // literal.LType= TLiteral::ltString;
  // literal_stack_.push_back(literal);
// }
TEMPLATE_DEC
void XCLASS::PushLiteralInt (t_iterator first, t_iterator last)
{
  if(!in_literal_list_)
  {
    TLiteral literal;
    literal.LInt= ConvertFromStr<pt_int>(join_iterators(first,last));
    literal.LType= TLiteral::ltInt;
    literal_stack_.push_back(literal);
  }
  else
  {
    LASSERT(!literal_stack_.empty());
    literal_stack_.back().LRealList.push_back(
        static_cast<pt_real>(ConvertFromStr<pt_int>(join_iterators(first,last))));
  }
}
TEMPLATE_DEC
void XCLASS::PushLiteralReal (t_iterator first, t_iterator last)
{
  if(!in_literal_list_)
  {
    TLiteral literal;
    literal.LReal= ConvertFromStr<pt_real>(join_iterators(first,last));
    literal.LType= TLiteral::ltReal;
    literal_stack_.push_back(literal);
  }
  else
  {
    LASSERT(!literal_stack_.empty());
    literal_stack_.back().LRealList.push_back(
        ConvertFromStr<pt_real>(join_iterators(first,last)));
  }
}
TEMPLATE_DEC
void XCLASS::PushLiteralBool (t_iterator first, t_iterator last)
{
  if(!in_literal_list_)
  {
    TLiteral literal;
    literal.LBool= ConvertFromStr<pt_bool>(join_iterators(first,last));
    literal.LType= TLiteral::ltBool;
    literal_stack_.push_back(literal);
  }
  else
  {
    LASSERT(!literal_stack_.empty());
    literal_stack_.back().LRealList.push_back(
        static_cast<pt_real>(ConvertFromStr<pt_bool>(join_iterators(first,last))));
  }
}
TEMPLATE_DEC
void XCLASS::PushLiteralString (t_iterator first, t_iterator last)
{
  if(!in_literal_list_)
  {
    TLiteral literal;
    literal.LString= DecodeString(join_iterators(first,last));
    literal.LType= TLiteral::ltString;
    literal_stack_.push_back(literal);
  }
  else
  {
    LERROR("(l."<<line_num_<<") string type cannot be an element of a list literal");
    LASSERT(!literal_stack_.empty());
    literal_stack_.back().LRealList.push_back(0.0l);
  }
}
TEMPLATE_DEC
void XCLASS::BeginLiteralListAny (t_iterator first, t_iterator last)
{
  TLiteral literal;
  literal.LType= TLiteral::ltBeginListAny;
  literal_stack_.push_back(literal);
}
TEMPLATE_DEC
void XCLASS::BeginLiteralListPrimitive (t_iterator first, t_iterator last)
{
  TLiteral literal;
  literal.LType= TLiteral::ltRealList;
  literal_stack_.push_back(literal);
  in_literal_list_= true;
}
TEMPLATE_DEC
void XCLASS::EndLiteralListPrimitive (t_iterator first, t_iterator last)
{
  in_literal_list_= false;
}

TEMPLATE_DEC
void XCLASS::PrimitiveAssignToMember (t_iterator first, t_iterator last)
{
  LASSERT(!literal_stack_.empty());
  LASSERT(!id_stack_.empty());
  LASSERT(!variable_stack_.empty());
  TLiteral value= literal_stack_.back();  literal_stack_.pop_back();
  std::string identifier(id_stack_.back());  id_stack_.pop_back();

  CONV_ERR_CATCHER_S
  variable_stack_.back().SetMember(TVariable(identifier), value.ToVariable());
  CONV_ERR_CATCHER_E
}

TEMPLATE_DEC
void XCLASS::CompositeAssignToMemberS (t_iterator first, t_iterator last)
{
  LASSERT(!id_stack_.empty());
  LASSERT(!variable_stack_.empty());
  std::string identifier(id_stack_.back());  id_stack_.pop_back();

  TExtVariable  member;
  VAR_ERR_CATCHER_S
  member= variable_stack_.back().GetMember(TVariable(identifier));
  VAR_ERR_CATCHER_E

  variable_stack_.push_back(member);
  context_stack_.push_back(cCompositeMemberAssign);
}

TEMPLATE_DEC
void XCLASS::ElementalPrimitiveAssignToMember (t_iterator first, t_iterator last)
{
  LASSERT1op1(literal_stack_.size(),>=,2);
  LASSERT(!variable_stack_.empty());
  TLiteral value= literal_stack_.back();  literal_stack_.pop_back();
  TLiteral key= literal_stack_.back();  literal_stack_.pop_back();

  CONV_ERR_CATCHER_S
  variable_stack_.back().SetMember(key.ToVariable(), value.ToVariable());
  CONV_ERR_CATCHER_E
}

TEMPLATE_DEC
void XCLASS::ElementalCompositeAssignToMemberS (t_iterator first, t_iterator last)
{
  LASSERT(!literal_stack_.empty());
  LASSERT(!variable_stack_.empty());
  TLiteral key= literal_stack_.back();  literal_stack_.pop_back();

  TExtVariable  member;
  VAR_ERR_CATCHER_S
  member= variable_stack_.back().GetMember(key.ToVariable());
  VAR_ERR_CATCHER_E

  variable_stack_.push_back(member);
  context_stack_.push_back(cCompositeMemberAssign);
}

TEMPLATE_DEC
void XCLASS::PushPrimitive (t_iterator first, t_iterator last)
{
  LASSERT(!literal_stack_.empty());
  LASSERT(!variable_stack_.empty());
  TLiteral value= literal_stack_.back();  literal_stack_.pop_back();

  CONV_ERR_CATCHER_S
  variable_stack_.back().Push().DirectAssign(value.ToVariable());
  CONV_ERR_CATCHER_E
}

TEMPLATE_DEC
void XCLASS::PushComposite (t_iterator first, t_iterator last)
{
  LASSERT(!variable_stack_.empty());

  TExtVariable  new_var;
  VAR_ERR_CATCHER_S
  new_var= variable_stack_.back().Push();
  VAR_ERR_CATCHER_E

  variable_stack_.push_back(new_var);
  context_stack_.push_back(cCompositeMemberAssign);
}

TEMPLATE_DEC
void XCLASS::FillAllPrimitive (t_iterator first, t_iterator last)
{
  LASSERT(!literal_stack_.empty());
  LASSERT(!variable_stack_.empty());
  TLiteral value= literal_stack_.back();  literal_stack_.pop_back();
  TVariable  var_value (value.ToVariable());
  TExtForwardIterator itr,ilast;

  VAR_ERR_CATCHER_S
  variable_stack_.back().GetBegin(itr);
  variable_stack_.back().GetEnd(ilast);
  VAR_ERR_CATCHER_E

  CONV_ERR_CATCHER_S
  for (; itr!=ilast; ++itr)
    itr->DirectAssign(var_value);
  CONV_ERR_CATCHER_E
}

TEMPLATE_DEC
void XCLASS::FillAllComposite (t_iterator first, t_iterator last)
{
  LASSERT(!variable_stack_.empty());
  TExtForwardIterator itr,ilast;

  VAR_ERR_CATCHER_S
  variable_stack_.back().GetBegin(itr);
  variable_stack_.back().GetEnd(ilast);
  VAR_ERR_CATCHER_E

  variable_stack_.push_back(TExtVariable(itr,ilast));
  context_stack_.push_back(cCompositeFillAll);
}

TEMPLATE_DEC
void XCLASS::FunctionCall (t_iterator first, t_iterator last)
{
  LASSERT(!id_stack_.empty());
  LASSERT(!variable_stack_.empty());

  std::string identifier(id_stack_.back());  id_stack_.pop_back();
  TVariableList  argv;
  argv.push_back(TVariable()); // means TVariable of void
  typename TLiteralList::iterator  ilast(literal_stack_.end()),ifirst(literal_stack_.begin());
  typename TLiteralList::iterator  itr(ilast);
  for(--itr; itr!=ifirst && itr->LType!=TLiteral::ltBeginListAny; --itr) {}
  if(itr->LType!=TLiteral::ltBeginListAny)  {LERROR("fatal!"); lexit(df);}
  for(++itr; itr!=ilast; ++itr)
    argv.push_back(itr->ToVariable());

  CONV_ERR_CATCHER_S
  variable_stack_.back().FunctionCall(identifier, argv);
  CONV_ERR_CATCHER_E

  while(literal_stack_.back().LType!=TLiteral::ltBeginListAny)
    literal_stack_.pop_back();
  literal_stack_.pop_back();
}

#undef VAR_ERR_CATCHER_S
#undef VAR_ERR_CATCHER_E

#undef CONV_ERR_CATCHER_S
#undef CONV_ERR_CATCHER_E

#undef TEMPLATE_DEC
#undef XCLASS
//-------------------------------------------------------------------------------------------


template <typename t_iterator>
template <typename ScannerT>
TCodeParser<t_iterator>::definition<ScannerT>::definition (const TCodeParser &self)
{
  using namespace boost::spirit::classic;

  #define ALIAS_ACTION(x_action,x_as)  \
    boost::function<void(t_iterator,t_iterator)>  \
      x_as= boost::bind(&TPAgent::x_action,&self.pvar_,_1,_2)
  ALIAS_ACTION(EndOfLine, f_end_of_line);
  ALIAS_ACTION(PushIdentifier, f_push_identifier);
  // ALIAS_ACTION(PushLiteral, f_push_literal);
  ALIAS_ACTION(PushLiteralInt, f_push_literal_int);
  ALIAS_ACTION(PushLiteralReal, f_push_literal_real);
  ALIAS_ACTION(PushLiteralBool, f_push_literal_bool);
  ALIAS_ACTION(PushLiteralString, f_push_literal_string);
  ALIAS_ACTION(BeginLiteralListAny, f_begin_list_literal_any);
  ALIAS_ACTION(BeginLiteralListPrimitive, f_begin_list_literal_primitive);
  ALIAS_ACTION(EndLiteralListPrimitive, f_end_list_literal_primitive);
  ALIAS_ACTION(PrimitiveAssignToMember, f_primitive_assign);
  ALIAS_ACTION(CompositeAssignToMemberS, f_composite_assign_s);
  ALIAS_ACTION(ElementalPrimitiveAssignToMember, f_elemental_primitive_assign);
  ALIAS_ACTION(ElementalCompositeAssignToMemberS, f_elemental_composite_assign_s);
  ALIAS_ACTION(PushPrimitive, f_push_primitive);
  ALIAS_ACTION(PushComposite, f_push_composite);
  ALIAS_ACTION(FillAllPrimitive, f_fill_all_primitive);
  ALIAS_ACTION(FillAllComposite, f_fill_all_composite);
  ALIAS_ACTION(FunctionCall, f_function_call);
  ALIAS_ACTION(SyntaxError, f_syntax_error);
  ALIAS_ACTION(CloseByBrace, f_close_by_brace);
  ALIAS_ACTION(DebugMessage, f_debug);
  #undef ALIAS_ACTION

  identifier_p
    = ((alpha_p | '_') >> *(alnum_p | '_'));

  list_literal_any
    = !(literal_any >> *(op_comma >> literal_any));

  list_literal_primitive
    = !(literal_primitive >> *(op_comma >> literal_primitive));

  literal_any
    = ( literal_primitive | literal_parenthesized_list );

  literal_primitive
    = (
        literal_real      [f_push_literal_real]
        | literal_int     [f_push_literal_int]
        | literal_boolean [f_push_literal_bool]
        | literal_string  [f_push_literal_string]
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

  literal_int
    = int_parser<pt_int, 10, 1, -1>();

  literal_real
    = (real_parser<pt_real, strict_real_parser_policies<pt_real> >()
      | (!(ch_p('+') | ch_p('-')) >> str_p("inf")));
    // = real_parser<pt_real, real_parser_policies<pt_real> >();

  literal_boolean
    = (str_p("false") | str_p("true") | int_p);

  literal_string
    = confix_p('"', *c_escape_ch_p, '"');

  lcomment
    = str_p("//")>>*(anychar_p - eol_p)
        >> *blank_p ;

  end_of_line
    = eol_p [f_end_of_line];
  blank_eol_p
    = blank_p | end_of_line;

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
    = identifier_p [f_push_identifier]
      >> (
        statement_composite_assign
        | statement_function_call
        | statement_primitive_assign
        );

  statement_elemental_assign
    = op_bracket_l >> literal_any >> op_bracket_r
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
    = (op_eq >> literal_any) [f_primitive_assign];
  statement_composite_assign
    = op_eq >> *blank_eol_p
      >> op_brace_l [f_composite_assign_s]
        >> statements_b;

  statement_elemental_primitive_assign
    = (op_eq >> literal_any) [f_elemental_primitive_assign];
  statement_elemental_composite_assign
    = op_eq >> *blank_eol_p
      >> op_brace_l [f_elemental_composite_assign_s]
        >> statements_b;

  statement_push_primitive
    = (op_eq >> literal_any) [f_push_primitive];
  statement_push_composite
    = op_eq >> *blank_eol_p
      >> op_brace_l [f_push_composite]
        >> statements_b;

  statement_fill_all_primitive
    = (op_eq >> literal_any) [f_fill_all_primitive];
  statement_fill_all_composite
    = op_eq >> *blank_eol_p
      >> op_brace_l [f_fill_all_composite]
        >> statements_b;

  statement_function_call
    = op_parenthesis_l [f_begin_list_literal_any]
        >> list_literal_any
          >> op_parenthesis_r [f_function_call];

}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_parser_impl_h
//-------------------------------------------------------------------------------------------
