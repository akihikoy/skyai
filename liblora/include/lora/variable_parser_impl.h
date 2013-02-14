//-------------------------------------------------------------------------------------------
/*! \file    variable_parser_impl.h
    \brief   liblora - parser for variable-space  (implement header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    May.17, 2010-
    \date    Jun.24, 2010    Bug fixed for FillAllComposite ([@]={...})

    Copyright (C) 2010, 2012  Akihiko Yamaguchi

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
#include <lora/variable_bindef.h>
#include <lora/variable_space_fwd.h>
#include <lora/string.h>
//-------------------------------------------------------------------------------------------
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


template <typename t_iterator>
class TCodeParser : public boost::spirit::classic::grammar<TCodeParser<t_iterator> >
{
public:

  TCodeParser() : bin_stack_(NULL), file_name_(NULL), line_num_(NULL), error_(NULL) {}

  const TBinaryStack& BinStack() const {return *bin_stack_;}
  const std::string& FileName() const {return file_name_;}
  int LineNum() const {return *line_num_;}
  bool Error() const {return *error_;}

  void SetBinStack(TBinaryStack *p)  {bin_stack_= p;}
  void SetFileName(std::string *p)  {file_name_= p;}
  void SetLineNum(int *p)  {line_num_= p;}
  void SetError(bool *p)  {error_= p;}
  void SetCallbacks(TParserCallbacks callbacks)  {callbacks_= callbacks;}

  void LoraError (message_system::TMessageType type, int linenum, const char *filename, const char *functionname, std::stringstream &ss) const
    {
      if(type==message_system::mtError)
        print_error(ss.str());
      else
        DefaultFormat(type, linenum, filename, functionname, ss);
    }

  const pt_string& TmpIdentifier() const {return tmp_identifier_ ;}
  const pt_int&    TmpInt()        const {return tmp_int_        ;}
  const pt_real&   TmpReal()       const {return tmp_real_       ;}
  const pt_bool&   TmpBool()       const {return tmp_bool_       ;}
  const pt_string& TmpString()     const {return tmp_string_     ;}

  template <typename ScannerT>
  struct definition
    {
      typedef boost::spirit::classic::rule<ScannerT> rule_t;
      rule_t  statements, statement, end_of_statement;
      rule_t  statement_std;
      rule_t  statement_print;
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

      rule_t  parenthesized_list_expr_any, list_expr_any, list_expr_primitive;
      rule_t  expr_block_list, expr_block, expr_additive, expr_multiplicative, expr_primary, expr_literal;
      rule_t  expr_or, expr_and, expr_equality, expr_relational, expr_unary, expr_postfix;
      rule_t  expr_identifier;

      rule_t  type_block, type_primitive, type_list;

      rule_t  int_p_i;
      rule_t  lex_identifier, lex_int, lex_real, lex_boolean, lex_string;

      rule_t  literal_identifier, literal_int, literal_real, literal_boolean, literal_string;
      rule_t  end_of_line;

      rule_t  lcomment, blank_eol_p;
      rule_t  op_semicolon, op_comma, op_dot, op_eq, op_at;
      rule_t  op_brace_l, op_brace_r, op_parenthesis_l, op_parenthesis_r;
      rule_t  op_bracket_l, op_bracket_r;
      rule_t  op_plus, op_minus, op_star, op_slash, op_percent;
      rule_t  op_lt, op_gt, op_lteq, op_gteq, op_deq, op_exeq;
      rule_t  op_amp, op_bar, op_excl, op_damp, op_dbar, op_cat;

      definition (const TCodeParser &self);
      const rule_t& start() const {return statements;}
    };

private:

  TBinaryStack *bin_stack_;
  std::string  *file_name_;
  int          *line_num_;
  bool         *error_;

  TParserCallbacks  callbacks_;

  mutable pt_string tmp_identifier_;
  mutable pt_int    tmp_int_;
  mutable pt_real   tmp_real_;
  mutable pt_bool   tmp_bool_;
  mutable pt_string tmp_string_;

  template <typename ScannerT>
  friend struct TCodeParser<t_iterator>::definition;

  void add_single_command(t_iterator first, t_iterator last, int command) const;

  #define DECL_ACTION(x_func)  void x_func (t_iterator first, t_iterator last) const
  DECL_ACTION(end_of_line);
  DECL_ACTION(store_literal_identifier);
  DECL_ACTION(store_literal_int);
  DECL_ACTION(store_literal_real);
  DECL_ACTION(store_literal_bool);
  DECL_ACTION(store_literal_string);
  DECL_ACTION(push_literal_identifier);
  DECL_ACTION(push_literal_int);
  DECL_ACTION(push_literal_real);
  DECL_ACTION(push_literal_bool);
  DECL_ACTION(push_literal_string);
  DECL_ACTION(push_type);

  DECL_ACTION(debug_message);
  DECL_ACTION(syntax_error);
  #undef DECL_ACTION


  static std::string join_iterators (t_iterator first, t_iterator last)
    {
      std::stringstream ss;
      for(;first!=last;++first)
        ss<<*first;
      return ss.str();
    }

  void print_error (const std::string &str) const
    {
      *error_= true;
      std::cerr<<"("<<*file_name_<<":"<<*line_num_<<") "<<str<<std::endl;
    }

  void on_command_pushed() const
    {
      if(callbacks_.OnCommandPushed)
        callbacks_.OnCommandPushed(*file_name_,*line_num_,*error_);
    }

};
//-------------------------------------------------------------------------------------------

#define TEMPLATE_DEC  template <typename t_iterator>
#define XCLASS        TCodeParser <t_iterator>

TEMPLATE_DEC
void XCLASS::add_single_command(t_iterator first, t_iterator last, int command) const
{
  bin_stack_->Push(command);
  on_command_pushed();
}

TEMPLATE_DEC
void XCLASS::end_of_line (t_iterator first, t_iterator last) const
{
  if(callbacks_.OnEndOfLine)
    callbacks_.OnEndOfLine(*file_name_,*line_num_,*error_);

  ++(*line_num_);
}

TEMPLATE_DEC
void XCLASS::store_literal_identifier (t_iterator first, t_iterator last) const
{
  tmp_identifier_= pt_string(join_iterators(first,last));
}
TEMPLATE_DEC
void XCLASS::store_literal_int (t_iterator first, t_iterator last) const
{
  tmp_int_= ConvertFromStr<pt_int>(join_iterators(first,last));
}
TEMPLATE_DEC
void XCLASS::store_literal_real (t_iterator first, t_iterator last) const
{
  tmp_real_= ConvertFromStr<pt_real>(join_iterators(first,last));
}
TEMPLATE_DEC
void XCLASS::store_literal_bool (t_iterator first, t_iterator last) const
{
  tmp_bool_= ConvertFromStr<pt_bool>(join_iterators(first,last));
}
TEMPLATE_DEC
void XCLASS::store_literal_string (t_iterator first, t_iterator last) const
{
  tmp_string_= pt_string(DecodeString(join_iterators(first,last)));
}

TEMPLATE_DEC
void XCLASS::push_literal_identifier (t_iterator first, t_iterator last) const
{
  bin_stack_->Push(bin::cmd::PUSH);
  bin_stack_->Push(bin::vtype::ID);
  bin_stack_->Push(tmp_identifier_);
  on_command_pushed();
}
TEMPLATE_DEC
void XCLASS::push_literal_int (t_iterator first, t_iterator last) const
{
  bin_stack_->Push(bin::cmd::PUSH);
  bin_stack_->Push(bin::vtype::INT);
  bin_stack_->Push(tmp_int_);
  on_command_pushed();
}
TEMPLATE_DEC
void XCLASS::push_literal_real (t_iterator first, t_iterator last) const
{
  bin_stack_->Push(bin::cmd::PUSH);
  bin_stack_->Push(bin::vtype::REAL);
  bin_stack_->Push(tmp_real_);
  on_command_pushed();
}
TEMPLATE_DEC
void XCLASS::push_literal_bool (t_iterator first, t_iterator last) const
{
  bin_stack_->Push(bin::cmd::PUSH);
  bin_stack_->Push(bin::vtype::BOOL);
  bin_stack_->Push(tmp_bool_);
  on_command_pushed();
}
TEMPLATE_DEC
void XCLASS::push_literal_string (t_iterator first, t_iterator last) const
{
  bin_stack_->Push(bin::cmd::PUSH);
  bin_stack_->Push(bin::vtype::STR);
  bin_stack_->Push(tmp_string_);
  on_command_pushed();
}
TEMPLATE_DEC
void XCLASS::push_type (t_iterator first, t_iterator last) const
{
  bin_stack_->Push(bin::cmd::PUSH);
  bin_stack_->Push(bin::vtype::TYPE);
  bin_stack_->Push(bin::TypeCode(tmp_identifier_));
  on_command_pushed();
}

TEMPLATE_DEC
void XCLASS::debug_message (t_iterator first, t_iterator last) const
{
  LDEBUG(join_iterators(first,last));
}

TEMPLATE_DEC
void XCLASS::syntax_error (t_iterator first, t_iterator last) const
{
  print_error("syntax error:");
  std::cerr<<"  > "<<join_iterators(first,last)<<std::endl;
}

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
      x_as= boost::bind(&TCodeParser::x_action,&self,_1,_2)
  ALIAS_ACTION(end_of_line                 , f_end_of_line                    );
  ALIAS_ACTION(store_literal_identifier    , f_store_literal_identifier       );
  ALIAS_ACTION(store_literal_int           , f_store_literal_int              );
  ALIAS_ACTION(store_literal_real          , f_store_literal_real             );
  ALIAS_ACTION(store_literal_bool          , f_store_literal_bool             );
  ALIAS_ACTION(store_literal_string        , f_store_literal_string           );
  ALIAS_ACTION(push_literal_identifier     , f_push_literal_identifier        );
  ALIAS_ACTION(push_literal_int            , f_push_literal_int               );
  ALIAS_ACTION(push_literal_real           , f_push_literal_real              );
  ALIAS_ACTION(push_literal_bool           , f_push_literal_bool              );
  ALIAS_ACTION(push_literal_string         , f_push_literal_string            );
  ALIAS_ACTION(push_type                   , f_push_type                      );

  ALIAS_ACTION(debug_message               , f_debug                          );
  ALIAS_ACTION(syntax_error                , f_syntax_error                   );
  #undef ALIAS_ACTION

  #define SCMD(x_command)  \
    boost::bind(&TCodeParser::add_single_command,&self,_1,_2,bin::cmd::x_command)

#include <lora/bits/basic_syntax.h>

  statements
    = *(*blank_eol_p >> statement);
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
      statement_print
      | statement_starting_with_identifier
      | statement_fill_all
      | statement_elemental_assign
      | statement_push
      );

  statement_print
    = str_p("print") >> +blank_eol_p >> expr_block [SCMD(PRINT)];

  statement_starting_with_identifier
    = expr_identifier
      >> (
        statement_composite_assign
        | statement_function_call
        | statement_primitive_assign
        );

  statement_elemental_assign
    = op_bracket_l >> expr_block >> op_bracket_r
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
    = (op_eq >> expr_block) [SCMD(M_ASGN_P)];
  statement_composite_assign
    = op_eq >> *blank_eol_p
      >> op_brace_l [SCMD(M_ASGN_CS)]
        >> statements >> *blank_eol_p >> op_brace_r [SCMD(CASGN_END)];

  statement_elemental_primitive_assign
    = (op_eq >> expr_block) [SCMD(E_ASGN_P)];
  statement_elemental_composite_assign
    = op_eq >> *blank_eol_p
      >> op_brace_l [SCMD(E_ASGN_CS)]
        >> statements >> *blank_eol_p >> op_brace_r [SCMD(CASGN_END)];

  statement_push_primitive
    = (op_eq >> expr_block) [SCMD(P_ASGN_P)];
  statement_push_composite
    = op_eq >> *blank_eol_p
      >> op_brace_l [SCMD(P_ASGN_CS)]
        >> statements >> *blank_eol_p >> op_brace_r [SCMD(CASGN_END)];

  statement_fill_all_primitive
    = (op_eq >> expr_block) [SCMD(F_ASGN_P)];
  statement_fill_all_composite
    = op_eq >> *blank_eol_p
      >> op_brace_l [SCMD(F_ASGN_CS)]
        >> statements >> *blank_eol_p >> op_brace_r [SCMD(CASGN_END)];

  statement_function_call
    = parenthesized_list_expr_any [SCMD(FUNC_CALL)] [SCMD(POP)];


  //-----------------------------------------------------------------------------------------

  parenthesized_list_expr_any  // FIXME
    = op_parenthesis_l [SCMD(LLISTS)]
      >> *blank_eol_p
        >> !(expr_block >> *(*blank_eol_p >> op_comma >> expr_block))
          >> *blank_eol_p >> op_parenthesis_r;

//   list_expr_any  // FIXME
//     = !(expr_any >> *(op_comma >> expr_any));

//   list_expr_primitive  // FIXME
//     = !(expr_primitive >> *(op_comma >> expr_primitive));

  // expr_any  // FIXME
    // = (
      // expr_add
      // | expr_value
      // );

//   expr_any
  expr_block_list
    = expr_block
      >> *(
          *blank_eol_p
            >> (op_comma >> *blank_eol_p >> expr_block)  [SCMD(LAPPEND)]
          );
  expr_block
    = expr_or;
  expr_or
    = expr_and
      >> *(
          (op_dbar >> expr_and)  [SCMD(OR)]
          );
  expr_and
    = expr_equality
      >> *(
          (op_damp >> expr_equality)  [SCMD(AND)]
          );

  expr_equality
    = expr_relational
      >> *(
          (op_deq >> expr_relational)  [SCMD(EQ)]
          | (op_exeq >> expr_relational)  [SCMD(NEQ)]
          );

  expr_relational
    = expr_additive
      >> *(
          (op_lteq >> expr_additive)  [SCMD(LTEQ)]
          | (op_gteq >> expr_additive)  [SCMD(GTEQ)]
          | (op_lt >> expr_additive)  [SCMD(LT)]
          | (op_gt >> expr_additive)  [SCMD(GT)]
          );

  expr_additive
    = expr_multiplicative
      >> *(
          (op_plus >> expr_multiplicative)  [SCMD(ADD)]
          | (op_minus >> expr_multiplicative)  [SCMD(SUBT)]
          );
  expr_multiplicative
    = expr_unary
      >> *(
          (op_star >> expr_unary)  [SCMD(MULT)]
          | (op_slash >> expr_unary)  [SCMD(DIV)]
          | (op_percent >> expr_unary)  [SCMD(MOD)]
          );

  expr_unary
    = (op_excl >> expr_unary) [SCMD(NOT)]
      | expr_postfix;

  expr_postfix
    = expr_primary
      >> *(
          (op_dot >> expr_primary)  [SCMD(MEMBER)]
          | (op_bracket_l >> *blank_eol_p
              >> expr_block_list >> *blank_eol_p >> op_bracket_r)  [SCMD(ELEM)]
          );

  expr_primary
    = (
      (op_parenthesis_l >> *blank_eol_p
          >> expr_block_list >> *blank_eol_p >> op_parenthesis_r)
      | (op_parenthesis_l >> *blank_p >> op_parenthesis_r) [SCMD(PUSH_EMPL)]
      | (
          str_p("cast") >> op_lt >> type_block >> op_gt
            >> (op_parenthesis_l >> expr_block >> op_parenthesis_r) [SCMD(CAST)]
        )
      | expr_literal
      | (
          expr_identifier
          >> !(parenthesized_list_expr_any [SCMD(FUNC_CALL)])
        )
      );

  expr_identifier
    = literal_identifier
      >> (
        (op_cat >> expr_identifier)  [SCMD(CONCAT)]
        | eps_p
        );

  expr_literal
    = (
        literal_real
        | literal_int
        | literal_boolean
        | literal_string
      ) >> *blank_p;

  type_block
    = (
        type_list
        | type_primitive
      );

  type_primitive
    = (
        str_p("int")
        | str_p("real")
        | str_p("bool")
        | str_p("str")
      )  [f_store_literal_identifier] [f_push_type];

  type_list
    = ( str_p("list") >> op_lt >> type_block >> op_gt )  [SCMD(T_TO_LIST)];

  #undef SCMD

  //-----------------------------------------------------------------------------------------

  end_of_line
    = eol_p [f_end_of_line];


  int_p_i
    = int_parser<pt_int, 10, 1, -1>();

  lex_identifier
    = ((alpha_p | '_') >> *(alnum_p | '_'))  [f_store_literal_identifier];

  lex_int
    = int_p_i  [f_store_literal_int];

  lex_real
    = (
      real_parser<pt_real, strict_real_parser_policies<pt_real> >()
      | (!(ch_p('+') | ch_p('-')) >> str_p("inf"))
      )  [f_store_literal_real];
      // = real_parser<pt_real, real_parser_policies<pt_real> >();

  lex_boolean
    = (str_p("false") | str_p("true") | lex_int)  [f_store_literal_bool];

  lex_string
    = (confix_p('"', *c_escape_ch_p, '"'))  [f_store_literal_string];


  literal_identifier
    = lex_identifier  [f_push_literal_identifier];

  literal_int
    = lex_int  [f_push_literal_int];

  literal_real
    = lex_real  [f_push_literal_real];

  literal_boolean
    = lex_boolean  [f_push_literal_bool];

  literal_string
    = lex_string  [f_push_literal_string];

}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_parser_impl_h
//-------------------------------------------------------------------------------------------
