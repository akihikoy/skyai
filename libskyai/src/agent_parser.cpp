//-------------------------------------------------------------------------------------------
/*! \file    agent_parser.cpp
    \brief   libskyai - certain program (source)
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
#include <skyai/agent_parser.h>
#include <skyai/agent_bindef.h>
#include <lora/variable_parser_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace agent_parser
{


template <typename t_iterator>
class TCodeParser : public boost::spirit::classic::grammar<TCodeParser<t_iterator> >
{
public:

  TCodeParser() : bin_stack_(NULL), file_name_(NULL), line_num_(NULL), error_(NULL), ctrl_id_stack(0) {}

  const TBinaryStack& BinStack() const {return *bin_stack_;}
  const std::string& FileName() const {return file_name_;}
  int LineNum() const {return *line_num_;}
  bool Error() const {return *error_;}

  void SetBinStack(TBinaryStack *p)  {bin_stack_= p; var_parser.SetBinStack(p);}
  void SetFileName(std::string *p)  {file_name_= p; var_parser.SetFileName(p);}
  void SetLineNum(int *p)  {line_num_= p; var_parser.SetLineNum(p);}
  void SetError(bool *p)  {error_= p; var_parser.SetError(p);}
  void SetCallbacks(TParserCallbacks callbacks)
    {
      callbacks_= callbacks;
      var_space::TParserCallbacks vcallbacks;
      vcallbacks.OnCommandPushed = callbacks.OnCommandPushed;
      vcallbacks.OnEndOfLine     = callbacks.OnEndOfLine;
      var_parser.SetCallbacks(vcallbacks);
    }

  void LoraError (message_system::TMessageType type, int linenum, const char *filename, const char *functionname, std::stringstream &ss) const
    {
      if(type==message_system::mtError)
        print_error(ss.str());
      else
        DefaultFormat(type, linenum, filename, functionname, ss);
    }

  template <typename ScannerT>
  struct definition
    {
      typedef boost::spirit::classic::rule<ScannerT> rule_t;
      rule_t  list_identifier;
      rule_t  statements, end_of_statement;
      rule_t  statements_global, statement_global;
      rule_t  statements_in_def, statement_in_def;
      rule_t  statements_in_cmp, statement_in_cmp;
      rule_t  statements_in_edit, statement_in_edit;
      rule_t  statement_composite, statement_edit, statement_def, statement_return;
      rule_t  statement_include, statement_include_once, statement_linclude;
      rule_t  statement_dump1, statement_dump2;
      rule_t  statement_destroy;
      rule_t  statement_module, statement_remove, statement_connect, statement_disconnect;
      rule_t  statement_inherit, statement_inherit_prv;
      rule_t  statement_export, statement_export_config, statement_export_memory, statement_export_port;
      rule_t  statement_assign_agent_config;
      rule_t  statement_starting_with_identifier, statement_assign_config, statement_assign_memory;
      rule_t  statement_ctrl, statement_if;
      rule_t  statement_unexpected;

      typename var_space::TCodeParser<t_iterator>::template definition<ScannerT>  var_parser_def;
      const rule_t &var_statements;

      const rule_t &lex_string;
      const rule_t &literal_identifier, &literal_int, &literal_real, &literal_boolean, &literal_string;
      const rule_t &end_of_line;
      const rule_t &expr_block;
      const rule_t &expr_identifier;
      const rule_t &statement_function_call;

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

  mutable long ctrl_id_stack;

  mutable var_space::TCodeParser<t_iterator>  var_parser;

  // actions:
  template <typename ScannerT>
  friend struct TCodeParser<t_iterator>::definition;

  void add_single_command(t_iterator first, t_iterator last, int command) const;

  #define DECL_ACTION(x_func)  void x_func (t_iterator first, t_iterator last) const
  DECL_ACTION(include_file);
  DECL_ACTION(include_file_once);
  DECL_ACTION(syntax_error);
  DECL_ACTION(ctrl_if);
  DECL_ACTION(ctrl_else);
  DECL_ACTION(ctrl_end_if);
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
void XCLASS::include_file (t_iterator first, t_iterator last) const
{
  if(callbacks_.OnInclude)
  {
    std::string file_name(var_parser.TmpString());
    std::string abs_file_name;
    if(!callbacks_.OnInclude(var_parser.TmpString(),abs_file_name))
    {
      print_error(file_name+": no such file");
      return;
    }
    if(!ParseFile(abs_file_name, *bin_stack_, callbacks_))
      *error_= true;
  }
  else
    print_error("error: callbacks_.OnInclude is not given;`include' is not available");
}
TEMPLATE_DEC
void XCLASS::include_file_once (t_iterator first, t_iterator last) const
{
  if(callbacks_.OnIncludeOnce)
  {
    std::string file_name(var_parser.TmpString());
    std::string abs_file_name;
    if(!callbacks_.OnIncludeOnce(var_parser.TmpString(),abs_file_name))
    {
      print_error(file_name+": no such file");
      return;
    }
    if(abs_file_name=="")  return;
    if(!ParseFile(abs_file_name, *bin_stack_, callbacks_))
      *error_= true;
  }
  else
    print_error("error: callbacks_.OnIncludeOnce is not given;`include_once' is not available");
}

TEMPLATE_DEC
void XCLASS::syntax_error (t_iterator first, t_iterator last) const
{
  print_error("syntax error:");
  std::cout<<"  > "<<join_iterators(first,last)<<std::endl;
}

TEMPLATE_DEC
void XCLASS::ctrl_if (t_iterator first, t_iterator last) const
{
  bin_stack_->Push(bin::cmd::CTRL_IF);
  bin_stack_->Push(ctrl_id_stack);
  ++ctrl_id_stack;
  on_command_pushed();
}
TEMPLATE_DEC
void XCLASS::ctrl_else (t_iterator first, t_iterator last) const
{
  bin_stack_->Push(bin::cmd::CTRL_ELSE);
  bin_stack_->Push(ctrl_id_stack-1);
  on_command_pushed();
}
TEMPLATE_DEC
void XCLASS::ctrl_end_if (t_iterator first, t_iterator last) const
{
  bin_stack_->Push(bin::cmd::CTRL_END_IF);
  --ctrl_id_stack;
  bin_stack_->Push(ctrl_id_stack);
  on_command_pushed();
}

#undef TEMPLATE_DEC
#undef XCLASS
//-------------------------------------------------------------------------------------------

template <typename t_iterator>
template <typename ScannerT>
TCodeParser<t_iterator>::definition<ScannerT>::definition (const TCodeParser &self)
  :
    var_parser_def     (self.var_parser            ),
    var_statements     (var_parser_def.statements  ),
    lex_string         (var_parser_def.lex_string          ),
    literal_identifier (var_parser_def.literal_identifier  ),
    literal_int        (var_parser_def.literal_int         ),
    literal_real       (var_parser_def.literal_real        ),
    literal_boolean    (var_parser_def.literal_boolean     ),
    literal_string     (var_parser_def.literal_string      ),
    end_of_line        (var_parser_def.end_of_line         ),
    expr_block         (var_parser_def.expr_block          ),
    expr_identifier    (var_parser_def.expr_identifier     ),
    statement_function_call (var_parser_def.statement_function_call )
{
  using namespace boost::spirit::classic;

  #define ALIAS_ACTION(x_action,x_as)  \
    boost::function<void(t_iterator,t_iterator)>  \
      x_as= boost::bind(&TCodeParser::x_action,&self,_1,_2)
  ALIAS_ACTION(syntax_error        , f_syntax_error           );
  ALIAS_ACTION(include_file        , f_include_file           );
  ALIAS_ACTION(include_file_once   , f_include_file_once      );
  ALIAS_ACTION(ctrl_if             , f_ctrl_if                );
  ALIAS_ACTION(ctrl_else           , f_ctrl_else              );
  ALIAS_ACTION(ctrl_end_if         , f_ctrl_end_if            );
  #undef ALIAS_ACTION

  #define SCMD(x_command)  \
    boost::bind(&TCodeParser::add_single_command,&self,_1,_2,bin::cmd::x_command)

#include <lora/bits/basic_syntax.h>

  list_identifier
    = !(expr_identifier >> *(op_comma >> expr_identifier));

  end_of_statement
    = *blank_p
      >> (
        op_semicolon
        | end_of_line
        | end_p
        | (lcomment>>end_of_line)
        );

  statements
    = statements_global;

  statements_global
    = *(
      +blank_eol_p
      | (
        (lcomment >> (end_of_line | end_p))
        | (statement_global >> end_of_statement)
        | op_semicolon
        | (statement_unexpected [f_syntax_error] >> end_of_statement)
        )
      );
  statements_in_def
    = *(
      +blank_eol_p
      | (
        (lcomment >> (end_of_line | end_p))
        | (statement_in_def >> end_of_statement)
        | op_semicolon
        | (statement_unexpected [f_syntax_error] >> end_of_statement)
        )
      );
  statements_in_cmp
    = *(
      +blank_eol_p
      | (
        (lcomment >> (end_of_line | end_p))
        | (statement_in_cmp >> end_of_statement)
        | op_semicolon
        | (statement_unexpected [f_syntax_error] >> end_of_statement)
        )
      );
  statements_in_edit
    = *(
      +blank_eol_p
      | (
        (lcomment >> (end_of_line | end_p))
        | (statement_in_edit >> end_of_statement)
        | op_semicolon
        | (statement_unexpected [f_syntax_error] >> end_of_statement)
        )
      );

  statement_global
    = (
      statement_def
      | statement_in_def
      );
  statement_in_def
    = (
      statement_composite
      | statement_linclude [SCMD(LINCLUDE)]
      | statement_assign_agent_config
      | statement_return
      | statement_in_cmp
      );
  statement_in_cmp
    = (
      statement_module [SCMD(MODULE)]
      | statement_remove [SCMD(REMOVE)]
      | statement_connect [SCMD(CONNECT)]
      | statement_disconnect [SCMD(DISCNCT)]
      | statement_inherit [SCMD(INHERIT)]
      | statement_inherit_prv [SCMD(INHERITPR)]
      | statement_export
      | statement_in_edit
      );
  statement_in_edit
    = (
      statement_edit
      | statement_include [f_include_file]
      | statement_include_once [f_include_file_once]
      | statement_dump1 [SCMD(DUMP1)]
      | statement_dump2 [SCMD(DUMP2)]
      | statement_destroy [SCMD(DESTROY)]
      | statement_ctrl
      | statement_starting_with_identifier
      );

  statement_composite
    = str_p("composite") >> +blank_eol_p
      >> expr_identifier >> *blank_eol_p
        >> op_brace_l [SCMD(COMPOSITE)]
          >> statements_in_cmp
            >> op_brace_r [SCMD(CMP_END)];

  statement_edit
    = str_p("edit") >> +blank_eol_p
      >> expr_identifier >> *blank_eol_p
        >> op_brace_l [SCMD(EDIT)]
          >> statements_in_edit
            >> op_brace_r [SCMD(EDIT_END)];

  statement_def
    = (str_p("def") >> +blank_eol_p) [SCMD(S_PARAMS)]
      >> expr_identifier >> *blank_eol_p
        >> op_parenthesis_l >> list_identifier >> op_parenthesis_r >> *blank_eol_p
          >> op_brace_l [SCMD(FUNC_DEF)]
            >> statements_in_def
              >> op_brace_r [SCMD(FDEF_END)];

  statement_return
    = str_p("return") >> +blank_p >> expr_block [SCMD(RETURN)];

  statement_include
    = str_p("include")
      >> +blank_p >> lex_string;

  statement_include_once
    = str_p("include_once")
      >> +blank_p >> lex_string;

  statement_linclude
    = str_p("linclude") >> +blank_eol_p
      >> expr_identifier >> +blank_eol_p >> literal_string;

  statement_dump1
    = str_p("dump1")
      >> +blank_eol_p >> literal_string >> +blank_eol_p >> literal_string;

  statement_dump2
    = str_p("dump2")
      >> +blank_eol_p >> literal_string >> +blank_eol_p >> literal_string >> +blank_eol_p >> literal_string;

  statement_destroy
    = str_p("_destroy")
      >> +blank_eol_p >> literal_string >> +blank_eol_p >> expr_identifier;

  statement_module
    = str_p("module")
      >> +blank_p >> expr_identifier
        >> (op_comma | +blank_p)
          >> expr_identifier;

  statement_remove
    = str_p("remove")
      >> +blank_p >> expr_identifier;

  statement_connect
    = str_p("connect") >> +blank_p
      >> expr_identifier
        >> op_dot >> expr_identifier >> *blank_eol_p
          >> op_comma >> *blank_eol_p
            >> expr_identifier
            >> op_dot >> expr_identifier ;

  statement_disconnect
    = str_p("disconnect") >> +blank_p
      >> expr_identifier
        >> op_dot >> expr_identifier >> *blank_eol_p
          >> op_comma >> *blank_eol_p
            >> expr_identifier
            >> op_dot >> expr_identifier ;

  statement_inherit
    = str_p("inherit") >> +blank_eol_p
      >> expr_identifier ;
  statement_inherit_prv
    = str_p("inherit_prv") >> +blank_eol_p
      >> expr_identifier ;

  statement_export
    = str_p("export") >> +blank_eol_p
      >> expr_identifier
        >> op_dot
          >> (statement_export_config
            | statement_export_memory
            | statement_export_port   );

  statement_export_config
    = str_p("config") >> op_dot
      >> expr_identifier >> +blank_eol_p
        >> (
          str_p("as_is") [SCMD(EXPO_C)]
          | (str_p("as") >> +blank_eol_p >> expr_identifier) [SCMD(EXPO_C_AS)]
          );

  statement_export_memory
    = str_p("memory") >> op_dot
      >> expr_identifier >> +blank_eol_p
        >> (
          str_p("as_is") [SCMD(EXPO_M)]
          | (str_p("as") >> +blank_eol_p >> expr_identifier) [SCMD(EXPO_M_AS)]
          );

  statement_export_port
    = expr_identifier >> +blank_eol_p
        >> (
          str_p("as_is") [SCMD(EXPO_P)]
          | (str_p("as") >> +blank_eol_p >> expr_identifier) [SCMD(EXPO_P_AS)]
          );

  statement_assign_agent_config
    = str_p("config") >> op_eq >> *blank_eol_p
      >> op_brace_l [SCMD(ASGN_GCNF)]
        >> var_statements >> *blank_eol_p
          >> op_brace_r [SCMD(ASGN_END)];

  statement_ctrl
    = (
      statement_if
      // |
      );

  statement_if
    = str_p("if") >> *blank_eol_p >> op_parenthesis_l
        >> expr_block >> *blank_eol_p >> op_parenthesis_r >> *blank_eol_p
          >> op_brace_l [f_ctrl_if]
            >> statements_global >> op_brace_r
              >> (!(
                *blank_eol_p
                  >> str_p("else") >> *blank_eol_p >> op_brace_l [f_ctrl_else]
                    >> statements_global >> op_brace_r
              )) [f_ctrl_end_if];

  statement_starting_with_identifier
    = expr_identifier
      >> (statement_assign_config
        | statement_assign_memory
        | statement_function_call);

  statement_assign_config
    = op_dot >> str_p("config") >> op_eq >> *blank_eol_p
      >> op_brace_l [SCMD(ASGN_CNF)]
        >> var_statements >> *blank_eol_p
          >> op_brace_r [SCMD(ASGN_END)];

  statement_assign_memory
    = op_dot >> str_p("memory") >> op_eq >> *blank_eol_p
      >> op_brace_l [SCMD(ASGN_MEM)]
        >> var_statements >> *blank_eol_p
          >> op_brace_r [SCMD(ASGN_END)];

  statement_unexpected
    = (anychar_p - eol_p - op_brace_r) >> *(anychar_p - eol_p);

}
//-------------------------------------------------------------------------------------------


bool ParseFile (const std::string &file_name, TBinaryStack &bin_stack, const TParserCallbacks &callbacks)
{
  using namespace boost::spirit::classic;
  typedef file_iterator<char> TIterator;
  TIterator  first(file_name);
  if (!first)
  {
    LERROR("failed to open file: "<<file_name);
    return false;
  }

  TIterator last= first.make_end();

  int linenum(1);
  std::string file_name2(file_name);
  bool error(false);

  TCodeParser<TIterator> parser;
  parser.SetBinStack(&bin_stack);
  parser.SetFileName(&file_name2);
  parser.SetLineNum(&linenum);
  parser.SetError(&error);
  parser.SetCallbacks(callbacks);

  LORA_MESSAGE_FORMAT_FUNCTION tmp_msg_format= message_system::GetFormat<LORA_MESSAGE_FORMAT_FUNCTION>();
  message_system::SetFormat(LORA_MESSAGE_FORMAT_FUNCTION(boost::bind(&TCodeParser<TIterator>::LoraError,&parser,_1,_2,_3,_4,_5)) );

  parse_info<TIterator> info= parse(first, last, parser);
  message_system::SetFormat(tmp_msg_format);


  LMESSAGE("loaded "<<linenum<<" lines,");
  LMESSAGE("which is "<<(info.stop==last ? "the last" : "not the last"));

  return !parser.Error();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of agent_parser
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
