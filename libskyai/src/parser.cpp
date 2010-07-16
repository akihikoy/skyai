//-------------------------------------------------------------------------------------------
/*! \file    parser.cpp
    \brief   libskyai - agent file (script) parser (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.17, 2009-

    Copyright (C) 2009, 2010  Akihiko Yamaguchi

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
#include <skyai/parser.h>
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
#include <lora/variable_parser_impl.h>
#include <lora/stl_ext.h>
#include <string>
#include <sstream>
#include <fstream>
#include <list>
#include <boost/spirit/include/classic.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


inline boost::filesystem::path  operator+ (const boost::filesystem::path &file_path, const std::string &rhs)
{
  return file_path.parent_path()/(file_path.filename()+rhs);
}

//===========================================================================================
template <typename t_iterator>
class TSKYAIParseAgent
//===========================================================================================
{
public:

  TSKYAIParseAgent (const boost::filesystem::path &current_dir,
                    std::list<boost::filesystem::path> &path_list, std::list<std::string> &included_list,
                    TCompositeModuleGenerator &cmp_module_generator)
    :
      current_dir_    (current_dir),
      path_list_      (path_list),
      included_list_  (included_list),
      cmp_module_generator_ (cmp_module_generator),
      var_cparser_    (var_pagent_),
      error_          (false)
    {}

  boost::spirit::classic::parse_info<t_iterator>  Parse (TCompositeModule &cmodule, const std::string &file_name, t_iterator first, t_iterator last, int start_line_num=1);

  const var_space::TCodeParser<t_iterator>&  VarCParser() const {return var_cparser_;}

  bool Error() const {return error_;}

  void IncludeFile (t_iterator first, t_iterator last);
  void IncludeFileOnce (t_iterator first, t_iterator last);
  void EndOfLine (t_iterator first, t_iterator last);
  void SyntaxError (t_iterator first, t_iterator last);
  void PushIdentifier (t_iterator first, t_iterator last);
  void PushLiteralString (t_iterator first, t_iterator last);
  void AddModule (t_iterator, t_iterator);
  void Connect (t_iterator, t_iterator);
  void AssignAgentConfigS (t_iterator first, t_iterator last);
  void AssignAgentConfigE (t_iterator first, t_iterator last);
  void AssignConfigS (t_iterator first, t_iterator last);
  void AssignConfigE (t_iterator first, t_iterator last);
  void AssignMemoryS (t_iterator first, t_iterator last);
  void AssignMemoryE (t_iterator first, t_iterator last);
  void CompositeDefS (t_iterator first, t_iterator last);
  void CompositeDefE (t_iterator first, t_iterator last);
  void EditS (t_iterator first, t_iterator last);
  void EditE (t_iterator first, t_iterator last);
  void Inherit (t_iterator, t_iterator);
  void ExportPortAs (t_iterator, t_iterator);
  void ExportPortAsIs (t_iterator, t_iterator);
  void ExportConfigAs (t_iterator, t_iterator);
  void ExportConfigAsIs (t_iterator, t_iterator);
  void ExportMemoryAs (t_iterator, t_iterator);
  void ExportMemoryAsIs (t_iterator, t_iterator);

private:
  std::list<TCompositeModule*>         cmodule_stack_;
  boost::filesystem::path              current_dir_;
  std::list<boost::filesystem::path>   &path_list_;
  std::list<std::string>               &included_list_;
  TCompositeModuleGenerator            &cmp_module_generator_;
  var_space::TParserAgent<t_iterator>  var_pagent_;
  var_space::TCodeParser<t_iterator>   var_cparser_;

  std::list<std::string>               id_stack_;
  std::list<std::string>               str_stack_;
  std::list<TCompositeModule>          cmodule_entity_stack_;
  // std::list<t_iterator>                t_itr_stack_;
  bool  error_;
  int  line_num_;
  std::string  file_name_;

  int  tmp_line_num_;

  std::string pop_id() {std::string res(id_stack_.back()); id_stack_.pop_back(); return res;}
  std::string pop_str() {std::string res(str_stack_.back()); str_stack_.pop_back(); return res;}

  static std::string join_iterators (t_iterator first, t_iterator last)
    {
      std::stringstream ss;
      for(;first!=last;++first)
        ss<<*first;
      return ss.str();
    }

  bool search_agent_file (const boost::filesystem::path &file_path, boost::filesystem::path &complete_path);
  void include_file (t_iterator first, t_iterator last, bool once);

  bool is_allowed_in_composite(const std::string &x)
    {
      if(cmodule_entity_stack_.size()>0)
      {
        std::cout<<"("<<file_name_<<":"<<line_num_<<") "<<x<<" is not allowed within a composite module definition"<<std::endl;
        error_= true;
        return false;
      }
      return true;
    }

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
template <typename t_iterator>
class TSKYAICodeParser : public boost::spirit::classic::grammar<TSKYAICodeParser<t_iterator> >
//===========================================================================================
{
public:
  typedef TSKYAIParseAgent<t_iterator> TPAgent;

  TSKYAICodeParser (TPAgent &a) : pagent_(a)
    {
    }

  template <typename ScannerT>
  struct definition
    {
      typedef boost::spirit::classic::rule<ScannerT> rule_t;
      rule_t  identifier_p;
      rule_t  literal_string;
      rule_t  lcomment;
      rule_t  end_of_line, blank_eol_p;
      rule_t  op_semicolon, op_comma, op_dot, op_eq, op_at;
      rule_t  op_brace_l, op_brace_r, op_parenthesis_l, op_parenthesis_r;
      rule_t  op_bracket_l, op_bracket_r;
      rule_t  statements, statement, end_of_statement;
      rule_t  statement_composite, statement_edit;
      rule_t  statement_std;
      rule_t  statement_include, statement_include_once;
      rule_t  statement_module, statement_connect;
      rule_t  statement_inherit;
      rule_t  statement_export, statement_export_config, statement_export_memory, statement_export_port;
      rule_t  statement_export_config_as, statement_export_memory_as, statement_export_port_as;
      rule_t  statement_export_config_as_is, statement_export_memory_as_is, statement_export_port_as_is;
      rule_t  statement_assign_agent_config;
      rule_t  statement_assign, statement_assign_config, statement_assign_memory;
      rule_t  statement_unexpected;

      typename var_space::TCodeParser<t_iterator>::template definition<ScannerT>  var_cparser_def;
      const rule_t &var_parser;

      definition (const TSKYAICodeParser &self);
      const rule_t& start() const {return statements;}
    };

private:
  TPAgent &pagent_;

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TSKYAIParseAgent
//===========================================================================================

#define TEMPLATE_DEC  template <typename t_iterator>
#define XCLASS        TSKYAIParseAgent <t_iterator>

TEMPLATE_DEC
boost::spirit::classic::parse_info<t_iterator>  XCLASS::Parse (TCompositeModule &cmodule, const std::string &file_name, t_iterator first, t_iterator last, int start_line_num)
{
  using namespace boost::spirit::classic;
  error_= false;
  line_num_= start_line_num;
  file_name_= file_name;
  TSKYAICodeParser<t_iterator> parser(*this);
  cmodule_stack_.push_back(&cmodule);
  parse_info<t_iterator> res= parse(first, last, parser);

  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.pop_back();

  #define STACK_CHECK(x_stack)  do{if(!x_stack.empty()) {LERROR(#x_stack " is not empty."); error_=true;}} while(0)
  #define STACK_CHECK_S(x_stack)  do{if(!x_stack.empty()) {LERROR(#x_stack " is not empty."); PrintContainer(x_stack,"  " #x_stack "= "); error_=true;}} while(0)
  STACK_CHECK(cmodule_stack_);
  STACK_CHECK_S(id_stack_);
  STACK_CHECK_S(str_stack_);
  STACK_CHECK(cmodule_entity_stack_);
  // STACK_CHECK(t_itr_stack_);
  #undef STACK_CHECK
  #undef STACK_CHECK_S

  return res;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
bool  XCLASS::search_agent_file (const boost::filesystem::path &file_path, boost::filesystem::path &complete_path)
{
  using namespace boost::filesystem;
  if (file_path.is_complete())
  {
    if (exists((complete_path= file_path)))  return true;
    if (exists((complete_path= file_path.parent_path()/(file_path.filename()+".agent"))))  return true;
    return false;
  }

  if (exists((complete_path= current_dir_/file_path)))  return true;
  if (exists((complete_path= current_dir_/file_path+".agent")))  return true;

  path  tmp_path(file_path);
  for (std::list<path>::const_iterator ditr(path_list_.begin()), ditr_last(path_list_.end()); ditr!=ditr_last; ++ditr)
    if (exists((complete_path= *ditr/tmp_path)))  return true;
  tmp_path= file_path+".agent";
  for (std::list<path>::const_iterator ditr(path_list_.begin()), ditr_last(path_list_.end()); ditr!=ditr_last; ++ditr)
    if (exists((complete_path= *ditr/tmp_path)))  return true;
  return false;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
void XCLASS::include_file (t_iterator first, t_iterator last, bool once)
{
  LASSERT(!str_stack_.empty());
  std::string  filename(pop_str());
  boost::filesystem::path file_path;
  if (!search_agent_file(filename,file_path))
  {
    error_= true;
    std::cout<<"("<<file_name_<<":"<<line_num_<<") error: "<<filename<<" : no such file"<<std::endl;
    return;
  }
  filename= file_path.file_string();
  if (once && std::find(included_list_.begin(),included_list_.end(), filename)!=included_list_.end())
    return;
  // LDEBUG("including file: "<<filename);
  bool is_last;
  LASSERT(!cmodule_stack_.empty());
  if (!LoadAgentFromFile(*cmodule_stack_.back(), file_path, &is_last, &path_list_, &included_list_, &cmp_module_generator_))
    error_= true;

  if(!is_last)
  {
    error_= true;
    LERROR("unexpected end of file in "<<filename);
  }
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
void XCLASS::IncludeFile (t_iterator first, t_iterator last)
{
  if(!is_allowed_in_composite("`include\'"))  return;
  include_file(first,last,false);
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
void XCLASS::IncludeFileOnce (t_iterator first, t_iterator last)
{
  if(!is_allowed_in_composite("`include_once\'"))  return;
  include_file(first,last,true);
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
void XCLASS::EndOfLine (t_iterator first, t_iterator last)
{
  ++line_num_;
}

TEMPLATE_DEC
void XCLASS::SyntaxError (t_iterator first, t_iterator last)
{
  error_= true;
  std::cout<<"("<<file_name_<<":"<<line_num_<<") syntax error:"<<std::endl<<"  > ";
  for(;first!=last;++first)
    std::cout<<*first;
  std::cout<<std::endl;
}

TEMPLATE_DEC
void XCLASS::PushIdentifier (t_iterator first, t_iterator last)
{
  id_stack_.push_back(join_iterators(first,last));
}

TEMPLATE_DEC
void XCLASS::PushLiteralString (t_iterator first, t_iterator last)
{
  str_stack_.push_back (DecodeString(join_iterators(first,last)));
}

TEMPLATE_DEC
void XCLASS::AddModule (t_iterator, t_iterator)
{
  LASSERT1op1(id_stack_.size(),>=,2);
  std::string identifier(pop_id()), type(pop_id());
  // std::cout<<"add module: "<<type<<": "<<identifier<<std::endl;
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->AddSubModule(type, identifier);
}

TEMPLATE_DEC
void XCLASS::Connect (t_iterator, t_iterator)
{
  LASSERT1op1(id_stack_.size(),>=,4);
  std::string port2(pop_id()), module2(pop_id()), port1(pop_id()), module1(pop_id());
  // std::cout<<"connect cmodule: "
    // <<module1<<"."<<port1<<" ---> "<<module2<<"."<<port2<<std::endl;
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->SubConnect(module1, port1,  module2, port2);
}

TEMPLATE_DEC
void XCLASS::AssignAgentConfigS (t_iterator first, t_iterator last)
{
  if(!is_allowed_in_composite("global `config\'"))  return;
  LASSERT(!cmodule_stack_.empty());
  var_pagent_.StartSubParse (cmodule_stack_.back()->ParamBoxConfig(), line_num_, file_name_);
}
TEMPLATE_DEC
void XCLASS::AssignAgentConfigE (t_iterator first, t_iterator last)
{
  if (!var_pagent_.EndSubParse(&line_num_))  error_= true;
}

TEMPLATE_DEC
void XCLASS::AssignConfigS (t_iterator first, t_iterator last)
{
  LASSERT(!id_stack_.empty());
  std::string identifier(pop_id());
  // std::cout<<"assign config to "<<identifier<<std::endl; sf.PrintToStream(std::cout,"  ");
  LASSERT(!cmodule_stack_.empty());
  var_pagent_.StartSubParse (cmodule_stack_.back()->SubModule(identifier).ParamBoxConfig(), line_num_, file_name_);
}
TEMPLATE_DEC
void XCLASS::AssignConfigE (t_iterator first, t_iterator last)
{
  if (!var_pagent_.EndSubParse(&line_num_))  error_= true;
}

TEMPLATE_DEC
void XCLASS::AssignMemoryS (t_iterator first, t_iterator last)
{
  LASSERT(!id_stack_.empty());
  std::string identifier(pop_id());
  LASSERT(!cmodule_stack_.empty());
  var_pagent_.StartSubParse (cmodule_stack_.back()->SubModule(identifier).ParamBoxMemory(), line_num_, file_name_);
}
TEMPLATE_DEC
void XCLASS::AssignMemoryE (t_iterator first, t_iterator last)
{
  if (!var_pagent_.EndSubParse(&line_num_))  error_= true;
}

TEMPLATE_DEC
void XCLASS::CompositeDefS (t_iterator first, t_iterator last)
{
  if(!is_allowed_in_composite("`composite\'"))  return;
  LASSERT(!id_stack_.empty());
  std::string module_name(pop_id());
  cmodule_entity_stack_.push_back (TCompositeModule(module_name,"temporary"));
  cmodule_entity_stack_.back().SetAgent (cmodule_stack_.back()->Agent());
  cmodule_stack_.push_back (&(cmodule_entity_stack_.back()));
  // t_itr_stack_.push_back (last);
  tmp_line_num_= line_num_;
}
TEMPLATE_DEC
void XCLASS::CompositeDefE (t_iterator first, t_iterator last)
{
  LASSERT(!cmodule_stack_.empty());
  LASSERT(!cmodule_entity_stack_.empty());
  cmodule_stack_.pop_back();
  std::string  module_name(cmodule_entity_stack_.back().InheritedModuleName());
  if (cmp_module_generator_.GeneratorExists (module_name))
  {
    error_= true;
    std::cout<<"("<<file_name_<<":"<<line_num_<<") composite module "<<module_name<<" is already defined"<<std::endl;
    return;
  }
  // LASSERT(!t_itr_stack_.empty());
  TCompositeModuleGenerator::TGeneratorInfo  generator;
  // generator.Script   =  join_iterators(t_itr_stack_.back(),first);
  std::stringstream ss;
  cmodule_entity_stack_.back().WriteToStream(ss);
  cmodule_entity_stack_.pop_back();
  generator.Script   =  ss.str();
  generator.FileName =  file_name_;
  generator.LineNum  =  tmp_line_num_;
  // t_itr_stack_.pop_back();
  if (!cmp_module_generator_.AddGenerator (module_name, generator))
  {
    error_= true;
    return;
  }
}

TEMPLATE_DEC
void XCLASS::EditS (t_iterator first, t_iterator last)
{
  LASSERT(!id_stack_.empty());
  std::string identifier(pop_id());
  LASSERT(!cmodule_stack_.empty());
  TCompositeModule *cmodule= dynamic_cast<TCompositeModule*>(&cmodule_stack_.back()->SubModule(identifier));
  if (cmodule==NULL)
  {
    error_= true;
    std::cout<<"("<<file_name_<<":"<<line_num_<<") not a composite module: "<<identifier<<std::endl;
    return;
  }
  cmodule_stack_.push_back (cmodule);
}
TEMPLATE_DEC
void XCLASS::EditE (t_iterator first, t_iterator last)
{
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.pop_back();
}

TEMPLATE_DEC
void XCLASS::Inherit (t_iterator, t_iterator)
{
  LASSERT(!id_stack_.empty());
  std::string  cmodule_name(pop_id());
  LASSERT(!cmodule_stack_.empty());
  if(!cmp_module_generator_.Create(*cmodule_stack_.back(), cmodule_name, cmodule_stack_.back()->InstanceName()))
  {
    error_= true;
    std::cout<<"("<<file_name_<<":"<<line_num_<<") failed to inherit "<<cmodule_name<<std::endl;
    return;
  }
}

TEMPLATE_DEC
void XCLASS::ExportPortAs (t_iterator, t_iterator)
{
  LASSERT1op1(id_stack_.size(),>=,3);
  std::string  export_name(pop_id()), port_name(pop_id()), module_name(pop_id());
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->ExportPort (module_name, port_name, export_name);
}
TEMPLATE_DEC
void XCLASS::ExportPortAsIs (t_iterator, t_iterator)
{
  LASSERT1op1(id_stack_.size(),>=,2);
  std::string  port_name(pop_id()), module_name(pop_id());
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->ExportPort (module_name, port_name, port_name);
}

TEMPLATE_DEC
void XCLASS::ExportConfigAs (t_iterator, t_iterator)
{
  LASSERT1op1(id_stack_.size(),>=,3);
  std::string  export_name(pop_id()), param_name(pop_id()), module_name(pop_id());
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->ExportConfig (module_name, param_name, export_name);
}
TEMPLATE_DEC
void XCLASS::ExportConfigAsIs (t_iterator, t_iterator)
{
  LASSERT1op1(id_stack_.size(),>=,2);
  std::string  param_name(pop_id()), module_name(pop_id());
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->ExportConfig (module_name, param_name, param_name);
}

TEMPLATE_DEC
void XCLASS::ExportMemoryAs (t_iterator, t_iterator)
{
  LASSERT1op1(id_stack_.size(),>=,3);
  std::string  export_name(pop_id()), param_name(pop_id()), module_name(pop_id());
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->ExportMemory (module_name, param_name, export_name);
}
TEMPLATE_DEC
void XCLASS::ExportMemoryAsIs (t_iterator, t_iterator)
{
  LASSERT1op1(id_stack_.size(),>=,2);
  std::string  param_name(pop_id()), module_name(pop_id());
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->ExportMemory (module_name, param_name, param_name);
}

#undef TEMPLATE_DEC
#undef XCLASS
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TSKYAICodeParser
//===========================================================================================

template <typename t_iterator>
template <typename ScannerT>
TSKYAICodeParser<t_iterator>::definition<ScannerT>::definition (const TSKYAICodeParser &self)
  :
    var_cparser_def (self.pagent_.VarCParser()),
    var_parser (var_cparser_def.start())
{
  using namespace boost::spirit::classic;

  #define ALIAS_ACTION(x_action,x_as)  \
    boost::function<void(t_iterator,t_iterator)>  \
      x_as= boost::bind(&TPAgent::x_action,&self.pagent_,_1,_2)
  ALIAS_ACTION(EndOfLine           , f_end_of_line            );
  ALIAS_ACTION(SyntaxError         , f_syntax_error           );
  ALIAS_ACTION(PushIdentifier      , f_push_identifier        );
  ALIAS_ACTION(PushLiteralString   , f_push_literal_string    );
  ALIAS_ACTION(IncludeFile         , f_include_file           );
  ALIAS_ACTION(IncludeFileOnce     , f_include_file_once      );
  ALIAS_ACTION(AddModule           , f_add_module             );
  ALIAS_ACTION(Connect             , f_connect                );
  ALIAS_ACTION(AssignAgentConfigS  , f_assign_agent_config_s  );
  ALIAS_ACTION(AssignAgentConfigE  , f_assign_agent_config_e  );
  ALIAS_ACTION(AssignConfigS       , f_assign_config_s        );
  ALIAS_ACTION(AssignConfigE       , f_assign_config_e        );
  ALIAS_ACTION(AssignMemoryS       , f_assign_memory_s        );
  ALIAS_ACTION(AssignMemoryE       , f_assign_memory_e        );
  ALIAS_ACTION(CompositeDefS       , f_composite_def_s        );
  ALIAS_ACTION(CompositeDefE       , f_composite_def_e        );
  ALIAS_ACTION(EditS               , f_edit_s                 );
  ALIAS_ACTION(EditE               , f_edit_e                 );
  ALIAS_ACTION(Inherit             , f_inherit                );
  ALIAS_ACTION(ExportPortAs        , f_export_port_as         );
  ALIAS_ACTION(ExportPortAsIs      , f_export_port_as_is      );
  ALIAS_ACTION(ExportConfigAs      , f_export_config_as       );
  ALIAS_ACTION(ExportConfigAsIs    , f_export_config_as_is    );
  ALIAS_ACTION(ExportMemoryAs      , f_export_memory_as       );
  ALIAS_ACTION(ExportMemoryAsIs    , f_export_memory_as_is    );
  #undef ALIAS_ACTION

  identifier_p
    = ((alpha_p | '_') >> *(alnum_p | '_'));

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
    = *(+blank_eol_p | statement);
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

  statement_std
    = (
      statement_composite
      | statement_edit
      | statement_include [f_include_file]
      | statement_include_once [f_include_file_once]
      | statement_module [f_add_module]
      | statement_connect [f_connect]
      | statement_inherit [f_inherit]
      | statement_export
      | statement_assign_agent_config
      | statement_assign
      );

  statement_composite
    = str_p("composite") >> *blank_p
      >> identifier_p [f_push_identifier] >> *blank_eol_p
        >> op_brace_l [f_composite_def_s]
          >> statements
            >> op_brace_r [f_composite_def_e];

  statement_edit
    = str_p("edit") >> *blank_p
      >> identifier_p [f_push_identifier] >> *blank_eol_p
        >> op_brace_l [f_edit_s]
          >> statements
            >> op_brace_r [f_edit_e];

  statement_include
    = str_p("include")
      >> *blank_p >> literal_string [f_push_literal_string];

  statement_include_once
    = str_p("include_once")
      >> *blank_p >> literal_string [f_push_literal_string];

  statement_module
    = str_p("module")
      >> *blank_p >> identifier_p [f_push_identifier]
        >> (op_comma | *blank_p)
          >> identifier_p [f_push_identifier];

  statement_connect
    = str_p("connect")
      >> *blank_p >> identifier_p [f_push_identifier]
        >> op_dot >> identifier_p [f_push_identifier] >> *blank_eol_p
          >> op_comma >> identifier_p [f_push_identifier]
            >> op_dot >> identifier_p [f_push_identifier] ;

  statement_inherit
    = str_p("inherit") >> *blank_eol_p
      >> identifier_p [f_push_identifier] ;

  statement_export
    = str_p("export") >> *blank_eol_p
      >> identifier_p [f_push_identifier]
        >> op_dot
          >> (statement_export_config
            | statement_export_memory
            | statement_export_port );

  statement_export_config
    = str_p("config") >> op_dot
      >> identifier_p [f_push_identifier] >> *blank_eol_p
        >> (statement_export_config_as_is [f_export_config_as_is]
          | statement_export_config_as    [f_export_config_as] );
  statement_export_config_as_is
    = str_p("as_is");
  statement_export_config_as
    = str_p("as") >> *blank_eol_p
      >> identifier_p [f_push_identifier];

  statement_export_memory
    = str_p("memory") >> op_dot
      >> identifier_p [f_push_identifier] >> *blank_eol_p
        >> (statement_export_memory_as_is [f_export_memory_as_is]
          | statement_export_memory_as    [f_export_memory_as] );
  statement_export_memory_as_is
    = str_p("as_is");
  statement_export_memory_as
    = str_p("as") >> *blank_eol_p
      >> identifier_p [f_push_identifier];

  statement_export_port
    = identifier_p [f_push_identifier] >> *blank_eol_p
      >> (statement_export_port_as_is [f_export_port_as_is]
        | statement_export_port_as    [f_export_port_as] );
  statement_export_port_as_is
    = str_p("as_is");
  statement_export_port_as
    = str_p("as") >> *blank_eol_p
      >> identifier_p [f_push_identifier];

  statement_assign_agent_config
    = str_p("config") >> op_eq >> *blank_eol_p
      >> op_brace_l [f_assign_agent_config_s]
        >> var_parser >> *blank_eol_p
          >> op_brace_r [f_assign_agent_config_e];

  statement_assign
    = identifier_p [f_push_identifier]
      >> (statement_assign_config
        | statement_assign_memory);

  statement_assign_config
    = op_dot >> str_p("config") >> op_eq >> *blank_eol_p
      >> op_brace_l [f_assign_config_s]
        >> var_parser >> *blank_eol_p
          >> op_brace_r [f_assign_config_e];

  statement_assign_memory
    = op_dot >> str_p("memory") >> op_eq >> *blank_eol_p
      >> op_brace_l [f_assign_memory_s]
        >> var_parser >> *blank_eol_p
          >> op_brace_r [f_assign_memory_e];

  statement_unexpected
    = (anychar_p - eol_p - op_brace_r) >> *(anychar_p - eol_p);

}
//-------------------------------------------------------------------------------------------



//===========================================================================================
//! Create an instance of cmodule_name; return true for success
bool TCompositeModuleGenerator::Create(TCompositeModule &instance, const std::string &cmodule_name, const std::string &instance_name) const
//===========================================================================================
{
  using namespace boost::spirit::classic;
  std::map<std::string, TCompositeModuleGenerator::TGeneratorInfo>::const_iterator  igenerator(generators_.find(cmodule_name));
  if (igenerator==generators_.end())
  {
    LERROR("error in TCompositeModuleGenerator: does not have a generator for "<<cmodule_name);
    return false;
  }
  typedef std::string::const_iterator TIterator;
  TIterator  first(igenerator->second.Script.begin());
  TIterator  last(igenerator->second.Script.end());

  std::list<boost::filesystem::path>  null_path_list;
  std::list<std::string> null_included_list;
  TCompositeModuleGenerator null_cmp_module_generator;

  TSKYAIParseAgent<TIterator> pagent(boost::filesystem::path(""), null_path_list, null_included_list, null_cmp_module_generator);

  parse_info<TIterator> info= pagent.Parse(instance, igenerator->second.FileName, first, last, igenerator->second.LineNum);
  if (info.stop!=last || pagent.Error())
  {
    LERROR("failed to instantiate "<<instance_name<<" as "<<cmodule_name);
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief load modules, connections, configurations from the file [filename] (native path format)
    \param [in,out]included_list  :  included full-path (native) list
    \note  If you use include_once for multiple LoadFromFile, the same included_list should be specified */
bool TAgent::LoadFromFile (const std::string &filename, bool *is_last, std::list<std::string> *included_list)
//===========================================================================================
{
  using namespace  boost::filesystem;
  path  file_path(complete(path(filename,native)));

  return LoadAgentFromFile (Modules(), file_path, is_last, path_list_, included_list, &cmp_module_generator_);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief load modules, connections, configurations from the file [filename]
    \param [in]path_list : path-list from which an agent file is searched
    \param [in,out]included_list  :  included full-path (native) list
    \note  If you use include_once for multiple LoadAgentFromFile, the same included_list should be specified */
bool LoadAgentFromFile (TCompositeModule &cmodule, boost::filesystem::path file_path, bool *is_last,
                        std::list<boost::filesystem::path> *path_list, std::list<std::string> *included_list,
                        TCompositeModuleGenerator *cmp_module_generator)
//===========================================================================================
{
  using namespace boost::spirit::classic;
  typedef file_iterator<char> TIterator;
  file_path= boost::filesystem::complete(file_path);
  TIterator  first(file_path.file_string());
  if (!first)
  {
    LERROR("failed to open file: "<<file_path.file_string());
    return false;
  }

  std::list<boost::filesystem::path>  null_path_list;
  if (path_list==NULL)  path_list= &null_path_list;
  std::list<std::string> null_included_list;
  if (included_list==NULL)  included_list= &null_included_list;
  TCompositeModuleGenerator null_cmp_module_generator;
  if (cmp_module_generator==NULL)  cmp_module_generator= &null_cmp_module_generator;

  included_list->push_back(file_path.file_string());
  TSKYAIParseAgent<TIterator> pagent(file_path.parent_path(), *path_list, *included_list, *cmp_module_generator);

  TIterator last= first.make_end();

  parse_info<TIterator> info= pagent.Parse(cmodule, file_path.file_string(), first, last);
  if (is_last)  *is_last= (info.stop==last);
  return !pagent.Error();
}
//-------------------------------------------------------------------------------------------


//===========================================================================================


static bool save_module_to_stream (const TModuleInterface *module, ostream *os, const std::string &indent)
{
  (*os)<<indent<< "module  "<< module->InheritedModuleName() << "  " << module->InstanceName() << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

void find_module_port_ids(const TPortInterface &port, const std::vector<const TModuleInterface*> &modules, std::string &port_name, std::string &module_name)
{
  if (std::find(modules.begin(),modules.end(), &port.OuterBase())!=modules.end())
  {
    port_name= port.Name();
    module_name= port.OuterBase().InstanceName();
    return;
  }

  const TCompositeModule *parent= port.OuterBase().ParentCModule();
  while(true)
  {
    if (std::find(modules.begin(),modules.end(), parent)!=modules.end())
    {
      port_name= parent->SearchPortByPtr(&port);
      module_name= parent->InstanceName();
      if(port_name=="")  {LERROR("fatal!"); lexit(df);}
      return;
    }
    parent= parent->ParentCModule();
  }
}
//-------------------------------------------------------------------------------------------

static bool save_connection_to_stream (
    const TPortInterface* from_port_ptr, const TPortInterface* to_port_ptr,
    ostream *os, const std::string &indent, const std::vector<const TModuleInterface*> &modules)
{
  std::string  from_port_name, from_module_name, to_port_name, to_module_name;
  find_module_port_ids(*from_port_ptr, modules, from_port_name, from_module_name);
  find_module_port_ids(*to_port_ptr, modules, to_port_name, to_module_name);

  (*os)<<indent<< "connect  "<< from_module_name << "." << from_port_name
                    << " ,   "<< to_module_name << "." << to_port_name << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_config_to_stream (const TModuleInterface *module, ostream *os, const std::string &indent)
{
  if(module->ParamBoxConfig().NoMember())  return true;
  (*os) <<indent<< module->InstanceName() << ".config ={" << endl;
  module->ParamBoxConfig().WriteToStream (*os, true, indent+"    ");
  (*os) <<indent<< "  }" << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_memory_to_stream (const TModuleInterface *module, ostream *os, const std::string &indent)
{
  if(module->ParamBoxMemory().NoMember())  return true;
  (*os) <<indent<< module->InstanceName() << ".memory ={" << endl;
  module->ParamBoxMemory().WriteToStream (*os, true, indent+"    ");
  (*os) <<indent<< "  }" << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_params_to_stream (const TModuleInterface *module, ostream *os, const std::string &indent)
{
  if(const TCompositeModule *cmodule= dynamic_cast<const TCompositeModule*>(module))
  {
    (*os) <<indent<< "edit  " << module->InstanceName() << endl;
    (*os) <<indent<< "{" << endl;
    cmodule->ForEachSubModule (boost::bind(save_params_to_stream,_1,os,indent+"  "));
    (*os) <<indent<< "}" << endl;
  }
  else
  {
    save_config_to_stream(module,os,indent);
    save_memory_to_stream(module,os,indent);
  }
  return true;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief save modules, connections, configurations to a stream */
bool TCompositeModule::WriteToStream (std::ostream &os, const std::string &indent) const
//===========================================================================================
{
  std::vector<const TModuleInterface*>  modules(sub_modules_.size(),NULL);
  TCompositeModule::TModuleSet::const_iterator  sub_modules_itr(sub_modules_.begin());
  for (std::vector<const TModuleInterface*>::iterator itr(modules.begin()),last(modules.end()); itr!=last; ++itr,++sub_modules_itr)
    *itr= sub_modules_itr->second;

  ForEachSubModule (boost::bind(save_module_to_stream,_1,&os,indent));
  os<<endl;
  ForEachSubConnection (boost::bind(save_connection_to_stream,_1,_2,&os,indent,modules));

  if (!export_list_.empty())
  {
    os<<endl;
    for (std::list<std::pair<std::string,std::string> >::const_iterator itr(export_list_.begin()),last(export_list_.end()); itr!=last; ++itr)
      os<<indent<< "export  "<<itr->first<<"  as  "<<itr->second<<endl;
  }

  os<<endl;
  ForEachSubModule (boost::bind(save_params_to_stream,_1,&os,indent));
  return true;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! Write all composite module definitions to a stream
bool TCompositeModuleGenerator::WriteToStream (std::ostream &os, const std::string &indent) const
//===========================================================================================
{
  std::map<std::string, TCompositeModuleGenerator::TGeneratorInfo>::const_iterator  igenerator;
  for (std::list<std::string>::const_iterator itr(cmodule_name_list_.begin()),last(cmodule_name_list_.end()); itr!=last; ++itr)
  {
    igenerator= generators_.find(*itr);
    if (igenerator==generators_.end())
    {
      LERROR("fatal!");
      lexit(df);
    }
    os<<indent<<"composite  "<<*itr<<endl;
    os<<indent<<"{"<<endl;
    os<<TIndentString(igenerator->second.Script, indent+"  ");
    os<<indent<<"}"<<endl;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief save modules, connections, configurations to the file [filename] (native path format) */
bool TAgent::SaveToFile (const std::string &filename) const
//===========================================================================================
{
  boost::filesystem::path  file_path (filename, boost::filesystem::native);
  return SaveAgentToFile (*this, file_path);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief save modules, connections, configurations to the file [filename] */
bool SaveAgentToFile (const TAgent &agent, const boost::filesystem::path &file_path)
//===========================================================================================
{
  using namespace boost::filesystem;

  if (!exists(file_path.parent_path()))
  {
    cerr<<"Cannot save data into the file: "<<file_path.file_string();
    cerr<<" because the parent path: "<<file_path.parent_path().file_string();
    cerr<<" does not exists."<<endl;
    return false;
  }
  if (exists(file_path))
  {
    cerr<<file_path.file_string()<<" already exists. Will you overwrite?"<<endl;
    cerr<<"  answer Yes    : overwrite"<<endl;
    cerr<<"  answer No     : not overwrite (continue)"<<endl;
    cerr<<"  answer Cancel : stop running"<<endl;
    switch(AskYesNoCancel())
    {
      case ryncYes    :  break;
      case ryncNo     :  return false;
      case ryncCancel :  lexit(qfail); break;
      default : lexit(abort);
    }
  }

  ofstream  ofs (file_path.file_string().c_str());
  agent.CompositeModuleGenerator().WriteToStream(ofs);
  ofs<<endl;
  return  agent.Modules().WriteToStream(ofs);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

