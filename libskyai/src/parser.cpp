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
#include <skyai/base.h>
#include <skyai/types.h>
//-------------------------------------------------------------------------------------------
#include <lora/variable_parser_impl.h>
#include <lora/stl_ext.h>
#include <lora/file.h>
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
class TSKYAIParseAgent : public var_space::TBasicParserAgent
//===========================================================================================
{
public:

  TSKYAIParseAgent (const boost::filesystem::path &current_dir,
                    std::list<boost::filesystem::path> &path_list, std::list<std::string> &included_list,
                    TCompositeModuleGenerator &cmp_module_generator, TFunctionManager &function_manager,
                    std::stringstream &equivalent_code)
    :
      var_space::TBasicParserAgent(),
      current_dir_    (current_dir),
      path_list_      (path_list),
      included_list_  (included_list),
      cmp_module_generator_ (cmp_module_generator),
      function_manager_     (function_manager),
      equivalent_code_(equivalent_code),
      var_cparser_    (var_pagent_)
    {
      keywords_.insert("_destroy");
      keywords_.insert("as");
      keywords_.insert("as_is");
      keywords_.insert("def");
      keywords_.insert("composite");
      keywords_.insert("edit");
      keywords_.insert("include");
      keywords_.insert("include_once");
      keywords_.insert("dump1");
      keywords_.insert("dump2");
      keywords_.insert("module");
      keywords_.insert("remove");
      keywords_.insert("connect");
      keywords_.insert("disconnect");
      keywords_.insert("inherit");
      keywords_.insert("inherit_prv");
      keywords_.insert("export");
      keywords_.insert("config");
      keywords_.insert("memory");

      var_space::AddToKeywordSet(keywords_);
    }

  boost::spirit::classic::parse_info<t_iterator>  Parse (TCompositeModule &cmodule, const std::string &file_name, t_iterator first, t_iterator last, int start_line_num, TAgentParseMode parse_mode);

  const var_space::TCodeParser<t_iterator>&  VarCParser() const {return var_cparser_;}

  bool Error() const {return error_;}
  int  LineNum() const {return line_num_;}

  void SetLiteralTable (const var_space::TLiteralTable *ltable) {literal_table_= ltable;}

  #define DECL_ACTION(x_func)  void x_func (t_iterator first, t_iterator last)
  DECL_ACTION(IncludeFile);
  DECL_ACTION(IncludeFileOnce);
  DECL_ACTION(DumpInfo1);
  DECL_ACTION(DumpInfo2);
  DECL_ACTION(DestroyDef);
  DECL_ACTION(SyntaxError);
  DECL_ACTION(PushKeyword);
  DECL_ACTION(AddModule);
  DECL_ACTION(RemoveModule);
  DECL_ACTION(Connect);
  DECL_ACTION(Disconnect);
  DECL_ACTION(AssignAgentConfigS);
  DECL_ACTION(AssignAgentConfigE);
  DECL_ACTION(AssignConfigS);
  DECL_ACTION(AssignConfigE);
  DECL_ACTION(AssignMemoryS);
  DECL_ACTION(AssignMemoryE);
  DECL_ACTION(CompositeDefS);
  DECL_ACTION(CompositeDefE);
  DECL_ACTION(EditS);
  DECL_ACTION(EditE);
  DECL_ACTION(Inherit);
  DECL_ACTION(InheritPrv);
  DECL_ACTION(ExportPort);
  DECL_ACTION(ExportConfig);
  DECL_ACTION(ExportMemory);
  DECL_ACTION(FunctionDefS);
  DECL_ACTION(FunctionDefE);
  DECL_ACTION(FunctionCall);
  #undef DECL_ACTION

private:

  enum TInternalParseMode {ipmNormal=0, ipmFunctionDef, ipmCompositeDef, ipmEdit};

  std::list<TCompositeModule*>         cmodule_stack_;
  std::list<TInternalParseMode>        ipmode_stack_;
  boost::filesystem::path              current_dir_;
  std::list<boost::filesystem::path>   &path_list_;
  std::list<std::string>               &included_list_;
  TCompositeModuleGenerator            &cmp_module_generator_;
  TFunctionManager                     &function_manager_;
  std::stringstream                    &equivalent_code_;
  var_space::TParserAgent<t_iterator>  var_pagent_;
  var_space::TCodeParser<t_iterator>   var_cparser_;

  std::list<TCompositeModule>          cmodule_entity_stack_;
  std::list<int>                       line_num_stack_;
  std::list<std::string>               func_param_stack_;
  TAgentParseMode                      parse_mode_;
  std::set<std::string>                keywords_;

  std::string                          tmp_func_name_;

  LORA_MESSAGE_FORMAT_FUNCTION tmp_lora_msg_format_;

  void set_parse_mode (const TAgentParseMode &parse_mode)
    {
      parse_mode_= parse_mode;
      expand_id_= !parse_mode_.Phantom;
    }

  override void on_push_literal_identifier (const std::string &id)
    {
      if(keywords_.find(id)!=keywords_.end())
        PRINT_ERROR(id<<" is a reserved keyword, but used as an identifier");
    }

  std::string pop_id_x (void)
    {
      std::string identifier(var_space::TBasicParserAgent::pop_id_x());
      if(identifier=="as" || identifier=="as_is" || identifier=="def" || identifier=="") return identifier;
      return identifier;
    }

  bool search_agent_file (const boost::filesystem::path &file_path, boost::filesystem::path &complete_path);
  void include_file (bool once);
  void inherit_module (bool no_export);

  bool is_allowed_in_composite(const std::string &x)
    {
      LASSERT(!ipmode_stack_.empty());
      if(ipmode_stack_.back()==ipmCompositeDef)
      {
        PRINT_ERROR (x<<" is not allowed within a composite module definition");
        return false;
      }
      return true;
    }
  bool is_allowed_in_funcdef(const std::string &x)
    {
      LASSERT(!ipmode_stack_.empty());
      if(ipmode_stack_.back()==ipmFunctionDef)
      {
        PRINT_ERROR (x<<" is not allowed within a function definition");
        return false;
      }
      return true;
    }
  bool is_allowed_in_edit(const std::string &x)
    {
      LASSERT(!ipmode_stack_.empty());
      if(ipmode_stack_.back()==ipmEdit)
      {
        PRINT_ERROR (x<<" is not allowed within an edit mode");
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
  struct definition : var_space::basic_parser_definition<t_iterator,ScannerT>
    {
      typedef var_space::basic_parser_definition<t_iterator,ScannerT> TParent;
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

      typedef boost::spirit::classic::rule<ScannerT> rule_t;
      rule_t  list_identifier;
      rule_t  block_as;
      rule_t  statements, statement, end_of_statement;
      rule_t  statement_composite, statement_edit, statement_def;
      rule_t  statement_std;
      rule_t  statement_include, statement_include_once;
      rule_t  statement_dump1, statement_dump2;
      rule_t  statement_destroy;
      rule_t  statement_module, statement_remove, statement_connect, statement_disconnect;
      rule_t  statement_inherit, statement_inherit_prv;
      rule_t  statement_export, statement_export_config, statement_export_memory, statement_export_port;
      rule_t  statement_assign_agent_config;
      rule_t  statement_starting_with_identifier, statement_assign_config, statement_assign_memory, statement_function_call;
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
boost::spirit::classic::parse_info<t_iterator>  XCLASS::Parse (TCompositeModule &cmodule, const std::string &file_name, t_iterator first, t_iterator last, int start_line_num, TAgentParseMode parse_mode)
{
  using namespace boost::spirit::classic;
  error_= false;
  in_literal_list_= false;
  line_num_= start_line_num;
  file_name_= file_name;
  set_parse_mode(parse_mode);
  TSKYAICodeParser<t_iterator> parser(*this);
  cmodule_stack_.push_back(&cmodule);
  ipmode_stack_.push_back(ipmNormal);

  tmp_lora_msg_format_= message_system::GetFormat<LORA_MESSAGE_FORMAT_FUNCTION>();
  message_system::SetFormat(LORA_MESSAGE_FORMAT_FUNCTION(boost::bind(&XCLASS::lora_error,this,_1,_2,_3,_4,_5)) );

  parse_info<t_iterator> res= parse(first, last, parser);

  message_system::SetFormat(tmp_lora_msg_format_);

  LASSERT(!cmodule_stack_.empty());
  LASSERT(!ipmode_stack_.empty());
  cmodule_stack_.pop_back();
  LASSERT1op1(ipmode_stack_.back(),==,ipmNormal);
  ipmode_stack_.pop_back();

  #define STACK_CHECK(x_stack)  do{if(!x_stack.empty()) {LERROR(#x_stack " is not empty:"); error_=true;}} while(0)
  #define STACK_CHECK_S(x_stack)  do{if(!x_stack.empty()) {LERROR(#x_stack " is not empty:"); PrintContainer(x_stack,"  " #x_stack "= "); error_=true;}} while(0)
  if(!error_)
  {
    STACK_CHECK(cmodule_stack_);
    STACK_CHECK(ipmode_stack_);
    STACK_CHECK(literal_stack_);
    STACK_CHECK(cmodule_entity_stack_);
    STACK_CHECK_S(line_num_stack_);
    STACK_CHECK_S(func_param_stack_);
  }
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
void XCLASS::include_file (bool once)
{
  var_space::TLiteral  lfilename(pop_literal_x());
  LASSERT1op1(lfilename.LType,==,var_space::TLiteral::ltPrimitive);
  LASSERT1op1(lfilename.LPrimitive.Type,==,static_cast<int>(var_space::TAnyPrimitive::ptString));
  std::string &filename(lfilename.LPrimitive.EString);
  boost::filesystem::path file_path;
  if (!search_agent_file(filename,file_path))
  {
    PRINT_ERROR(filename<<" : no such file");
    return;
  }
  filename= file_path.file_string();
  if (once && std::find(included_list_.begin(),included_list_.end(), filename)!=included_list_.end())
    return;

  TCompositeModule null_cmod("NULL","null"), *cmod_ptr(&null_cmod);
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    cmod_ptr= cmodule_stack_.back();
  }
  else
  {
    equivalent_code_<<"// start[include from "<<file_path.file_string()<<"]:"<<std::endl;
  }
  TAgentParserInfoIn  in(*cmod_ptr);
  TAgentParserInfoOut out;
  in.ParseMode          = parse_mode_;
  in.PathList           = &path_list_;
  in.IncludedList       = &included_list_;
  in.CmpModuleGenerator = &cmp_module_generator_;
  in.FunctionManager    = &function_manager_;
  out.EquivalentCode    = &equivalent_code_;

  if (!LoadAgentFromFile(file_path, in, out))
    error_= true;

  if(!out.IsLast)
  {
    error_= true;
    LERROR("unexpected end of file in "<<filename);
  }
  if(parse_mode_.Phantom)
  {
    equivalent_code_<<"// end[include from "<<file_path.file_string()<<"]"<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::IncludeFile (t_iterator first, t_iterator last)
{
  include_file(false);
}
TEMPLATE_DEC
void XCLASS::IncludeFileOnce (t_iterator first, t_iterator last)
{
  include_file(true);
}

TEMPLATE_DEC
void XCLASS::DumpInfo1 (t_iterator, t_iterator)
{
  var_space::TLiteral  lstr2(pop_literal_x()), lstr1(pop_literal_x());
  LASSERT1op1(lstr1.LType,==,var_space::TLiteral::ltPrimitive);
  LASSERT1op1(lstr1.LPrimitive.Type,==,static_cast<int>(var_space::TAnyPrimitive::ptString));
  std::string &str1(lstr1.LPrimitive.EString);
  LASSERT1op1(lstr2.LType,==,var_space::TLiteral::ltPrimitive);
  LASSERT1op1(lstr2.LPrimitive.Type,==,static_cast<int>(var_space::TAnyPrimitive::ptString));
  std::string &filename(lstr2.LPrimitive.EString);

  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    if (!DumpCModInfo(*cmodule_stack_.back(), filename, str1, NULL))
    {
      error_= true;
      LERROR("failed to dump info");
    }
  }
  else
  {
    equivalent_code_<<"dump1 "<<ConvertToStr(str1)<<" "<<ConvertToStr(filename)<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::DumpInfo2 (t_iterator, t_iterator)
{
  var_space::TLiteral  lstr3(pop_literal_x()), lstr2(pop_literal_x()), lstr1(pop_literal_x());
  LASSERT1op1(lstr1.LType,==,var_space::TLiteral::ltPrimitive);
  LASSERT1op1(lstr1.LPrimitive.Type,==,static_cast<int>(var_space::TAnyPrimitive::ptString));
  std::string &str1(lstr1.LPrimitive.EString);
  LASSERT1op1(lstr2.LType,==,var_space::TLiteral::ltPrimitive);
  LASSERT1op1(lstr2.LPrimitive.Type,==,static_cast<int>(var_space::TAnyPrimitive::ptString));
  std::string &str2(lstr2.LPrimitive.EString);
  LASSERT1op1(lstr3.LType,==,var_space::TLiteral::ltPrimitive);
  LASSERT1op1(lstr3.LPrimitive.Type,==,static_cast<int>(var_space::TAnyPrimitive::ptString));
  std::string &filename(lstr3.LPrimitive.EString);

  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    if (!DumpCModInfo(*cmodule_stack_.back(), filename, str1, &str2))
    {
      error_= true;
      LERROR("failed to dump info");
    }
  }
  else
  {
    equivalent_code_<<"dump2 "<<ConvertToStr(str1)<<" "<<ConvertToStr(str2)<<" "<<ConvertToStr(filename)<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::DestroyDef (t_iterator, t_iterator)
{
  std::string identifier(pop_id_x());

  var_space::TLiteral  lstr1(pop_literal_x());
  LASSERT1op1(lstr1.LType,==,var_space::TLiteral::ltPrimitive);
  LASSERT1op1(lstr1.LPrimitive.Type,==,static_cast<int>(var_space::TAnyPrimitive::ptString));
  std::string &str1(lstr1.LPrimitive.EString);

  if(!parse_mode_.Phantom)
  {
    if (str1=="func")
      {if (!function_manager_.RemoveFunction(identifier))  error_= true;}
    else if (str1=="cmp")
      {if (!cmp_module_generator_.RemoveGenerator(identifier))  error_= true;}
    else
      {LERROR("invalid destroy kind: "<<ConvertToStr(str1));}
  }
  else
  {
    equivalent_code_<<"_destroy "<<ConvertToStr(str1)<<" "<<identifier<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::SyntaxError (t_iterator first, t_iterator last)
{
  PRINT_ERROR("syntax error:");
  std::cout<<"  > "<<join_iterators(first,last)<<std::endl;
}

TEMPLATE_DEC
void XCLASS::PushKeyword (t_iterator first, t_iterator last)
{
  using namespace var_space;
  std::string kw(join_iterators(first,last));
  TrimBoth(kw);
  TLiteral literal;
  literal.LType= TLiteral::ltIdentifier;
  literal.LPrimitive= TAnyPrimitive(pt_string(kw));
  literal_stack_.push_back(TLiteralCell(ltValue,literal));
  if(kw=="as_is")
  {
    literal.LPrimitive= TAnyPrimitive(pt_string(""));  // dummy id
    literal_stack_.push_back(TLiteralCell(ltValue,literal));
  }
}

TEMPLATE_DEC
void XCLASS::AddModule (t_iterator, t_iterator)
{
  if(!is_allowed_in_edit("`module\'"))  return;

  std::string identifier(pop_id_x()), type(pop_id_x());
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    cmodule_stack_.back()->AddSubModule(type, identifier);
  }
  else
  {
    equivalent_code_<<"module "<<type<<" "<<identifier<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::RemoveModule (t_iterator, t_iterator)
{
  if(!is_allowed_in_edit("`remove\'"))  return;

  std::string identifier(pop_id_x());
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    cmodule_stack_.back()->RemoveSubModule(identifier);
  }
  else
  {
    equivalent_code_<<"remove "<<identifier<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::Connect (t_iterator, t_iterator)
{
  if(!is_allowed_in_edit("`connect\'"))  return;

  std::string port2(pop_id_x()), module2(pop_id_x()), port1(pop_id_x()), module1(pop_id_x());
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    cmodule_stack_.back()->SubConnect(module1, port1,  module2, port2);
  }
  else
  {
    equivalent_code_<<"connect "<<module1<<"."<<port1<<","<<module2<<"."<<port2<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::Disconnect (t_iterator, t_iterator)
{
  if(!is_allowed_in_edit("`disconnect\'"))  return;

  std::string port2(pop_id_x()), module2(pop_id_x()), port1(pop_id_x()), module1(pop_id_x());
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    cmodule_stack_.back()->SubDisconnect(module1, port1,  module2, port2);
  }
  else
  {
    equivalent_code_<<"disconnect "<<module1<<"."<<port1<<","<<module2<<"."<<port2<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::AssignAgentConfigS (t_iterator first, t_iterator last)
{
  if(!is_allowed_in_composite("global `config\'"))  return;
  var_space::TVariable void_var, *var_ptr(&void_var);
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    var_ptr= &(cmodule_stack_.back()->ParamBoxConfig());
  }
  else
  {
    equivalent_code_<<"config={"<<std::endl;
  }
  var_space::TParserInfoIn  pinfo(*var_ptr);
  pinfo.FileName= file_name_;
  pinfo.StartLineNum= line_num_;
  pinfo.LiteralTable= literal_table_;
  if(parse_mode_.Phantom) {pinfo.ParseMode= var_space::pmPhantom; pinfo.EquivalentCode= &equivalent_code_;}
  var_pagent_.StartSubParse (pinfo);
}
TEMPLATE_DEC
void XCLASS::AssignAgentConfigE (t_iterator first, t_iterator last)
{
  var_space::TParserInfoOut poutinfo;
  if (!var_pagent_.EndSubParse(&poutinfo))  error_= true;
  line_num_= poutinfo.LastLineNum;
  if(parse_mode_.Phantom)
  {
    equivalent_code_<<"}"<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::AssignConfigS (t_iterator first, t_iterator last)
{
  std::string identifier(pop_id_x());
  var_space::TVariable void_var, *var_ptr(&void_var);
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    var_ptr= &(cmodule_stack_.back()->SubModule(identifier).ParamBoxConfig());
  }
  else
  {
    equivalent_code_<<identifier<<".config={"<<std::endl;
  }
  var_space::TParserInfoIn  pinfo(*var_ptr);
  pinfo.FileName= file_name_;
  pinfo.StartLineNum= line_num_;
  pinfo.LiteralTable= literal_table_;
  if(parse_mode_.Phantom) {pinfo.ParseMode= var_space::pmPhantom; pinfo.EquivalentCode= &equivalent_code_;}
  var_pagent_.StartSubParse (pinfo);
}
TEMPLATE_DEC
void XCLASS::AssignConfigE (t_iterator first, t_iterator last)
{
  var_space::TParserInfoOut poutinfo;
  if (!var_pagent_.EndSubParse(&poutinfo))  error_= true;
  line_num_= poutinfo.LastLineNum;
  if(parse_mode_.Phantom)
  {
    equivalent_code_<<"}"<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::AssignMemoryS (t_iterator first, t_iterator last)
{
  std::string identifier(pop_id_x());
  var_space::TVariable void_var, *var_ptr(&void_var);
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    var_ptr= &(cmodule_stack_.back()->SubModule(identifier).ParamBoxMemory());
  }
  else
  {
    equivalent_code_<<identifier<<".memory={"<<std::endl;
  }
  var_space::TParserInfoIn  pinfo(*var_ptr);
  pinfo.FileName= file_name_;
  pinfo.StartLineNum= line_num_;
  pinfo.LiteralTable= literal_table_;
  if(parse_mode_.Phantom) {pinfo.ParseMode= var_space::pmPhantom; pinfo.EquivalentCode= &equivalent_code_;}
  var_pagent_.StartSubParse (pinfo);
}
TEMPLATE_DEC
void XCLASS::AssignMemoryE (t_iterator first, t_iterator last)
{
  var_space::TParserInfoOut poutinfo;
  if (!var_pagent_.EndSubParse(&poutinfo))  error_= true;
  line_num_= poutinfo.LastLineNum;
  if(parse_mode_.Phantom)
  {
    equivalent_code_<<"}"<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::CompositeDefS (t_iterator first, t_iterator last)
{
  if(!is_allowed_in_composite("`composite\'"))  return;
  if(!is_allowed_in_edit("`edit\'"))  return;

  std::string module_name(pop_id_x());
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    cmodule_entity_stack_.push_back (TCompositeModule(module_name,"temporary"));
    cmodule_entity_stack_.back().SetAgent (cmodule_stack_.back()->Agent());
    cmodule_stack_.push_back (&(cmodule_entity_stack_.back()));
    line_num_stack_.push_back(line_num_);
  }
  else
  {
    cmodule_entity_stack_.push_back(TCompositeModule("DUMMY","dummy"));
    equivalent_code_<<"composite "<<module_name<<"{"<<std::endl;
  }
  ipmode_stack_.push_back(ipmCompositeDef);
}
TEMPLATE_DEC
void XCLASS::CompositeDefE (t_iterator first, t_iterator last)
{
  LASSERT(!ipmode_stack_.empty());
  LASSERT1op1(ipmode_stack_.back(),==,ipmCompositeDef);
  ipmode_stack_.pop_back();
  if(parse_mode_.Phantom)
  {
    LASSERT(!cmodule_entity_stack_.empty());
    cmodule_entity_stack_.pop_back();
    equivalent_code_<<"}"<<std::endl;
    return;
  }
  LASSERT(!cmodule_stack_.empty());
  LASSERT(!cmodule_entity_stack_.empty());
  cmodule_stack_.pop_back();
  std::string  module_name(cmodule_entity_stack_.back().InheritedModuleName());
  if (cmp_module_generator_.GeneratorExists (module_name))
  {
    PRINT_ERROR("composite module "<<module_name<<" is already defined");
    const TCompositeModuleGenerator::TGeneratorInfo *info= cmp_module_generator_.Generator(module_name);
    PRINT_ERROR("first definition is: "<<info->FileName<<":"<<info->LineNum);
    return;
  }
  TCompositeModuleGenerator::TGeneratorInfo  generator;
  std::stringstream ss;
  cmodule_entity_stack_.back().WriteToStream(ss);
  cmodule_entity_stack_.pop_back();
  generator.Script   =  ss.str();
  generator.FileName =  file_name_;
  LASSERT(!line_num_stack_.empty());
  generator.LineNum  =  line_num_stack_.back();  line_num_stack_.pop_back();
  if (!cmp_module_generator_.AddGenerator (module_name, generator))
  {
    error_= true;
    return;
  }
}

TEMPLATE_DEC
void XCLASS::EditS (t_iterator first, t_iterator last)
{
  std::string identifier(pop_id_x());
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    TCompositeModule *cmodule= dynamic_cast<TCompositeModule*>(&cmodule_stack_.back()->SubModule(identifier));
    if (cmodule==NULL)
    {
      PRINT_ERROR("not a composite module: "<<identifier);
      return;
    }
    cmodule_stack_.push_back (cmodule);
  }
  else
  {
    equivalent_code_<<"edit "<<identifier<<"{"<<std::endl;
  }
  ipmode_stack_.push_back(ipmEdit);
}
TEMPLATE_DEC
void XCLASS::EditE (t_iterator first, t_iterator last)
{
  LASSERT(!ipmode_stack_.empty());
  LASSERT1op1(ipmode_stack_.back(),==,ipmEdit);
  ipmode_stack_.pop_back();
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    cmodule_stack_.pop_back();
  }
  else
  {
    equivalent_code_<<"}"<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::inherit_module (bool no_export)
{
  std::string  cmodule_name(pop_id_x());
  if(!parse_mode_.Phantom)
  {
    LASSERT(!ipmode_stack_.empty());
    if(ipmode_stack_.back()!=ipmCompositeDef)
    {
      PRINT_ERROR("inherit/inherit_prv is available within only a composite definition");
      return;
    }

    LASSERT(!cmodule_stack_.empty());
    if(!cmp_module_generator_.Create(*cmodule_stack_.back(), cmodule_name, cmodule_stack_.back()->InstanceName(), no_export))
    {
      PRINT_ERROR("failed to inherit "<<cmodule_name);
      return;
    }
  }
  else
  {
    if(!no_export) equivalent_code_<<"inherit "<<cmodule_name<<std::endl;
    else           equivalent_code_<<"inherit_prv "<<cmodule_name<<std::endl;
  }
}
TEMPLATE_DEC
void XCLASS::Inherit (t_iterator, t_iterator)
{
  inherit_module(false);
}
TEMPLATE_DEC
void XCLASS::InheritPrv (t_iterator, t_iterator)
{
  inherit_module(true);
}

TEMPLATE_DEC
void XCLASS::ExportPort (t_iterator, t_iterator)
{
  if(!is_allowed_in_edit("`export\'"))  return;

  std::string  export_name(pop_id_x()), as_type(pop_id_x()), port_name(pop_id_x()), module_name(pop_id_x());
  if(as_type=="as_is")  export_name= port_name;
  if(parse_mode_.NoExport)  return;
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    cmodule_stack_.back()->ExportPort (module_name, port_name, export_name);
  }
  else
  {
    equivalent_code_<<"export "<<module_name<<"."<<port_name<<" as "<<export_name<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::ExportConfig (t_iterator, t_iterator)
{
  if(!is_allowed_in_edit("`export\'"))  return;

  std::string  export_name(pop_id_x()), as_type(pop_id_x()), param_name(pop_id_x()), module_name(pop_id_x());
  if(as_type=="as_is")  export_name= param_name;
  if(parse_mode_.NoExport)  return;
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    cmodule_stack_.back()->ExportConfig (module_name, param_name, export_name);
  }
  else
  {
    equivalent_code_<<"export "<<module_name<<".config."<<param_name<<" as "<<export_name<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::ExportMemory (t_iterator, t_iterator)
{
  if(!is_allowed_in_edit("`export\'"))  return;

  std::string  export_name(pop_id_x()), as_type(pop_id_x()), param_name(pop_id_x()), module_name(pop_id_x());
  if(as_type=="as_is")  export_name= param_name;
  if(parse_mode_.NoExport)  return;
  if(!parse_mode_.Phantom)
  {
    LASSERT(!cmodule_stack_.empty());
    cmodule_stack_.back()->ExportMemory (module_name, param_name, export_name);
  }
  else
  {
    equivalent_code_<<"export "<<module_name<<".memory."<<param_name<<" as "<<export_name<<std::endl;
  }
}

TEMPLATE_DEC
void XCLASS::FunctionDefS (t_iterator first, t_iterator last)
{
  if(!is_allowed_in_composite("`def\'"))  return;
  if(!is_allowed_in_funcdef("`def\'"))  return;
  if(!is_allowed_in_edit("`def\'"))  return;
  LASSERT(func_param_stack_.empty());
  do
  {
    func_param_stack_.push_front(pop_id_x());
  } while(func_param_stack_.front()!="def");
  func_param_stack_.pop_front();  // pop "def"
  tmp_func_name_= func_param_stack_.front();
  func_param_stack_.pop_front();  // pop tmp_func_name_
  if(!parse_mode_.Phantom)
  {
    LASSERT1op1(equivalent_code_.str(),==,"");
    parse_mode_.Phantom= true;
    set_parse_mode(parse_mode_);
    ipmode_stack_.push_back(ipmFunctionDef);
    line_num_stack_.push_back(line_num_);
  }
  else
  {
    equivalent_code_<<"def "<<tmp_func_name_<<"(";
    PrintContainer(func_param_stack_.begin(),func_param_stack_.end(), equivalent_code_, ",");
    equivalent_code_<<")"<<"{"<<std::endl;
    ipmode_stack_.push_back(ipmNormal);
  }
}
TEMPLATE_DEC
void XCLASS::FunctionDefE (t_iterator first, t_iterator last)
{
  LASSERT(!ipmode_stack_.empty());
  if(ipmode_stack_.back()==ipmNormal && parse_mode_.Phantom)
  {
    func_param_stack_.clear();
    tmp_func_name_="";
    equivalent_code_<<"}"<<std::endl;
    return;
  }
  LASSERT1op1(ipmode_stack_.back(),==,ipmFunctionDef);
  ipmode_stack_.pop_back();
  parse_mode_.Phantom= false;
  set_parse_mode(parse_mode_);
  if (function_manager_.FunctionExists (tmp_func_name_))
  {
    PRINT_ERROR("function "<<tmp_func_name_<<" is already defined");
    const TFunctionManager::TFunctionInfo *info= function_manager_.Function(tmp_func_name_);
    PRINT_ERROR("first definition is: "<<info->FileName<<":"<<info->LineNum);
    return;
  }
  TFunctionManager::TFunctionInfo  function;
  function.Script= equivalent_code_.str();  equivalent_code_.str("");
  function.ParamList= func_param_stack_;  func_param_stack_.clear();
  function.FileName =  file_name_;
  LASSERT(!line_num_stack_.empty());
  function.LineNum  =  line_num_stack_.back();  line_num_stack_.pop_back();
  if (!function_manager_.AddFunction (tmp_func_name_, function))
  {
    error_= true;
  }
  tmp_func_name_="";
}

TEMPLATE_DEC
void XCLASS::FunctionCall (t_iterator first, t_iterator last)
{
  std::list<var_space::TLiteral>  argv_entity;
  var_space::TEvaluateLiteralConfig pop_config; pop_config.AllowId=true;
  pop_literal_list_x(argv_entity,pop_config);
  std::string identifier(pop_id_x());
  if(parse_mode_.Phantom)
  {
    equivalent_code_<<identifier<<"("<<var_space::TVarListFormat(argv_entity.begin(),argv_entity.end())<<")"<<std::endl;
    return;
  }

  LASSERT(!cmodule_stack_.empty());
  if (!function_manager_.ExecuteFunction(
          identifier, argv_entity, *cmodule_stack_.back(),
          current_dir_, &path_list_, &included_list_,
          &cmp_module_generator_,  parse_mode_.NoExport))
  {
    PRINT_ERROR("failed to execute the function "<<identifier);
  }
}

#undef TEMPLATE_DEC
#undef XCLASS
//-------------------------------------------------------------------------------------------
#undef PRINT_ERROR
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TSKYAICodeParser
//===========================================================================================

template <typename t_iterator>
template <typename ScannerT>
TSKYAICodeParser<t_iterator>::definition<ScannerT>::definition (const TSKYAICodeParser &self)
  :
    var_space::basic_parser_definition<t_iterator,ScannerT>(self.pagent_),
    var_cparser_def (self.pagent_.VarCParser()),
    var_parser (var_cparser_def.start())
{
  using namespace boost::spirit::classic;

  #define ALIAS_ACTION(x_action,x_as)  \
    boost::function<void(t_iterator,t_iterator)>  \
      x_as= boost::bind(&TPAgent::x_action,&self.pagent_,_1,_2)
  ALIAS_ACTION(SyntaxError         , f_syntax_error           );
  ALIAS_ACTION(PushKeyword         , f_push_keyword           );
  ALIAS_ACTION(IncludeFile         , f_include_file           );
  ALIAS_ACTION(IncludeFileOnce     , f_include_file_once      );
  ALIAS_ACTION(DumpInfo1           , f_dump1                  );
  ALIAS_ACTION(DumpInfo2           , f_dump2                  );
  ALIAS_ACTION(DestroyDef          , f_destroy_def            );
  ALIAS_ACTION(AddModule           , f_add_module             );
  ALIAS_ACTION(RemoveModule        , f_remove_module          );
  ALIAS_ACTION(Connect             , f_connect                );
  ALIAS_ACTION(Disconnect          , f_disconnect             );
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
  ALIAS_ACTION(InheritPrv          , f_inherit_prv            );
  ALIAS_ACTION(ExportPort          , f_export_port            );
  ALIAS_ACTION(ExportConfig        , f_export_config          );
  ALIAS_ACTION(ExportMemory        , f_export_memory          );
  ALIAS_ACTION(FunctionDefS        , f_function_def_s         );
  ALIAS_ACTION(FunctionDefE        , f_function_def_e         );
  ALIAS_ACTION(FunctionCall        , f_function_call          );
  #undef ALIAS_ACTION

  list_identifier
    = !(expr_identifier >> *(op_comma >> expr_identifier));

  block_as
    = ( str_p("as_is") [f_push_keyword]
      | (str_p("as") >> +blank_eol_p) [f_push_keyword]
          >> expr_identifier );

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
      | statement_def
      | statement_include [f_include_file]
      | statement_include_once [f_include_file_once]
      | statement_dump1 [f_dump1]
      | statement_dump2 [f_dump2]
      | statement_module [f_add_module]
      | statement_remove [f_remove_module]
      | statement_connect [f_connect]
      | statement_disconnect [f_disconnect]
      | statement_inherit [f_inherit]
      | statement_inherit_prv [f_inherit_prv]
      | statement_export
      | statement_assign_agent_config
      | statement_destroy [f_destroy_def]
      | statement_starting_with_identifier
      );

  statement_composite
    = str_p("composite") >> +blank_eol_p
      >> expr_identifier >> *blank_eol_p
        >> op_brace_l [f_composite_def_s]
          >> statements
            >> op_brace_r [f_composite_def_e];

  statement_edit
    = str_p("edit") >> +blank_eol_p
      >> expr_identifier >> *blank_eol_p
        >> op_brace_l [f_edit_s]
          >> statements
            >> op_brace_r [f_edit_e];

  statement_def
    = (str_p("def") >> +blank_eol_p) [f_push_keyword]
      >> expr_identifier >> *blank_eol_p
        >> op_parenthesis_l >> list_identifier >> op_parenthesis_r >> *blank_eol_p
          >> op_brace_l [f_function_def_s]
            >> statements
              >> op_brace_r [f_function_def_e];

  statement_include
    = str_p("include")
      >> +blank_p >> literal_string;

  statement_include_once
    = str_p("include_once")
      >> +blank_p >> literal_string;

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
          >> (statement_export_config [f_export_config]
            | statement_export_memory [f_export_memory]
            | statement_export_port   [f_export_port]  );

  statement_export_config
    = str_p("config") >> op_dot
      >> expr_identifier >> +blank_eol_p
        >> block_as;

  statement_export_memory
    = str_p("memory") >> op_dot
      >> expr_identifier >> +blank_eol_p
        >> block_as;

  statement_export_port
    = expr_identifier >> +blank_eol_p
      >> block_as;

  statement_assign_agent_config
    = str_p("config") >> op_eq >> *blank_eol_p
      >> op_brace_l [f_assign_agent_config_s]
        >> var_parser >> *blank_eol_p
          >> op_brace_r [f_assign_agent_config_e];

  statement_starting_with_identifier
    = expr_identifier
      >> (statement_assign_config
        | statement_assign_memory
        | statement_function_call);

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

  statement_function_call
    = parenthesized_list_expr_any [f_function_call];

  statement_unexpected
    = (anychar_p - eol_p - op_brace_r) >> *(anychar_p - eol_p);

}
//-------------------------------------------------------------------------------------------



//===========================================================================================
//! Create an instance of cmodule_name; return true for success
bool TCompositeModuleGenerator::Create(TCompositeModule &instance, const std::string &cmodule_name, const std::string &instance_name, bool no_export) const
//===========================================================================================
{
  using namespace boost::spirit::classic;
  std::map<std::string, TCompositeModuleGenerator::TGeneratorInfo>::const_iterator  igenerator(generators_.find(cmodule_name));
  if (igenerator==generators_.end())
  {
    LERROR(cmodule_name<<": module (composite) not found");
    return false;
  }
  typedef std::string::const_iterator TIterator;
  TIterator  first(igenerator->second.Script.begin());
  TIterator  last(igenerator->second.Script.end());

  std::list<boost::filesystem::path>  null_path_list;
  std::list<std::string> null_included_list;
  TCompositeModuleGenerator null_cmp_module_generator;
  TFunctionManager null_function_manager;
  std::stringstream dummy_equivalent_code;

  TSKYAIParseAgent<TIterator> pagent(boost::filesystem::path(""), null_path_list, null_included_list, null_cmp_module_generator, null_function_manager, dummy_equivalent_code);

  TAgentParseMode parse_mode;
  parse_mode.NoExport= no_export;
  parse_info<TIterator> info= pagent.Parse(instance, igenerator->second.FileName, first, last, igenerator->second.LineNum, parse_mode);
  if (info.stop!=last || pagent.Error())
  {
    LERROR("failed to instantiate "<<instance_name<<" as "<<cmodule_name);
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
bool TFunctionManager::ExecuteFunction(
        const std::string &func_name, const std::list<var_space::TLiteral> &argv,
        TCompositeModule &context_cmodule,
        const boost::filesystem::path &current_dir,
        std::list<boost::filesystem::path> *path_list, std::list<std::string> *included_list,
        TCompositeModuleGenerator *cmp_module_generator,  bool no_export) const
//===========================================================================================
{
  const TFunctionInfo *pfunction= Function(func_name);
  if(pfunction==NULL)  {return false;}

  var_space::TLiteralTable  literal_table;
  if(pfunction->ParamList.size()!=argv.size())
  {
    LERROR("the function "<<func_name<<" takes "
            <<pfunction->ParamList.size()<<" arguments, but given "<<argv.size());
    return false;
  }
  std::list<std::string>::const_iterator  param_itr(pfunction->ParamList.begin());
  for(std::list<var_space::TLiteral>::const_iterator itr(argv.begin()),last(argv.end()); itr!=last; ++itr,++param_itr)
    literal_table.AddLiteral(*param_itr, *itr);

  TAgentParserInfoIn  in(context_cmodule);
  TAgentParserInfoOut out;
  TAgentParseMode parse_mode;
  parse_mode.NoExport= no_export;
  in.ParseMode          = parse_mode;
  in.StartLineNum       = pfunction->LineNum;
  in.PathList           = path_list;
  in.IncludedList       = included_list;
  in.CmpModuleGenerator = cmp_module_generator;
  in.FunctionManager    = const_cast<TFunctionManager*>(this);  /*! this const_cast is possible since defining a function
                                                                  in a function is not allowed (thus, FunctionManager is not changed) */
  in.LiteralTable       = &literal_table;
  out.EquivalentCode    = NULL;

  return loco_rabbits::ExecuteScript(pfunction->Script, current_dir, pfunction->FileName, in, out);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
bool TAgent::ExecuteScript(
        const std::string &exec_script, TCompositeModule &context_cmodule, std::list<std::string> *included_list,
        const std::string &file_name, int start_line_num, bool no_export)
//===========================================================================================
{
  TAgentParserInfoIn  in(context_cmodule);
  TAgentParserInfoOut out;
  TAgentParseMode parse_mode;
  parse_mode.NoExport= no_export;
  in.ParseMode          = parse_mode;
  in.StartLineNum       = start_line_num;
  in.PathList           = path_list_;
  in.IncludedList       = included_list;
  in.CmpModuleGenerator = &cmp_module_generator_;
  in.FunctionManager    = &function_manager_;

  return loco_rabbits::ExecuteScript(exec_script,  boost::filesystem::current_path(), file_name, in, out);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Execute script
bool ExecuteScript(const std::string &exec_script,
      const boost::filesystem::path &current_dir, const std::string &file_name,
      TAgentParserInfoIn &in, TAgentParserInfoOut &out)
//===========================================================================================
{
  using namespace boost::spirit::classic;
  typedef std::string::const_iterator TIterator;
  TIterator  first(exec_script.begin());
  TIterator  last(exec_script.end());

  std::list<boost::filesystem::path>  null_path_list;
  if (in.PathList==NULL)  in.PathList= &null_path_list;
  std::list<std::string> null_included_list;
  if (in.IncludedList==NULL)  in.IncludedList= &null_included_list;
  TCompositeModuleGenerator null_cmp_module_generator;
  if (in.CmpModuleGenerator==NULL)  in.CmpModuleGenerator= &null_cmp_module_generator;
  TFunctionManager null_function_manager;
  if (in.FunctionManager==NULL)  in.FunctionManager= &null_function_manager;
  std::stringstream dummy_equivalent_code;
  if (out.EquivalentCode==NULL)  out.EquivalentCode= &dummy_equivalent_code;

  TSKYAIParseAgent<TIterator> pagent(current_dir, *in.PathList, *in.IncludedList, *in.CmpModuleGenerator, *in.FunctionManager, *out.EquivalentCode);
  if (in.LiteralTable)  pagent.SetLiteralTable(in.LiteralTable);

  parse_info<TIterator> info= pagent.Parse(in.CModule, file_name, first, last, in.StartLineNum, in.ParseMode);

  out.IsLast= (info.stop==last);
  out.LastLineNum= pagent.LineNum();
  return out.IsLast && !pagent.Error();
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief load modules, connections, configurations from the file [filename] (native path format)
    \param [in,out]included_list  :  included full-path (native) list
    \note  If you use include_once for multiple LoadFromFile, the same included_list should be specified */
bool TAgent::LoadFromFile (const std::string &filename, std::list<std::string> *included_list)
//===========================================================================================
{
  using namespace  boost::filesystem;
  path  file_path(complete(path(filename,native)));

  TAgentParserInfoIn  in(Modules());
  TAgentParserInfoOut out;
  in.PathList           = path_list_;
  in.IncludedList       = included_list;
  in.CmpModuleGenerator = &cmp_module_generator_;
  in.FunctionManager    = &function_manager_;

  return LoadAgentFromFile (file_path, in, out);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief load modules, connections, configurations from the file [path_list] */
bool LoadAgentFromFile (boost::filesystem::path file_path, TAgentParserInfoIn &in, TAgentParserInfoOut &out)
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
  if (in.PathList==NULL)  in.PathList= &null_path_list;
  std::list<std::string> null_included_list;
  if (in.IncludedList==NULL)  in.IncludedList= &null_included_list;
  TCompositeModuleGenerator null_cmp_module_generator;
  if (in.CmpModuleGenerator==NULL)  in.CmpModuleGenerator= &null_cmp_module_generator;
  TFunctionManager null_function_manager;
  if (in.FunctionManager==NULL)  in.FunctionManager= &null_function_manager;
  std::stringstream dummy_equivalent_code;
  if (out.EquivalentCode==NULL)  out.EquivalentCode= &dummy_equivalent_code;

  if (std::find(in.IncludedList->begin(),in.IncludedList->end(),file_path.file_string())==in.IncludedList->end())
    in.IncludedList->push_back(file_path.file_string());
  TSKYAIParseAgent<TIterator> pagent(file_path.parent_path(), *in.PathList, *in.IncludedList, *in.CmpModuleGenerator, *in.FunctionManager, *out.EquivalentCode);
  if (in.LiteralTable)  pagent.SetLiteralTable(in.LiteralTable);

  TIterator last= first.make_end();

  parse_info<TIterator> info= pagent.Parse(in.CModule, file_path.file_string(), first, last, in.StartLineNum, in.ParseMode);

  out.IsLast= (info.stop==last);
  out.LastLineNum= pagent.LineNum();

  return !pagent.Error();
}
//-------------------------------------------------------------------------------------------


//===========================================================================================


static bool save_module_to_stream (const TCompositeModule::TModuleCell &module, ostream *os, const std::string &indent)
{
  if (module.Managed)
    (*os)<<indent<< "module  "<< module.Ptr->InheritedModuleName() << "  " << module.Ptr->InstanceName() << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_connection_to_stream (const TConstPortInfo *from_port, const TConstPortInfo *to_port, ostream *os, const std::string &indent)
{
  (*os)<<indent<< "connect  "<< from_port->OuterModule->InstanceName() << "." << from_port->Name
                    << " ,   "<< to_port->OuterModule->InstanceName() << "." << to_port->Name << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_agent_config_to_stream (const TAgent *agent, ostream *os, const std::string &indent)
{
  (*os) <<indent<< "config ={" << endl;
  agent->ParamBoxConfig().WriteToStream (*os, true, indent+"    ");
  (*os) <<indent<< "  }" << endl;
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

static bool save_cmp_params_to_stream (const TCompositeModule::TModuleCell &module, ostream *os, const std::string &indent,
              bool save_conf, bool save_mem);

static bool save_cmp_params_to_stream_base (const TModuleInterface *module, ostream *os, const std::string &indent,
              bool save_conf, bool save_mem)
{
  if(const TCompositeModule *cmodule= dynamic_cast<const TCompositeModule*>(module))
  {
    (*os) <<indent<< "edit  " << module->InstanceName() << endl;
    (*os) <<indent<< "{" << endl;
    cmodule->ForEachSubModuleCell (boost::bind(save_cmp_params_to_stream,_1,os,indent+"  ",save_conf,save_mem));
    (*os) <<indent<< "}" << endl;
  }
  else
  {
    if(save_conf)  save_config_to_stream(module,os,indent);
    if(save_mem)   save_memory_to_stream(module,os,indent);
  }
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_cmp_params_to_stream (const TCompositeModule::TModuleCell &module, ostream *os, const std::string &indent,
              bool save_conf, bool save_mem)
{
  if(!module.Managed)  return true;

  return save_cmp_params_to_stream_base(module.Ptr, os, indent, save_conf, save_mem);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief save modules, connections, configurations to a stream */
bool TCompositeModule::WriteToStream (std::ostream &os, const std::string &indent) const
//===========================================================================================
{
  ForEachSubModuleCell (boost::bind(save_module_to_stream,_1,&os,indent));

  os<<endl;
  ForEachSubConnection (boost::bind(save_connection_to_stream,_1,_2,&os,indent));

  if (!export_list_.empty())
  {
    os<<endl;
    for (std::list<TExportItem>::const_iterator itr(export_list_.begin()),last(export_list_.end()); itr!=last; ++itr)
    {
      os<<indent<< "export  "<<itr->ModuleName;
      switch (itr->Kind)
      {
      case ekPort   : break;
      case ekConfig : os<<".config"; break;
      case ekMemory : os<<".memory"; break;
      default : LERROR("fatal!"); lexit(df);
      }
      os<<"."<<itr->ElemName<<"  as  "<<itr->ExportName<<endl;
    }
  }

  os<<endl;
  ForEachSubModuleCell (boost::bind(save_cmp_params_to_stream,_1,&os,indent,true,true));
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_cmodule_generator (const std::string &id, const TCompositeModuleGenerator::TGeneratorInfo &generator,
              std::ostream &os, const std::string &indent)
{
  os<<indent<<"composite  "<<id<<endl;
  os<<indent<<"{"<<endl;
  os<<TIndentString(generator.Script, indent+"  ");
  os<<indent<<"}"<<endl;
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
      LERROR("fatal! composite module generator is lost: "<<igenerator->first);
      lexit(df);
    }
    save_cmodule_generator(*itr,igenerator->second,os,indent);
  }
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_function (const std::string &id, const TFunctionManager::TFunctionInfo &finfo,
              std::ostream &os, const std::string &indent)
{
  os<<indent<<"def  "<<id<<"("<<ContainerToStr(finfo.ParamList.begin(),finfo.ParamList.end(),", ")<<")"<<endl;
  os<<indent<<"{"<<endl;
  os<<TIndentString(finfo.Script, indent+"  ");
  os<<indent<<"}"<<endl;
  return true;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! Write all function definitions to a stream
bool TFunctionManager::WriteToStream (std::ostream &os, const std::string &indent) const
//===========================================================================================
{
  for (std::map<std::string, TFunctionInfo>::const_iterator itr(functions_.begin()),last(functions_.end()); itr!=last; ++itr)
  {
    save_function (itr->first,itr->second,os,indent);
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
/*!\brief save modules, connections, configurations to the file [path_list] */
bool SaveAgentToFile (const TAgent &agent, const boost::filesystem::path &file_path)
//===========================================================================================
{
  using namespace boost::filesystem;

  if (!exists(file_path.parent_path()))
  {
    cerr<<"Cannot save data into the file: "<<file_path.file_string();
    cerr<<" because the parent path: "<<file_path.parent_path().file_string();
    cerr<<" does not exist."<<endl;
    return false;
  }
  if (!CanOpenFile(file_path.file_string(),fopAsk))  return false;

  ofstream  ofs(file_path.file_string().c_str());
  return  WriteAgentToStream(agent, ofs);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief save modules, connections, configurations to the stream [os] */
bool WriteAgentToStream (const TAgent &agent, ostream &os)
//===========================================================================================
{
  save_agent_config_to_stream (&agent, &os, "");
  os<<endl;
  agent.CompositeModuleGenerator().WriteToStream(os);
  os<<endl;
  agent.FunctionManager().WriteToStream(os);
  os<<endl;
  return  agent.Modules().WriteToStream(os);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================


//===========================================================================================
/*!\brief dump information */
bool DumpCModInfo (const TCompositeModule &cmodule, const std::string &filename,
      const std::string &kind, const std::string *opt, const std::string &indent)
//===========================================================================================
{
  ostream  *p_os(NULL);
  ofstream  ofs;
  if (filename!="")
  {
    std::string file_path= cmodule.Agent().GetDataFileName(filename);
    if (!CanOpenFile(file_path,fopAsk))  return false;
    ofs.open(file_path.c_str());
    p_os= &ofs;
  }
  else
  {
    p_os= &std::cout;
  }

  if(kind=="agent")
  {
    return WriteAgentToStream (cmodule.Agent(), *p_os);
  }
  else if(kind=="config")
  {
    *p_os <<indent<< "config ={" << endl;
    cmodule.ParamBoxConfig().WriteToStream (*p_os, true, indent+"    ");
    *p_os <<indent<< "  }" << endl;
  }
  else if(kind=="mod_list")
  {
    cmodule.ForEachSubModuleCell (boost::bind(save_module_to_stream,_1,p_os,indent));
  }
  else if(kind=="conn_list")
  {
    cmodule.ForEachSubConnection (boost::bind(save_connection_to_stream,_1,_2,p_os,indent));
  }
  else if(kind=="all_mod")
  {
    cmodule.ForEachSubModuleCell (boost::bind(save_cmp_params_to_stream,_1,p_os,indent,true,true));
  }
  else if(kind=="all_mod_conf")
  {
    cmodule.ForEachSubModuleCell (boost::bind(save_cmp_params_to_stream,_1,p_os,indent,true,false));
  }
  else if(kind=="all_mod_mem")
  {
    cmodule.ForEachSubModuleCell (boost::bind(save_cmp_params_to_stream,_1,p_os,indent,false,true));
  }
  else if(kind=="all_cmp")
  {
    cmodule.Agent().CompositeModuleGenerator().WriteToStream(*p_os);
  }
  else if(kind=="all_func")
  {
    cmodule.Agent().FunctionManager().WriteToStream(*p_os);
  }
  else if(kind=="mod" || kind=="mod_conf" || kind=="mod_mem")
  {
    if (opt==NULL)  {LERROR("use dump2"); return false;}
    if (*opt=="")   {LERROR("invalid second option"); return false;}
    const TModuleInterface *module(cmodule.SubModulePtr(*opt));
    if (module==NULL)  {LERROR(*opt<<": module not found"); *p_os<<*opt<<": module not found"<<endl; return false;}
    if (kind=="mod")            save_cmp_params_to_stream_base (module, p_os, "",true,true);
    else if (kind=="mod_conf")  save_cmp_params_to_stream_base (module, p_os, "",true,false);
    else if (kind=="mod_mem")   save_cmp_params_to_stream_base (module, p_os, "",false,true);
  }
  else if(kind=="cmp")
  {
    if (opt==NULL)  {LERROR("use dump2"); return false;}
    if (*opt=="")   {LERROR("invalid second option"); return false;}
    const TCompositeModuleGenerator::TGeneratorInfo *pgenerator= cmodule.Agent().CompositeModuleGenerator().Generator(*opt);
    if (pgenerator)  save_cmodule_generator(*opt,*pgenerator,*p_os,indent);
    else  return false;
  }
  else if(kind=="func")
  {
    if (opt==NULL)  {LERROR("use dump2"); return false;}
    if (*opt=="")   {LERROR("invalid second option"); return false;}
    const TFunctionManager::TFunctionInfo *pfunction= cmodule.Agent().FunctionManager().Function(*opt);
    if (pfunction)  save_function(*opt,*pfunction,*p_os,indent);
    else  return false;
  }
  else
  {
    LERROR("invalid dump kind: "<<ConvertToStr(kind));
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

