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
#include <lora/variable_parser_impl.h>
//-------------------------------------------------------------------------------------------
#include <skyai/parser.h>
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
#include <lora/stl_ext.h>
#include <string>
#include <sstream>
#include <fstream>
#include <list>
#include <boost/spirit/include/classic.hpp>
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;

using namespace boost::spirit::classic;

//===========================================================================================

template <typename t_iterator>
class TSKYAIParseAgent
{
public:

  TSKYAIParseAgent (TAgent &a) : agent_(a), var_cparser_(var_pagent_), error_(false) {}

  parse_info<t_iterator>  Parse (t_iterator first, t_iterator last);

  const var_space::TCodeParser<t_iterator>&  VarCParser() const {return var_cparser_;}

  bool Error() const {return error_;}

  void EndOfLine (t_iterator first, t_iterator last)
    {
      ++line_num_;
    }

  void Push (t_iterator first, t_iterator last)
    {
      std::stringstream ss;
      for(;first!=last;++first)
        ss<<*first;
      imemory_.push_back(ss.str());
    }

  void AddModule (t_iterator, t_iterator)
    {
      std::string identifier(pop()), type(pop());
      // std::cout<<"add module: "<<type<<": "<<identifier<<std::endl;
      agent_.AddModule(type, identifier);
    }

  void Connect (t_iterator, t_iterator)
    {
      std::string port2(pop()), module2(pop()), port1(pop()), module1(pop());
      // std::cout<<"connect modules: "
        // <<module1<<"."<<port1<<" ---> "<<module2<<"."<<port2<<std::endl;
      agent_.Connect(module1, port1,  module2, port2);
    }

  void AssignConfigS (t_iterator first, t_iterator last)
    {
      std::string identifier(pop());
      // std::cout<<"assign config to "<<identifier<<std::endl; sf.PrintToStream(std::cout,"  ");
      var_pagent_.StartSubParse (agent_.Module(identifier).ParamBoxConfig(), line_num_);
    }
  void AssignConfigE (t_iterator first, t_iterator last)
    {
      if (!var_pagent_.EndSubParse(&line_num_))  error_= true;
    }

  void AssignMemoryS (t_iterator first, t_iterator last)
    {
      std::string identifier(pop());
      var_pagent_.StartSubParse (agent_.Module(identifier).ParamBoxMemory(), line_num_);
    }
  void AssignMemoryE (t_iterator first, t_iterator last)
    {
      if (!var_pagent_.EndSubParse(&line_num_))  error_= true;
    }

  void SyntaxError (t_iterator first, t_iterator last)
    {
      error_= true;
      std::cout<<"(l."<<line_num_<<") Syntax error:"<<std::endl<<"  > ";
      for(;first!=last;++first)
        std::cout<<*first;
      std::cout<<std::endl;
    }

private:
  TAgent &agent_;
  var_space::TParserAgent<t_iterator>  var_pagent_;
  var_space::TCodeParser<t_iterator>   var_cparser_;

  std::list<std::string>  imemory_;
  bool  error_;
  int  line_num_;

  std::string pop() {std::string res(imemory_.back()); imemory_.pop_back(); return res;}

};
//-------------------------------------------------------------------------------------------


template <typename t_iterator>
class TSKYAICodeParser : public grammar<TSKYAICodeParser<t_iterator> >
{
public:
  typedef TSKYAIParseAgent<t_iterator> TPAgent;

  TSKYAICodeParser (TPAgent &a) : pagent_(a)
    {
    }

  template <typename ScannerT>
  struct definition
    {
      typedef rule<ScannerT> rule_t;
      rule_t  identifier;
      rule_t  int_literal, real_literal, boolean_literal, string_literal;
      rule_t  lcomment;
      rule_t  end_of_line, blank_eol_p;
      rule_t  op_semicolon, op_comma, op_dot, op_eq, op_at;
      rule_t  op_brace_l, op_brace_r, op_parenthesis_l, op_parenthesis_r;
      rule_t  op_bracket_l, op_bracket_r;
      rule_t  statements, statement;
      rule_t  module_statement, connect_statement;
      rule_t  assign_statement, assign_config_statement, assign_memory_statement;
      rule_t  unexpected_statement;

      typename var_space::TCodeParser<t_iterator>::template definition<ScannerT>  var_cparser_def;
      const rule_t &var_parser;

      definition (const TSKYAICodeParser &self)
          : var_cparser_def (self.pagent_.VarCParser()),
            var_parser (var_cparser_def.start())
        {

          identifier
            = ((alpha_p | '_') >> *(alnum_p | '_'));

          int_literal
            = int_p
              >> *blank_p ;

          real_literal
            = real_parser<TReal>()
              >> *blank_p ;

          boolean_literal
            = (str_p("false") | str_p("true") | int_p)
              >> *blank_p ;

          string_literal
            = confix_p('"', *c_escape_ch_p, '"')
              >> *blank_p ;

          // literal
            // = confix_p('\'', *c_escape_ch_p, '\'')
              // >> *blank_p ;

          lcomment
            = str_p("//")>>*(anychar_p - eol_p)
              >> *blank_p ;

          end_of_line
            = eol_p [boost::bind(&TPAgent::EndOfLine,&self.pagent_,_1,_2)];
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
            = *((*blank_p >> end_of_line)
                | statement) ;

          statement
            = *blank_p
              >> (module_statement [boost::bind(&TPAgent::AddModule,&self.pagent_,_1,_2)]
                | connect_statement [boost::bind(&TPAgent::Connect,&self.pagent_,_1,_2)]
                | assign_statement
                | lcomment
                | unexpected_statement [boost::bind(&TPAgent::SyntaxError,&self.pagent_,_1,_2)] )
              >> *blank_p >> ((ch_p(';') >> *blank_p) | end_of_line | end_p | (lcomment>>end_of_line)); ;

          module_statement
            = str_p("module")
              >> *blank_p >> identifier [boost::bind(&TPAgent::Push,&self.pagent_,_1,_2)]
              >> op_comma >> identifier [boost::bind(&TPAgent::Push,&self.pagent_,_1,_2)];

          connect_statement
            = str_p("connect")
              >> *blank_p >> identifier [boost::bind(&TPAgent::Push,&self.pagent_,_1,_2)]
                  >> op_dot >> identifier [boost::bind(&TPAgent::Push,&self.pagent_,_1,_2)]
              >> op_comma >> identifier [boost::bind(&TPAgent::Push,&self.pagent_,_1,_2)]
                  >> op_dot >> identifier [boost::bind(&TPAgent::Push,&self.pagent_,_1,_2)] ;

          assign_statement
            = identifier [boost::bind(&TPAgent::Push,&self.pagent_,_1,_2)]
              >> (assign_config_statement
                | assign_memory_statement);

          assign_config_statement
            = op_dot >> str_p("config") >> op_eq >> *blank_eol_p
              >> op_brace_l [boost::bind(&TPAgent::AssignConfigS,&self.pagent_,_1,_2)]
                >> var_parser >> *blank_eol_p
                  >> op_brace_r [boost::bind(&TPAgent::AssignConfigE,&self.pagent_,_1,_2)];

          assign_memory_statement
            = op_dot >> str_p("memory") >> op_eq >> *blank_eol_p
              >> op_brace_l [boost::bind(&TPAgent::AssignMemoryS,&self.pagent_,_1,_2)]
                >> var_parser >> *blank_eol_p
                  >> op_brace_r [boost::bind(&TPAgent::AssignMemoryE,&self.pagent_,_1,_2)];

          unexpected_statement
            = +(anychar_p - eol_p);

        }

        const rule_t& start() const {return statements;}
    };

private:
  TPAgent &pagent_;

};
//-------------------------------------------------------------------------------------------

template <typename t_iterator>
parse_info<t_iterator>  TSKYAIParseAgent<t_iterator>::Parse (t_iterator first, t_iterator last)
{
  error_= false;
  line_num_= 1;
  TSKYAICodeParser<t_iterator> parser(*this);
  parse_info<t_iterator> res= parse(first, last, parser);

  // #define STACK_CHECH(x_stack)  do{if(!x_stack.empty()) {LERROR(#x_stack " is not empty."); error_=true;}} while(0)
  // STACK_CHECH(imemory_);
  // #undef STACK_CHECH
  if (!imemory_.empty())
  {
    LERROR("imemory_ is not empty.");
    PrintContainer(imemory_,"  imemory_= ");
    error_=true;
  }

  return res;
}
//-------------------------------------------------------------------------------------------


/*!\brief load modules, connections, configurations from filename */
bool LoadAgentFromFile (TAgent &agent, const std::string &filename, bool *is_last)
{
  typedef file_iterator<char> TIterator;
  TIterator  first(filename);
  if (!first)
  {
    LERROR("failed to open file: "<<filename);
    return false;
  }

  TSKYAIParseAgent<TIterator> pagent(agent);

  TIterator last= first.make_end();

  parse_info<TIterator> info= pagent.Parse(first, last);
  if (is_last)  *is_last= (info.stop==last);
  return !pagent.Error();
}
//-------------------------------------------------------------------------------------------


//===========================================================================================


static bool save_module_to_string_list (const TModuleInterface *module, ostream *os)
{
  (*os)<< "module  "<< module->InheritedModuleName() << ",  " << module->InstanceName() << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_connection_to_string_list (const TPortInterface* from_port_ptr, const TPortInterface* to_port_ptr, ostream *os)
{
  (*os)<< "connect  "<< from_port_ptr->OuterBase().InstanceName() << "." << from_port_ptr->Name()
           << " ,   "<< to_port_ptr->OuterBase().InstanceName() << "." << to_port_ptr->Name() << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_config_to_string_list (const TModuleInterface *module, ostream *os)
{
  (*os) << module->InstanceName() << ".config ={" << endl;
  module->ParamBoxConfig().WriteToStream (*os, true, "    ");
  (*os) << "  }" << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_memory_to_string_list (const TModuleInterface *module, ostream *os)
{
  (*os) << module->InstanceName() << ".memory ={" << endl;
  module->ParamBoxMemory().WriteToStream (*os, true, "    ");
  (*os) << "  }" << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

/*!\brief save modules, connections, configurations to filename */
bool SaveAgentToFile (const TAgent &agent, const std::string &filename)
{
  ofstream  ofs (filename.c_str());
  agent.ForEachModule (boost::bind(save_module_to_string_list,_1,&ofs));
  ofs<<endl;
  agent.ForEachConnection (boost::bind(save_connection_to_string_list,_1,_2,&ofs));
  ofs<<endl;
  agent.ForEachModule (boost::bind(save_config_to_string_list,_1,&ofs));
  ofs<<endl;
  agent.ForEachModule (boost::bind(save_memory_to_string_list,_1,&ofs));
  ofs.close();
  return false;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

