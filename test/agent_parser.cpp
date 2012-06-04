//-------------------------------------------------------------------------------------------
/*! \file    agent_parser.cpp
    \brief   Test program of agent parser
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.04, 2012

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
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
}
//-------------------------------------------------------------------------------------------

loco_rabbits::TBinaryStack bin_stack;

void callback(const std::string& file_name,int line_num,bool error_stat)
{
  if(error_stat)  return;
  std::cout<<"--eol@"<<file_name<<":"<<line_num<<std::endl;
  loco_rabbits::agent_parser::PrintToStream(bin_stack);
  bin_stack.Clear();
}

bool file_finder(const std::string &file_name, std::string &abs_file_name)
{
  abs_file_name= file_name;
  return true;
}

int main(int argc, char**argv)
{
  using namespace std;
  using namespace loco_rabbits;
  using namespace agent_parser;
  string filename= (argc>1)?argv[1]:"(file is not specified)";
  TParserCallbacks callbacks;
  // callbacks.OnCommandPushed= callback;
  callbacks.OnEndOfLine= callback;
  callbacks.OnInclude= file_finder;
  if (ParseFile (filename,bin_stack,callbacks))
  {
    cout<<"remaining bin_stack:"<<endl;
    PrintToStream(bin_stack);
  }
  return 0;
}
//-------------------------------------------------------------------------------------------
