//-------------------------------------------------------------------------------------------
/*! \file    agent_writer.cpp
    \brief   Test program of agent writer
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.04, 2012

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
#include <skyai/agent_binexec.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
}
//-------------------------------------------------------------------------------------------

void callback(const std::string& file_name,int line_num,bool error_stat)
{
  if(error_stat)  return;
  std::cerr<<"--eol@"<<file_name<<":"<<line_num<<std::endl;
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
  loco_rabbits::TBinaryStack bin_stack;
  string filename= (argc>1)?argv[1]:"(file is not specified)";
  TParserCallbacks callbacks;
  callbacks.OnEndOfLine= callback;
  callbacks.OnInclude= file_finder;
  if (ParseFile (filename,bin_stack,callbacks))
  {
    cerr<<"loaded:"<<endl;
    PrintToStream(bin_stack,cerr);
    cerr<<"----"<<endl;
    agent_parser::TBinWriter writer;
    writer.SetBinStack(&bin_stack);
    writer.SetOutStream(&cout);
    writer.Execute();
  }
  return 0;
}
//-------------------------------------------------------------------------------------------
