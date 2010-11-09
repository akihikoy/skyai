//-------------------------------------------------------------------------------------------
/*! \file    agent_talker.cpp
    \brief   talking with agent
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.04, 2010

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
#include <skyai/utility.h>
#include <skyai/skyai.h>
#include <lora/small_classes.h>
//-------------------------------------------------------------------------------------------
#ifdef USING_GNU_READLINE
#  include <readline/readline.h>
#  include <readline/history.h>
#  include <cstdlib>
#endif
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

void readline_str(std::string &line, const std::string &prompt=">> ")
{
#ifdef USING_GNU_READLINE
  char *s= readline(prompt.c_str());
  line=s;
  if(line!="") add_history(s);
  free(s);
#else
  std::cout<<prompt<<std::flush;
  getline(std::cin,line);
#endif
}
//-------------------------------------------------------------------------------------------

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  TOptionParser option(argc,argv);

  TAgent  agent;
  std::ofstream debug;
  std::list<std::string> included_list;
  if (!ParseCmdLineOption (agent, option, debug, &included_list,/*agent_option_required=*/false))  return 0;

  string script;
  while (true)
  {
    string line;
    readline_str(line);
    if (line!="")
    {
      if (line[line.length()-1]=='\\')
      {
        if (line.length()==1 || line[line.length()-2]!='\\')
        {
          if (script!="")  script+="\n"+line.substr(0,line.length()-1);
          else             script=line.substr(0,line.length()-1);
        }
        else
        {
          if (script!="")  script+="\n"+line;
          else             script=line;
        }
      }
      else
      {
        if (script!="")  script+="\n"+line;
        else             script=line;
      }
      if (line=="exit" || line=="quit" || line=="q")  break;
    }
    if (script!="")
    {
      agent.ExecuteScript(script, agent.Modules(), &included_list, /*file_name=*/"stdin");
      script= "";
    }
  }

  return 0;
}
//-------------------------------------------------------------------------------------------
