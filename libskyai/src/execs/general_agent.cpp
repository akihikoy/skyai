//-------------------------------------------------------------------------------------------
/*! \file    general_agent.cpp
    \brief   libskyai - general_agent executable (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Dec.16, 2012

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
#include <skyai/execs/general_agent.h>
#include <skyai/utility.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
using namespace loco_rabbits;
using namespace std;

int main(int argc,char **argv)
{
  TOptionParser option(argc,argv);

  TAgent  agent;
  std::ofstream debug;
  if (!ParseCmdLineOption (agent, option, debug))  return 0;

  int exit_status(0);
  if (TGAGlobalVariables::GetSkyAIMain())
    exit_status= TGAGlobalVariables::GetSkyAIMain()(option,agent);

  {
    stringstream optss;
    if (option("help")!="")
      {cerr<<"valid options:"<<endl; option.PrintUsed(); return 0;}
    if (option.PrintNotAccessed(optss))
      {cerr<<"invalid options:"<<endl<<optss.str(); return 1;}
  }

  return exit_status;
}
//-------------------------------------------------------------------------------------------

