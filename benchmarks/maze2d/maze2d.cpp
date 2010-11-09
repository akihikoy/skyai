//-------------------------------------------------------------------------------------------
/*! \file    maze2d.cpp
    \brief   benchmarks - test libskyai on a simple toyproblem: navigation task in 2d-maze
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.23, 2009-

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
#include "libmaze2d.h"
#include <skyai/utility.h>
#include <skyai/modules_core/learning_manager.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
}
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------


int main(int argc, char**argv)
{
  TOptionParser option(argc,argv);

  TAgent  agent;
  std::ofstream debug;
  if (!ParseCmdLineOption (agent, option, debug))  return 0;

  MBasicLearningManager *p_lmanager = dynamic_cast<MBasicLearningManager*>(agent.SearchModule("lmanager"));
  MMazeEnvironment *p_environment = dynamic_cast<MMazeEnvironment*>(agent.SearchModule("environment"));
  if(p_lmanager==NULL)  {LERROR("module `lmanager' is not defined correctly"); return 1;}
  if(p_environment==NULL)  {LERROR("module `environment' is not defined correctly"); return 1;}
  MBasicLearningManager &lmanager(*p_lmanager);
  MMazeEnvironment &environment(*p_environment);


  agent.SaveToFile (agent.GetDataFileName("maze2d-before.agent"));

  {
    stringstream optss;
    if (option("help")!="")
      {cerr<<"valid options:"<<endl; option.PrintUsed(); return 0;}
    if (option.PrintNotAccessed(optss))
      {cerr<<"invalid options:"<<endl<<optss.str(); return 1;}
  }

  /// start learning:

  lmanager.Initialize();
  lmanager.StartLearning();

  while (lmanager.IsLearning())
  {
    environment.StepLoop();
  }

  /// result:

  agent.SaveToFile (agent.GetDataFileName("maze2d-after.agent"));

  return 0;
}
//-------------------------------------------------------------------------------------------
