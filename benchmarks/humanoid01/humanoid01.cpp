//-------------------------------------------------------------------------------------------
/*! \file    humanoid01.cpp
    \brief   benchmarks - motion learning task of a simulation humanoid robot
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
#include "libhumanoid01.h"
#include <skyai/execs/general_agent.h>
#include <skyai/modules_core/learning_manager.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
}
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------

int Maze2dSkyAIMain(TOptionParser &option, TAgent &agent)
{
  option["notex"]; option["noshadow"]; option["noshadows"]; option["pause"];  // these options are used by ODE
  bool console_mode= ConvertFromStr<bool>(option("console","false"));
  string default_textures_path(agent.SearchFileName("textures"));
  string textures_path= option("texture",default_textures_path);
  int xwindow_width(ConvertFromStr<int>(option("winx","400"))), xwindow_height(ConvertFromStr<int>(option("winy","400")));

  MBasicLearningManager *p_lmanager = dynamic_cast<MBasicLearningManager*>(agent.SearchModule("lmanager"));
  MHumanoidEnvironment *p_environment = dynamic_cast<MHumanoidEnvironment*>(agent.SearchModule("environment"));
  if(p_lmanager==NULL)  {LERROR("module `lmanager' is not defined correctly"); return 1;}
  if(p_environment==NULL)  {LERROR("module `environment' is not defined correctly"); return 1;}
  MBasicLearningManager &lmanager(*p_lmanager);
  MHumanoidEnvironment &environment(*p_environment);

  agent.SaveToFile (agent.GetDataFileName("before.agent"),"before-");

  //////////////////////////////////////////////////////
  /// start learning:
  ptr_environment= &environment;
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &ODEDS_Start;
  fn.step = &ODEDS_StepLoop;
  fn.command = &ODEDS_KeyEvent;
  fn.stop = &ODEDS_Stop;
  fn.path_to_textures = textures_path.c_str();

  lmanager.Initialize();
  lmanager.StartLearning();

  environment.SetConsoleMode(console_mode);
  while(environment.Executing())
  {
    if (!environment.ConsoleMode())
      {dsSimulationLoop (option.ArgC(),const_cast<char**>(option.ArgV()),xwindow_width,xwindow_height,&fn);}
    else
      {while(environment.Executing()) environment.StepLoop();}
  }
  TerminateODE();
  //////////////////////////////////////////////////////

  /// result:

  agent.SaveToFile (agent.GetDataFileName("after.agent"),"after-");

  return 0;
}
//-------------------------------------------------------------------------------------------
SKYAI_SET_MAIN(Maze2dSkyAIMain)
//-------------------------------------------------------------------------------------------
