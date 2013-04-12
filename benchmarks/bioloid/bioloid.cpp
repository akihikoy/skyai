//-------------------------------------------------------------------------------------------
/*! \file    bioloid.cpp
    \brief   benchmarks - motion learning task of a real robot (Bioloid, ROBOTIS)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Mar.08, 2010-

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
#include "libbioloid.h"
#include <skyai/execs/general_agent.h>
#include <skyai/modules_core/learning_manager.h>
#include <fstream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
}
//-------------------------------------------------------------------------------------------
#include <signal.h>
#include <lora/sys.h>
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------


static bool Executing(true);
// static bool ForceSetup(false);
int ExitCode(0);
const TAgent *PtrAgent(NULL);
MBioloidEnvironment  *PtrEnvironment(NULL);
void SaveLearningParams ();
void FinishLearningProc ();

void sig_handler(int signo)
{
  using namespace std;
  if(!Executing)
    {std::cerr<<"quit immediately!"<<std::endl;  exit(1);}

  if(signo==SIGINT)
  {
    std::cerr<<ioscc::blue<<"interrupted."<<std::endl;
    std::cerr<<
      "  Q:  quit with returning a failure code"<<std::endl<<
      "  X:  quit with returning a success code"<<std::endl<<
      "  l/L:  save the learned valuetable"<<std::endl<<
      "  r/R:  reset the simulation (including connection setup) and start a new episode"<<std::endl<<
      "  g/G:  emit a penalty signal, reset the simulation, and start a new episode"<<std::endl<<
      "  a/A:  set a success flag on the episode"<<std::endl;
    while(true)
    {
      std::cerr<<"  Q|X|l|r|g|a > "<<std::flush;
      int res= WaitKBHit();
      if(res=='Q')
      {
        Executing=false;
        ExitCode=1;
        FinishLearningProc();
        break;
      }
      else if(res=='X')
      {
        Executing=false;
        ExitCode=0;
        FinishLearningProc();
        break;
      }
      else if(res=='l'||res=='L')
      {
        SaveLearningParams();
        std::cerr<<"reset the simulation..."<<std::endl;
        PtrEnvironment->ForceFinishEpisode();
        break;
      }
      else if(res=='r'||res=='R')
      {
        std::cerr<<"r: reset the simulation..."<<std::endl;
        PtrEnvironment->Setup();
        PtrEnvironment->ForceFinishEpisode();
        break;
      }
      else if(res=='g'||res=='G')
      {
        std::cerr<<"g: emit penalty..."<<std::endl;
        PtrEnvironment->ForcePenalty();
        std::cerr<<"reset the simulation..."<<std::endl;
        PtrEnvironment->Setup();
        PtrEnvironment->ForceFinishEpisode();
        break;
      }
      else if(res=='a'||res=='A')
      {
        std::cerr<<"a: episode success..."<<std::endl;
        PtrEnvironment->SetMissionSuccess(true);
        break;
      }
      else {std::cerr<<"unknown action."<<std::endl;}
    }
  }
  else if (signo==SIGQUIT)
  {
    std::cerr<<"quit..."<<std::endl;
    Executing=false; FinishLearningProc();
  }
  else  std::cerr<<"signal code "<<signo<<" is ignored."<<std::endl;

  // com::client_state::ClearServerQueue();
}
//-------------------------------------------------------------------------------------------

void ConnectionErrorHandler(void)
{
  LERROR("connection error! force reset");
  PtrEnvironment->ForceFinishEpisode();
}
//-------------------------------------------------------------------------------------------

void SaveLearningParams ()
{
  static int params_index(0);
  std::string  suffix (IntToStr(params_index));
  std::string  filename (PtrAgent->GetDataFileName("learning"+suffix+".agent"));
  LMESSAGE("writing learned parameters to "<<filename);
  PtrAgent->SaveToFile (filename,"learning"+suffix+"-");
  params_index++;
}
//-------------------------------------------------------------------------------------------

void FinishLearningProc ()
{
  std::string  filename (PtrAgent->GetDataFileName("after.agent"));
  LMESSAGE("writing learned parameters to "<<filename);
  PtrAgent->SaveToFile (filename,"after-");
}
//-------------------------------------------------------------------------------------------


int Maze2dSkyAIMain(TOptionParser &option, TAgent &agent)
{
  signal(SIGINT,sig_handler);
  signal(SIGQUIT,sig_handler);

  // [-- setup the agent

  MManualLearningManager *p_lmanager = dynamic_cast<MManualLearningManager*>(agent.SearchModule("lmanager"));
  MBioloidEnvironment *p_environment = dynamic_cast<MBioloidEnvironment*>(agent.SearchModule("environment"));
  if(p_lmanager==NULL)  {LERROR("module `lmanager' is not defined correctly"); return 1;}
  if(p_environment==NULL)  {LERROR("module `environment' is not defined correctly"); return 1;}
  MManualLearningManager &lmanager(*p_lmanager);
  MBioloidEnvironment &environment(*p_environment);


  agent.SaveToFile (agent.GetDataFileName("before.agent"),"before-");

  // setup the agent --]

  PtrAgent= &agent;
  PtrEnvironment= &environment;

  //////////////////////////////////////////////////////
  // [-- learning

  lmanager.Initialize();

  double time(0.0), time_offset(0.0), sleep_time(0.0);
  while(Executing)
  {
    // if (ForceSetup)  environment.Setup();

    LMESSAGE("can you start learning?");
    cout<<"hit space key > "<<flush;
    WaitKBHit(' ');
    LMESSAGE("bioloid: learning...");

    lmanager.StartEpisode();

    time= GetCurrentTime();
    time_offset= time;
    while(Executing)
    {
      environment.StartTimeStep();

      sleep_time= time + environment.TimeStep() - GetCurrentTime();
      if(sleep_time>0.0)  usleep(static_cast<int>(sleep_time*1.0e+6));
      else  LDEBUG("at "<<time-time_offset<<"[s]: sleep_time="<<sleep_time);

      time= GetCurrentTime();
      environment.FinishTimeStep();
      if(!lmanager.IsInEpisode())  break;
    }
    if(!lmanager.IsLearning())  break;
  }

  // learning --]
  //////////////////////////////////////////////////////

  FinishLearningProc();

  return ExitCode;
}
//-------------------------------------------------------------------------------------------
SKYAI_SET_MAIN(Maze2dSkyAIMain)
//-------------------------------------------------------------------------------------------
