//-------------------------------------------------------------------------------------------
/*! \file    mtracker-module.cpp
    \brief   skyai - certain application
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Sep.06, 2012

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
#include <skyai/skyai.h>
#include <skyai/utility.h>
#include <highgui.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

//===========================================================================================
class TTestMarkerTrackerConfigurations
//===========================================================================================
{
public:

  TInt   SleepTime;

  TTestMarkerTrackerConfigurations (var_space::TVariableMap &mmap) :
      SleepTime   (0)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( SleepTime );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Test MMarkerTracker module
class MTestMarkerTracker
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef MTestMarkerTracker   TThis;
  SKYAI_MODULE_NAMES(MTestMarkerTracker)

  MTestMarkerTracker (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      slot_start            (*this),
      signal_initialization (*this),
      signal_step           (*this)
    {
      add_slot_port (slot_start);
      add_signal_port (signal_initialization);
      add_signal_port (signal_step);
    }

  void Start(void)
    {
      slot_start.Exec();
    }

protected:

  TTestMarkerTrackerConfigurations  conf_;

  MAKE_SLOT_PORT(slot_start, void, (void), (), TThis);
  MAKE_SIGNAL_PORT(signal_initialization, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_step, void (void), TThis);

  virtual void slot_start_exec (void)
    {
      signal_initialization.ExecAll();
      while(true)
      {
        signal_step.ExecAll();
        usleep(conf_.SleepTime);
      }
    }
};  // end of MTestMarkerTracker
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MTestMarkerTracker)
//-------------------------------------------------------------------------------------------

}
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  TOptionParser option(argc,argv);
  TAgent  agent;
  if (!ParseCmdLineOption (agent, option))  return 0;

  MTestMarkerTracker *p_test_mt = dynamic_cast<MTestMarkerTracker*>(agent.SearchModule("test_mt"));
  if(p_test_mt==NULL)  {LERROR("module `test_mt' is not defined as an instance of MTestMarkerTracker"); return 1;}

  // agent.SaveToFile (agent.GetDataFileName("before.agent"),"before-");

  p_test_mt->Start();

  // agent.SaveToFile (agent.GetDataFileName("after.agent"),"after-");

  return 0;
}
//-------------------------------------------------------------------------------------------
