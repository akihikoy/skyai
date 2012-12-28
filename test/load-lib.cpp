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
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

//===========================================================================================
//!\brief TestX module
class MTestX
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface  TParent;
  typedef MTestX            TThis;
  SKYAI_MODULE_NAMES(MTestX)

  MTestX (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_start     (*this),
      signal_start   (*this)
    {
      add_slot_port (slot_start);
      add_signal_port (signal_start);
    }

  void Start(void)
    {
      slot_start.Exec();
    }

protected:

  MAKE_SLOT_PORT(slot_start, void, (void), (), TThis);
  MAKE_SIGNAL_PORT(signal_start, void (void), TThis);

  virtual void slot_start_exec (void)
    {
      signal_start.ExecAll();
    }
};  // end of MTestX
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MTestX)
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

  MTestX *p_test = dynamic_cast<MTestX*>(agent.SearchModule("test"));
  if(p_test==NULL)  {LERROR("module `test' is not defined as an instance of MTestX"); return 1;}

  // agent.SaveToFile (agent.GetDataFileName("before.agent"),"before-");

  p_test->Start();

  // agent.SaveToFile (agent.GetDataFileName("after.agent"),"after-");

  return 0;
}
//-------------------------------------------------------------------------------------------
