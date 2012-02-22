//-------------------------------------------------------------------------------------------
/*! \file    agent_binexec_test.cpp
    \brief   Test program of agent binary executor
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.14, 2012

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
#include <lora/small_classes.h>
#include <fstream>
#include <boost/filesystem/operations.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
}
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace loco_rabbits;

int main(int argc,char**argv)
{
  TOptionParser option(argc,argv);

  TAgent  agent;
  std::ofstream debug;
  std::list<std::string> included_list;
  if (!ParseCmdLineOption(agent, option, debug, &included_list,/*agent_option_required=*/true))  return 0;

  // agent.SaveToFile (agent.GetDataFileName("after.agent"),"after-");
  agent.SaveToFile (boost::filesystem::complete("after.agent").file_string(),"after-");

  return 0;
}
//-------------------------------------------------------------------------------------------
