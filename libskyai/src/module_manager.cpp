//-------------------------------------------------------------------------------------------
/*! \file    module_manager.cpp
    \brief   libskyai - implement of TModuleManager class
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
#include <skyai/module_manager.h>
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class TModuleManager
//===========================================================================================

/*static*/void TModuleManager::ShowModule (const std::string &name, const std::string &option, std::ostream &os)
{
  TModuleInterface::TShowConf  show_conf;
  show_conf.ShowInstanceName= false;
  show_conf.ShowInstanceName= false;
  show_conf.ShowPortsConnection= false;
  show_conf.ShowPortsMaxConnection= false;

  TModuleInterface::ParseShowConfOption (option, show_conf);
  TModuleInterface *p= Generator(name)("x");
  p->ShowModule (show_conf, os);
  delete p; p=NULL;
}
//-------------------------------------------------------------------------------------------

/*static*/void TModuleManager::ShowAllModules (const std::string &option, std::ostream &os)
{
  TModuleInterface::TShowConf  show_conf;
  show_conf.ShowInstanceName= false;
  show_conf.ShowInstanceName= false;
  show_conf.ShowPortsConnection= false;
  show_conf.ShowPortsMaxConnection= false;

  TModuleInterface::ParseShowConfOption (option, show_conf);
  for (TMap::const_iterator itr(instance().module_generators_.begin()); itr!=instance().module_generators_.end(); ++itr)
  {
    TModuleInterface *p= itr->second("x");
    p->ShowModule (show_conf, os);
    delete p; p=NULL;
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

