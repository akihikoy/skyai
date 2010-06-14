//-------------------------------------------------------------------------------------------
/*! \file    module_manager.h
    \brief   libskyai - define the TModuleManager class that allocates an instance of
              a subclass of TModuleInterface from subclass' name.
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.28, 2009-

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

    -----------------------------------------------------------------------------------------

    usage:
    \code
      // add the class TModuleA into the TModuleManager:
      SKYAI_ADD_MODULE(TModuleA)

      // allocate an instance of TModuleA from the string "TModuleA"
      //   with the constructor option "module_A"
      TModuleInterface *p= TModuleManager::Generator("TModuleA")("module_A");
    \endcode
*/
//-------------------------------------------------------------------------------------------
#ifndef skyai_module_manager_h
#define skyai_module_manager_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <string>
#include <map>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

class TModuleInterface;

#define SKYAI_ADD_MODULE(x_type_module)    \
  namespace skyai_detail                   \
  {                                       \
    volatile const bool xx_registered_##x_type_module =                                               \
        ::loco_rabbits::TModuleManager::SetModule(x_type_module::ModuleName(),                        \
                                    &::loco_rabbits::skyai_detail::InstantiateModule<x_type_module>);  \
  }
//-------------------------------------------------------------------------------------------

namespace skyai_detail
{

template <typename t_module>
TModuleInterface* InstantiateModule (const std::string &v_instance_name)
{
  return new t_module(v_instance_name);
}

}  // end of namespace skyai_detail
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief TModuleManager that allocates an instance of a subclass of TModuleInterface from subclass' name.
    TModuleManager is designed as a singleton */
class TModuleManager
//===========================================================================================
{
public:

  typedef TModuleInterface* (*TGenerator)(const std::string &v_instance_name);

  /*!\brief Register a module by identifier, 'name'
      \todo Output error if a module is already registered with the same name.
          However, in order to allow multiple-registration of template modules (from some units),
          implement a allow-multiple-registration mode. */
  static bool SetModule (const std::string &name, TGenerator generator)
    {
      instance().module_generators_[name]= generator;
      return true;
    }

  static TGenerator Generator (const std::string &name)
    {
      TMap::const_iterator itr= instance().module_generators_.find(name);
      if (itr==instance().module_generators_.end())
      {
        LERROR("TModuleManager does not have a generator of "<<name);
        return NULL;
      }
      return itr->second;
    }

  static void ShowModule (const std::string &name, const std::string &option="", std::ostream &os=std::cout);
  static void ShowAllModules (const std::string &option="", std::ostream &os=std::cout);

private:

  typedef std::map<std::string, TGenerator> TMap;

  TMap  module_generators_;

  TModuleManager() {};
  TModuleManager(const TModuleManager&);
  ~TModuleManager() {};

  static TModuleManager& instance()
    {
      static TModuleManager module_manager;
      return module_manager;
    }

};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_module_manager_h
//-------------------------------------------------------------------------------------------
