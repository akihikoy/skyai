//-------------------------------------------------------------------------------------------
/*! \file    general_agent.h
    \brief   libskyai - general_agent executable (header)
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
#ifndef skyai_general_agent_h
#define skyai_general_agent_h
//-------------------------------------------------------------------------------------------
#include <skyai/base.h>
#include <lora/small_classes.h>
#include <boost/function.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief TGAGlobalVariables holds global variables for the general_agent unit that are able to
    assign in the initialization stage by initializing a global constant by a SetXXX member function.
    TModuleManager is designed as a singleton */
class TGAGlobalVariables
//===========================================================================================
{
public:
  typedef boost::function<int(TOptionParser &option,TAgent &agent)> TSkyAIMain;

  static bool SetSkyAIMain(TSkyAIMain f)  {instance().SkyAIMain= f;  return true;}
  static TSkyAIMain GetSkyAIMain()  {return instance().SkyAIMain;}

private:
  TGAGlobalVariables() {}

  TGAGlobalVariables(const TGAGlobalVariables&);
  ~TGAGlobalVariables() {}
  const TGAGlobalVariables& operator=(const TGAGlobalVariables&);

  static TGAGlobalVariables& instance()
    {
      static TGAGlobalVariables global_variables;
      return global_variables;
    }

  // variables:
  TSkyAIMain SkyAIMain;  //!< SkyAI's main function called by general_agent

};
//-------------------------------------------------------------------------------------------
//! assign to SkyAIMain
#define SKYAI_SET_MAIN(x_main)    \
  namespace skyai_detail          \
  {                               \
    volatile const bool xx_registered_##x_main =  \
        ::loco_rabbits::TGAGlobalVariables::SetSkyAIMain(x_main); \
  }
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_general_agent_h
//-------------------------------------------------------------------------------------------
