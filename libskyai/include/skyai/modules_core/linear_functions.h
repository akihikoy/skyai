//-------------------------------------------------------------------------------------------
/*! \file    linear_functions.h
    \brief   libskyai - linear function modules  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.04, 2010-

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
#ifndef skyai_linear_functions_h
#define skyai_linear_functions_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <skyai/interfaces/functions.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TLinearFunctionRvConfigurations
//===========================================================================================
{
public:

  TRealMatrix            Factor;

  TLinearFunctionRvConfigurations(var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Factor     );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief Linear function (y = A x)
    \note this module is similar to the MConstMultiplierRv. the difference is
        this module is in/out version, MConstMultiplierRv is signal/slot version. */
class MLinearFunctionRv
    : public MFunctionSISOInterface <TRealVector, TRealVector>
//===========================================================================================
{
public:
  typedef MFunctionSISOInterface <
                        TRealVector,
                        TRealVector>     TParent;
  typedef MLinearFunctionRv              TThis;
  SKYAI_MODULE_NAMES(MLinearFunctionRv)

  MLinearFunctionRv (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map())
    {
    }

  virtual ~MLinearFunctionRv() {}

protected:

  TLinearFunctionRvConfigurations  conf_;

  override void slot_initialize_exec (void)  {}

  override void function (const TInput &x, TOutput &y) const
    {
      y= conf_.Factor * x;
    }

};  // end of MLinearFunctionRv
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_linear_functions_h
//-------------------------------------------------------------------------------------------
