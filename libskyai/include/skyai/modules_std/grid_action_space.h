//-------------------------------------------------------------------------------------------
/*! \file    grid_action_space.h
    \brief   libskyai - Cohen's hierarchical RL (HRL) implementation
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.22, 2009-

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
#ifndef grid_action_space_h
#define grid_action_space_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/action_space.h>
#include <lora/small_classes.h>
#include <lora/octave_str.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace grid_action_space_detail
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TConfigurations
//===========================================================================================
{
public:

  TIntVector           Levels;      //!< vector of the number of partitions into which each dimension is divided
  TRealVector          ActionMin;  //!< minimum vector
  TRealVector          ActionMax;  //!< maximum vector
  TReal                Interval;    //!< time during which an action is executed

  TConfigurations(var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Levels         );
      ADD( ActionMin      );
      ADD( ActionMax      );
      ADD( Interval       );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief standard grid action module.
        this module catches a discrete action signal, and
        emits a execute-command signal of a real vector for Interval
    \todo TODO: <b>REPLACE THIS MODULE BY MDiscretizer AND A CONTINUOUS ACTION SPACE</b>  */
class MGridActionSpace
    : public MDiscreteActionSpaceInterface <TRealVector>
//===========================================================================================
{
public:
  typedef MDiscreteActionSpaceInterface <TRealVector>  TParent;
  typedef MGridActionSpace                             TThis;
  SKYAI_MODULE_NAMES(MGridActionSpace)

  MGridActionSpace (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      ltime_         (-1.0l)
    {
    }

  virtual ~MGridActionSpace() {}

protected:

  TConfigurations  conf_;

  TGridGenerator   grid_;

  TReal            ltime_;  //!< if positive, an action is active
  TRealVector      current_command_;

  override void slot_initialize_exec (void);
  override void slot_reset_exec (void)  {}
  override void slot_execute_action_exec (const TAction &a);
  override void slot_start_time_step_exec (const TContinuousTime &time_step);
  override void slot_finish_time_step_exec (const TContinuousTime &time_step);
  override void slot_finish_action_immediately_exec (void);

  mutable TInt tmp_assize_;
  override const TInt& out_action_set_size_get (void) const  {return (tmp_assize_= grid_.Size());}

};  // end of MGridActionSpace
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of grid_action_space_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // grid_action_space_h
//-------------------------------------------------------------------------------------------
