//-------------------------------------------------------------------------------------------
/*! \file    dcob.h
    \brief   libskyai - DCOB module (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.08, 2010-

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
#ifndef skyai_dcob_h
#define skyai_dcob_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <lora/stl_ext.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TActionSetDCOBConfigurations
//===========================================================================================
{
public:

  TRealVector           IntervalSet;

  TActionSetDCOBConfigurations (var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( IntervalSet    );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief DCOB which discretizes the action space BFTrans (see skyai/modules_std/as_bftrans.h) */
class MActionSetDCOB
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef MActionSetDCOB       TThis;
  SKYAI_MODULE_NAMES(MActionSetDCOB)

  MActionSetDCOB (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      slot_execute_action     (*this),
      signal_execute_command  (*this),
      out_action_set_size     (*this),
      in_center_state_set     (*this)
    {
      add_slot_port   (slot_execute_action     );
      add_signal_port (signal_execute_command  );
      add_out_port    (out_action_set_size     );
      add_in_port     (in_center_state_set     );
    }

  virtual ~MActionSetDCOB() {}

protected:

  TActionSetDCOBConfigurations  conf_;

  TContinuousAction  bf_trans_action_;

  //!\brief if this slot catch a signal, this module starts to generate a sequence of control command according to `a'
  MAKE_SLOT_PORT(slot_execute_action, void, (const TDiscreteAction &a), (a), TThis);

  //!\brief this signal is emitted to request to execute the control command
  MAKE_SIGNAL_PORT(signal_execute_command, void(const TContinuousAction&), TThis);

  MAKE_OUT_PORT(out_action_set_size, const TInt&, (void), (), TThis);

  //!\brief input a set of the center state of the basis functions
  MAKE_IN_PORT(in_center_state_set, const std::vector<TRealVector>& (void), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(center_state_set, const std::vector<TRealVector>&, (void), ())

  #undef GET_FROM_IN_PORT

  virtual void slot_execute_action_exec (const TDiscreteAction &a);

  mutable TInt tmp_assize_;
  const TInt& out_action_set_size_get (void) const
    {return (tmp_assize_= get_center_state_set().size() * conf_.IntervalSet.length());}

};  // end of MActionSetDCOB
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_dcob_h
//-------------------------------------------------------------------------------------------
