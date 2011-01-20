//-------------------------------------------------------------------------------------------
/*! \file    lc_pd.h
    \brief   libskyai - PD-controller  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.09, 2010-

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
#ifndef skyai_lc_pd_h
#define skyai_lc_pd_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/action_space.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

enum TPDEndOfActionCondition
{
  eoacNone      =0, //!< signal_end_of_action is never emitted
  eoacError       , //!< signal_end_of_action is emitted when the first time the error becomes less than a threshold, ThError
  eoacInterval      //!< signal_end_of_action is emitted at the specified running time, Interval
};
ENUM_STR_MAP_BEGIN(TPDEndOfActionCondition)
  ENUM_STR_MAP_ADD( eoacNone           )
  ENUM_STR_MAP_ADD( eoacError          )
  ENUM_STR_MAP_ADD( eoacInterval       )
ENUM_STR_MAP_END  (TPDEndOfActionCondition)
SPECIALIZE_TVARIABLE_TO_ENUM(TPDEndOfActionCondition)


//===========================================================================================
class TLCSimplePDConfigurations
//===========================================================================================
{
public:

  TPDEndOfActionCondition       EOACondition;
  TReal                         ThError;    //!< error threshold
  TReal                         Interval;

  TRealVector                   Kp, Kd;   //!< gain parameter with no interference among the dimensions

  TLCSimplePDConfigurations(var_space::TVariableMap &mmap)
    :
      EOACondition   (eoacNone),
      ThError        (0.0l),
      Interval       (0.0l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( EOACondition      );
      ADD( ThError           );
      ADD( Interval          );
      ADD( Kp                );
      ADD( Kd                );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief low-level controller - a simple PD controller
    \TODO NOTE and TODO: this module have not been tested yet. make sure that it works well.  */
class MLCSimplePD
    : public MActionSpaceInterface <TRealVector, TRealVector>
//===========================================================================================
{
public:
  typedef MActionSpaceInterface
              <TRealVector, TRealVector>   TParent;
  typedef MLCSimplePD                      TThis;
  SKYAI_MODULE_NAMES(MLCSimplePD)

  MLCSimplePD (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      ltime_         (-1.0l),
      is_active_     (false),
      in_state                  (*this),
      in_extract_proportional   (*this),
      in_extract_derivative     (*this)
    {
      add_in_port (in_state                 );
      add_in_port (in_extract_proportional  );
      add_in_port (in_extract_derivative    );
    }

  virtual ~MLCSimplePD() {}

protected:

  TLCSimplePDConfigurations  conf_;

  TContinuousTime    ltime_;
  bool               is_active_;
  TRealVector  target_;  //!< only proportional elements (derivative elements are zero)
  TRealVector  state_proportional_, state_derivative_; // , state_pd_;
  TRealVector  command_;

  //!\brief input the current state
  MAKE_IN_PORT(in_state, const TContinuousState& (void), TThis);

  //!\brief extract proportional elements of a state (Cp)
  MAKE_IN_PORT(in_extract_proportional, void (const TRealVector &in, TRealVector &out), TThis);

  //!\brief extract derivative elements of a state (Cd)
  MAKE_IN_PORT(in_extract_derivative, void (const TRealVector &in, TRealVector &out), TThis);

  override void slot_initialize_exec (void)  {is_active_=false;}
  override void slot_reset_exec (void)  {is_active_=false;}
  override void slot_execute_action_exec (const TRealVector &a);
  override void slot_start_time_step_exec (const TContinuousTime &time_step);
  override void slot_finish_time_step_exec (const TContinuousTime &time_step);
  override void slot_finish_action_immediately_exec (void);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(state, const TContinuousState&, (void), ())

  GET_FROM_IN_PORT(extract_proportional, void, (const TRealVector &in, TRealVector &out), (in,out))

  GET_FROM_IN_PORT(extract_derivative, void, (const TRealVector &in, TRealVector &out), (in,out))

  #undef GET_FROM_IN_PORT

  using TParent::signal_end_of_action;
  using TParent::signal_execute_command;

  void extract_pd_from_state (const TContinuousState &state, TRealVector &p, TRealVector &d/*, TRealVector &pd*/) const
    {
      get_extract_proportional (state, p);
      get_extract_derivative (state, d);
      // pd.resize (p.length()+d.length());
      // std::copy (GenBegin(p),GenEnd(p), GenBegin(pd));
      // std::copy (GenBegin(d),GenEnd(d), GenBegin(pd)+p.length());
    }

};  // end of MLCSimplePD
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_lc_pd_h
//-------------------------------------------------------------------------------------------
