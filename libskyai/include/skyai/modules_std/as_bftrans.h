//-------------------------------------------------------------------------------------------
/*! \file    as_bftrans.h
    \brief   libskyai - action space BFTrans (part of DCOB) (header)
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
#ifndef skyai_as_bftrans_h
#define skyai_as_bftrans_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/action_space.h>
#include <lora/ctrl_tools.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


enum TTrajectoryGeneratorType
{
  tgtJerkMin      =0,
  tgtAccMin       ,
  tgtAccMin2
};
ENUM_STR_MAP_BEGIN(TTrajectoryGeneratorType)
  ENUM_STR_MAP_ADD( tgtJerkMin       )
  ENUM_STR_MAP_ADD( tgtAccMin        )
  ENUM_STR_MAP_ADD( tgtAccMin2       )
ENUM_STR_MAP_END  (TTrajectoryGeneratorType)
SPECIALIZE_TVARIABLE_TO_ENUM(TTrajectoryGeneratorType)


//===========================================================================================
class TBFTransConfigurations
//===========================================================================================
{
public:

  TInt                       ProportionalDim;  //! dimension of proportional elements of state
  bool                       SetTargetByState;  //! set true for DCOB, false for WF-DCOB

  bool                       AbbreviateTrajectory;  //!< abbreviating trajectory (set false for OldDCOB)

  /*! scale factor that scales the abbreviation;
      1: abbreviated trajectory is about a distance to the nearest BF. */
  TReal                      AbbreviatingScale;

  TTrajectoryGeneratorType   TrajectoryGeneratorType;

  // parameters for TTrjGeneratorCondition
  bool                       ZeroUnspecifiedState; /*!< true: in TTrajectoryGeneratorBase::Step,
                                                        if jstate's dimension is smaller than required,
                                                        use zero instead of the internal state
                                                        (set false for OldDCOB) */

  /*! using maximum norm as a distance instead of L-2 norm (in AbbreviateTrajectory)
      \note this value should be the same as one which used in in_distance_to_nearest_bf */
  bool                       UsingMaxNorm;

  TBFTransConfigurations(var_space::TVariableMap &mmap)
    :
      ProportionalDim          (-1),
      SetTargetByState         (true),
      AbbreviateTrajectory     (true),
      AbbreviatingScale        (1.0l),
      TrajectoryGeneratorType  (tgtAccMin2),
      ZeroUnspecifiedState     (true),
      UsingMaxNorm             (false)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( ProportionalDim           );
      ADD( SetTargetByState          );
      ADD( AbbreviateTrajectory      );
      ADD( AbbreviatingScale         );
      ADD( TrajectoryGeneratorType   );
      ADD( ZeroUnspecifiedState      );
      ADD( UsingMaxNorm              );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief Action space BFTrans: get input (g, x_target) as an action and output q_target (desired proportional elements of state)  */
class MBFTrans
    : public MActionSpaceInterface <TContinuousAction, TRealVector>
//===========================================================================================
{
public:
  typedef MActionSpaceInterface
                  <TContinuousAction,
                          TRealVector>   TParent;
  typedef MBFTrans                       TThis;
  SKYAI_MODULE_NAMES(MBFTrans)

  MBFTrans (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      in_control_    (false),
      trj_gen_       (NULL),
      trj_conf_      (0),
      in_feature                 (*this),
      in_state                   (*this),
      in_distance_to_nearest_bf  (*this),
      in_extract_proportional    (*this),
      in_extract_derivative      (*this)
    {
      add_in_port (in_feature                 );
      add_in_port (in_state                   );
      add_in_port (in_distance_to_nearest_bf  );
      add_in_port (in_extract_proportional    );
      add_in_port (in_extract_derivative      );
    }

  virtual ~MBFTrans() {clear_trj_gen();}

protected:

  TBFTransConfigurations  conf_;

  bool         in_control_;

  TRealVector  target_;
  TRealVector  target_proportional_, target_derivative_, target_pd_;
  TRealVector  state_proportional_, state_derivative_, state_pd_;
  TReal        interval_neighbor_;

  TTrajectoryGeneratorBase  *trj_gen_;
  TTrjGeneratorCondition    trj_conf_;

  //!\brief input the current feature vector (output of the basis functions at the current state)
  MAKE_IN_PORT(in_feature, const TRealVector& (void), TThis);

  //!\brief input the current state
  MAKE_IN_PORT(in_state, const TContinuousState& (void), TThis);

  //!\brief distance vector to the nearest basis function from each BF (dim == num. of BFs)
  MAKE_IN_PORT(in_distance_to_nearest_bf, const TRealVector& (void), TThis);

  //!\brief extract proportional elements of a state (Cp)
  MAKE_IN_PORT(in_extract_proportional, void (const TRealVector &in, TRealVector &out), TThis);

  //!\brief extract derivative elements of a state (Cd)
  MAKE_IN_PORT(in_extract_derivative, void (const TRealVector &in, TRealVector &out), TThis);

  override void slot_initialize_exec (void);
  override void slot_reset_exec (void)  {}
  override void slot_execute_action_exec (const TAction &a);
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

  GET_FROM_IN_PORT(feature, const TRealVector&, (void), ())

  GET_FROM_IN_PORT(state, const TContinuousState&, (void), ())

  GET_FROM_IN_PORT(distance_to_nearest_bf, const TRealVector&, (void), ())

  GET_FROM_IN_PORT(extract_proportional, void, (const TRealVector &in, TRealVector &out), (in,out))

  GET_FROM_IN_PORT(extract_derivative, void, (const TRealVector &in, TRealVector &out), (in,out))

  #undef GET_FROM_IN_PORT

  using TParent::signal_end_of_action;
  using TParent::signal_execute_command;

  void clear_trj_gen (void)
    {
      if (trj_gen_)  delete trj_gen_;
      trj_gen_= NULL;
    }

  void extract_pd_from_state (const TContinuousState &state, TRealVector &p, TRealVector &d, TRealVector &pd) const
    {
      get_extract_proportional (state, p);
      get_extract_derivative (state, d);
      pd.resize (p.length()+d.length());
      std::copy (GenBegin(p),GenEnd(p), GenBegin(pd));
      std::copy (GenBegin(d),GenEnd(d), GenBegin(pd)+p.length());
    }

};  // end of MBFTrans
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_as_bftrans_h
//-------------------------------------------------------------------------------------------
