//-------------------------------------------------------------------------------------------
/*! \file    st_value_calculator.h
    \brief   libskyai - state/action value calculator
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jan.01, 2010-

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
#ifndef skyai_st_value_calculator_h
#define skyai_st_value_calculator_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <lora/stl_ext.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TStateValueCalculator1Configurations
//===========================================================================================
{
public:

  TInt                      Index;  //! dimension to be changed
  TReal                     Min;
  TReal                     Max;
  TInt                      Division;

  TStateValueCalculator1Configurations (var_space::TVariableMap &mmap)
    :
      Index     (0),
      Min       (-1.0l),
      Max       (1.0l),
      Division  (50)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Index     );
      ADD( Min       );
      ADD( Max       );
      ADD( Division  );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief get a state value at the current state and its surround ({x|x[i]=state[i](i!=Index), x[Index]={Min,..,Max}).
          this module is designed for an RL module which uses a feature vector as a state input
    \note 1F denotes 1-dim and using Feature vector */
class MStateValueCalculator1F
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface         TParent;
  typedef MStateValueCalculator1F  TThis;
  SKYAI_MODULE_NAMES(MStateValueCalculator1F)

  MStateValueCalculator1F (const std::string &v_instance_name)
    : TParent              (v_instance_name),
      conf_                (TParent::param_box_config_map()),
      in_state_value       (*this),
      in_state             (*this),
      in_state_to_feature  (*this),
      slot_calculate       (*this),
      out_state_values     (*this)
    {
      add_in_port   ( in_state_value       );
      add_in_port   ( in_state             );
      add_in_port   ( in_state_to_feature  );
      add_slot_port ( slot_calculate       );
      add_out_port  ( out_state_values     );
    }

protected:

  TStateValueCalculator1Configurations  conf_;
  TRealVector  state_values_;

  MAKE_IN_PORT(in_state_value, void (const TRealVector &phi, TValue &v), TThis);

  MAKE_IN_PORT(in_state, const TContinuousState& (void), TThis);

  MAKE_IN_PORT(in_state_to_feature, void (const TContinuousState&, TRealVector&), TThis);

  MAKE_SLOT_PORT(slot_calculate, void, (void), (), TThis);

  MAKE_OUT_PORT(out_state_values, const TRealVector&, (void), (), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(state_value, void, (const TRealVector &phi, TValue &v), (phi,v))

  GET_FROM_IN_PORT(state, const TContinuousState&, (void), ())

  GET_FROM_IN_PORT(state_to_feature, void, (const TContinuousState &x, TRealVector &phi), (x,phi))

  #undef GET_FROM_IN_PORT


  virtual void slot_calculate_exec (void);

  virtual const TRealVector& out_state_values_get (void) const
    {
      return state_values_;
    }

};  // end of MStateValueCalculator1F
//-------------------------------------------------------------------------------------------



//===========================================================================================
class TCalculatorOnGridConfigurations
//===========================================================================================
{
public:

  TRealVector               Min;  //! vector on the state space which decides the lower bound of the grid
  TRealVector               Max;  //! vector on the state space which decides the upper bound of the grid
  TIntVector                Division;  //! the numbers into which the grid is divided

  TCalculatorOnGridConfigurations (var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Min         );
      ADD( Max         );
      ADD( Division    );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief get a state value on the grid made from Min, Max, and Division.
          this module is designed for an RL module which uses a feature vector as a state input
    \note NF denotes N-dim and using Feature vector */
class MStateValueCalculatorNF
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface         TParent;
  typedef MStateValueCalculatorNF  TThis;
  SKYAI_MODULE_NAMES(MStateValueCalculatorNF)

  MStateValueCalculatorNF (const std::string &v_instance_name)
    : TParent              (v_instance_name),
      conf_                (TParent::param_box_config_map()),
      in_state_value       (*this),
      in_state_to_feature  (*this),
      slot_calculate       (*this),
      out_state_values     (*this)
    {
      add_in_port   ( in_state_value       );
      add_in_port   ( in_state_to_feature  );
      add_slot_port ( slot_calculate       );
      add_out_port  ( out_state_values     );
    }

protected:

  TCalculatorOnGridConfigurations  conf_;

  /*! state_0[0] state_0[1] .. state_0[state-dim-1]  state-value_0
      state_1[0] state_1[1] .. state_1[state-dim-1]  state-value_1
      ...  */
  TRealMatrix   state_values_;

  MAKE_IN_PORT(in_state_value, void (const TRealVector &phi, TValue &v), TThis);

  MAKE_IN_PORT(in_state_to_feature, void (const TContinuousState&, TRealVector&), TThis);

  MAKE_SLOT_PORT(slot_calculate, void, (void), (), TThis);

  MAKE_OUT_PORT(out_state_values, const TRealMatrix&, (void), (), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(state_value, void, (const TRealVector &phi, TValue &v), (phi,v))

  GET_FROM_IN_PORT(state_to_feature, void, (const TContinuousState &x, TRealVector &phi), (x,phi))

  #undef GET_FROM_IN_PORT


  virtual void slot_calculate_exec (void);

  virtual const TRealMatrix& out_state_values_get (void) const
    {
      return state_values_;
    }

};  // end of MStateValueCalculatorNF
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief get action value on the grid made from Min, Max, and Division.
          this module is designed for an RL module which uses a feature vector as a state input
    \note this module can be used with a discrete-action RL module
    \note NF denotes N-dim and using Feature vector */
class MActionValueCalculatorNF
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface          TParent;
  typedef MActionValueCalculatorNF  TThis;
  SKYAI_MODULE_NAMES(MActionValueCalculatorNF)

  MActionValueCalculatorNF (const std::string &v_instance_name)
    : TParent              (v_instance_name),
      conf_                (TParent::param_box_config_map()),
      in_action_value      (*this),
      in_state_to_feature  (*this),
      slot_calculate       (*this),
      out_action_values    (*this)
    {
      add_in_port   ( in_action_value      );
      add_in_port   ( in_state_to_feature  );
      add_slot_port ( slot_calculate       );
      add_out_port  ( out_action_values    );
    }

protected:

  TCalculatorOnGridConfigurations  conf_;

  /*! state_0[0] state_0[1] .. state_0[state-dim-1]  action-value_0[0] .. action-value_0[NA-1]
      state_1[0] state_1[1] .. state_1[state-dim-1]  action-value_1[0] .. action-value_1[NA-1]
      ...  */
  TRealMatrix   action_values_;

  MAKE_IN_PORT(in_action_value, void (const TRealVector &phi, TRealVector &v), TThis);

  MAKE_IN_PORT(in_state_to_feature, void (const TContinuousState&, TRealVector&), TThis);

  MAKE_SLOT_PORT(slot_calculate, void, (void), (), TThis);

  MAKE_OUT_PORT(out_action_values, const TRealMatrix&, (void), (), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(action_value, void, (const TRealVector &phi, TRealVector &v), (phi,v))

  GET_FROM_IN_PORT(state_to_feature, void, (const TContinuousState &x, TRealVector &phi), (x,phi))

  #undef GET_FROM_IN_PORT


  virtual void slot_calculate_exec (void);

  virtual const TRealMatrix& out_action_values_get (void) const
    {
      return action_values_;
    }

};  // end of MActionValueCalculatorNF
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_st_value_calculator_h
//-------------------------------------------------------------------------------------------
