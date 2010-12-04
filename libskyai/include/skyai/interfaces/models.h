//-------------------------------------------------------------------------------------------
/*! \file    models.h
    \brief   libskyai - interface of model modules
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.08, 2010

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
#ifndef skyai_interfaces_models_h
#define skyai_interfaces_models_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief interface module of dynamics model
class MDynamicsModelInterface
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface         TParent;
  typedef MDynamicsModelInterface  TThis;
  SKYAI_MODULE_NAMES(MDynamicsModelInterface)

  MDynamicsModelInterface (const std::string &v_instance_name)
    :
      TParent          (v_instance_name),
      out_trans_probability      (*this),
      out_next_feature           (*this),
      out_most_probable_action   (*this),
      slot_initialize            (*this),
      slot_start_action          (*this),
      slot_finish_action         (*this),
      in_feature                 (*this),
      in_action_set_size         (*this)
    {
      add_out_port   (out_trans_probability     );
      add_out_port   (out_next_feature          );
      add_out_port   (out_most_probable_action  );
      add_slot_port  (slot_initialize           );
      add_slot_port  (slot_start_action         );
      add_slot_port  (slot_finish_action        );
      add_in_port    (in_feature                );
      add_in_port    (in_action_set_size        );
    }

  virtual ~MDynamicsModelInterface() {}

protected:

  /*!\brief output estimated probability to transit from symbol state curr_s to next_s with action curr_a */
  MAKE_OUT_PORT(out_trans_probability, const TReal&, (const TInt &curr_s, const TInt &next_s, const TDiscreteAction &curr_a), (curr_s,next_s,curr_a), TThis);

  /*!\brief output next feature by taking action curr_a from current feature curr_phi */
  MAKE_OUT_PORT(out_next_feature, const TRealVector&, (const TRealVector &curr_phi, const TDiscreteAction &curr_a), (curr_phi,curr_a), TThis);

  /*!\brief output most probable action with which the current symbol state curr_s transits to next_s;
            its probability is stored into trans_prob */
  MAKE_OUT_PORT(out_most_probable_action, const TDiscreteAction&, (const TInt &curr_s, const TInt &next_s, TReal &trans_prob), (curr_s,next_s,trans_prob), TThis);


  /*!\brief initialize this module */
  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  /*!\brief start action curr_a */
  MAKE_SLOT_PORT(slot_start_action, void, (const TDiscreteAction &curr_a), (curr_a), TThis);

  /*!\brief finish action */
  MAKE_SLOT_PORT(slot_finish_action, void, (void), (), TThis);


  /*!\brief input current feature */
  MAKE_IN_PORT(in_feature, const TRealVector& (void), TThis);

  //!\brief input a size of action-set
  MAKE_IN_PORT(in_action_set_size, const TInt& (void), TThis);

  // generate get_##x_in (e.g. get_reward)
  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(feature, const TRealVector&, (void), ())

  GET_FROM_IN_PORT(action_set_size, const TInt&, (void), ())

  #undef GET_FROM_IN_PORT

  virtual const TReal& out_trans_probability_get (const TInt &curr_s, const TInt &next_s, const TDiscreteAction &curr_a) const = 0;
  virtual const TRealVector& out_next_feature_get (const TRealVector &curr_phi, const TDiscreteAction &curr_a) const = 0;
  virtual const TDiscreteAction& out_most_probable_action_get (const TInt &curr_s, const TInt &next_s, TReal &trans_prob) const = 0;

  virtual void slot_initialize_exec (void) = 0;
  virtual void slot_start_action_exec (const TDiscreteAction &curr_a) = 0;
  virtual void slot_finish_action_exec (void) = 0;

};  // end of MDynamicsModelInterface
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief interface module of reward model
class MRewardModelInterface
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface         TParent;
  typedef MRewardModelInterface    TThis;
  SKYAI_MODULE_NAMES(MRewardModelInterface)

  MRewardModelInterface (const std::string &v_instance_name)
    :
      TParent          (v_instance_name),
      out_trans_reward            (*this),
      out_trans_reward_at_feature (*this),
      slot_initialize             (*this),
      slot_start_action           (*this),
      slot_finish_action          (*this),
      in_feature                  (*this),
      in_reward                   (*this),
      in_action_set_size          (*this)
    {
      add_out_port   (out_trans_reward             );
      add_out_port   (out_trans_reward_at_feature  );
      add_slot_port  (slot_initialize              );
      add_slot_port  (slot_start_action            );
      add_slot_port  (slot_finish_action           );
      add_in_port    (in_feature                   );
      add_in_port    (in_reward                    );
      add_in_port    (in_action_set_size           );
    }

  virtual ~MRewardModelInterface() {}

  /*!\brief embed a prior knowledge to model; set reward source */
  virtual void SetRewardSource (const std::list<TRealVector> &features, const std::list<TReal> &rewards) = 0;

protected:

  /*!\brief output extimated reward by taking action curr_a at the current symbol state curr_s */
  MAKE_OUT_PORT(out_trans_reward, const TReal&, (const TInt &curr_s, const TDiscreteAction &curr_a), (curr_s,curr_a), TThis);

  /*!\brief output extimated reward by taking action curr_a at the current feature curr_phi */
  MAKE_OUT_PORT(out_trans_reward_at_feature, const TReal&, (const TRealVector &curr_phi, const TDiscreteAction &curr_a), (curr_phi,curr_a), TThis);


  /*!\brief initialize this module */
  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  /*!\brief start action curr_a */
  MAKE_SLOT_PORT(slot_start_action, void, (const TDiscreteAction &curr_a), (curr_a), TThis);

  /*!\brief finish action */
  MAKE_SLOT_PORT(slot_finish_action, void, (void), (), TThis);


  /*!\brief input current feature */
  MAKE_IN_PORT(in_feature, const TRealVector& (void), TThis);

  /*!\brief input current reward */
  MAKE_IN_PORT(in_reward, const TSingleReward& (void), TThis);

  //!\brief input a size of action-set
  MAKE_IN_PORT(in_action_set_size, const TInt& (void), TThis);

  // generate get_##x_in (e.g. get_reward)
  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(feature, const TRealVector&, (void), ())

  GET_FROM_IN_PORT(reward, const TSingleReward&, (void), ())

  GET_FROM_IN_PORT(action_set_size, const TInt&, (void), ())

  #undef GET_FROM_IN_PORT

  virtual const TReal& out_trans_reward_get (const TInt &curr_s, const TDiscreteAction &curr_a) const = 0;
  virtual const TReal& out_trans_reward_at_feature_get (const TRealVector &curr_phi, const TDiscreteAction &curr_a) const = 0;
  virtual void slot_initialize_exec (void) = 0;
  virtual void slot_start_action_exec (const TDiscreteAction &curr_a) = 0;
  virtual void slot_finish_action_exec (void) = 0;

};  // end of MRewardModelInterface
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_interfaces_models_h
//-------------------------------------------------------------------------------------------
