//-------------------------------------------------------------------------------------------
/*! \file    h_tdgfa_cohen.h
    \brief   libskyai - Cohen's hierarchical RL (HRL) implementation
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.22, 2010-

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
#ifndef h_tdgfa_cohen_h
#define h_tdgfa_cohen_h
//-------------------------------------------------------------------------------------------
#include <skyai/modules_std/td_generic_fa.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
namespace tdgfa_tmpl_detail
{
//-------------------------------------------------------------------------------------------



//===========================================================================================
/*!\brief Cohen's hierarchical reinforcement learning (TD-learning) module for generic function approximator
  \note this function inherits the TD(lambda)-learning module, but remember that the convergence of the HRL
      is guaranteed only for lambda==0. (i.e. do not use the activity-trace) */
template <typename t_action>
class MHierarchicalTDGenericFACohen
    : public MTDGenericFuncApprox <t_action>
//===========================================================================================
{
public:
  typedef MTDGenericFuncApprox <t_action>  TParent;
  typedef typename TParent::TAction        TAction;
  typedef MHierarchicalTDGenericFACohen    TThis;
  SKYAI_MODULE_NAMES(MHierarchicalTDGenericFACohen)

  MHierarchicalTDGenericFACohen (const std::string &v_instance_name)
    : TParent                       (v_instance_name),
      // conf_                         (TParent::param_box_config_map()),
      slot_execute_higher_action    (*this),
      slot_update                   (*this),
      slot_clear_flag_greedy        (*this),
      signal_reset_lower            (*this),
      signal_end_of_higher_action   (*this),
      signal_update_lower           (*this),
      signal_clear_flag_greedy      (*this)
    {
      add_slot_port   (slot_execute_higher_action   );
      add_slot_port   (slot_update                  );
      add_slot_port   (slot_clear_flag_greedy       );
      add_signal_port (signal_reset_lower           );
      add_signal_port (signal_end_of_higher_action  );
      add_signal_port (signal_update_lower          );
      add_signal_port (signal_clear_flag_greedy     );
    }

protected:

  bool         higher_action_is_active_;

  bool         old_flag_greedy_;
  bool         next_flag_greedy_;

  TRealVector  root_values_;

  MAKE_SLOT_PORT(slot_execute_higher_action, void, (void), (), TThis);

  /*!\brief  update this module, and emit update signal to lower modules
      \note nextQroot is used in on-policy learning (Sarsa), nextVroot is used in off-policy learning (Q-L)
      \note values=(reward, nextQroot, nextVroot) must be supplied by the root node (module) */
  MAKE_SLOT_PORT(slot_update, void, (const TRealVector &values), (values), TThis);

  MAKE_SLOT_PORT(slot_clear_flag_greedy, void, (void), (), TThis);

  MAKE_SIGNAL_PORT(signal_reset_lower, void (void), TThis);

  MAKE_SIGNAL_PORT(signal_end_of_higher_action, void (void), TThis);

  MAKE_SIGNAL_PORT(signal_update_lower, void (const TAction &old_action, const TRealVector &values), TThis);

  MAKE_SIGNAL_PORT(signal_clear_flag_greedy, void (void), TThis);

  override void slot_start_episode_exec (void);
  override void slot_finish_action_exec (void);
  virtual void slot_execute_higher_action_exec (void);
  virtual void slot_update_exec (const TRealVector &values);
  virtual void slot_clear_flag_greedy_exec (void)
    {
      next_flag_greedy_= false;
      signal_clear_flag_greedy.ExecAll();
    }

  using TParent::conf_;

  using TParent::signal_end_of_episode   ;
  using TParent::signal_execute_action   ;
  using TParent::signal_end_of_action    ;

  using TParent::ModuleUniqueCode        ;

  using TParent::get_reward;

  using TParent::old_action;
  using TParent::next_action;
  using TParent::nextQ;
  using TParent::nextV;

  using TParent::is_active_;
  using TParent::total_episodes_;
  using TParent::return_in_episode_;
  using TParent::actions_in_episode_;
  using TParent::is_end_of_episode_;
  using TParent::current_action_value_;


  override bool is_updatable() const
    {
      return next_flag_greedy_ && TParent::is_updatable();
    }

  override void prepare_next_action ()
    {
      TParent::prepare_next_action();
      if (signal_update_lower.ConnectionSize()==0)  // terminal node
      {
        next_flag_greedy_= true;
      }
      else  // non-terminal node
      {
        next_flag_greedy_= (nextQ==nextV);
        if (!next_flag_greedy_)
          signal_clear_flag_greedy.ExecAll();
      }
    }

  using TParent::reset_episode;

  override TAction select_action ()
    {
      old_flag_greedy_= next_flag_greedy_;
      return TParent::select_action();
    }

  using TParent::update;

};  // end of MHierarchicalTDGenericFACohen
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MHierarchicalTDGenericFACohen,TContinuousAction)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MHierarchicalTDGenericFACohen,TDiscreteAction)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
} // end of tdgfa_tmpl_detail
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // h_tdgfa_cohen_h
//-------------------------------------------------------------------------------------------
