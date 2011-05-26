//-------------------------------------------------------------------------------------------
/*! \file    models.h
    \brief   libskyai - dynamics and reward models
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.09, 2010-

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
#ifndef skyai_std_models_h
#define skyai_std_models_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/models.h>
#include <skyai/skyai.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TCommonModelConfigurations
//===========================================================================================
{
public:

  TReal    Alpha;     //!< step-size parameter
  TReal    AlphaMin;  //!< lower bound of step-size parameter
  TReal    AlphaDecreasingFactor;   //!< damping coefficient of the step-size parameter. larger is decreasing faster (becoming greedy). do not set a value greater than 1.

  TCommonModelConfigurations (var_space::TVariableMap &mmap)
    :
      Alpha                   (0.05l),
      AlphaMin                (DBL_TINY),
      AlphaDecreasingFactor   (0.0001)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Alpha                  );
      ADD( AlphaMin               );
      ADD( AlphaDecreasingFactor  );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
template <typename t_param_act>
class TCommonModelMemories
//===========================================================================================
{
public:

  TInt  EpisodeNumber;  //!< episode number

  std::vector<t_param_act>   Param;

  TCommonModelMemories (var_space::TVariableMap &mmap)
    :
      EpisodeNumber(0)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( EpisodeNumber );
      ADD( Param         );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
struct TSimpleDMParamAct
//===========================================================================================
{
  TRealMatrix  Fa;

  TSimpleDMParamAct() {}
  TSimpleDMParamAct(int feature_size) : Fa(feature_size,feature_size,0.0l) {}

  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Fa );
      #undef ADD
    }
};
namespace var_space{void Register (TSimpleDMParamAct &x, TVariableMap &mmap);}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief simple dynamics model such that phi' = Fa*phi(x) */
class MSimpleDynamicsModel
    : public MDynamicsModelInterface
//===========================================================================================
{
public:
  typedef MDynamicsModelInterface  TParent;
  typedef MSimpleDynamicsModel     TThis;
  SKYAI_MODULE_NAMES(MSimpleDynamicsModel)

  MSimpleDynamicsModel (const std::string &v_instance_name)
    :
      TParent          (v_instance_name),
      conf_            (TParent::param_box_config_map()),
      mem_             (TParent::param_box_memory_map()),
      slot_start_episode   (*this)
    {
      add_slot_port(slot_start_episode);
    }

  ~MSimpleDynamicsModel() {}

  std::vector<TSimpleDMParamAct>& ModelParam()  {return mem_.Param;}
  const std::vector<TSimpleDMParamAct>& ModelParam() const {return mem_.Param;}

protected:

  TCommonModelConfigurations                       conf_;
  mutable TCommonModelMemories<TSimpleDMParamAct>  mem_;

  mutable TReal            tmp_trans_prob_;
  mutable TRealVector      tmp_next_phi_, tmp_old_phi_;
  mutable TDiscreteAction  tmp_trans_a_, tmp_old_a_;

  //!\brief start an episode
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  override const TReal& out_trans_probability_get (const TInt &curr_s, const TInt &next_s, const TDiscreteAction &curr_a) const;
  override const TRealVector& out_next_feature_get (const TRealVector &curr_phi, const TDiscreteAction &curr_a) const;
  override const TDiscreteAction& out_most_probable_action_get (const TInt &curr_s, const TInt &next_s, TReal &trans_prob) const;

  override void slot_initialize_exec (void);
  override void slot_start_action_exec (const TDiscreteAction &curr_a);
  override void slot_finish_action_exec (void);

  virtual void slot_start_episode_exec (void)  {++mem_.EpisodeNumber;}

  void setup_param() const;

  inline TReal get_alpha (void) const;

};  // end of MSimpleDynamicsModel
//-------------------------------------------------------------------------------------------


//===========================================================================================
struct TMixFS2DMParamAct
//===========================================================================================
{
  TRealMatrix  Fa;
  TRealMatrix  Wa;
  TRealVector  Dxa;

  mutable TRealMatrix CachedFa;  //!< cached transition matrix for fast computation (not saved)
  mutable bool        IsCached;  //!< cached or not (not saved)

  TMixFS2DMParamAct() : IsCached(false) {}
  TMixFS2DMParamAct(int feature_size, int state_dim)
    :
      Fa        (feature_size,feature_size,0.0l),
      Wa        (state_dim,feature_size,0.0),
      Dxa       (state_dim,0.0),
      CachedFa  (feature_size,feature_size,0.0l),
      IsCached  (false)
    {}

  void Resize(int feature_size, int state_dim)
    {
      Fa        .resize(feature_size,feature_size,0.0l);
      Wa        .resize(state_dim,feature_size,0.0);
      Dxa       .resize(state_dim,0.0);
      CachedFa  .resize(feature_size,feature_size,0.0l);
      IsCached  = false;
    }

  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Fa  );
      ADD( Wa  );
      ADD( Dxa );
      #undef ADD
    }
};
namespace var_space{void Register (TMixFS2DMParamAct &x, TVariableMap &mmap);}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief MixFS2 dynamics model such that phi' = Fa*phi(x) + phi(x+Wa*phi(x)+Dxa) */
class MMixFS2DynamicsModel
    : public MDynamicsModelInterface
//===========================================================================================
{
public:
  typedef MDynamicsModelInterface  TParent;
  typedef MMixFS2DynamicsModel     TThis;
  SKYAI_MODULE_NAMES(MMixFS2DynamicsModel)

  MMixFS2DynamicsModel (const std::string &v_instance_name)
    :
      TParent          (v_instance_name),
      conf_            (TParent::param_box_config_map()),
      mem_             (TParent::param_box_memory_map()),
      slot_start_episode   (*this),
      in_state             (*this),
      in_center_state_set  (*this),
      in_state_to_feature  (*this)
    {
      add_slot_port  (slot_start_episode   );
      add_in_port    (in_state             );
      add_in_port    (in_center_state_set  );
      add_in_port    (in_state_to_feature  );
    }

  ~MMixFS2DynamicsModel() {}

  std::vector<TMixFS2DMParamAct>& ModelParam()  {return mem_.Param;}
  const std::vector<TMixFS2DMParamAct>& ModelParam() const {return mem_.Param;}

protected:

  TCommonModelConfigurations                       conf_;
  mutable TCommonModelMemories<TMixFS2DMParamAct>  mem_;

  mutable TReal            tmp_trans_prob_;
  mutable TRealVector      tmp_next_phi_, tmp_old_phi_;
  mutable TRealVector      tmp_next_x_, tmp_old_x_;
  mutable TDiscreteAction  tmp_trans_a_, tmp_old_a_;

  //!\brief start an episode
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  /*!\brief input current state */
  MAKE_IN_PORT(in_state, const TRealVector& (void), TThis);

  //!\brief input a set of the center state of the basis functions
  MAKE_IN_PORT(in_center_state_set, const TRealVectorSet& (void), TThis);

  /*!\brief converter from state to feature */
  MAKE_IN_PORT(in_state_to_feature, void (const TRealVector &, TRealVector &), TThis);

  // generate get_##x_in (e.g. get_reward)
  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(state, const TRealVector&, (void), ())

  GET_FROM_IN_PORT(center_state_set, const TRealVectorSet&, (void), ())

  GET_FROM_IN_PORT(state_to_feature, void, (const TRealVector &x, TRealVector &y), (x,y))

  #undef GET_FROM_IN_PORT

  override const TReal& out_trans_probability_get (const TInt &curr_s, const TInt &next_s, const TDiscreteAction &curr_a) const;
  override const TRealVector& out_next_feature_get (const TRealVector &curr_phi, const TDiscreteAction &curr_a) const;
  override const TDiscreteAction& out_most_probable_action_get (const TInt &curr_s, const TInt &next_s, TReal &trans_prob) const;

  override void slot_initialize_exec (void);
  override void slot_start_action_exec (const TDiscreteAction &curr_a);
  override void slot_finish_action_exec (void);

  virtual void slot_start_episode_exec (void)  {++mem_.EpisodeNumber;}

  void setup_param() const;

  void  update_cache (const TDiscreteAction &a) const;

  inline TReal get_alpha (void) const;

};  // end of MMixFS2DynamicsModel
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TMixFS3DMConfigurations
//===========================================================================================
{
public:

  TReal    AaAlphaRate;     //!< step-size parameter rate for learning Aa

  TMixFS3DMConfigurations (var_space::TVariableMap &mmap)
    :
      AaAlphaRate   (0.1l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( AaAlphaRate    );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
struct TMixFS3DMParamAct
//===========================================================================================
{
  TRealMatrix Fa;
  TRealMatrix Aa;
  TRealMatrix Ba;
  TRealVector Dxa;

  mutable TRealMatrix CachedFa;  //!< cached transition matrix for fast computation (not saved)
  mutable bool        IsCached;  //!< cached or not (not saved)

  TMixFS3DMParamAct() : IsCached(false) {}
  TMixFS3DMParamAct(int feature_size, int state_dim)
    :
      Fa        (feature_size,feature_size,0.0),
      Aa        (state_dim,state_dim,0.0),
      Ba        (state_dim,feature_size,0.0),
      Dxa       (state_dim,0.0),
      CachedFa  (feature_size,feature_size,0.0l),
      IsCached  (false)
    {}

  void Resize(int feature_size, int state_dim)
    {
      Fa        .resize(feature_size,feature_size,0.0);
      Aa        .resize(state_dim,state_dim,0.0);
      Ba        .resize(state_dim,feature_size,0.0);
      Dxa       .resize(state_dim,0.0);
      CachedFa  .resize(feature_size,feature_size,0.0l);
      IsCached  = false;
    }

  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Fa  );
      ADD( Aa  );
      ADD( Ba  );
      ADD( Dxa );
      #undef ADD
    }
};
namespace var_space{void Register (TMixFS3DMParamAct &x, TVariableMap &mmap);}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief MixFS3 dynamics model such that  phi' = F*phi(x) + phi(x+Aa*x+Ba*phi(x)+dxa)
    \note Composition of feature space dynamics model and state space dynamics model. */
class MMixFS3DynamicsModel
    : public MDynamicsModelInterface
//===========================================================================================
{
public:
  typedef MDynamicsModelInterface  TParent;
  typedef MMixFS3DynamicsModel     TThis;
  SKYAI_MODULE_NAMES(MMixFS3DynamicsModel)

  MMixFS3DynamicsModel (const std::string &v_instance_name)
    :
      TParent          (v_instance_name),
      conf_            (TParent::param_box_config_map()),
      mconf_           (TParent::param_box_config_map()),
      mem_             (TParent::param_box_memory_map()),
      slot_start_episode   (*this),
      in_state             (*this),
      in_center_state_set  (*this),
      in_state_to_feature  (*this)
    {
      add_slot_port  (slot_start_episode   );
      add_in_port    (in_state             );
      add_in_port    (in_center_state_set  );
      add_in_port    (in_state_to_feature  );
    }

  ~MMixFS3DynamicsModel() {}

  std::vector<TMixFS3DMParamAct>& ModelParam()  {return mem_.Param;}
  const std::vector<TMixFS3DMParamAct>& ModelParam() const {return mem_.Param;}

protected:

  TCommonModelConfigurations               conf_;
  TMixFS3DMConfigurations                          mconf_;
  mutable TCommonModelMemories<TMixFS3DMParamAct>  mem_;

  mutable TReal            tmp_trans_prob_;
  mutable TRealVector      tmp_next_phi_, tmp_old_phi_;
  mutable TRealVector      tmp_next_x_, tmp_old_x_;
  mutable TDiscreteAction  tmp_trans_a_, tmp_old_a_;

  //!\brief start an episode
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  /*!\brief input current state */
  MAKE_IN_PORT(in_state, const TRealVector& (void), TThis);

  //!\brief input a set of the center state of the basis functions
  MAKE_IN_PORT(in_center_state_set, const TRealVectorSet& (void), TThis);

  /*!\brief converter from state to feature */
  MAKE_IN_PORT(in_state_to_feature, void (const TRealVector &, TRealVector &), TThis);

  // generate get_##x_in (e.g. get_reward)
  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(state, const TRealVector&, (void), ())

  GET_FROM_IN_PORT(center_state_set, const TRealVectorSet&, (void), ())

  GET_FROM_IN_PORT(state_to_feature, void, (const TRealVector &x, TRealVector &y), (x,y))

  #undef GET_FROM_IN_PORT

  override const TReal& out_trans_probability_get (const TInt &curr_s, const TInt &next_s, const TDiscreteAction &curr_a) const;
  override const TRealVector& out_next_feature_get (const TRealVector &curr_phi, const TDiscreteAction &curr_a) const;
  override const TDiscreteAction& out_most_probable_action_get (const TInt &curr_s, const TInt &next_s, TReal &trans_prob) const;

  override void slot_initialize_exec (void);
  override void slot_start_action_exec (const TDiscreteAction &curr_a);
  override void slot_finish_action_exec (void);

  virtual void slot_start_episode_exec (void)  {++mem_.EpisodeNumber;}

  void setup_param() const;

  void  update_cache (const TDiscreteAction &a) const;

  inline TReal get_alpha (void) const;

};  // end of MMixFS3DynamicsModel
//-------------------------------------------------------------------------------------------


//===========================================================================================
struct TSimpleRMParamAct
//===========================================================================================
{
  TRealVector  Ba;

  TSimpleRMParamAct() {}
  TSimpleRMParamAct(int feature_size) : Ba(feature_size,0.0l) {}

  void Resize(int feature_size)
    {
      Ba.resize(feature_size,0.0l);
    }

  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Ba );
      #undef ADD
    }
};
namespace var_space{void Register (TSimpleRMParamAct &x, TVariableMap &mmap);}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief simple reward model such that R(x,a) = Ba*phi(x) */
class MSimpleRewardModel
    : public MRewardModelInterface
//===========================================================================================
{
public:
  typedef MRewardModelInterface    TParent;
  typedef MSimpleRewardModel       TThis;
  SKYAI_MODULE_NAMES(MSimpleRewardModel)

  MSimpleRewardModel (const std::string &v_instance_name)
    :
      TParent          (v_instance_name),
      conf_            (TParent::param_box_config_map()),
      mem_             (TParent::param_box_memory_map()),
      slot_start_episode   (*this)
    {
      add_slot_port(slot_start_episode);
    }

  ~MSimpleRewardModel() {}

  std::vector<TSimpleRMParamAct>& ModelParam()  {return mem_.Param;}
  const std::vector<TSimpleRMParamAct>& ModelParam() const {return mem_.Param;}

  /*!\brief embed a prior knowledge to model; set reward source */
  override void SetRewardSource (const std::list<TRealVector> &features, const std::list<TReal> &rewards);

protected:

  TCommonModelConfigurations                       conf_;
  mutable TCommonModelMemories<TSimpleRMParamAct>  mem_;

  mutable TReal            tmp_r_;
  mutable TRealVector      tmp_old_phi_;
  mutable TDiscreteAction  tmp_old_a_;

  //!\brief start an episode
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  override const TReal& out_trans_reward_get (const TInt &curr_s, const TDiscreteAction &curr_a) const;
  override const TReal& out_trans_reward_at_feature_get (const TRealVector &curr_phi, const TDiscreteAction &curr_a) const;
  override void slot_initialize_exec (void);
  override void slot_start_action_exec (const TDiscreteAction &curr_a);
  override void slot_finish_action_exec (void);

  virtual void slot_start_episode_exec (void)  {++mem_.EpisodeNumber;}

  void setup_param() const;

  inline TReal get_alpha (void) const;

};  // end of MSimpleRewardModel
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_std_models_h
//-------------------------------------------------------------------------------------------
