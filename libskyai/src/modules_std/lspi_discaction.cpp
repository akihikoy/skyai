//-------------------------------------------------------------------------------------------
/*! \file    lspi_discaction.cpp
    \brief   libskyai - LSPI implementation for a discrete action set [Lagoudakis and Parr, 2001]
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Mar.29, 2010-

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
#include <skyai/modules_std/lspi_discaction.h>
#include <lora/stl_ext.h>
#include <octave/dbleSVD.h>
#include <iomanip>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace lspi_discaction_detail
{
using namespace std;
// using namespace boost;
using namespace disc_action;


/*!\brief calculate action value at the feature phi with action value parameter W, and store into resQs
    \note size of the resQs must be the same as the action set */
void calc_action_value (const TRealVector &W, const TRealVector &phi, TRealVector &resQs)
{
  TypeExtS<TRealVector>::const_iterator  witr(GenBegin(W));
  for (TypeExtS<TRealVector>::iterator qsitr(GenBegin(resQs)); qsitr!=GenEnd(resQs); ++qsitr)
  {
    *qsitr= 0.0l;
    for (TypeExtS<TRealVector>::const_iterator phiitr(GenBegin(phi)); phiitr!=GenEnd(phi); ++witr,++phiitr)
      (*qsitr)+= (*witr)*(*phiitr);
  }
}
//-------------------------------------------------------------------------------------------

/*!\brief calculate action value at the (sparse) feature sphi with action value parameter W, and store into resQs
    \note size of the resQs must be the same as the action set */
void calc_action_value (const TRealVector &W, const TSparseFeature &sphi, TRealVector &resQs, TInt size_of_feature)
{
  TInt row_base(0);
  for (TypeExtS<TRealVector>::iterator qsitr(GenBegin(resQs)); qsitr!=GenEnd(resQs); ++qsitr,row_base+=size_of_feature)
  {
    *qsitr= 0.0l;
    for (TSparseFeature::const_iterator sphiitr(sphi.begin()); sphiitr!=sphi.end(); ++sphiitr)
      (*qsitr)+= W(row_base+sphiitr->first)*(sphiitr->second);
  }
}
//-------------------------------------------------------------------------------------------

//! calculate the feature vector over (state,action)
void make_complete_feature (const TRealVector &state_feature, const TDiscreteAction &action, SparseMatrix &phi,
    const TReal &nonzero_feature_threshold, TInt nonzero_feature_max_size)
{
  // phi= SparseMatrix(wsize,1,nonzero_feature_max_size);
  TInt count=0, row=action*GenSize(state_feature);
  for (TypeExtS<TRealVector>::const_iterator itr(GenBegin(state_feature)),last(GenEnd(state_feature));
      itr!=last; ++itr,++row)
  {
    if (*itr>nonzero_feature_threshold)
    {
      if (count<nonzero_feature_max_size)
      {
        phi(row,0)=*itr;
        ++count;
      }
      else
        {LWARNING("number of nonzero elements ("<<count<<") of state_feature is larger than nonzero_feature_max_size("<<nonzero_feature_max_size<<")");}
    }
  }
}
//-------------------------------------------------------------------------------------------

//! calculate the feature vector over (state,action)
void make_complete_feature (const TSparseFeature &state_feature, const TDiscreteAction &action, SparseMatrix &phi,
    TInt size_of_feature)
{
  // phi= SparseMatrix(wsize,1,nonzero_feature_max_size);
  const TInt row_base=action*size_of_feature;
  for (TSparseFeature::const_iterator itr(state_feature.begin()),last(state_feature.end()); itr!=last; ++itr)
    phi(row_base+itr->first,0)= itr->second;
}
//-------------------------------------------------------------------------------------------

void feature_to_sparse_feature (const TRealVector &feature, TSparseFeature &sfeature,
    const TReal &nonzero_feature_threshold, TInt nonzero_feature_max_size)
{
  TInt count=0, row=0;
  for (TypeExtS<TRealVector>::const_iterator itr(GenBegin(feature)),last(GenEnd(feature));
      itr!=last; ++itr,++row)
  {
    if (*itr>nonzero_feature_threshold)
    {
      if (count<nonzero_feature_max_size)
      {
        sfeature.push_back(TFeatureElement(row,*itr));
        ++count;
      }
      else
        {LWARNING("number of nonzero elements ("<<count<<") of feature is larger than nonzero_feature_max_size("<<nonzero_feature_max_size<<")");}
    }
  }
}
//-------------------------------------------------------------------------------------------

void solve_singularity_handler (double rcond)
{
  LWARNING("(in MLSPI_TDiscreteAction) liboctave fatal: SparseMatrix::solve matrix singular to machine precision, rcond= "<<rcond);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// struct TLSPISample
//-------------------------------------------------------------------------------------------
TLSPISample::TLSPISample (const TSingleReward v_reward, const TDiscreteAction &v_old_action, const TRealVector &old_feature, const TRealVector &next_feature,
                          const TReal &nonzero_feature_threshold, TInt nonzero_feature_max_size)
{
  reward= v_reward;
  old_action= v_old_action;
  feature_to_sparse_feature (old_feature, old_sfeature, nonzero_feature_threshold, nonzero_feature_max_size);
  feature_to_sparse_feature (next_feature, next_sfeature, nonzero_feature_threshold, nonzero_feature_max_size);
}
//-------------------------------------------------------------------------------------------

void add_to_sample (TLSPIData &data, const TLSPISample &sample, TInt size_of_feature, TInt max_data_size_per_key)
{
  TInt key= sample.old_action*size_of_feature + std::max_element(sample.old_sfeature.begin(), sample.old_sfeature.end())->first;
  data.insert(key,sample);
  if (static_cast<int>(data.size(key))>max_data_size_per_key)
  {
    data.erase (data.begin(key));
    // LDEBUG("erased: "<<key);
  }
}
//-------------------------------------------------------------------------------------------

void apply_LSTDQ (const TLSPIData &data, TRealVector &W, const TReal &Gamma, TInt size_of_feature, TInt size_of_action,
    TInt nonzero_feature_size, TInt nonzero_feature_max_size, TInt max_A_capacity,
    TMatrixSolver &matrix_solver, const TReal &small_value=1.0e-6)
{
  const TInt wsize(size_of_feature*size_of_action);

  SparseMatrix  lspi_A (wsize,wsize,  std::min(Square(size_of_action*nonzero_feature_size),max_A_capacity));
  SparseMatrix  lspi_b (wsize,1,  wsize);
  SparseMatrix  tmp_old_phi;
  SparseMatrix  tmp_next_phi;

  // to avoid "liboctave error: SparseMatrix::solve numeric factorization failed", the next code is needed
  for (TInt r(0); r<wsize; ++r)  lspi_A(r,r)= small_value;

  TRealVector Qs;
  GenResize(Qs, size_of_action);

  for (TLSPIData::const_iterator data_itr(data.begin()),data_last(data.end()); data_itr!=data_last; ++data_itr)
  {
    const TLSPISample &sample(*data_itr);
    //! \todo FIXME: following two codes are not efficient since memory is re-allocated
    tmp_old_phi= SparseMatrix(wsize,1,nonzero_feature_max_size);
    tmp_next_phi= SparseMatrix(wsize,1,nonzero_feature_max_size);

    // calculate tmp_next_action with the current policy
    TDiscreteAction tmp_next_action(-1);
    calc_action_value (W, sample.next_sfeature, Qs, size_of_feature);
    tmp_next_action= max_element_index (GenBegin(Qs),GenEnd(Qs));

    // calculate the feature vector over (state,action)
    make_complete_feature (sample.old_sfeature, sample.old_action, tmp_old_phi, size_of_feature);
    make_complete_feature (sample.next_sfeature, tmp_next_action, tmp_next_phi, size_of_feature);

    //! \todo TODO: make the following code efficient
    lspi_A+= tmp_old_phi * (tmp_old_phi - Gamma*tmp_next_phi).transpose();
    lspi_b+= tmp_old_phi * sample.reward;
  }

// LDEBUG("lspi_b="<<endl<<lspi_b);
/*dbg*/std::cout<<"lspi_A.nnz()="<<lspi_A.nnz()<<",  data.size()="<<data.size()<<endl;

  if (matrix_solver==msSparse)
  {
    int info(0); double rcond;
    W= lspi_A.solve(lspi_b,info,rcond,solve_singularity_handler).matrix_value().column(0);
  }
  else if (matrix_solver==msSVD)
  {
    SVD svd(lspi_A.matrix_value());
    DiagMatrix isigma(lspi_A.rows(),lspi_A.cols());
    for(int r(0);r<lspi_A.rows();++r)
      if(fabs(svd.singular_values()(r,r))>small_value)  isigma(r,r)=1.0/svd.singular_values()(r,r);
      else  isigma(r,r)=0.0;
    // std::cout<<"isigma="<<endl<<isigma.diag().transpose()<<endl;
    W= (svd.right_singular_matrix()*isigma*(svd.left_singular_matrix().transpose()*lspi_b)).column(0);
  }
  else {LERROR("invalid matrix_solver: "<<(int)matrix_solver); lexit(df);}

/*dbg*/std::cout<<"W="<<endl<<W.transpose()<<endl;
}
//-------------------------------------------------------------------------------------------

void apply_LSTDQ_opt (const TLSPIData &data, TRealVector &W, const TReal &Gamma, TInt size_of_feature, TInt size_of_action,
    TInt nonzero_feature_size, TInt nonzero_feature_max_size, TInt max_A_capacity, const TReal &large_value=1.0e+6)
{
  const TInt wsize(size_of_feature*size_of_action);

  SparseMatrix  lspi_B (wsize,wsize,  std::min(Square(size_of_action*nonzero_feature_size),max_A_capacity));
  SparseMatrix  lspi_b (wsize,1,  wsize);
  SparseMatrix  tmp_old_phi;
  SparseMatrix  tmp_next_phi;

  for (TInt r(0); r<wsize; ++r)  lspi_B(r,r)= large_value;

  TRealVector Qs;
  GenResize(Qs, size_of_action);

  for (TLSPIData::const_iterator data_itr(data.begin()),data_last(data.end()); data_itr!=data_last; ++data_itr)
  {
    const TLSPISample &sample(*data_itr);
    //! \todo FIXME: following two codes are not efficient since memory is re-allocated
    tmp_old_phi= SparseMatrix(wsize,1,nonzero_feature_max_size);
    tmp_next_phi= SparseMatrix(wsize,1,nonzero_feature_max_size);

    // calculate tmp_next_action with the current policy
    TDiscreteAction tmp_next_action(-1);
    calc_action_value (W, sample.next_sfeature, Qs, size_of_feature);
    tmp_next_action= max_element_index (GenBegin(Qs),GenEnd(Qs));

    // calculate the feature vector over (state,action)
    make_complete_feature (sample.old_sfeature, sample.old_action, tmp_old_phi, size_of_feature);
    make_complete_feature (sample.next_sfeature, tmp_next_action, tmp_next_phi, size_of_feature);

    //! \todo TODO: make the following code efficient
    lspi_B+= lspi_B * tmp_old_phi * (tmp_old_phi - Gamma*tmp_next_phi).transpose() * lspi_B
            * (-1.0 / (1.0 + ((tmp_old_phi - Gamma*tmp_next_phi).transpose() * lspi_B * tmp_old_phi).elem(0,0)));
    lspi_b+= tmp_old_phi * sample.reward;
  }

/*dbg*/std::cout<<"lspi_B.nnz()="<<lspi_B.nnz()<<",  data.size()="<<data.size()<<endl;
  W= (lspi_B*lspi_b).matrix_value().column(0);
/*dbg*/std::cout<<"W="<<endl<<W.transpose()<<endl;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MLSPI_TDiscreteAction
//===========================================================================================

override void MLSPI_TDiscreteAction::slot_initialize_exec (void)
{
  // mem_.EpisodeNumber= 0;
  return_in_episode_= 0.0l;
  is_end_of_episode_= false;
  is_active_= false;
  current_action_value_= 0.0l;
  initializeW();
}
//-------------------------------------------------------------------------------------------

override void MLSPI_TDiscreteAction::slot_start_episode_exec (void)
{
  return_in_episode_= 0.0l;
  actions_in_episode_= 0;
  current_action_value_= 0.0l;
  reset_episode();
  is_end_of_episode_= false;
  is_active_= true;
  current_action_value_= GenAt(nextQs,next_action);
  signal_execute_action.ExecAll (select_action());
}
//-------------------------------------------------------------------------------------------

override void MLSPI_TDiscreteAction::slot_finish_episode_exec (void)
{
  if (is_active_)
    is_end_of_episode_= true;
}
//-------------------------------------------------------------------------------------------

override void MLSPI_TDiscreteAction::slot_finish_episode_immediately_exec (void)
{
  if (is_active_)
  {
    is_end_of_episode_= true;
    TParent::slot_finish_action.Exec();
  }
}
//-------------------------------------------------------------------------------------------

override void MLSPI_TDiscreteAction::slot_finish_action_exec (void)
{
  if (is_active_)
  {
    /*! reward */
    TSingleReward reward (0.0l);
    if (is_updatable())  reward= get_reward();

    /*! select next action; since it is used in Sarsa */
    prepare_next_action ();

    if (!is_end_of_episode_)
    {
      current_action_value_= GenAt(nextQs,next_action);
      signal_execute_action.ExecAll (next_action);
    }

    update (reward);
    select_action();
    ++actions_in_episode_;

    signal_end_of_action.ExecAll();

    if (is_end_of_episode_)
    {
      ++mem_.EpisodeNumber;
      is_active_= false;
      signal_end_of_episode.ExecAll();
    }
  }
}
//-------------------------------------------------------------------------------------------

/*virtual*/void MLSPI_TDiscreteAction::out_state_value_get (const TFeature &phi, TValue &v) const
{
  TRealVector Qs;
  GenResize(Qs, getActionCount());
  calc_action_value (mem_.W, phi, Qs);
  v= *std::max_element (GenBegin(Qs),GenEnd(Qs));
}
//-------------------------------------------------------------------------------------------

/*virtual*/void MLSPI_TDiscreteAction::out_action_value_get (const TFeature &phi, TRealVector &v) const
{
  TRealVector Qs;
  GenResize(Qs, getActionCount());
  calc_action_value (mem_.W, phi, Qs);
  GenResize(v, getActionCount());
  std::copy (GenBegin(Qs),GenEnd(Qs), GenBegin(v));
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// MLSPI_TDiscreteAction implementation of protected member functions
//===========================================================================================

inline TReal MLSPI_TDiscreteAction::get_tau (void) const
{
  TReal tau (0.0l);
  switch (conf_.PolicyImprovement)
  {
    case piConst              :
          tau= conf_.Tau;
          break;
    case piExpReduction       :
          tau= conf_.Tau * real_exp (-conf_.TauDecreasingFactor * static_cast<TReal>(mem_.EpisodeNumber));
          break;
    default  :  LERROR("invalid PolicyImprovement "<<static_cast<int>(conf_.PolicyImprovement)); lexit(df);
  }
  return ApplyRange(tau,conf_.TauMin,conf_.Tau);
}
//-------------------------------------------------------------------------------------------
inline TReal MLSPI_TDiscreteAction::get_eps (void) const
{
  switch (conf_.PolicyImprovement)
  {
    case piConst              :
          return conf_.Eps;
    case piExpReduction       :
          return conf_.Eps * real_exp (-conf_.EpsDecreasingFactor * static_cast<TReal>(mem_.EpisodeNumber));
    default  :  LERROR("invalid PolicyImprovement "<<static_cast<int>(conf_.PolicyImprovement)); lexit(df);
  }
  return -1.0l;
}
//-------------------------------------------------------------------------------------------


void MLSPI_TDiscreteAction::initializeW (void)
{
  TInt wsize(getFAUnitCount()*getActionCount());
  if (wsize==0)  return;
  if (static_cast<int>(GenSize(mem_.W))<wsize)
    GenResize(mem_.W, wsize, 0.0);
}
//-------------------------------------------------------------------------------------------


//!\brief select \p MLSPI_TDiscreteAction::next_action from real actions and internal actions
void MLSPI_TDiscreteAction::prepare_next_action ()
{
  next_feature= get_feature();

  //! calculate action value at current state x
  GenResize (nextQs, getActionCount());
  calc_action_value (mem_.W, next_feature, nextQs);

  //! select next_action
  if (conf_.ActionSelection==asGreedy)
  {
    next_action = max_element_index (GenBegin(nextQs),GenEnd(nextQs));
    std::fill (OctBegin(next_policy),OctEnd(next_policy),0.0l);
    next_policy(next_action)= 1.0l;
    return;
  }
  else if (conf_.ActionSelection==asEpsGreedy)
    EpsGreedy (get_eps(), GenBegin(nextQs),GenEnd(nextQs), GenBegin(next_policy),GenEnd(next_policy));
  else if (conf_.ActionSelection==asBoltzman)
    Boltzmann  ((next_tau=get_tau()), GenBegin(nextQs),GenEnd(nextQs), GenBegin(next_policy),GenEnd(next_policy));
  else if (conf_.ActionSelection==asHeuristicWeightedBoltzman)
  {
    LERROR("ActionSelection==asHeuristicWeightedBoltzman is not implemented in this version yet.."); lexit(df);
    // calcHeuristicBoltzmanWeight (x, hbweight, &SCACHE);
    // WeightedBoltzmann ((next_tau=get_tau()), GenBegin(nextQs),GenEnd(nextQs),
            // hbweight.begin(), hbweight.end(),
            // GenBegin(next_policy),GenEnd(next_policy));
  }

  next_action= SelectActionFromPolicy(GenBegin(next_policy),GenEnd(next_policy));
}
//-------------------------------------------------------------------------------------------


void MLSPI_TDiscreteAction::reset_episode () //! execute this function to start new episode
{
  // TParent::reset_episode(x,dt);

  TInt wsize(getFAUnitCount()*getActionCount());

  if (wsize==0)  {LERROR("getFAUnitCount()*getActionCount()==0"); lexit(df);}

  initializeW();

  next_policy.resize (getActionCount(),0.0l);
  next_tau= 0.0l;

  prepare_next_action();
}
//-------------------------------------------------------------------------------------------

MLSPI_TDiscreteAction::TAction MLSPI_TDiscreteAction::select_action ()
{
  //! start new state (\in func_approx) transition
  old_action=next_action;  // NOTE next_action has been selected in prepare_next_action(x)
  old_policy=next_policy;
  oldQs=nextQs;
  old_feature= next_feature;
  old_tau= next_tau;

  return old_action;
}
//-------------------------------------------------------------------------------------------

bool MLSPI_TDiscreteAction::update (const TSingleReward &reward)
{
  // if (!learningFromOthersExpMode || (learningFromOthersExpMode && isThisBehaviorLM))
    // TParent::update (ares);

  return_in_episode_+= reward;

  /*! update each module with LSPI */
  if (is_updatable())
  {
    add_to_sample(
        lspi_data_,
        TLSPISample(reward, old_action, old_feature, next_feature,
                    conf_.NonzeroFeatureThreshold, conf_.NonzeroFeatureMaxSize),
        GenSize(old_feature), conf_.MaxDataSizePerDim);
  }

  if (is_end_of_episode_)
  {
    if (is_updatable())
    {
      if ((mem_.EpisodeNumber+1)%conf_.LSPICycle==0)
      {
        // LSTD-Q
        for (int i(0); i<conf_.LSTDQIterations; ++i)
        {
          switch (conf_.LSTDQKind)
          {
          case lkDirect:
            apply_LSTDQ (lspi_data_, mem_.W, conf_.Gamma, getFAUnitCount(), getActionCount(),
                          conf_.NonzeroFeatureSize, conf_.NonzeroFeatureMaxSize, conf_.MaxACapacity, conf_.MatrixSolver);
            break;
          case lkRecursive:
            apply_LSTDQ_opt (lspi_data_, mem_.W, conf_.Gamma, getFAUnitCount(), getActionCount(),
                              conf_.NonzeroFeatureSize, conf_.NonzeroFeatureMaxSize, conf_.MaxACapacity);
            break;
          default:
            LERROR("invalid conf_.LSTDQKind: "<<(int)conf_.LSTDQKind);
            lexit(df);
          }
        }
      }
    }
    return true;
  }
  return false;
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MLSPI_TDiscreteAction)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
} // end of lspi_discaction_detail
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

