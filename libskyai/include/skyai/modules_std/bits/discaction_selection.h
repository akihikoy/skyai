//-------------------------------------------------------------------------------------------
/*! \file    discaction_selection.h
    \brief   libskyai - action selection functions for discrete action-set (general implementation)
    \author  Akihiko Yamaguchi
    \date    Oct.31,2008  For PG, separated the calculation of policy from an action value function and the decision of action from policy
    \date    Mar.01,2010  Modified so that the functions can be used for the case where available actions are limited
    \date    Apr.19,2010  Added functions to calculate a state calue function

    Copyright (C) 2008, 2009, 2010  Akihiko Yamaguchi

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
#ifndef discaction_selection_h
#define discaction_selection_h
//-------------------------------------------------------------------------------------------
#include <algorithm>
#include <lora/common.h>
#include <lora/string.h>
#include <lora/rand.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

namespace disc_action
{
  enum TActionSelection {
    asGreedy=0,
    asEpsGreedy,
    asBoltzman,
    asHeuristicWeightedBoltzman,
    asActionReplay=100 };
}
ENUM_STR_MAP_BEGIN_NS(disc_action, TActionSelection)
  ENUM_STR_MAP_ADD_NS(disc_action, asGreedy                      )
  ENUM_STR_MAP_ADD_NS(disc_action, asEpsGreedy                   )
  ENUM_STR_MAP_ADD_NS(disc_action, asBoltzman                    )
  ENUM_STR_MAP_ADD_NS(disc_action, asHeuristicWeightedBoltzman   )
  ENUM_STR_MAP_ADD_NS(disc_action, asActionReplay                )
ENUM_STR_MAP_END_NS  (disc_action, TActionSelection)
SPECIALIZE_TVARIABLE_TO_ENUM(disc_action::TActionSelection)

namespace disc_action
{


//===========================================================================================
// state value
//===========================================================================================
/*!\brief return the state value for a given action value at a state
   \param[in]  QsFirst~QsLast : action-value at the state s
   \return : maximum Q-value
   \todo test this function (not tested enough)  */
template <typename QFwdItr>
inline typename dereferenced_type<QFwdItr>::type  StateValue (QFwdItr QsFirst, QFwdItr QsLast)
{
  LASSERT(QsFirst!=QsLast);
  return *std::max_element (QsFirst, QsLast);
}
//-------------------------------------------------------------------------------------------
/*!\brief return the state value for a given action value at a state (in case where available actions are limited)
   \param[in]  QsFirst~QsLast : action-value at the state s
   \return : maximum Q-value
   \param[in]  avlactfirst~avlactlast : boolean vector which specifies the set of available action; e.g. (true,false,true,..,true)
   \todo test this function (not tested enough)  */
template <typename QFwdItr, typename AvlActFwdItr>
inline typename dereferenced_type<QFwdItr>::type  StateValue (
    QFwdItr QsFirst, QFwdItr QsLast,
    AvlActFwdItr avlactfirst, AvlActFwdItr avlactlast)
{
  LASSERT(QsFirst!=QsLast);
  LASSERT(avlactfirst!=avlactlast);
  QFwdItr  max_qitr (QsLast);
  for (;QsFirst!=QsLast; ++QsFirst,++avlactfirst)
    if (*avlactfirst)
      {if (max_qitr==QsLast || *QsFirst > *max_qitr)  {max_qitr=QsFirst;}}
  // checksum
  LASSERT(avlactfirst==avlactlast);
  return *max_qitr;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// greedy
//===========================================================================================
/*!\brief Greedy: greedy policy
   \param[in]  QsFirst~QsLast : action-value at the state s
   \param[out] pfirst~plast : calculated policy
   \return : index of maximum Q-value (QsFirst is zero)
   \todo test this function (not tested enough)  */
template <typename QFwdItr, typename PFwdItr>
inline int Greedy (QFwdItr QsFirst, QFwdItr QsLast, PFwdItr pfirst, PFwdItr plast)
{
  LASSERT(QsFirst!=QsLast);
  LASSERT(pfirst!=plast);
  QFwdItr  max_qitr (QsLast);
  PFwdItr  max_pitr (plast);
  int      index(0), max_index(-1);
  std::fill (pfirst,plast,0.0l);
  for (;QsFirst!=QsLast; ++QsFirst,++pfirst,++index)
    if (max_index<0 || *QsFirst > *max_qitr)  {max_qitr=QsFirst; max_pitr=pfirst; max_index=index;}
  *max_pitr= 1.0l;
  // checksum
  LASSERT(pfirst==plast);
  return max_index;
}
//-------------------------------------------------------------------------------------------
/*!\brief Greedy: greedy policy (in case where available actions are limited)
   \param[in]  QsFirst~QsLast : action-value at the state s
   \param[out] pfirst~plast : calculated policy
   \return : index of maximum Q-value (QsFirst is zero)
   \param[in]  avlactfirst~avlactlast : boolean vector which specifies the set of available action; e.g. (true,false,true,..,true)
   \todo test this function (not tested enough)  */
template <typename QFwdItr, typename PFwdItr, typename AvlActFwdItr>
inline int Greedy (
    QFwdItr QsFirst, QFwdItr QsLast,
    PFwdItr pfirst, PFwdItr plast,
    AvlActFwdItr avlactfirst, AvlActFwdItr avlactlast)
{
  LASSERT(QsFirst!=QsLast);
  LASSERT(pfirst!=plast);
  LASSERT(avlactfirst!=avlactlast);
  QFwdItr  max_qitr (QsLast);
  PFwdItr  max_pitr (plast);
  int      index(0), max_index(-1);
  std::fill (pfirst,plast,0.0l);
  for (;QsFirst!=QsLast; ++QsFirst,++pfirst,++index,++avlactfirst)
    if (*avlactfirst)
    {
      if (max_index<0 || *QsFirst > *max_qitr)  {max_qitr=QsFirst; max_pitr=pfirst; max_index=index;}
    }
  *max_pitr= 1.0l;
  // checksum
  LASSERT(pfirst==plast);
  LASSERT(avlactfirst==avlactlast);
  return max_index;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// epsilon-greedy
//===========================================================================================

/*!\brief EpsGreedy: epsilon-greedy policy
   \param[in]  eps  : epsilon
   \param[in]  QsFirst~QsLast : action-value at the state s
   \param[out] pfirst~plast : calculated policy  */
template <typename QFwdItr, typename PFwdItr>
inline void EpsGreedy (const TReal &eps, QFwdItr QsFirst, QFwdItr QsLast, PFwdItr pfirst, PFwdItr plast)
{
  LASSERT(QsFirst!=QsLast);
  LASSERT(pfirst!=plast);
  int acount(0);
  QFwdItr max_itr (QsFirst);
  {
    QFwdItr itr(QsFirst);
    for (++itr, ++acount; itr!=QsLast; ++itr, ++acount)
      if (*itr > *max_itr)  max_itr=itr;
  }
  if(acount==1)
  {
    std::fill (pfirst,plast, 0.0l);
    *pfirst=1.0l;
    return;
  }
  TReal prand= eps/static_cast<TReal>(acount-1);
  for (; pfirst!=plast; ++pfirst,++QsFirst)
    if(QsFirst==max_itr)  *pfirst= 1.0l-eps;
    else                  *pfirst= prand;
  // checksum
  LASSERT(QsFirst==QsLast);
}
//-------------------------------------------------------------------------------------------
/*!\brief EpsGreedy: epsilon-greedy policy (in case where available actions are limited)
   \param[in]  eps  : epsilon
   \param[in]  QsFirst~QsLast : action-value at the state s
   \param[out] pfirst~plast : calculated policy
   \param[in]  avlactfirst~avlactlast : boolean vector which specifies the set of available action; e.g. (true,false,true,..,true)
   \todo test this function (not tested yet)  */
template <typename QFwdItr, typename PFwdItr, typename AvlActFwdItr>
inline void EpsGreedy (
    const TReal &eps,
    QFwdItr QsFirst, QFwdItr QsLast,
    PFwdItr pfirst, PFwdItr plast,
    AvlActFwdItr avlactfirst, AvlActFwdItr avlactlast)
{
  LASSERT(QsFirst!=QsLast);
  LASSERT(pfirst!=plast);
  LASSERT(avlactfirst!=avlactlast);
  int avlacount(0);
  QFwdItr  max_qitr (QsLast);
  PFwdItr  max_pitr (plast);
  {
    QFwdItr       qitr (QsFirst);
    PFwdItr       pitr (pfirst);
    AvlActFwdItr  avlactitr (avlactfirst);
    for (; qitr!=QsLast; ++qitr,++pitr,++avlactitr)
    {
      if (*avlactitr)
      {
        if (avlacount==0 || *qitr > *max_qitr)  {max_qitr=qitr; max_pitr=pitr;}
        ++avlacount;
      }
    }
  }
  if(avlacount==0)  {std::fill (pfirst,plast, 0.0l);  return;}
  if(avlacount==1)
  {
    std::fill (pfirst,plast, 0.0l);
    *max_pitr= 1.0l;
    return;
  }
  TReal prand= eps/static_cast<TReal>(avlacount-1);
  for (; pfirst!=plast; ++pfirst,++QsFirst,++avlactfirst)
  {
    if (*avlactfirst)
    {
      if (QsFirst==max_qitr)  *pfirst= 1.0l-eps;
      else                    *pfirst= prand;
    }
    else
    {
      *pfirst= 0.0l;
    }
  }
  // checksum
  LASSERT(QsFirst==QsLast);
  LASSERT(avlactfirst==avlactlast);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// Boltzmann selection
//===========================================================================================

/*!\brief Boltzmann: Boltzmann (softmax) policy
   \param[in]  tau  : temperature parameter
   \param[in]  QsFirst~QsLast : action-value at the state s
   \param[out] pfirst~plast : calculated policy  */
template <typename QFwdItr, typename PFwdItr>
inline void Boltzmann (const TReal &tau, QFwdItr QsFirst, QFwdItr QsLast, PFwdItr pfirst, PFwdItr plast)
{
  LASSERT(QsFirst!=QsLast);
  LASSERT(pfirst!=plast);
  typename dereferenced_type<QFwdItr>::type maxQ = *std::max_element (QsFirst,QsLast);
  typename dereferenced_type<QFwdItr>::type sumQ(0.0l);
  int acount(0);
  for (QFwdItr itr(QsFirst); itr!=QsLast; ++itr, ++acount)
    sumQ += real_exp ((*itr-maxQ)/tau);
  if (sumQ<DBL_TINY)
  {
    std::fill (pfirst,plast, 1.0l/static_cast<TReal>(acount));
    return;
  }
  for (; pfirst!=plast; ++pfirst,++QsFirst)
    *pfirst= real_exp ((*QsFirst-maxQ)/tau)/sumQ;
  // checksum
  LASSERT(QsFirst==QsLast);
}
//-------------------------------------------------------------------------------------------
/*!\brief Boltzmann: Boltzmann (softmax) policy (in case where available actions are limited)
   \param[in]  tau  : temperature parameter
   \param[in]  QsFirst~QsLast : action-value at the state s
   \param[out] pfirst~plast : calculated policy
   \param[in]  avlactfirst~avlactlast : boolean vector which specifies the set of available action; e.g. (true,false,true,..,true)
   */
template <typename QFwdItr, typename PFwdItr, typename AvlActFwdItr>
inline void Boltzmann (
    const TReal &tau,
    QFwdItr QsFirst, QFwdItr QsLast,
    PFwdItr pfirst, PFwdItr plast,
    AvlActFwdItr avlactfirst, AvlActFwdItr avlactlast)
{
  LASSERT(QsFirst!=QsLast);
  LASSERT(pfirst!=plast);
  LASSERT(avlactfirst!=avlactlast);
  typename dereferenced_type<QFwdItr>::type maxQ(-0.12345);
  typename dereferenced_type<QFwdItr>::type sumQ(0.0l);
  int avlacount(0);
  {
    QFwdItr       qitr (QsFirst);
    AvlActFwdItr  avlactitr (avlactfirst);
    for (; qitr!=QsLast; ++qitr,++avlactitr)
    {
      if (*avlactitr)
      {
        if (avlacount==0 || *qitr > maxQ)  maxQ= *qitr;
        ++avlacount;
      }
    }
  }
  std::fill (pfirst,plast, 0.0l);
  if(avlacount==0)  return;
  {
    TReal tmp_prob= 1.0l/static_cast<TReal>(avlacount);
    PFwdItr       pitr (pfirst);
    AvlActFwdItr  avlactitr (avlactfirst);
    for (QFwdItr qitr(QsFirst); qitr!=QsLast; ++qitr,++pitr,++avlactitr)
      if (*avlactitr)
      {
        sumQ += real_exp ((*qitr-maxQ)/tau);
        *pitr= tmp_prob;  // assign for too small sumQ
      }
  }
  if (sumQ<DBL_TINY)  return;
  for (; pfirst!=plast; ++pfirst,++QsFirst,++avlactfirst)
    if (*avlactfirst)
      *pfirst= real_exp ((*QsFirst-maxQ)/tau)/sumQ;
  // checksum
  LASSERT(QsFirst==QsLast);
  LASSERT(avlactfirst==avlactlast);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// weighted-Boltzmann selection
//===========================================================================================

/*!\brief WeightedBoltzmann: weighted-Boltzmann (softmax) policy
   \param[in]  tau  : computational temparature
   \param[in]  QsFirst~QsLast : action-value at the state s
   \param[in]  wfirst~wlast : weight for each action
   \param[out] pfirst~plast : calculated policy  */
template <typename QFwdItr, typename WFwdItr, typename PFwdItr>
inline void WeightedBoltzmann (const TReal &tau, QFwdItr QsFirst, QFwdItr QsLast, WFwdItr wfirst, WFwdItr wlast, PFwdItr pfirst, PFwdItr plast)
{
  LASSERT(QsFirst!=QsLast);
  LASSERT(pfirst!=plast);
  LASSERT(wfirst!=wlast);
  typename dereferenced_type<QFwdItr>::type maxQ = *std::max_element (QsFirst,QsLast);
  typename dereferenced_type<QFwdItr>::type sumQ(0.0l);
  WFwdItr  witr (wfirst);
  int acount(0);
  for (QFwdItr itr(QsFirst); itr!=QsLast; ++itr,++witr,++acount)
    sumQ += *witr*real_exp((*itr-maxQ)/tau);
  if (sumQ<DBL_TINY)
  {
    std::fill (pfirst,plast, 1.0l/static_cast<TReal>(acount));
    return;
  }
  for (; pfirst!=plast; ++pfirst,++QsFirst,++wfirst)
    *pfirst= *wfirst*real_exp((*QsFirst-maxQ)/tau)/sumQ;
  // checksum
  LASSERT(QsFirst==QsLast);
  LASSERT(wfirst==wlast);
}
//-------------------------------------------------------------------------------------------
/*!\brief WeightedBoltzmann: weighted-Boltzmann (softmax) policy
   \param[in]  tau  : computational temparature
   \param[in]  QsFirst~QsLast : action-value at the state s
   \param[in]  wfirst~wlast : weight for each action
   \param[out] pfirst~plast : calculated policy  */
template <typename QFwdItr, typename WFwdItr, typename PFwdItr, typename AvlActFwdItr>
inline void WeightedBoltzmann (
    const TReal &tau,
    QFwdItr QsFirst, QFwdItr QsLast,
    WFwdItr wfirst, WFwdItr wlast,
    PFwdItr pfirst, PFwdItr plast,
    AvlActFwdItr avlactfirst, AvlActFwdItr avlactlast)
{
  LASSERT(QsFirst!=QsLast);
  LASSERT(pfirst!=plast);
  LASSERT(wfirst!=wlast);
  LASSERT(avlactfirst!=avlactlast);
  typename dereferenced_type<QFwdItr>::type maxQ(-0.12345);
  typename dereferenced_type<QFwdItr>::type sumQ(0.0l);
  int avlacount(0);
  {
    QFwdItr       qitr (QsFirst);
    AvlActFwdItr  avlactitr (avlactfirst);
    for (; qitr!=QsLast; ++qitr,++avlactitr)
    {
      if (*avlactitr)
      {
        if (avlacount==0 || *qitr > maxQ)  maxQ= *qitr;
        ++avlacount;
      }
    }
  }
  std::fill (pfirst,plast, 0.0l);
  if(avlacount==0)  return;
  {
    TReal tmp_prob= 1.0l/static_cast<TReal>(avlacount);
    WFwdItr       witr (wfirst);
    PFwdItr       pitr (pfirst);
    AvlActFwdItr  avlactitr (avlactfirst);
    for (QFwdItr qitr(QsFirst); qitr!=QsLast; ++qitr,++witr,++pitr,++avlactitr)
      if (*avlactitr)
      {
        sumQ += (*witr)*real_exp ((*qitr-maxQ)/tau);
        *pitr= tmp_prob;  // assign for too small sumQ
      }
  }
  if (sumQ<DBL_TINY)  return;
  for (; pfirst!=plast; ++pfirst,++QsFirst,++wfirst,++avlactfirst)
    if (*avlactfirst)
      *pfirst= (*wfirst)*real_exp ((*QsFirst-maxQ)/tau)/sumQ;
  // checksum
  LASSERT(QsFirst==QsLast);
  LASSERT(wfirst==wlast);
  LASSERT(avlactfirst==avlactlast);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// select action from policy
//===========================================================================================

/*!\brief select an (discrete) action from policy
   \param[in] pfirst~plast : policy  */
template <typename PFwdItr>
inline int SelectActionFromPolicy (PFwdItr pfirst, PFwdItr plast)
{
  int action(0);
  TReal p (Rand(1.0l));
  for (; pfirst!=plast; ++pfirst, ++action)
  {
    if (p<=*pfirst)  return action;
    p -= *pfirst;
  }
  return action-1;
}
//-------------------------------------------------------------------------------------------


} // end of disc_action

} // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // discaction_selection_h
//-------------------------------------------------------------------------------------------
