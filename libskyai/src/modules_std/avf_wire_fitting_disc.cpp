//-------------------------------------------------------------------------------------------
/*! \file    avf_wire_fitting_disc.cpp
    \brief   libskyai - wire-fitting function approximator module for action value function
              over continuous state-action space (Grid-base, DCOB) (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Dec.17, 2009-

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
#include <skyai/modules_std/avf_wire_fitting_disc.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace avf_wire_fitting_detail
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class TAVFWireFittingGridBaseConfigurations
//===========================================================================================

override void MAVFWireFittingGridBase::slot_initialize_exec (void)
{
  LASSERT1op1(conf_.ActionMax.length(),==,conf_.ActionDim);
  LASSERT1op1(conf_.ActionMin.length(),==,conf_.ActionDim);
  LASSERT1op1(static_cast<int>(conf_grid_.Levels.size()),==,conf_.ActionDim);

  // generate center_of_u_
  {
    TGridGenerator   grid;
    grid.Init (GenBegin(conf_grid_.Levels), GenEnd(conf_grid_.Levels),
               GenBegin(conf_.ActionMax),
               GenBegin(conf_.ActionMin));
    TRealVector u_i(conf_.ActionDim);
    center_of_u_.clear();
    for (grid.Init(); grid.Cont(); grid.Increment())
    {
      grid.CurrentContVector (GenBegin(u_i));
      center_of_u_.push_back (u_i);
    }
  }

  if (param_.BFSize()*param_.CtrlDim()*param_.WireSize()==0)
  {
    // parameter initialization...

    param_.Init (get_feature_dim(), conf_.ActionDim, /*wire size=*/center_of_u_.size());

    LASSERT1op1(param_.BFSize(),>,0);
    LASSERT1op1(param_.CtrlDim(),>,0);
    LASSERT1op1(param_.WireSize(),>,0);

    std::vector<TRealVector>::const_iterator  cou_itr (center_of_u_.begin());
    for (std::vector<TRealVector>::iterator desitr (GenBegin(param_.CtrlVec)); desitr!=GenEnd(param_.CtrlVec); ++desitr, ++cou_itr)
    {
      *desitr = *cou_itr;
    }
  }

  LASSERT1op1(param_.WireSize(),==,static_cast<int>(center_of_u_.size()));

  policy.resize (param_.WireSize());

  init_gauss_noise();
}
//-------------------------------------------------------------------------------------------

override void MAVFWireFittingGridBase::slot_add_to_parameter_exec (const TParameter &diff)
{
  param_+= diff;

  // constrain the parameters
  if (conf_grid_.GConstraintKind==gckEllipseOnGrid || conf_grid_.GConstraintKind==gckEoGMinMax)
  {
    if (conf_grid_.Radiuses.length()!=conf_.ActionDim)
      {LERROR("for conf_grid_.GConstraintKind=="<<ConvertToStr(conf_grid_.GConstraintKind)
        <<", conf_grid_.Radiuses.length() should be "<<conf_.ActionDim);
        LDBGVAR(conf_grid_.Radiuses.length()); lexit(df);}
    TRealVector du(conf_.ActionDim);
    std::vector<TRealVector>::iterator  cou_itr (center_of_u_.begin());
    for (std::vector<TRealVector>::iterator desitr (GenBegin(param_.CtrlVec)); desitr!=GenEnd(param_.CtrlVec); ++desitr, ++cou_itr)
    {
      du = *desitr - *cou_itr;
      for (double *duptr(GenBegin(du)), *r_itr(GenBegin(conf_grid_.Radiuses)),
                  *qptr(GenBegin(*desitr)), *ci_itr(GenBegin(*cou_itr));
          duptr!=GenEnd(du); ++duptr,++r_itr, ++qptr,++ci_itr)
      {
        if ((*r_itr)>0.0)
          (*duptr)=(*duptr)/(*r_itr);
        else
        {
          (*duptr)=0.0;
          *qptr = *ci_itr;
        }
      }
      double norm= GetNorm(du);
      if (norm>1.0)
      {
        du*=(1.0/norm);
        for (double *duptr(GenBegin(du)), *r_itr(GenBegin(conf_grid_.Radiuses)); duptr!=GenEnd(du); ++duptr,++r_itr)
          (*duptr)=(*duptr)*(*r_itr);
        *desitr = *cou_itr + du;
      }

      // if (conf_grid_.GConstraintKind==gckEoGMinMax)
      // {
        // const double *amaxptr(OctBegin(conf_.ActionMax)), *aminptr(OctBegin(conf_.ActionMin));
        // for (double *qptr(OctBegin(*desitr)); qptr!=OctEnd(*desitr); ++qptr,++amaxptr,++aminptr)
          // *qptr = ApplyRange(*qptr, *aminptr, *amaxptr);
      // }
    }
    if (conf_grid_.GConstraintKind==gckEoGMinMax)
      apply_min_max_constraint_to_des_i();
  }
  else if (conf_grid_.GConstraintKind==gckMinMax)
  {
    // for (std::vector<TRealVector>::iterator desitr (GenBegin(CtrlVec)); desitr!=GenEnd(param_.CtrlVec); ++desitr)
    // {
      // const double *amaxptr(OctBegin(conf_.ActionMax)), *aminptr(OctBegin(conf_.ActionMin));
      // for (double *qptr(OctBegin(*desitr)); qptr!=OctEnd(*desitr); ++qptr,++amaxptr,++aminptr)
        // *qptr = ApplyRange(*qptr, *aminptr, *amaxptr);
    // }
    apply_min_max_constraint_to_des_i();
  }
  else if (conf_grid_.GConstraintKind!=gckNone)
    {LERROR("invalid conf_grid_.GConstraintKind: "<<(int)conf_grid_.GConstraintKind); lexit(df);}
}
//-------------------------------------------------------------------------------------------




//===========================================================================================
// class MAVFWireFittingDCOB
//===========================================================================================

override void MAVFWireFittingDCOB::slot_initialize_exec (void)
{
  LASSERT1op1(conf_.ActionMax.length(),==,conf_.ActionMin.length());
  LASSERT1op1(conf_wfdcob_.Interval2Set.cols(),==,2);

  if (param_.BFSize()*param_.CtrlDim()*param_.WireSize()==0)
  {
    // parameter initialization...

    /*!\todo TODO: conf_.ActionDim is not used in MAVFWireFittingDCOB.
            remove it from param_box at the constructor of TAVFWireFittingDCOBConfigurations */

    int action_dim= 1 + get_center_state_set().front().length();
    param_.Init (get_feature_dim(), action_dim, /*wire size=*/get_wire_size());

    LASSERT1op1(param_.BFSize(),>,0);
    LASSERT1op1(param_.CtrlDim(),>,0);
    LASSERT1op1(param_.WireSize(),>,0);

    // std::vector<TRealVector>::const_iterator  cou_itr (center_of_u_.begin());
    // for (std::vector<TRealVector>::iterator desitr (GenBegin(param_.CtrlVec)); desitr!=GenEnd(param_.CtrlVec); ++desitr, ++cou_itr)
    // {
      // *desitr = *cou_itr;
    // }

    /*set des_i*/{
      int  target(0), interval(0);
      for (std::vector<TRealVector>::iterator desitr (GenBegin(param_.CtrlVec));
              desitr!=GenEnd(param_.CtrlVec);   ++desitr, increment_action(target, interval))
      {
        (*desitr)(0)= 0.5l * (conf_wfdcob_.Interval2Set(interval,0) + conf_wfdcob_.Interval2Set(interval,1));
        std::copy (GenBegin(get_center_state_set()[target]), GenEnd(get_center_state_set()[target]), GenBegin(*desitr)+1);
      }
    }
  }

  LASSERT1op1(param_.WireSize(),==,get_wire_size());

  policy.resize (param_.WireSize());

  init_gauss_noise();
}
//-------------------------------------------------------------------------------------------

override void MAVFWireFittingDCOB::slot_add_to_parameter_exec (const TParameter &diff)
{
  param_+= diff;

  // constrain the parameters
  if (conf_wfdcob_.DConstraintKind==dckSpheres || conf_wfdcob_.DConstraintKind==dckSphMinMax)
  {
    LASSERT1op1(get_distance_to_nearest_bf().length(),==,(int)get_center_state_set().size());

    TReal distance;
    des_i_diff.resize (get_center_state_set().front().length());
    int  target(0), interval(0);
    for (std::vector<TRealVector>::iterator desitr (GenBegin(param_.CtrlVec));
            desitr!=GenEnd(param_.CtrlVec);   ++desitr, increment_action(target, interval))
    {
      // constraint on interval:
      if      ((*desitr)(0) < conf_wfdcob_.Interval2Set(interval,0))  (*desitr)(0)= conf_wfdcob_.Interval2Set(interval,0);
      else if ((*desitr)(0) > conf_wfdcob_.Interval2Set(interval,1))  (*desitr)(0)= conf_wfdcob_.Interval2Set(interval,1);

      // constraint on target ((*desitr)(1,2,...)):

      // calculate distance(norm of difference):
      const double  *bf_itr   = GenBegin(get_center_state_set()[target]),
                    *bf_end   = GenEnd(get_center_state_set()[target]);
      double        *qitr     = GenBegin(*desitr)+1,
                    *diff_itr = GenBegin(des_i_diff);
      // calculate difference vector (des_i_diff) and distance between the center of target and *desitr(1,..)
      distance= 0.0l;
      for (; bf_itr!=bf_end; ++bf_itr, ++qitr, ++diff_itr)
      {
        *diff_itr = *qitr - *bf_itr;
        distance += Square(*diff_itr);
      }
      distance= real_sqrt(distance);
      /* if distance is greater than get_distance_to_nearest_bf()(target):
          1. adjust the difference vector (des_i_diff) so that its norm is equal to get_distance_to_nearest_bf()(target)
          2. add the adjusted des_i_diff to the center of target  */
      if (distance > get_distance_to_nearest_bf()(target))
      {
        TReal factor= get_distance_to_nearest_bf()(target)/distance;
        bf_itr   = GenBegin(get_center_state_set()[target]);
        qitr     = GenBegin(*desitr)+1;
        diff_itr = GenBegin(des_i_diff);
        for (; bf_itr!=bf_end; ++bf_itr, ++qitr, ++diff_itr)
          *qitr = *bf_itr + factor * (*diff_itr);
      }
    }

    if (conf_wfdcob_.DConstraintKind==dckSphMinMax)
      apply_min_max_constraint_to_des_i();
  }
  else if (conf_wfdcob_.DConstraintKind==dckMinMax)
  {
    apply_min_max_constraint_to_des_i();
  }
  else if (conf_wfdcob_.DConstraintKind!=dckNone)
    {LERROR("invalid conf_wfdcob_.DConstraintKind: "<<(int)conf_wfdcob_.DConstraintKind); lexit(df);}
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MAVFWireFittingGridBase)
SKYAI_ADD_MODULE(MAVFWireFittingDCOB)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of avf_wire_fitting_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

