//-------------------------------------------------------------------------------------------
/*! \file    ctrl_tools.h
    \brief   liblora - control tool-box (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    2008-

    Copyright (C) 2008, 2010  Akihiko Yamaguchi

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
#ifndef loco_rabbits_ctrl_tools_h
#define loco_rabbits_ctrl_tools_h
//-------------------------------------------------------------------------------------------
#include <lora/octave.h>
#include <lora/small_classes.h>
#include <lora/variable_space_fwd.h>
#include <vector>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//! \brief PD controller class
class TPDController
{
private:
  int dim;
  ColumnVector u;
public:
  ColumnVector Kp, Kd;
  ColumnVector UMax;
  TPDController(int v_d, const double &v_kp, const double &v_kd, const double &v_umax)
    : dim(v_d), u(dim,0.0), Kp(dim,v_kp), Kd(dim,v_kd), UMax(dim,v_umax) {};
  const ColumnVector& operator() (const ColumnVector &jstate/*angle+angvel*/, const ColumnVector &target_angle)
    {
      for(int j(0); j<dim; ++j)
      {
        u(j)=Kp(j)*(target_angle(j)-jstate(j))-Kd(j)*jstate(dim+j);
        if(UMax(j)>0.0)  u(j)=ApplyRange(u(j), -UMax(j), UMax(j));
      }
      return u;
    };
};
//-------------------------------------------------------------------------------------------


struct TTrjGeneratorCondition
{
  bool                 DynamicParamUpdate;
  std::vector<double>  MaxDdq; //!< Upper limit of the angular acceleration (internal state); Invalid if negative or zero
  bool                 ZeroUnspecifiedState;  /*!< true: in TTrajectoryGeneratorBase::Step,
                                                  if jstate's dimension is smaller than required,
                                                  use zero instead of the internal state */

  virtual void InitParam (int joint_dim)
    {
      MaxDdq.resize(joint_dim,-1.0l);
    };

  TTrjGeneratorCondition (int joint_dim) :
      DynamicParamUpdate   (false),
      MaxDdq               (joint_dim, -1.0l),
      ZeroUnspecifiedState (false)
    {};
};
//-------------------------------------------------------------------------------------------

namespace var_space{
  void Register (TTrjGeneratorCondition &cond, TVariableMap &mmap);}
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*! \brief trajectory generator base */
class TTrajectoryGeneratorBase
//===========================================================================================
{
protected:
  const TTrjGeneratorCondition  *cnd;
  int           dim;      //! dimension of the trajectory
  const int     trj_order;  //! order of each dimension of the trajectory
  TReal         ltime;
  bool          in_control;
  TReal         IntervalFactor;
  TReal         Tf, TrjInterval;
  ColumnVector  qf, dqf, ddqf;   //!< target joint-state
  ColumnVector  q0, dq0, ddq0;   //!< current joint-state
  ColumnVector  qd, dqd, ddqd;   //!< desired trajectory (==the next joint-state)
  TReal         old_t_dis;
  ColumnVector  polyDT;      //! (1,t,t^2,t^3,t^4,t^5)
  ColumnVector  polyDTdiff;  //! (0,1,2t,3t^2,4t^3,5t^4)
  ColumnVector  polyDTdiff2; //! (0,0,2,6t,12t^2,20t^3)
  bool          init_control;
  bool          need_calc_interval;
  Matrix        cj;
  virtual void update_param (void) = 0;
  virtual void get_poly (const TReal &v_time) = 0;

protected:
  TTrajectoryGeneratorBase (int v_joint_dim, int v_trj_dim)
    : cnd(NULL), dim(v_joint_dim), trj_order(v_trj_dim), ltime(0.0l), in_control(false),
      Tf(0.0l), TrjInterval(0.0l),
      qf(dim,0.0), dqf(dim,0.0), ddqf(dim,0.0),
      q0(dim,0.0), dq0(dim,0.0), ddq0(dim,0.0),
      qd(dim,0.0), dqd(dim,0.0), ddqd(dim,0.0),
      old_t_dis(0.0l),
      polyDT(trj_order,0.0), polyDTdiff(trj_order,0.0), polyDTdiff2(trj_order,0.0),
      init_control(true), need_calc_interval(true), cj(dim,trj_order)
      {};
  TTrajectoryGeneratorBase (const TTrjGeneratorCondition &c, int v_joint_dim, int v_trj_dim)
    : cnd(&c), dim(v_joint_dim), trj_order(v_trj_dim), ltime(0.0l), in_control(false),
      Tf(0.0l), TrjInterval(0.0l),
      qf(dim,0.0), dqf(dim,0.0), ddqf(dim,0.0),
      q0(dim,0.0), dq0(dim,0.0), ddq0(dim,0.0),
      qd(dim,0.0), dqd(dim,0.0), ddqd(dim,0.0),
      old_t_dis(0.0l),
      polyDT(trj_order,0.0), polyDTdiff(trj_order,0.0), polyDTdiff2(trj_order,0.0),
      init_control(true), need_calc_interval(true), cj(dim,trj_order)
      {};

public:
  virtual ~TTrajectoryGeneratorBase (void) {};

  /*required*/void SetCondition (const TTrjGeneratorCondition &c)  {cnd= &c;};

  virtual void Reset (int v_dim=0);
  const int&    GetDim (void) const {return dim;};
  const TReal&  Time (void) const {return ltime;};
  bool          IsInControl (void) const {return in_control;};
  const TReal&  GetRemainTime (void) const {return Tf;};
  const TReal&  GetTrjInterval (void) const {return TrjInterval;};

  //! execute after SetTargetTf before Step
  void SetTrjInterval (const TReal &interval)  {TrjInterval=interval; Tf=TrjInterval;}

  //! for backward compatibility
  void SetTarget (const ColumnVector &target_jstate, const TReal &interval_factor)
    {SetTargetIF(target_jstate, interval_factor);}

  //! \brief set target with an interval factor
  //! \param [in] target_jstate   target (angle,angvel,angaccl), ColumnVector(dim~dim*3)
  //! \param [in] interval_factor Tf=interval_factor*norm()
  void SetTargetIF (const ColumnVector &target_jstate, const TReal &interval_factor);

  //! \brief set target with Tf (trajectory interval in time)
  //! \param [in] target_jstate   target (angle,angvel,angaccl), ColumnVector(dim~dim*3)
  //! \param [in] interval        Tf=interval (0 is possible, but set by SetTrjInterval)
  void SetTargetTf (const ColumnVector &target_jstate, const TReal &interval);

  const ColumnVector& GetQd (void)    //! return desired angle. \note this func is valid after executing Step()
    const {return qd;};
  const ColumnVector& GetDqd (void)   //! return desired velocity. \note this func is valid after executing Step()
    const {return dqd;};
  const ColumnVector& getDdqd (void)  //! return desired acceleration. \note this func is valid after executing Step()
    const {return ddqd;};

  const ColumnVector& Step (const ColumnVector &jstate, const TReal &dt);
      /*! return the next target joint angle (after dt)
          \param [in] jstate  current joint-state(angle,angvel,angaccl) ColumnVector (dim~dim*3)
          \param [in] dt      time step of simulation  */

  static TReal CalcIntervalFromFactor (int jdim, const ColumnVector &jangle, const ColumnVector &target_angle, const TReal &interval_factor)
    {
      TReal pose_error(0.0);
      for (int j(0); j<jdim; ++j)
        pose_error = std::max(pose_error, static_cast<TReal>(real_fabs(target_angle(j)-jangle(j))));
      return  (pose_error) * interval_factor;
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*! \brief jerk-minimum trajectory generator */
class TJerkMinTrjGenerator : public TTrajectoryGeneratorBase
//===========================================================================================
{
private:
  override void update_param (void);
  override void get_poly (const TReal &v_time);
public:
  TJerkMinTrjGenerator (int joint_dim)
    : TTrajectoryGeneratorBase(joint_dim,6) {};
  TJerkMinTrjGenerator (const TTrjGeneratorCondition &c, int joint_dim)
    : TTrajectoryGeneratorBase(c, joint_dim,6) {};
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*! \brief acceleration-minimum trajectory generator */
class TAccMinTrjGenerator : public TTrajectoryGeneratorBase
//===========================================================================================
{
private:
  override void update_param (void);
  override void get_poly (const TReal &v_time);
public:
  TAccMinTrjGenerator (int joint_dim)
    : TTrajectoryGeneratorBase(joint_dim,4) {};
  TAccMinTrjGenerator (const TTrjGeneratorCondition &c, int joint_dim)
    : TTrajectoryGeneratorBase(c, joint_dim,4) {};
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*! \brief acceleration-minimum trajectory generator2. (boundary condition: (q0,qf,dqf,ddqf)) */
class TAccMinTrjGenerator2 : public TAccMinTrjGenerator
//===========================================================================================
{
private:
  override void update_param (void);
public:
  TAccMinTrjGenerator2 (int joint_dim)
    : TAccMinTrjGenerator(joint_dim) {};
  TAccMinTrjGenerator2 (const TTrjGeneratorCondition &c, int joint_dim)
    : TAccMinTrjGenerator(c, joint_dim) {};
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_ctrl_tools_h
//-------------------------------------------------------------------------------------------
