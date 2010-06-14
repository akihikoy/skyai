//-------------------------------------------------------------------------------------------
/*! \file    ctrl_tools.cpp
    \brief   liblora - control tool-box (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-

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
#include <lora/ctrl_tools.h>
#include <lora/variable_space.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------


namespace var_space{
  void Register (TTrjGeneratorCondition &cond, TVariableMap &mmap)
  {
    #define ADD(x_member)  AddToVarMap(mmap, #x_member, cond.x_member)
    ADD( DynamicParamUpdate   );
    ADD( MaxDdq               );
    ADD( ZeroUnspecifiedState );
    #undef ADD
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TTrajectoryGeneratorBase
//===========================================================================================
/*virtual*/void TTrajectoryGeneratorBase::Reset (int v_dim)
{
  if (cnd==NULL)
    {LERROR("in TTrajectoryGeneratorBase::Reset: SetCondition "
            "should be executed before running this routine.");lexit(df);}
  if(v_dim>0)
  {
    dim=v_dim;
    qf.resize(dim); dqf.resize(dim); ddqf.resize(dim);
    q0.resize(dim); dq0.resize(dim); ddq0.resize(dim);
    qd.resize(dim); dqd.resize(dim); ddqd.resize(dim);
    cj.resize(dim,trj_order);
  }
  SetZero(ltime); SetZero(Tf); SetZero(TrjInterval);
  SetZero(qf); SetZero(dqf); SetZero(ddqf);
  SetZero(q0); SetZero(dq0); SetZero(ddq0);
  SetZero(qd); SetZero(dqd); SetZero(ddqd);
  in_control=(false);
  init_control=true;
  need_calc_interval=true;
}
//-------------------------------------------------------------------------------------------

//! \brief set target with an interval factor
//! \param [in] target_jstate   target (angle,angvel,angaccl), ColumnVector(dim~dim*3)
//! \param [in] interval_factor Tf=interval_factor*norm()
void TTrajectoryGeneratorBase::SetTargetIF (const ColumnVector &target_jstate, const TReal &interval_factor)
{
  SetTargetTf (target_jstate, 0.0l);
  IntervalFactor = interval_factor;
  need_calc_interval=true;
}
//-------------------------------------------------------------------------------------------

//! \brief set target with Tf (trajectory interval in time)
//! \param [in] target_jstate   target (angle,angvel,angaccl), ColumnVector(dim~dim*3)
//! \param [in] interval        Tf=interval (0 is possible, but set by SetTrjInterval)
void TTrajectoryGeneratorBase::SetTargetTf (const ColumnVector &target_jstate, const TReal &interval)
{
  const int tjdim(target_jstate.dim1());
  if(tjdim<dim)
    {LERROR("fatal! target_jstate.dim1()("<<tjdim<<") is less than dim("<<dim<<")");}
  IntervalFactor = 0.0l;
  ltime = 0.0l;
  Tf  = interval;
  TrjInterval = interval;
  for(int j(0); j<dim; ++j)
  {
    qf(j)   = ((j<tjdim)       ? target_jstate(j) : 0.0);
    dqf(j)  = ((dim+j<tjdim)   ? target_jstate(dim+j) : 0.0);
    ddqf(j) = ((2*dim+j<tjdim) ? target_jstate(2*dim+j) : 0.0);
  }
  in_control = true;
  init_control = true;
  need_calc_interval=false;
}
//-------------------------------------------------------------------------------------------

const ColumnVector& TTrajectoryGeneratorBase::Step (const ColumnVector &jstate, const TReal &dt)
  /*! return the next target joint angle (after dt)
      \param [in] jstate  current joint-state(angle,angvel,angaccl) ColumnVector (dim~dim*3)
      \param [in] dt      time step of simulation  */
{
  const int jsdim(jstate.dim1());
  // NOTE if dimenstion of jstate is not enough: internal state is used (e.g. ddq0).
  for (int j(0); j<dim; ++j)  // WARNING is this part not needed for cnd->DynamicParamUpdate==false?
  {
    if(j<jsdim)                            q0(j)   = jstate(j);
    else if(cnd->ZeroUnspecifiedState)     q0(j)   = 0.0l;
    if(dim+j<jsdim)                        dq0(j)  = jstate(dim+j);
    else if(cnd->ZeroUnspecifiedState)     dq0(j)  = 0.0l;
    if(2*dim+j<jsdim)                      ddq0(j) = jstate(2*dim+j);
    else if(cnd->ZeroUnspecifiedState)     ddq0(j) = 0.0l;
  }
  if (init_control && need_calc_interval)
  {
    // TReal pose_error(0.0);
    // for (int j(0); j<dim; ++j)
      // pose_error = std::max(pose_error, static_cast<TReal>(real_fabs(qf(j)-q0(j))));
    // TrjInterval = (pose_error) * IntervalFactor;
    TrjInterval= CalcIntervalFromFactor(dim,q0,qf,IntervalFactor);
    Tf = TrjInterval;
    // init_control = false;
    need_calc_interval= false;
  }
  if (Tf < 0.5l*dt)  {Tf = dt; in_control = false;}
  if (init_control)
  {
    update_param();
    init_control = false;
  }
  else if (cnd->DynamicParamUpdate)
    update_param();

  // calculate desired trajectory
  /* update polyDT{,diff,diff2} */{
    TReal t_dis;
    if (cnd->DynamicParamUpdate)  t_dis=dt;
    else                            t_dis=ltime;
    if (old_t_dis!=t_dis)
    {
      get_poly(t_dis);
      old_t_dis=t_dis;
    }
  }
  qd   = cj * polyDT;
  dqd  = cj * polyDTdiff;
  ddqd = cj * polyDTdiff2;
  if (static_cast<int>(cnd->MaxDdq.size())>=dim)
  {
    for (int j(0); j<dim; ++j)
      if (cnd->MaxDdq[j]>0.0l)
        ddqd(j)=ApplyRange(ddqd(j),-cnd->MaxDdq[j],cnd->MaxDdq[j]);
  }

  q0   = qd;
  dq0  = dqd;
  ddq0 = ddqd;
  Tf -= dt;
  ltime += dt;
  if (Tf < DBL_TINY)
    in_control = false;
  return qd;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class TJerkMinTrjGenerator : public TTrajectoryGeneratorBase
//===========================================================================================
override void TJerkMinTrjGenerator::update_param (void)
{
  for (int c(0); c<dim; ++c)
  {
    cj(c,0) = q0(c);
    cj(c,1) = dq0(c);
    cj(c,2) = 0.5l*ddq0(c);
    //*dbg*/LDEBUG(qf.transpose());
    TReal iTf = 1.0l/Tf;
    TReal A1  = (((qf(c)-cj(c,0))*iTf-cj(c,1))*iTf-cj(c,2))*iTf;
    TReal A2  = ((dqf(c)-cj(c,1))*iTf-2.0l*cj(c,2))*iTf;
    TReal A3  = (ddqf(c)-2.0l*cj(c,2))*iTf;
    cj(c,3) = 10.0l*A1-4.0l*A2+0.5l*A3;
    cj(c,4) = (7.0l*A2-15.0l*A1-A3)*iTf;
    cj(c,5) = (0.5l*A3-3.0l*A2+6.0l*A1)*Square(iTf);
    //*dbg*/LDEBUG(iTf<<" "<<A1<<" "<<A2<<" "<<A3<<" ;  Tf= "<<Tf);
  }
  //*dbg*/LDEBUG(cj<<std::endl);
}
//-------------------------------------------------------------------------------------------

override void TJerkMinTrjGenerator::get_poly (const TReal &v_time)
{
  polyDT(0)=1.0;
  for(int c(1);c<trj_order;++c)
    polyDT(c)=polyDT(c-1)*v_time;
  polyDTdiff(0)=0.0;
  for(int c(1);c<trj_order;++c)
    polyDTdiff(c)=polyDT(c-1)*static_cast<double>(c);
  polyDTdiff2(0)=0.0;
  polyDTdiff2(1)=0.0;
  polyDTdiff2(2)=2.0;
  polyDTdiff2(3)=6.0*polyDT(1);
  polyDTdiff2(4)=12.0*polyDT(2);
  polyDTdiff2(5)=20.0*polyDT(3);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class TAccMinTrjGenerator : public TTrajectoryGeneratorBase
//===========================================================================================
override void TAccMinTrjGenerator::update_param (void)
{
  for (int c(0); c<dim; ++c)
  {
    TReal tmp1 = qf(c)-q0(c)-dq0(c)*Tf;
    TReal tmp2 = dqf(c)-dq0(c);
    TReal iTf  = 1.0l / Tf;
    TReal ci = (3.0l*tmp1*iTf - tmp2)*iTf;
    TReal di = (-2.0l*tmp1*iTf + tmp2)*Square(iTf);
    cj(c,0) = q0(c);
    cj(c,1) = dq0(c);
    cj(c,2) = ci;
    cj(c,3) = di;
  }
}
//-------------------------------------------------------------------------------------------

override void TAccMinTrjGenerator::get_poly (const TReal &v_time)
{
  polyDT(0)=1.0;
  for(int c(1);c<trj_order;++c)
    polyDT(c)=polyDT(c-1)*v_time;
  polyDTdiff(0)=0.0;
  for(int c(1);c<trj_order;++c)
    polyDTdiff(c)=polyDT(c-1)*static_cast<double>(c);
  polyDTdiff2(0)=0.0;
  polyDTdiff2(1)=0.0;
  polyDTdiff2(2)=2.0;
  polyDTdiff2(3)=6.0*polyDT(1);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class TAccMinTrjGenerator2 : public TAccMinTrjGenerator
//===========================================================================================
override void TAccMinTrjGenerator2::update_param (void)
{
  for (int c(0); c<dim; ++c)
  {
    TReal iTf= 1.0l / Tf;
    TReal A1= iTf* (qf(c)-q0(c));
    TReal A2= dqf(c);
    TReal A3= ddqf(c)*Tf;
    cj(c,0)= q0(c);
    cj(c,1)= 0.5l* (6.0l*A1 - 4.0l*A2 + A3);
    cj(c,2)= iTf* (-3.0l*A1 + 3.0l*A2 - A3);
    cj(c,3)= 0.5l*iTf*iTf* (2.0l*A1 - 2.0l*A2 + A3);
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
