//-------------------------------------------------------------------------------------------
/*! \file    robot_model.cpp
    \brief   liblora - Robot model definition for ODE
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jan.31, 2013

    Copyright (C) 2013  Akihiko Yamaguchi

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
#include <lora/robot_model.h>
#include <lora/variable_binexec.h>
#include <lora/variable_space_impl.h>
#include <lora/ode_ds.h>
#include <fstream>
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


//-------------------------------------------------------------------------------------------
namespace var_space
{
//-------------------------------------------------------------------------------------------
#define ADD(x_member)  AddToVarMap(mmap, #x_member, x.x_member)

void Register (xode::TGeometryParameters &x, TVariableMap &mmap)
{
  ADD( Type         );
  ADD( Lx           );
  ADD( Ly           );
  ADD( Lz           );
  ADD( Radius       );
  ADD( Length       );
  ADD( A            );
  ADD( B            );
  ADD( C            );
  ADD( D            );
  ADD( Color        );
  ADD( Vertices     );
  ADD( Indices      );
  ADD( Directions   );
  ADD( Colors       );
}
void Register (xode::TLinkParameters &x, TVariableMap &mmap)
{
  ADD( Geometry        );
  ADD( InertiaType     );
  ADD( Mass            );
  ADD( Density         );
  ADD( CoMOffset       );
  ADD( InertiaMatrix   );
  ADD( MoveCoMToCenter );
  ADD( Position        );
  ADD( Rotation        );
}
void Register (xode::TStaticLinkParameters &x, TVariableMap &mmap)
{
  ADD( Geometry      );
  ADD( Position      );
  ADD( Rotation      );
}
void Register (xode::TAxisParameters &x, TVariableMap &mmap)
{
  ADD( LoStop        );
  ADD( HiStop        );
  ADD( FMax          );
}
void Register (xode::TJointParameters &x, TVariableMap &mmap)
{
  ADD( Body1         );
  ADD( Body2         );
  ADD( Type          );
  ADD( Axis1         );
  ADD( Axis2         );
  ADD( Anchor        );
  ADD( AParam1       );
  ADD( AParam2       );
  ADD( Visualize     );
  ADD( VizType       );
  ADD( VizRadius     );
  ADD( VizLength     );
  ADD( VizColor      );
}
void Register (xode::TForceSensorParameters &x, TVariableMap &mmap)
{
  Register(static_cast<xode::TLinkParameters&>(x), mmap);
  ADD( Attached      );
}
void Register (xode::TSurfaceParameters &x, TVariableMap &mmap)
{
  ADD( ContactMu2           );
  ADD( ContactFDir1         );
  ADD( ContactBounce        );
  ADD( ContactSoftERP       );
  ADD( ContactSoftCFM       );
  ADD( ContactMotion1       );
  ADD( ContactMotion2       );
  ADD( ContactMotionN       );
  ADD( ContactSlip1         );
  ADD( ContactSlip2         );
  ADD( ContactApprox0       );
  ADD( ContactApprox1_1     );
  ADD( ContactApprox1_2     );
  ADD( ContactApprox1       );
  ADD( Mu                   );
  ADD( Mu2                  );
  ADD( Bounce               );
  ADD( BounceVel            );
  ADD( SoftERP              );
  ADD( SoftCFM              );
  ADD( Motion1              );
  ADD( Motion2              );
  ADD( MotionN              );
  ADD( Slip1                );
  ADD( Slip2                );
}
void Register (xode::TJointManipulator &x, TVariableMap &mmap)
{
  ADD( Name                 );
  ADD( Index                );
}
void Register (xode::TRobotParameters &x, TVariableMap &mmap)
{
  ADD( Links                );
  ADD( Joints               );
  ADD( ForceSensors         );
  ADD( RootLink             );
  ADD( Position             );
  ADD( Rotation             );
  ADD( InitialJointAngles   );
  ADD( JointAngleObserver   );
  ADD( JointAngVelObserver  );
  ADD( JointTorqueAdder     );
  ADD( ForceObserver        );
  ADD( LinkContactObserver  );
}
void Register (xode::TStaticObjectParameters &x, TVariableMap &mmap)
{
  ADD( Links         );
  ADD( Position      );
  ADD( Rotation      );
}
void Register (xode::TWorldParameters &x, TVariableMap &mmap)
{
  ADD( Robots                );
  ADD( StaticObjects         );
  ADD( DefaultSurface        );
  ADD( Gravity               );
  ADD( WorldCFM              );
  ADD( MaxContacts           );
  ADD( TimeStep              );
  ADD( UsingQuickStep        );
  ADD( QuickStepIterationNum );
  ADD( ConsoleMode           );
  ADD( DisplayFPS            );
  ADD( ViewPoint             );
}

#undef ADD
//-------------------------------------------------------------------------------------------
}  // end of var_space
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
namespace xode
{
//-------------------------------------------------------------------------------------------

static void builtin_function_RFromAxisAndAngle(var_space::TBinExecutor &context, var_space::TVariableList &argv)
{
  using namespace loco_rabbits::var_space;
  if (argv.size()!=5)
    {VAR_SPACE_ERR_EXIT("syntax of RFromAxisAndAngle should be list(real ax,real ay,real az,real angle)");}
  TVariableList::iterator itr(argv.begin());
  TVariable &res(*itr); ++itr;
  TVariable &arg1(*itr); ++itr;
  TVariable &arg2(*itr); ++itr;
  TVariable &arg3(*itr); ++itr;
  TVariable &arg4(*itr); ++itr;

  #define _R(R,i,j) ((R)[(i)*4+(j)])
  dMatrix3 R;
  dRFromAxisAndAngle(R, arg1.PrimitiveGetAs<pt_real>(), arg2.PrimitiveGetAs<pt_real>(),
      arg3.PrimitiveGetAs<pt_real>(), arg4.PrimitiveGetAs<pt_real>());
  res.Push().PrimitiveSetBy<pt_real>( _R(R,0,0) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,0,1) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,0,2) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,1,0) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,1,1) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,1,2) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,2,0) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,2,1) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,2,2) );
  #undef _R
}
//-------------------------------------------------------------------------------------------

static void builtin_function_RFromEulerAngles(var_space::TBinExecutor &context, var_space::TVariableList &argv)
{
  using namespace loco_rabbits::var_space;
  if (argv.size()!=4)
    {VAR_SPACE_ERR_EXIT("syntax of RFromEulerAngles should be list(real phi,real theta,real psi)");}
  TVariableList::iterator itr(argv.begin());
  TVariable &res(*itr); ++itr;
  TVariable &arg1(*itr); ++itr;
  TVariable &arg2(*itr); ++itr;
  TVariable &arg3(*itr); ++itr;

  #define _R(R,i,j) ((R)[(i)*4+(j)])
  dMatrix3 R;
  dRFromEulerAngles(R, arg1.PrimitiveGetAs<pt_real>(), arg2.PrimitiveGetAs<pt_real>(), arg3.PrimitiveGetAs<pt_real>());
  res.Push().PrimitiveSetBy<pt_real>( _R(R,0,0) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,0,1) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,0,2) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,1,0) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,1,1) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,1,2) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,2,0) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,2,1) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,2,2) );
  #undef _R
}
//-------------------------------------------------------------------------------------------

static void builtin_function_RFromQ(var_space::TBinExecutor &context, var_space::TVariableList &argv)
{
  using namespace loco_rabbits::var_space;
  if (argv.size()!=5)
    {VAR_SPACE_ERR_EXIT("syntax of RFromQ should be list(real w,real x,real y,real z)");}
  TVariableList::iterator itr(argv.begin());
  TVariable &res(*itr); ++itr;
  TVariable &arg1(*itr); ++itr;
  TVariable &arg2(*itr); ++itr;
  TVariable &arg3(*itr); ++itr;
  TVariable &arg4(*itr); ++itr;

  #define _R(R,i,j) ((R)[(i)*4+(j)])
  dMatrix3 R;
  dQuaternion Q;
  Q[0]= arg1.PrimitiveGetAs<pt_real>();
  Q[1]= arg2.PrimitiveGetAs<pt_real>();
  Q[2]= arg3.PrimitiveGetAs<pt_real>();
  Q[3]= arg4.PrimitiveGetAs<pt_real>();
  dRfromQ(R, Q);
  res.Push().PrimitiveSetBy<pt_real>( _R(R,0,0) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,0,1) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,0,2) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,1,0) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,1,1) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,1,2) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,2,0) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,2,1) );
  res.Push().PrimitiveSetBy<pt_real>( _R(R,2,2) );
  #undef _R
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TWorld
//===========================================================================================

void TWorld::Clear()
{
  time_= 0.0l;
  old_fps_= 0.0l;
  repaint_time_= 0;
  no_paint_step_= 0;

  body_.clear();
  geom_bx_.clear();
  geom_ca_.clear();
  geom_cy_.clear();
  geom_pl_.clear();
  geom_sp_.clear();
  geom_tm_.clear();
  joint_ba_.clear();
  joint_h1_.clear();
  joint_sl_.clear();
  joint_un_.clear();
  joint_h2_.clear();
  joint_fx_.clear();
  joint_feedback_.clear();

  name_to_body_.clear();
  name_to_jfeedback_.clear();
  robot_idxes_.clear();
  idx_to_robotparams_.clear();
  angle_observer_.clear();
  angvel_observer_.clear();
  torque_adder_.clear();
  force_observer_.clear();
  contact_observer_.clear();
  current_contact_.clear();
}
//-------------------------------------------------------------------------------------------

void TWorld::resize_elements()
{
  // Count the numbers of links/joints:
  TElementSize num;
  #define COUNT_GEOM(x_type)                    \
    do{switch(x_type)                           \
    {                                           \
    case gtBox        : ++num.GeomBX; break;    \
    case gtCapsule    : ++num.GeomCA; break;    \
    case gtCylinder   : ++num.GeomCY; break;    \
    case gtPlane      : ++num.GeomPL; break;    \
    case gtSphere     : ++num.GeomSP; break;    \
    case gtTriMesh    : ++num.GeomTM; break;    \
    case gtNone  :                              \
    default      :                              \
      LERROR("Invalid geometry type: "<<static_cast<int>(x_type)); \
      lexit(df);                                \
    }} while(0)
  params_.update();
  int ridx(0);
  for(TNamedParam<TRobotParameters>::IdxMapper::const_iterator ritr(params_.robots_.begin()),rlast(params_.robots_.end()); ritr!=rlast; ++ritr,++ridx)
  {
    (*ritr)->second.update();
    robot_idxes_[(*ritr)->first]= ridx;
    for(TNamedParam<TLinkParameters>::IdxMapper::const_iterator litr((*ritr)->second.links_.begin()),llast((*ritr)->second.links_.end()); litr!=llast; ++litr)
    {
      name_to_body_[(*ritr)->first][(*litr)->first]= num.Body;
      ++num.Body;
      COUNT_GEOM((*litr)->second.Geometry.Type);
    }
    for(TNamedParam<TJointParameters>::IdxMapper::const_iterator jitr((*ritr)->second.joints_.begin()),jlast((*ritr)->second.joints_.end()); jitr!=jlast; ++jitr)
    {
      switch((*jitr)->second.Type)
      {
      case jtBall      : ++num.JointBA; break;
      case jtHinge     : ++num.JointH1; break;
      case jtSlider    : ++num.JointSL; break;
      case jtUniversal : ++num.JointUN; break;
      case jtHinge2    : ++num.JointH2; break;
      case jtFixed     : ++num.JointFX; break;
      case jtNone  :
      default      :
        LERROR("Invalid joint type: "<<static_cast<int>((*jitr)->second.Type));
        lexit(df);
      }
    }
    for(TNamedParam<TForceSensorParameters>::IdxMapper::const_iterator fitr((*ritr)->second.force_sensors_.begin()),flast((*ritr)->second.force_sensors_.end()); fitr!=flast; ++fitr)
    {
      name_to_body_[(*ritr)->first][(*fitr)->first]= num.Body;
      ++num.Body;
      name_to_jfeedback_[(*ritr)->first][(*fitr)->first]= num.JointFeedback;
      ++num.JointFeedback;
      COUNT_GEOM((*fitr)->second.Geometry.Type);
      ++num.JointFX;
    }
  }
  for(TNamedParam<TStaticObjectParameters>::IdxMapper::const_iterator sitr(params_.static_objects_.begin()),slast(params_.static_objects_.end()); sitr!=slast; ++sitr)
  {
    (*sitr)->second.update();
    for(TNamedParam<TStaticLinkParameters>::IdxMapper::const_iterator litr((*sitr)->second.links_.begin()),llast((*sitr)->second.links_.end()); litr!=llast; ++litr)
    {
      COUNT_GEOM((*litr)->second.Geometry.Type);
    }
  }
  #undef COUNT_GEOM

  // Resize:
  body_.resize    (num.Body);
  geom_bx_.resize (num.GeomBX);
  geom_ca_.resize (num.GeomCA);
  geom_cy_.resize (num.GeomCY);
  geom_pl_.resize (num.GeomPL);
  geom_sp_.resize (num.GeomSP);
  geom_tm_.resize (num.GeomTM);
  joint_ba_.resize(num.JointBA);
  joint_h1_.resize(num.JointH1);
  joint_sl_.resize(num.JointSL);
  joint_un_.resize(num.JointUN);
  joint_h2_.resize(num.JointH2);
  joint_fx_.resize(num.JointFX);
  joint_feedback_.resize(num.JointFeedback);

  idx_to_robotparams_.resize(params_.Robots.size());
  angle_observer_.resize(params_.Robots.size());
  angvel_observer_.resize(params_.Robots.size());
  torque_adder_.resize(params_.Robots.size());
  force_observer_.resize(params_.Robots.size());
  contact_observer_.resize(params_.Robots.size());
  current_contact_.resize(params_.Robots.size());

  ridx= 0;
  for(TNamedParam<TRobotParameters>::IdxMapper::const_iterator ritr(params_.robots_.begin()),rlast(params_.robots_.end()); ritr!=rlast; ++ritr,++ridx)
  {
    idx_to_robotparams_[ridx]= &((*ritr)->second);
  }
}
//-------------------------------------------------------------------------------------------

dGeomID TWorld::create_geometry(TGeometryParameters &geom, TElementSize &idx)
{
  dGeomID res(NULL);
  switch(geom.Type)
  {
  case gtBox        :
    geom_bx_[idx.GeomBX].create(space_.id(), geom.Lx, geom.Ly, geom.Lz);
    res= geom_bx_[idx.GeomBX].id();
    ++idx.GeomBX; break;
  case gtCapsule    :
    geom_ca_[idx.GeomCA].create(space_.id(), geom.Radius, geom.Length);
    res= geom_ca_[idx.GeomCA].id();
    ++idx.GeomCA; break;
  case gtCylinder   :
    geom_cy_[idx.GeomCY].create(space_.id(), geom.Radius, geom.Length);
    res= geom_cy_[idx.GeomCY].id();
    ++idx.GeomCY; break;
  case gtPlane      :
    geom_pl_[idx.GeomPL].create(space_.id(), geom.A, geom.B, geom.C, geom.D);
    res= geom_pl_[idx.GeomPL].id();
    ++idx.GeomPL; break;
  case gtSphere     :
    geom_sp_[idx.GeomSP].create(space_.id(), geom.Radius);
    res= geom_sp_[idx.GeomSP].id();
    ++idx.GeomSP; break;
  case gtTriMesh    :
    LASSERT1op1(geom.Vertices.size()%3,==,0);
    LASSERT1op1(geom.Indices.size()%3,==,0);
    if(geom.Directions.size()>0)  {LASSERT1op1(geom.Indices.size()/3,==,geom.Directions.size());}
    geom_tm_[idx.GeomTM].setVertices(geom.Vertices);
    if(geom.Directions.size()>0)
      geom_tm_[idx.GeomTM].setIndices(geom.Indices, geom.Directions);
    else
      geom_tm_[idx.GeomTM].setIndices(geom.Indices);
    geom_tm_[idx.GeomTM].create(space_.id());
    res= geom_tm_[idx.GeomTM];
    ++idx.GeomTM; break;
  case gtNone  :
  default      :
    LERROR("Invalid geometry type: "<<static_cast<int>(geom.Type));
    lexit(df);
  }
  dGeomSetData(res, const_cast<TGeometryParameters*>(&geom));
  return res;
}
//-------------------------------------------------------------------------------------------

/*static*/void TWorld::set_mass_from_geometry(dMass &m, const std::string &link_name, const TLinkParameters &link, const dGeomID ode_geom)
{
  LASSERT(link.InertiaType==itFromGeometryWithDensity || link.InertiaType==itFromGeometryWithMass);
  if(link.InertiaType==itFromGeometryWithDensity && link.Density<=0.0l)
  {
    LERROR("link:"<<link_name<<": InertiaType=itFromGeometryWithDensity, but Density="<<link.Density);
    lexit(df);
  }
  const dReal density(link.Density>0 ? link.Density : 1.0);

  const TGeometryParameters &geom(link.Geometry);
  switch(geom.Type)
  {
  case gtBox        :
    m.setBox(density, geom.Lx, geom.Ly, geom.Lz);
    break;
  case gtCapsule    :
    m.setCapsule(density, /*direction=Z:*/3, geom.Radius, geom.Length);
    break;
  case gtCylinder   :
    // m.setCylinder(density, /*direction=Z:*/3, geom.Radius, geom.Length);
    dMassSetCylinder(&m, density, /*direction=Z:*/3, geom.Radius, geom.Length);
    break;
  case gtSphere     :
    m.setSphere(density, geom.Radius);
    break;
  case gtTriMesh    :
    // this should be after create_geometry
    dMassSetTrimesh(&m, density, ode_geom);
    break;
  case gtPlane      :
  case gtNone  :
  default      :
    LERROR("Invalid geometry type: "<<static_cast<int>(geom.Type));
    lexit(df);
  }

  if(link.InertiaType==itFromGeometryWithMass)
  {
    m.adjust(link.Mass);
  }
}
//-------------------------------------------------------------------------------------------

void TWorld::create_link(const std::string &link_name, TLinkParameters &link, TElementSize &idx)
{
  LASSERT1op1(link.Position.size(),==,3);
  dGeomID ode_geom= create_geometry(link.Geometry, idx);
  body_[idx.Body].create(world_.id());
  body_[idx.Body].setPosition(link.Position[0], link.Position[1], link.Position[2]);

  dMass m;
  if(link.InertiaType==itManual)
  {
    LASSERT1op1(link.CoMOffset.size(),==,3);
    dReal I11= link.InertiaMatrix[0];
    dReal I12= link.InertiaMatrix[1];
    dReal I13= link.InertiaMatrix[2];
    dReal I21= link.InertiaMatrix[3];
    dReal I22= link.InertiaMatrix[4];
    dReal I23= link.InertiaMatrix[5];
    dReal I31= link.InertiaMatrix[6];
    dReal I32= link.InertiaMatrix[7];
    dReal I33= link.InertiaMatrix[8];
    LASSERT1op1(I12,==,I21);  // check symmetry
    LASSERT1op1(I13,==,I31);  // check symmetry
    LASSERT1op1(I23,==,I32);  // check symmetry
    dMassSetParameters (&m, link.Mass,
      link.CoMOffset[0], link.CoMOffset[1], link.CoMOffset[2],
      I11, I22, I33,  I12, I13, I23);
  }
  else // link.InertiaType==itFromGeometryWithDensity or link.InertiaType==itFromGeometryWithMass
  {
    set_mass_from_geometry(m, link_name, link, ode_geom);
  }

  if(link.MoveCoMToCenter)
  {
    dGeomSetPosition(ode_geom, -m.c[0], -m.c[1], -m.c[2]);
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
  }

  if(link.Rotation.size()>0)
  {
    dMatrix3 R;
    VectorToRotation(link.Rotation, R);
    dMassRotate(&m, R);
    body_[idx.Body].setRotation(R);
  }

  body_[idx.Body].setMass (&m);
  dGeomSetBody(ode_geom, body_[idx.Body]);

  ++idx.Body;
}
//-------------------------------------------------------------------------------------------

dJointID TWorld::create_joint(const std::string &joint_name, TJointParameters &joint, TElementSize &idx, const std::string &robot_name)
{
  int b1= LinkBodyIndex(robot_name, joint.Body1);
  int b2= LinkBodyIndex(robot_name, joint.Body2);
  if(b1<0 || b2<0)
  {
    LERROR("In joint "<<robot_name<<"::"<<joint_name<<": cannot attach joint between "<<joint.Body1<<" and "<<joint.Body2);
    lexit(df);
  }
  TNCBody &body1(body_[b1]);
  TNCBody &body2(body_[b2]);
  dJointID res(NULL);
  switch(joint.Type)
  {
  case jtBall      :
    LASSERT1op1(joint.Anchor.size(),==,3);
    joint_ba_[idx.JointBA].create(world_.id());
    joint_ba_[idx.JointBA].attach(body1,body2);
    joint_ba_[idx.JointBA].setAnchor(joint.Anchor[0], joint.Anchor[1], joint.Anchor[2]);
    res= joint_ba_[idx.JointBA].id();
    ++idx.JointBA; break;
  case jtHinge     :
    LASSERT1op1(joint.Anchor.size(),==,3);
    LASSERT1op1(joint.Axis1.size(),==,3);
    joint_h1_[idx.JointH1].create(world_.id());
    joint_h1_[idx.JointH1].attach(body1,body2);
    joint_h1_[idx.JointH1].setAnchor(joint.Anchor[0], joint.Anchor[1], joint.Anchor[2]);
    joint_h1_[idx.JointH1].setAxis(joint.Axis1[0], joint.Axis1[1], joint.Axis1[2]);
    joint_h1_[idx.JointH1].setParam(dParamFMax,   joint.AParam1.FMax);
    joint_h1_[idx.JointH1].setParam(dParamHiStop, joint.AParam1.HiStop);
    joint_h1_[idx.JointH1].setParam(dParamLoStop, joint.AParam1.LoStop);
    res= joint_h1_[idx.JointH1].id();
    ++idx.JointH1; break;
  case jtSlider    :
    LASSERT1op1(joint.Axis1.size(),==,3);
    joint_sl_[idx.JointSL].create(world_.id());
    joint_sl_[idx.JointSL].attach(body1,body2);
    joint_sl_[idx.JointSL].setAxis(joint.Axis1[0], joint.Axis1[1], joint.Axis1[2]);
    joint_sl_[idx.JointSL].setParam(dParamFMax,   joint.AParam1.FMax);
    joint_sl_[idx.JointSL].setParam(dParamHiStop, joint.AParam1.HiStop);
    joint_sl_[idx.JointSL].setParam(dParamLoStop, joint.AParam1.LoStop);
    res= joint_sl_[idx.JointSL].id();
    ++idx.JointSL; break;
  case jtUniversal :
    LASSERT1op1(joint.Anchor.size(),==,3);
    LASSERT1op1(joint.Axis1.size(),==,3);
    LASSERT1op1(joint.Axis2.size(),==,3);
    joint_un_[idx.JointUN].create(world_.id());
    joint_un_[idx.JointUN].attach(body1,body2);
    joint_un_[idx.JointUN].setAnchor(joint.Anchor[0], joint.Anchor[1], joint.Anchor[2]);
    joint_un_[idx.JointUN].setAxis1(joint.Axis1[0], joint.Axis1[1], joint.Axis1[2]);
    joint_un_[idx.JointUN].setAxis2(joint.Axis2[0], joint.Axis2[1], joint.Axis2[2]);
    joint_un_[idx.JointUN].setParam(dParamFMax1,   joint.AParam1.FMax);
    joint_un_[idx.JointUN].setParam(dParamHiStop1, joint.AParam1.HiStop);
    joint_un_[idx.JointUN].setParam(dParamLoStop1, joint.AParam1.LoStop);
    joint_un_[idx.JointUN].setParam(dParamFMax2,   joint.AParam2.FMax);
    joint_un_[idx.JointUN].setParam(dParamHiStop2, joint.AParam2.HiStop);
    joint_un_[idx.JointUN].setParam(dParamLoStop2, joint.AParam2.LoStop);
    res= joint_un_[idx.JointUN].id();
    ++idx.JointUN; break;
  case jtHinge2    :
    LASSERT1op1(joint.Anchor.size(),==,3);
    LASSERT1op1(joint.Axis1.size(),==,3);
    LASSERT1op1(joint.Axis2.size(),==,3);
    joint_h2_[idx.JointH2].create(world_.id());
    joint_h2_[idx.JointH2].attach(body1,body2);
    joint_h2_[idx.JointH2].setAnchor(joint.Anchor[0], joint.Anchor[1], joint.Anchor[2]);
    joint_h2_[idx.JointH2].setAxis1(joint.Axis1[0], joint.Axis1[1], joint.Axis1[2]);
    joint_h2_[idx.JointH2].setAxis2(joint.Axis2[0], joint.Axis2[1], joint.Axis2[2]);
    joint_h2_[idx.JointH2].setParam(dParamFMax1,   joint.AParam1.FMax);
    joint_h2_[idx.JointH2].setParam(dParamHiStop1, joint.AParam1.HiStop);
    joint_h2_[idx.JointH2].setParam(dParamLoStop1, joint.AParam1.LoStop);
    joint_h2_[idx.JointH2].setParam(dParamFMax2,   joint.AParam2.FMax);
    joint_h2_[idx.JointH2].setParam(dParamHiStop2, joint.AParam2.HiStop);
    joint_h2_[idx.JointH2].setParam(dParamLoStop2, joint.AParam2.LoStop);
    res= joint_h2_[idx.JointH2].id();
    ++idx.JointH2; break;
  case jtFixed     :
    joint_fx_[idx.JointFX].create(world_.id());
    joint_fx_[idx.JointFX].attach(body1,body2);
    joint_fx_[idx.JointFX].set();
    res= joint_fx_[idx.JointFX].id();
    ++idx.JointFX; break;
  case jtNone  :
  default      :
    LERROR("Invalid joint type: "<<static_cast<int>(joint.Type));
    lexit(df);
  }
  dJointSetData(res, const_cast<TJointParameters*>(&joint));
  return res;
}
//-------------------------------------------------------------------------------------------

void TWorld::create_force_sensor(TNamedParam<TForceSensorParameters>::P::iterator iforce_sensor, TElementSize &idx, const std::string &robot_name)
{
  TForceSensorParameters &force_sensor(iforce_sensor->second);
  force_sensor.fixed_joint_.Body1= force_sensor.Attached;
  force_sensor.fixed_joint_.Body2= iforce_sensor->first;
  force_sensor.fixed_joint_.Type= jtFixed;
  force_sensor.fixed_joint_.Visualize= false;

  create_link(iforce_sensor->first, force_sensor, idx);
  dJointID sensor_joint= create_joint(iforce_sensor->first+"_fixed", force_sensor.fixed_joint_, idx, robot_name);

  dJointSetFeedback(sensor_joint, &joint_feedback_[idx.JointFeedback]);
  ++idx.JointFeedback;
}
//-------------------------------------------------------------------------------------------

void TWorld::create_robot(TNamedParam<TRobotParameters>::P::iterator irobot, TElementSize &idx)
{
  TRobotParameters &robot(irobot->second);
  std::map<std::string, TJointParameters*>  name_to_joint;
  for(TNamedParam<TLinkParameters>::IdxMapper::iterator litr(robot.links_.begin()),llast(robot.links_.end()); litr!=llast; ++litr)
  {
    create_link((*litr)->first, (*litr)->second,idx);
  }
  for(TNamedParam<TJointParameters>::IdxMapper::iterator jitr(robot.joints_.begin()),jlast(robot.joints_.end()); jitr!=jlast; ++jitr)
  {
    (*jitr)->second.id_= create_joint((*jitr)->first,(*jitr)->second,idx,irobot->first);
    name_to_joint[(*jitr)->first]= &((*jitr)->second);
  }
  for(TNamedParam<TForceSensorParameters>::IdxMapper::iterator fitr(robot.force_sensors_.begin()),flast(robot.force_sensors_.end()); fitr!=flast; ++fitr)
  {
    create_force_sensor(*fitr,idx,irobot->first);
  }

  construct_manipulators(irobot, name_to_joint);

  if(robot.RootLink!="")
  {
    int b= LinkBodyIndex(irobot->first,robot.RootLink);
    if(b<0)  {LERROR(irobot->first<<": Invalid RootLink value"); lexit(df);}
    if(robot.Position.size()>0 || robot.Rotation.size()>0)
    {
      dVector3 pos;
      dMatrix3 R;
      if(robot.Position.size()>0)
        VectorToPosition(robot.Position, pos);
      else
        {pos[0]=0.0;pos[1]=0.0;pos[2]=0.0;}
      if(robot.Rotation.size())
        VectorToRotation(robot.Rotation, R);
      else
        dRSetIdentity(R);
      ODESetArticulatedBodyPosRotR(body_[b],pos,R);
    }
  }

  if(robot.InitialJointAngles.size()>0)
  {
    LASSERT1op1(robot.InitialJointAngles.size(),==,robot.JointAngleObserver.size());
    if(robot.RootLink=="")  {LERROR(irobot->first<<": RootLink should be specified to use InitialJointAngles"); lexit(df);}
    int b= LinkBodyIndex(irobot->first,robot.RootLink);
    if(b<0)  {LERROR(irobot->first<<": Invalid RootLink value"); lexit(df);}
    TODEJointAngleMap  angle_map;

    std::vector<TReal>::const_iterator qitr(robot.InitialJointAngles.begin());
    for(std::vector<TJointManipulator>::const_iterator oitr(robot.JointAngleObserver.begin()), olast(robot.JointAngleObserver.end()); oitr!=olast; ++oitr,++qitr)
    {
      std::map<std::string, TJointParameters*>::const_iterator  joint_item= name_to_joint.find(oitr->Name);
      if(joint_item==name_to_joint.end())
      {
        LERROR("Robot "<<irobot->first<<" does not have a joint named "<<oitr->Name);
        lexit(df);
      }
      const TJointParameters *joint= joint_item->second;
      LASSERT(joint);
      switch(joint->Type)
      {
      case jtHinge     :
      case jtSlider    :
      case jtHinge2    :
        LASSERT(oitr->Index==0);
        angle_map[joint->id_].angle1= *qitr;
        break;
      case jtUniversal :
        LASSERT(oitr->Index==0 || oitr->Index==1);
        if(oitr->Index==0)
          angle_map[joint->id_].angle1= *qitr;
        else if(oitr->Index==1)
          angle_map[joint->id_].angle2= *qitr;
        break;
      case jtNone  :
      case jtBall  :
      case jtFixed :
      default      :
        LERROR("Invalid joint type (does not observable): "<<static_cast<int>(joint->Type));
        lexit(df);
      }
    }
    ODESetArticulatedBodyJointAngles(body_[b], angle_map);
  }
}
//-------------------------------------------------------------------------------------------

dGeomID TWorld::create_static_link(TNamedParam<TStaticLinkParameters>::P::iterator ilink, TElementSize &idx)
{
  TStaticLinkParameters &link(ilink->second);
  dGeomID ode_geom= create_geometry(link.Geometry, idx);
  if(link.Rotation.size()>0)
  {
    dMatrix3 R;
    VectorToRotation(link.Rotation, R);
    dGeomSetRotation(ode_geom, R);
  }
  if(link.Position.size()>0)
  {
    LASSERT1op1(link.Position.size(),==,3);
    dGeomSetPosition(ode_geom, link.Position[0], link.Position[1], link.Position[2]);
  }
  return ode_geom;
}
//-------------------------------------------------------------------------------------------

void TWorld::create_static_object(TNamedParam<TStaticObjectParameters>::P::iterator isobj, TElementSize &idx)
{
  TStaticObjectParameters &sobj(isobj->second);
  std::list<dGeomID> geoms;
  for(TNamedParam<TStaticLinkParameters>::IdxMapper::iterator litr(sobj.links_.begin()),llast(sobj.links_.end()); litr!=llast; ++litr)
  {
    geoms.push_back(create_static_link(*litr, idx));
  }
  if(sobj.Rotation.size()>0)
  {
    dMatrix3 rot;
    VectorToRotation(sobj.Rotation, rot);
    // rotate every geoms around (0,0,0)
    for(std::list<dGeomID>::iterator gitr(geoms.begin()),glast(geoms.end());gitr!=glast;++gitr)
    {
      dVector3 c, d;
      memcpy(d, dGeomGetPosition(*gitr), sizeof(dVector3));
      dMULTIPLY0_331 (c,rot,d);  // c = rot * d
      dMatrix3 R;
      dMULTIPLY0_333(R,rot,dGeomGetRotation(*gitr));  // R = rotR * rotR(i)
      dGeomSetPosition(*gitr,c[0],c[1],c[2]);
      dGeomSetRotation(*gitr,R);
    }
  }
  if(sobj.Position.size()>0)
  {
    dVector3 pos;
    VectorToPosition(sobj.Position, pos);
    for(std::list<dGeomID>::iterator gitr(geoms.begin()),glast(geoms.end());gitr!=glast;++gitr)
      dGeomSetPosition(*gitr, pos[0], pos[1], pos[2]);
  }
}
//-------------------------------------------------------------------------------------------

void TWorld::construct_manipulators(TNamedParam<TRobotParameters>::P::iterator irobot, const std::map<std::string, TJointParameters*> &name_to_joint)
{
  const TRobotParameters &robot(irobot->second);
  const int ridx= robot_idxes_[irobot->first];
  int oidx;
  oidx= 0;
  angle_observer_[ridx].resize(robot.JointAngleObserver.size());
  for(std::vector<TJointManipulator>::const_iterator oitr(robot.JointAngleObserver.begin()), olast(robot.JointAngleObserver.end()); oitr!=olast; ++oitr,++oidx)
  {
    std::map<std::string, TJointParameters*>::const_iterator  joint_item= name_to_joint.find(oitr->Name);
    if(joint_item==name_to_joint.end())
    {
      LERROR("Robot "<<irobot->first<<" does not have a joint named "<<oitr->Name);
      lexit(df);
    }
    const TJointParameters *joint= joint_item->second;
    LASSERT(joint);
    switch(joint->Type)
    {
    case jtHinge     :
      LASSERT(oitr->Index==0);
      angle_observer_[ridx][oidx]= boost::bind(&dJointGetHingeAngle,joint->id_);
      break;
    case jtSlider    :
      LASSERT(oitr->Index==0);
      angle_observer_[ridx][oidx]= boost::bind(&dJointGetSliderPosition,joint->id_);
      break;
    case jtUniversal :
      LASSERT(oitr->Index==0 || oitr->Index==1);
      if(oitr->Index==0)
        angle_observer_[ridx][oidx]= boost::bind(&dJointGetUniversalAngle1,joint->id_);
      else if(oitr->Index==1)
        angle_observer_[ridx][oidx]= boost::bind(&dJointGetUniversalAngle2,joint->id_);
      break;
    case jtHinge2    :
      LASSERT(oitr->Index==0);
      angle_observer_[ridx][oidx]= boost::bind(&dJointGetHinge2Angle1,joint->id_);
      break;
    case jtNone  :
    case jtBall  :
    case jtFixed :
    default      :
      LERROR("Invalid joint type (does not observable): "<<static_cast<int>(joint->Type));
      lexit(df);
    }
  }

  oidx= 0;
  angvel_observer_[ridx].resize(robot.JointAngVelObserver.size());
  for(std::vector<TJointManipulator>::const_iterator oitr(robot.JointAngVelObserver.begin()), olast(robot.JointAngVelObserver.end()); oitr!=olast; ++oitr,++oidx)
  {
    std::map<std::string, TJointParameters*>::const_iterator  joint_item= name_to_joint.find(oitr->Name);
    if(joint_item==name_to_joint.end())
    {
      LERROR("Robot "<<irobot->first<<" does not have a joint named "<<oitr->Name);
      lexit(df);
    }
    const TJointParameters *joint= joint_item->second;
    LASSERT(joint);
    switch(joint->Type)
    {
    case jtHinge     :
      LASSERT(oitr->Index==0);
      angvel_observer_[ridx][oidx]= boost::bind(&dJointGetHingeAngleRate,joint->id_);
      break;
    case jtSlider    :
      LASSERT(oitr->Index==0);
      angvel_observer_[ridx][oidx]= boost::bind(&dJointGetSliderPositionRate,joint->id_);
      break;
    case jtUniversal :
      LASSERT(oitr->Index==0 || oitr->Index==1);
      if(oitr->Index==0)
        angvel_observer_[ridx][oidx]= boost::bind(&dJointGetUniversalAngle1Rate,joint->id_);
      else if(oitr->Index==1)
        angvel_observer_[ridx][oidx]= boost::bind(&dJointGetUniversalAngle2Rate,joint->id_);
      break;
    case jtHinge2    :
      LASSERT(oitr->Index==0 || oitr->Index==1);
      if(oitr->Index==0)
        angvel_observer_[ridx][oidx]= boost::bind(&dJointGetHinge2Angle1Rate,joint->id_);
      else if(oitr->Index==1)
        angvel_observer_[ridx][oidx]= boost::bind(&dJointGetHinge2Angle2Rate,joint->id_);
      break;
    case jtNone  :
    case jtBall  :
    case jtFixed :
    default      :
      LERROR("Invalid joint type (does not observable): "<<static_cast<int>(joint->Type));
      lexit(df);
    }
  }

  oidx= 0;
  torque_adder_[ridx].resize(robot.JointTorqueAdder.size());
  for(std::vector<TJointManipulator>::const_iterator oitr(robot.JointTorqueAdder.begin()), olast(robot.JointTorqueAdder.end()); oitr!=olast; ++oitr,++oidx)
  {
    std::map<std::string, TJointParameters*>::const_iterator  joint_item= name_to_joint.find(oitr->Name);
    if(joint_item==name_to_joint.end())
    {
      LERROR("Robot "<<irobot->first<<" does not have a joint named "<<oitr->Name);
      lexit(df);
    }
    const TJointParameters *joint= joint_item->second;
    LASSERT(joint);
    switch(joint->Type)
    {
    case jtHinge     :
      LASSERT(oitr->Index==0);
      torque_adder_[ridx][oidx]= boost::bind(&dJointAddHingeTorque,joint->id_,_1);
      break;
    case jtSlider    :
      LASSERT(oitr->Index==0);
      torque_adder_[ridx][oidx]= boost::bind(&dJointAddSliderForce,joint->id_,_1);
      break;
    case jtUniversal :
      LASSERT(oitr->Index==0 || oitr->Index==1);
      if(oitr->Index==0)
        torque_adder_[ridx][oidx]= boost::bind(&dJointAddUniversalTorques,joint->id_,_1,0.0);
      else if(oitr->Index==1)
        torque_adder_[ridx][oidx]= boost::bind(&dJointAddUniversalTorques,joint->id_,0.0,_1);
      break;
    case jtHinge2    :
      LASSERT(oitr->Index==0 || oitr->Index==1);
      if(oitr->Index==0)
        torque_adder_[ridx][oidx]= boost::bind(&dJointAddHinge2Torques,joint->id_,_1,0.0);
      else if(oitr->Index==1)
        torque_adder_[ridx][oidx]= boost::bind(&dJointAddHinge2Torques,joint->id_,0.0,_1);
      break;
    case jtNone  :
    case jtBall  :
    case jtFixed :
    default      :
      LERROR("Invalid joint type (does not observable): "<<static_cast<int>(joint->Type));
      lexit(df);
    }
  }

  {
    const std::map<std::string, int> &fsensor_to_jfeedback= name_to_jfeedback_[irobot->first];
    force_observer_[ridx].resize(robot.ForceObserver.size());
    int idx(0);
    for(std::vector<std::string>::const_iterator itr(robot.ForceObserver.begin()),last(robot.ForceObserver.end()); itr!=last; ++itr,++idx)
      force_observer_[ridx][idx]= fsensor_to_jfeedback.find(*itr)->second;
  }

  {
    const std::map<std::string, int> &link_to_body= name_to_body_[irobot->first];
    contact_observer_[ridx].resize(robot.LinkContactObserver.size());
    int idx(0);
    for(std::vector<std::string>::const_iterator itr(robot.LinkContactObserver.begin()),last(robot.LinkContactObserver.end()); itr!=last; ++itr,++idx)
      contact_observer_[ridx][idx]= link_to_body.find(*itr)->second;

    current_contact_[ridx].resize(robot.LinkContactObserver.size());
    std::fill(current_contact_[ridx].begin(),current_contact_[ridx].end(),false);
  }
}
//-------------------------------------------------------------------------------------------

bool TWorld::Create()
{
  LASSERT1op1(params_.Gravity.size(),==,3);
  Clear();

  contactgroup_.create();
  world_.setGravity(params_.Gravity[0],params_.Gravity[1],params_.Gravity[2]);
  dWorldSetCFM (world_.id(),params_.WorldCFM);

  resize_elements();
  TElementSize idx;

  // Create ODE models:
  for(TNamedParam<TRobotParameters>::IdxMapper::const_iterator ritr(params_.robots_.begin()),rlast(params_.robots_.end()); ritr!=rlast; ++ritr)
  {
    create_robot(*ritr, idx);
  }
  for(TNamedParam<TStaticObjectParameters>::IdxMapper::const_iterator sitr(params_.static_objects_.begin()),slast(params_.static_objects_.end()); sitr!=slast; ++sitr)
  {
    create_static_object(*sitr,idx);
  }

  return true;
}
//-------------------------------------------------------------------------------------------

static bool set_color(const std::vector<TReal> &color)
{
  if(color.size()==1)
    dsSetColorAlpha(color[0], color[0], color[0], 1.0);
  else if(color.size()==3)
    dsSetColorAlpha(color[0], color[1], color[2], 1.0);
  else if(color.size()==4)
    dsSetColorAlpha(color[0], color[1], color[2], color[3]);
  else
    return false;
  return true;
}
//-------------------------------------------------------------------------------------------

// NOTE: data should be a pointer to an instance of TGeometryParameters
static bool geometry_set_color(void *data)
{
  if(!data)  return false;
  const TGeometryParameters *geom= reinterpret_cast<const TGeometryParameters*>(data);
  return set_color(geom->Color);
}
//-------------------------------------------------------------------------------------------

// anchor1, anchor2, axis1, axis2 should be dVector3
static void visualize_joint(void *data, dReal *anchor1, dReal *axis1, dReal *anchor2, dReal *axis2)
{
  static dMatrix3 I={1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0, 0.0,0.0,1.0,0.0};
  static dVector3 uz={0.0,0.0,1.0};
  if(!data)  return;
  const TJointParameters *joint= reinterpret_cast<const TJointParameters*>(data);
  if(!joint->Visualize)  return;
  if(!set_color(joint->VizColor))
    dsSetColorAlpha (1.0, 1.0, 1.0, 0.8);
  if(!axis1)  axis1= uz;
  if(!axis2)  axis2= uz;
  dMatrix3 R;
  switch(joint->VizType)
  {
  case jvtSphere:
    if(anchor1)
      dsDrawSphere(anchor1, I, joint->VizRadius);
    break;
  case jvtCapsule:
    if(anchor1)
    {
      dRFromAxisAndAngle(R,-axis1[1],axis1[0],0.0, 0.5*M_PI-std::asin(axis1[2]));
      dsDrawCapsule(anchor1, R, joint->VizLength, joint->VizRadius);
    }
    if(anchor2)
    {
      dRFromAxisAndAngle(R,-axis2[1],axis2[0],0.0, 0.5*M_PI-std::asin(axis2[2]));
      dsDrawCapsule(anchor2, R, joint->VizLength, joint->VizRadius);
    }
    break;
  case jvtCylinder:
    if(anchor1 && axis1)
    {
      dRFromAxisAndAngle(R,-axis1[1],axis1[0],0.0, 0.5*M_PI-std::asin(axis1[2]));
      dsDrawCylinder(anchor1, R, joint->VizLength, joint->VizRadius);
    }
    if(anchor2 && axis2)
    {
      dRFromAxisAndAngle(R,-axis2[1],axis2[0],0.0, 0.5*M_PI-std::asin(axis2[2]));
      dsDrawCylinder(anchor2, R, joint->VizLength, joint->VizRadius);
    }
    break;
  default:
    LERROR("Invalid joint visualization type: "<<static_cast<int>(joint->VizType));
    lexit(df);
  }
}
//-------------------------------------------------------------------------------------------

void TWorld::Draw() const
{
  dsSetTexture(DS_WOOD);

  dReal rad, len;
  dReal sides[4];
  for (std::vector<TNCBox>::const_iterator itr(geom_bx_.begin()),last(geom_bx_.end()); itr!=last; ++itr)
  {
    if(!geometry_set_color(itr->getData()))
      dsSetColorAlpha(0.0, 1.0, 0.0, 0.8);
    itr->getLengths(sides);
    dsDrawBox(itr->getPosition(), itr->getRotation(), sides);
  }
  for (std::vector<TNCCapsule>::const_iterator itr(geom_ca_.begin()),last(geom_ca_.end()); itr!=last; ++itr)
  {
    if(!geometry_set_color(itr->getData()))
      dsSetColorAlpha(0.0, 0.5, 1.0, 0.6);
    itr->getParams(&rad, &len);
    dsDrawCapsule(itr->getPosition(), itr->getRotation(), len,rad);
  }
  for (std::vector<TNCCylinder>::const_iterator itr(geom_cy_.begin()),last(geom_cy_.end()); itr!=last; ++itr)
  {
    if(!geometry_set_color(itr->getData()))
      dsSetColorAlpha(1.0, 0.0, 0.0, 0.6);
    itr->getParams(&rad, &len);
    dsDrawCylinder(itr->getPosition(), itr->getRotation(), len,rad);
  }
  // for (std::vector<TNCPlane>::const_iterator itr(geom_pl_.begin()),last(geom_pl_.end()); itr!=last; ++itr)
  // {
    // if(!geometry_set_color(itr->getData()))
      // dsSetColorAlpha(0.2, 0.1, 0.1, 1.0);
    // dsDrawPlane?(itr->getPosition(), itr->getRotation(), itr->getRadius());
  // }
  for (std::vector<TNCSphere>::const_iterator itr(geom_sp_.begin()),last(geom_sp_.end()); itr!=last; ++itr)
  {
    if(!geometry_set_color(itr->getData()))
      dsSetColorAlpha(0.0, 1.0, 0.5, 0.6);
    dsDrawSphere(itr->getPosition(), itr->getRotation(), itr->getRadius());
  }
  for (std::vector<TTriMeshGeom>::const_iterator itr(geom_tm_.begin()),last(geom_tm_.end()); itr!=last; ++itr)
  {
    const TGeometryParameters *geom= reinterpret_cast<const TGeometryParameters*>(itr->getData());
    bool cmn_col(true);
    if(geom->Colors.size()==geom->Indices.size()/3*4)
      cmn_col= false;
    else
    {
      if(!geometry_set_color(itr->getData()))
        dsSetColorAlpha(1.0, 0.0, 0.5, 0.6);
    }

    // const dReal *pos = itr->getPosition();
    // const dReal *rot = itr->getRotation();
    const dVector3 pos={0,0,0,0};
    const dMatrix3 rot={1,0,0,0 ,0,1,0,0, 0,0,1,0};

    for (int i(0),N(dGeomTriMeshGetTriangleCount(*itr)); i<N; ++i)
    {
      if(!cmn_col)
        dsSetColorAlpha(geom->Colors[i*4+0],geom->Colors[i*4+1],geom->Colors[i*4+2],geom->Colors[i*4+3]);
      dVector3 v[3];
      dGeomTriMeshGetTriangle(*itr, i, &v[0], &v[1], &v[2]);
      dsDrawTriangle(pos, rot, v[0], v[1], v[2], 1);
    }
  }

  dVector3 anchor1,anchor2,axis1,axis2;
  for (std::vector<TNCBallJoint>::const_iterator itr(joint_ba_.begin()),last(joint_ba_.end()); itr!=last; ++itr)
  {
    itr->getAnchor(anchor1);
    visualize_joint(itr->getData(), anchor1, NULL, NULL, NULL);
  }
  for (std::vector<TNCHingeJoint>::const_iterator itr(joint_h1_.begin()),last(joint_h1_.end()); itr!=last; ++itr)
  {
    itr->getAnchor(anchor1);
    itr->getAxis(axis1);
    visualize_joint(itr->getData(), anchor1, axis1, NULL, NULL);
  }
  for (std::vector<TNCSliderJoint>::const_iterator itr(joint_sl_.begin()),last(joint_sl_.end()); itr!=last; ++itr)
  {
    itr->getAxis(axis1);
    visualize_joint(itr->getData(), NULL, axis1, NULL, NULL);  // NOTE: doesn't work
  }
  for (std::vector<TNCUniversalJoint>::const_iterator itr(joint_un_.begin()),last(joint_un_.end()); itr!=last; ++itr)
  {
    itr->getAnchor(anchor1);
    itr->getAxis1(axis1);
    itr->getAnchor2(anchor2);
    itr->getAxis2(axis2);
    visualize_joint(itr->getData(), anchor1, axis1, anchor2, axis2);
  }
  for (std::vector<TNCHinge2Joint>::const_iterator itr(joint_h2_.begin()),last(joint_h2_.end()); itr!=last; ++itr)
  {
    itr->getAnchor(anchor1);
    itr->getAxis1(axis1);
    itr->getAnchor2(anchor2);
    itr->getAxis2(axis2);
    visualize_joint(itr->getData(), anchor1, axis1, anchor2, axis2);
  }
  for (std::vector<TNCFixedJoint>::const_iterator itr(joint_fx_.begin()),last(joint_fx_.end()); itr!=last; ++itr)
  {
    visualize_joint(itr->getData(), NULL, NULL, NULL, NULL);  // NOTE: doesn't work
  }
}
//-------------------------------------------------------------------------------------------

void TWorld::Start()
{
  SetupODE();

  SetupViewer();

  time_= 0.0l;
}
//-------------------------------------------------------------------------------------------

void TWorld::SetupViewer()
{
  if (!params_.ConsoleMode)
  {
    float view[6];
    for(int i(0);i<6;++i)  view[i]= params_.ViewPoint[i];
    dsSetViewpoint (view,view+3);
  }

  old_fps_= 0.0l;
  repaint_time_= 0;
  no_paint_step_= 0;
}
//-------------------------------------------------------------------------------------------

void TWorld::Stop()
{
  StopViewer();
}
//-------------------------------------------------------------------------------------------

void TWorld::StopViewer()
{
  dsStop();
}
//-------------------------------------------------------------------------------------------

// Supplementary function
static void near_callback(void *data, dGeomID o1, dGeomID o2)
{
  TWorld *world= reinterpret_cast<TWorld*>(data);
  world->NearCallback(o1,o2);
}
//-------------------------------------------------------------------------------------------

void TWorld::StepSimulation()
{
  if(callbacks_.StartOfTimeStep)  callbacks_.StartOfTimeStep(*this, params_.TimeStep);

  for(std::vector<std::vector<bool> >::iterator citr(current_contact_.begin()),clast(current_contact_.end());citr!=clast;++citr)
    std::fill(citr->begin(),citr->end(),false);

  space_.collide(this, &near_callback);

  if (!params_.UsingQuickStep)
    world_.step (params_.TimeStep);
  else
  {
    world_.setQuickStepNumIterations (params_.QuickStepIterationNum);
    world_.quickStep (params_.TimeStep);
  }
  time_+= params_.TimeStep;

  contactgroup_.empty();

  if(callbacks_.EndOfTimeStep)  callbacks_.EndOfTimeStep(*this, params_.TimeStep);
}
//-------------------------------------------------------------------------------------------

void TWorld::StepDrawing()
{
  TReal draw_timestep= 1.0l/params_.DisplayFPS;
  if(callbacks_.StartOfDrawing)  callbacks_.StartOfDrawing(*this, draw_timestep);
  Draw();
  if(callbacks_.EndOfDrawing)  callbacks_.EndOfDrawing(*this, draw_timestep);
  no_paint_step_= 0;
}
//-------------------------------------------------------------------------------------------

/*! Step simulation with drawing (FPS is controlled).
    If in ConsoleMode, return whether the world is drawn; if not in ConsoleMode, return true always. */
bool TWorld::Step()
{
  StepSimulation();

  if(!params_.ConsoleMode)
  {
    if(no_paint_step_==0 && old_fps_!=params_.DisplayFPS)
    {
      repaint_time_= int(real_round(1.0l/params_.TimeStep/params_.DisplayFPS));
      if(repaint_time_<=0)  repaint_time_= 1;
      old_fps_= params_.DisplayFPS;
    }

    ++no_paint_step_;

    if(no_paint_step_==repaint_time_)
    {
      StepDrawing();
      return true;
    }
    else
      return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

static void copy_surface_params(const TSurfaceParameters &src, dSurfaceParameters &dest)
{
  dest.mode=0;
  if(src.ContactMu2       )  dest.mode|= dContactMu2       ;
  if(src.ContactFDir1     )  dest.mode|= dContactFDir1     ;
  if(src.ContactBounce    )  dest.mode|= dContactBounce    ;
  if(src.ContactSoftERP   )  dest.mode|= dContactSoftERP   ;
  if(src.ContactSoftCFM   )  dest.mode|= dContactSoftCFM   ;
  if(src.ContactMotion1   )  dest.mode|= dContactMotion1   ;
  if(src.ContactMotion2   )  dest.mode|= dContactMotion2   ;
  if(src.ContactMotionN   )  dest.mode|= dContactMotionN   ;
  if(src.ContactSlip1     )  dest.mode|= dContactSlip1     ;
  if(src.ContactSlip2     )  dest.mode|= dContactSlip2     ;
  if(src.ContactApprox0   )  dest.mode|= dContactApprox0   ;
  if(src.ContactApprox1_1 )  dest.mode|= dContactApprox1_1 ;
  if(src.ContactApprox1_2 )  dest.mode|= dContactApprox1_2 ;
  if(src.ContactApprox1   )  dest.mode|= dContactApprox1   ;

  dest.mu         = src.Mu        ;
  dest.mu2        = src.Mu2       ;
  dest.bounce     = src.Bounce    ;
  dest.bounce_vel = src.BounceVel;
  dest.soft_erp   = src.SoftERP   ;
  dest.soft_cfm   = src.SoftCFM   ;
  dest.motion1    = src.Motion1   ;
  dest.motion2    = src.Motion2   ;
  dest.motionN    = src.MotionN   ;
  dest.slip1      = src.Slip1     ;
  dest.slip2      = src.Slip2     ;
}
//-------------------------------------------------------------------------------------------

void TWorld::NearCallback(dGeomID o1, dGeomID o2)
{
  // do nothing if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnected (b1,b2)) return;

  std::valarray<dContact> contact(params_.MaxContacts);   // up to MaxContacts contacts per link
  for (int i=0; i<params_.MaxContacts; i++)
  {
    copy_surface_params(params_.DefaultSurface, contact[i].surface);
  }
  if (int numc=dCollide(o1,o2,params_.MaxContacts,&contact[0].geom,sizeof(dContact)))
  {
    for (int i=0; i<numc; i++)
    {
      dJointID c= dJointCreateContact(world_.id(),contactgroup_,&(contact[i]));
      dJointAttach (c,b1,b2);
    }
    for (int ridx(0),rnum(contact_observer_.size()); ridx<rnum; ++ridx)
    {
      for (int cidx(0),cnum(contact_observer_[ridx].size()); cidx<cnum; ++cidx)
      {
        if(b1==body_[contact_observer_[ridx][cidx]].id() || b2==body_[contact_observer_[ridx][cidx]].id())
          current_contact_[ridx][cidx]= true;
      }
    }
  }
}
//-------------------------------------------------------------------------------------------

bool TWorld::LoadFromFile(const std::string &file_name)
{
  using namespace loco_rabbits::var_space;
  TBuiltinFunctions additional_funcs;
  additional_funcs.Add("RFromAxisAndAngle", TBuiltinFunctions::rtList, &builtin_function_RFromAxisAndAngle);
  additional_funcs.Add("RFromEulerAngles", TBuiltinFunctions::rtList, &builtin_function_RFromEulerAngles);
  additional_funcs.Add("RFromQ", TBuiltinFunctions::rtList, &builtin_function_RFromQ);

  var_space::TLiteralTable literal_table;
  var_space::TVariable var(params_);
  if(!var_space::LoadFromFile(file_name,var,literal_table,&additional_funcs))
  {
    LERROR("Failed to load: "<<file_name);
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

bool TWorld::SaveToFile(const std::string &file_name)
{
  std::ofstream ofs(file_name.c_str());
  var_space::TVariable var(params_);
  var.WriteToStream(ofs,true);
  return true;
}
//-------------------------------------------------------------------------------------------

TReal TWorld::TotalMass(const std::string &robot_name) const
{
  std::map<std::string, std::map<std::string, int> >::const_iterator ritr= name_to_body_.find(robot_name);
  if(ritr==name_to_body_.end())
    {LERROR("Robot \""<<robot_name<<"\" not found");  return -1.0l;}
  TReal mass(0.0l);
  for(std::map<std::string, int>::const_iterator bitr(ritr->second.begin()),blast(ritr->second.end());bitr!=blast;++bitr)
    mass+= body_[bitr->second].getMass().mass;
  return mass;
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of xode
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
