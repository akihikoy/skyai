//-------------------------------------------------------------------------------------------
/*! \file
    \brief   benchmarks - simulation tools for ODE
    \author  Akihiko Yamaguchi
    \date    May 6, 2008-

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
#ifndef sim_tools_h
#define sim_tools_h
//-------------------------------------------------------------------------------------------
#ifndef SIM_OBJECT_DEFS
  #error SIM_OBJECT_DEFS is not defined; the simulation objects should be defined before include sim-tools.h,
  #error and SIM_OBJECT_DEFS constant is defined
#endif
//-------------------------------------------------------------------------------------------
#include <fstream>
#include <iomanip>
#include <list>
#include <lora/octave.h>
#include <lora/octave_str.h>
#include <lora/small_classes.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
extern const int JOINT_NUM;
//-------------------------------------------------------------------------------------------
static TReal world_time (0.0l);
//-------------------------------------------------------------------------------------------
TInitializer SimulationInit;   //!< initialized at create_world
//-------------------------------------------------------------------------------------------
struct TContactInfo
{
  dReal p[3];
  dBodyID b1, b2;  //!< contacting bodies
  TContactInfo(dVector3 v, dBodyID _b1, dBodyID _b2)
    : b1(_b1), b2(_b2) {p[0]=v[0];p[1]=v[1];p[2]=v[2];};
};
static std::vector<TContactInfo> contact_points;
static bool use_contact_points(false);
static std::vector<float> _bodies_contact_with_ground(BODY_NUM,0.0f); // contact with ground
static std::vector<float> _bodies_contact_with_object(BODY_NUM,0.0f);   // contact with objects other than the ground
static TLHBPFilters<std::vector<float> >  _bodies_contact_with_ground_LPF;
static TLHBPFilters<std::vector<float> >  _bodies_contact_with_object_LPF;
static bool use_contact_LPF(true);
//-------------------------------------------------------------------------------------------


//===========================================================================================
// SIMULATION CONDITION
//===========================================================================================
struct TSimulationCondition
{
  int                 MaxContactNum;  //!< maximum number of contact points per body
  dSurfaceParameters  Surface;       //!< parameters of Surface
  double              BodyContactLPFParamF;
  double              BodyContactLPFParamQ;

  bool                ForceInitFeetContactWithGround;  //!< true: force to initialize _bodies_contact_with_ground[FEET]=1

  bool                UsingQuickStep;  //!< use quickStep to step the world. this mode is fast, but sometimes lose the accuracy
  int                 QuickStepIterationNum;  //!< number of iteration used in quickStep

  TSimulationCondition (void) :
      MaxContactNum             (10),
      BodyContactLPFParamF     (10.0),
      BodyContactLPFParamQ     (0.8),
      ForceInitFeetContactWithGround (false),
      UsingQuickStep           (false),
      QuickStepIterationNum        (20)
    {
      Surface.mode       = dContactBounce | dContactSoftCFM;
      Surface.mu         = dInfinity;
      Surface.mu2        = 0;
      Surface.bounce     = 0.00001;  // 0.1;
      Surface.bounce_vel = 0.1; // 0.1;
      Surface.soft_erp   = 0.0;
      Surface.soft_cfm   = 0.001;   //  0.01;
      Surface.motion1    = 0.0;
      Surface.motion2    = 0.0;
      Surface.slip1      = 0.0;
      Surface.slip2      = 0.0;
    };
};
extern TSimulationCondition simulationcnd;
//-------------------------------------------------------------------------------------------


//===========================================================================================
// TOOL
//===========================================================================================

void getCOM (dVector3 com)
{
  memset (com,0,sizeof(dVector3));
  dReal M(0.0);
  for (int i(0); i<BODY_NUM; ++i)
  {
    // dMass m; body[i].getMass(&m);  // for ode-0.9
    dMass m (body[i].getMass());  // for ode-0.10.1
    com[0]+=m.mass*body[i].getPosition()[0];
    com[1]+=m.mass*body[i].getPosition()[1];
    com[2]+=m.mass*body[i].getPosition()[2];
    M += m.mass;
  }
  com[0]/=M;
  com[1]/=M;
  com[2]/=M;
}
//-------------------------------------------------------------------------------------------

void bodies_contact_with_ground (std::vector<bool> &result, const float &sensitivity=0.1f)
{
  LASSERT(use_contact_LPF);
  LASSERT(_bodies_contact_with_ground_LPF.isInitialized());
  const std::vector<float>  &lpf (_bodies_contact_with_ground_LPF());
  result.resize(lpf.size());
  std::vector<float>::const_iterator lpf_itr(lpf.begin());
  for (std::vector<bool>::iterator res_itr(result.begin()),res_last(result.end()); res_itr!=res_last; ++res_itr,++lpf_itr)
    (*res_itr)= (*lpf_itr > sensitivity);
}
void bodies_contact_with_object (std::vector<bool> &result, const float &sensitivity=0.1f)
{
  LASSERT(use_contact_LPF);
  LASSERT(_bodies_contact_with_object_LPF.isInitialized());
  const std::vector<float>  &lpf (_bodies_contact_with_object_LPF());
  result.resize(lpf.size());
  std::vector<float>::const_iterator lpf_itr(lpf.begin());
  for (std::vector<bool>::iterator res_itr(result.begin()),res_last(result.end()); res_itr!=res_last; ++res_itr,++lpf_itr)
    (*res_itr)= (*lpf_itr > sensitivity);
}
//-------------------------------------------------------------------------------------------

void show_robot_info (void)
{
  #ifdef FLOATING_BASE
  std::cerr<<"base joint is floating"<<std::endl;
  #else
  std::cerr<<"base joint is fixed"<<std::endl;
  #endif

  dMass m;
  dReal sum_mass(0.0l), m_body(1.0l);
  for (int j(0); j<BODY_NUM; ++j)
  {
    #if ODE_MINOR_VERSION>=10
      m= body[j].getMass();  // for ode-0.10.1
    #else
      body[j].getMass(&m);  // for ode-0.9
    #endif
    if(j==0) m_body= m.mass;
    std::cerr<<"body "<<j<<": m= "<<std::setprecision(18)<<m.mass<<" kg"
      <<"\t"<<m.mass/m_body << std::endl;
    sum_mass+=m.mass;
  }
  std::cerr<<"total-mass= "<<std::setprecision(18)<<sum_mass<<" kg"<<std::endl;

  #ifdef humanoid_manoi_h
    std::cerr<<"init-body-com-height= "<<std::setprecision(18)<<get_init_body_com_height()<<" m"<<std::endl;
    std::cerr<<"standing-height= "<<std::setprecision(18)<<get_standing_height()<<" m"<<std::endl;
    std::cerr<<"z-pos[head]= "<<std::setprecision(18)<<body[HEADLINK_INDEX].getPosition()[2]<<" m"<<std::endl;
  #endif

  for (int j(0); j<JOINT_NUM; ++j)
  {
    std::cerr<<"joint "<<j<<": ang=["<<joint[j].getParam(dParamLoStop)/M_PI<<" (pi), "
              <<joint[j].getParam(dParamHiStop)/M_PI<<" (pi)]"<<std::endl;
  }

}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// SIMULATION FUNCTIONS
//===========================================================================================

//-------------------------------------------------------------------------------------------
int display_fps_set[]={5,10,20,50,100,200,500};
int display_fps_index=(3);
//-------------------------------------------------------------------------------------------


static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  #ifdef _IGNORING_CONTACT_CODE
    _IGNORING_CONTACT_CODE(data, o1, o2)
  #endif

  // make contact property
  dSurfaceParameters the_surface;
  the_surface= simulationcnd.Surface;
  dContact contact [simulationcnd.MaxContactNum];   // up to MaxContactNum contacts per box-box
  if (int numc = dCollide (o1,o2,simulationcnd.MaxContactNum,&contact[0].geom,sizeof(dContact)))
  {
    for (int i=0; i<numc; i++)
    {
      contact[i].surface= the_surface;
      dJointID c = dJointCreateContact (world.id(),contactgroup.id(),contact+i);
      dJointAttach (c,b1,b2);  // set b1 or b2 to zero - a zero body refers to the static environment
      if (use_contact_points)
        contact_points.push_back(TContactInfo(contact[i].geom.pos, b1, b2));
    }
    for (int j(0); j<BODY_NUM; ++j)
    {
      if( (o1==plane.id() && b2==body[j].id())
        ||(o2==plane.id() && b1==body[j].id()) )  {_bodies_contact_with_ground[j]=1.0f;}
      // if( (o1!=plane.id() && b2==body[j].id())
        // ||(o2!=plane.id() && b1==body[j].id()) )  {_bodies_contact_with_object[j]=1.0f;}
    }
    if (o1!=plane.id() && o2!=plane.id())
    {
      for (int j(0); j<BODY_NUM; ++j)
      {
        if (b1==body[j].id() || b2==body[j].id())
        {
          dBodyID bx= ((b1==body[j].id()) ? b2 : b1);
          bool bx_is_body(false);
          for (int jj(0); jj<BODY_NUM; ++jj)
            if (bx==body[jj].id())  {bx_is_body=true; break;}
          if (!bx_is_body)
            {_bodies_contact_with_object[j]=1.0f;}
          break;
        }
      }
    }
  }
}
//-------------------------------------------------------------------------------------------

inline void initSimulation (void)
{
  create_world();
  SimulationInit.Init();
  world_time= 0.0l;
  LMESSAGE("simulation is initialized.");
}
//-------------------------------------------------------------------------------------------

inline void stepSimulation (const dReal &time_step)
{
  contact_points.clear();
  std::fill (_bodies_contact_with_ground.begin(), _bodies_contact_with_ground.end(), 0.0f);
  std::fill (_bodies_contact_with_object.begin(), _bodies_contact_with_object.end(), 0.0f);
  space.collide (0,&nearCallback);
  if (!simulationcnd.UsingQuickStep)
    world.step (time_step);
  else
  {
    world.setQuickStepNumIterations (simulationcnd.QuickStepIterationNum);
    world.quickStep (time_step);
  }
  world_time+=time_step;
  if (use_contact_LPF)
  {
    static int init(-1);
    if (SimulationInit(init))
    {
      #ifdef humanoid_manoi_h
      if (simulationcnd.ForceInitFeetContactWithGround)
      {
        _bodies_contact_with_ground[LFOOT_INDEX]= 1.0f;
        _bodies_contact_with_ground[RFOOT_INDEX]= 1.0f;
      }
      #endif
      _bodies_contact_with_ground_LPF.Initialize (TLHBPFilters<std::vector<float> >::LPF2,
        time_step, simulationcnd.BodyContactLPFParamF/*f*/, simulationcnd.BodyContactLPFParamQ/*q*/,
        std::vector<float>(BODY_NUM,0.0f), _bodies_contact_with_ground);
      _bodies_contact_with_object_LPF.Initialize (TLHBPFilters<std::vector<float> >::LPF2,
        time_step, simulationcnd.BodyContactLPFParamF/*f*/, simulationcnd.BodyContactLPFParamQ/*q*/,
        std::vector<float>(BODY_NUM,0.0f), _bodies_contact_with_object);
    }
    _bodies_contact_with_ground_LPF (_bodies_contact_with_ground);
    _bodies_contact_with_object_LPF (_bodies_contact_with_object);
  }
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
};  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // sim_tools_h
//-------------------------------------------------------------------------------------------
