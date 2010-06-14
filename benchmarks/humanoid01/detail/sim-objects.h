//-------------------------------------------------------------------------------------------
/*! \file
    \brief   benchmarks - create objects for ODE
    \author  Akihiko Yamaguchi
    \date    April 26, 2008-

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
#ifndef sim_objects_h
#define sim_objects_h
//-------------------------------------------------------------------------------------------
#include <iostream>
#include <list>
#include <vector>
//-------------------------------------------------------------------------------------------
#include <lora/ode.h>
#include <lora/ode_ds.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
extern dWorld world;
extern dSimpleSpace space;
//-------------------------------------------------------------------------------------------

namespace sim_objects
{
//-------------------------------------------------------------------------------------------

template <typename T>
class _TListInterface
{
public:
  typedef typename std::list<T>::iterator         iterator          ;
  typedef typename std::list<T>::const_iterator   const_iterator    ;
  typedef typename std::list<T>::reference        reference         ;
  typedef typename std::list<T>::const_reference  const_reference   ;
protected:
  std::list<T>  ids;
  virtual void destroy (T&id) const = 0;
public:
  _TListInterface(void) {};
  void push_back()  {ids.push_back(T(0));};

  iterator begin(void)  {return ids.begin();};
  const_iterator begin(void) const {return ids.begin();};
  iterator end(void)  {return ids.end();};
  const_iterator end(void) const {return ids.end();};

  reference front(void)  {return ids.front();};
  const_reference front(void) const {return ids.front();};
  reference back(void)  {return ids.back();};
  const_reference back(void) const {return ids.back();};

  void clear(void)
    {
      for(iterator itr(ids.begin()); itr!=ids.end(); ++itr)  destroy(*itr);
      ids.clear();
    };
};
//-------------------------------------------------------------------------------------------

class TGeomParticle
{
public:
  dGeomID  id;
  float    color[4];
  TGeomParticle(void) : id(0) {color[0]=color[1]=color[2]=color[3]=1.0;};
  TGeomParticle(int rhs) : id(0) {color[0]=color[1]=color[2]=color[3]=1.0; operator=(rhs);};
  const TGeomParticle& operator=(int rhs)
    {
      if (rhs==0)
        id=NULL;
      else
        {std::cerr<<"invalid assignment in TGeomParticle::operator="<<std::endl;exit(1);}
      return *this;
    };
};
//-------------------------------------------------------------------------------------------

class TBodyList : public _TListInterface<dBodyID>
{
protected:
  /*override*/void destroy (dBodyID &id) const {if(id) dBodyDestroy(id);};
public:
  TBodyList(void) : _TListInterface<dBodyID>() {};
};
//-------------------------------------------------------------------------------------------

class TGeomList : public _TListInterface<TGeomParticle>
{
protected:
  /*override*/void destroy (TGeomParticle &x) const {if(x.id) dGeomDestroy(x.id);};
public:
  TGeomList(void) : _TListInterface<TGeomParticle>() {};
};
//-------------------------------------------------------------------------------------------

class TJointList : public _TListInterface<dJointID>
{
protected:
  /*override*/void destroy (dJointID &id) const {if(id) dJointDestroy(id);};
public:
  TJointList(void) : _TListInterface<dJointID>() {};
};
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
// dynamics and collision objects
//-------------------------------------------------------------------------------------------
static TBodyList    simBody     ;
static TJointList   simJoint;
//-------------------------------------------------------------------------------------------
static TGeomList    simBox     ;
static TGeomList    simCapsule ;
static TGeomList    simSphere  ;
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
const dReal  FMAX      = 25.0; // car engine fmax
//-------------------------------------------------------------------------------------------

struct TCarParticle
{
  dBodyID  body;
  dJointID wheels[4];

  void addForce(const dReal &fx, const dReal &fy, const dReal &fz)
    {
      dBodyAddForce(body,fx,fy,fz);
    };
};
//-------------------------------------------------------------------------------------------

class TCarList
{
private:
  std::vector<TCarParticle>  cars;
public:
  int size(void) const {return cars.size();};
  void push_back(void)  {cars.push_back(TCarParticle());};
  void clear(void)  {cars.clear();}

  TCarParticle& operator[](int i)  {return cars[i];};
  const TCarParticle& operator[](int i) const {return cars[i];};
};
TCarList  CarList;
//-------------------------------------------------------------------------------------------

void makeCar(dReal x, dReal y)
{
  const dReal  factor (0.1);
  const dReal  LENGTH    = 3.5*factor ; // chassis length
  const dReal  WIDTH     = 2.5*factor ; // chassis width
  const dReal  HEIGHT    = 2.0*factor ; // chassis height
  const dReal  RADIUS    = 0.5*factor ; // wheel radius
  const dReal  STARTZ    = 0.5*HEIGHT+RADIUS ; // starting height of chassis
//   const dReal  CMASS     = 1.0 ; // chassis mass
//   const dReal  WMASS     = 1.0 ; // wheel mass
  const dReal  CMASS     = 5.0 ;  // chassis mass
  const dReal  WMASS     = 5.0 ; // wheel mass
  const dReal  COMOFFSET = -5.0*factor; // center of mass offset

  int i;
  dMass m;

  CarList.push_back();

  TBodyList::iterator  ibody[5];
  for (i=0; i<5; ++i)
  {
    simBody.push_back();
    ibody[i]= simBody.end();
    --ibody[i];
  }

  // chassis body
  *(ibody[0])= dBodyCreate (world);
  dBodySetPosition (*(ibody[0]),x,y,STARTZ);
  dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
  dMassAdjust (&m,CMASS/2.0);
  dBodySetMass (*(ibody[0]),&m);

  simBox.push_back();
  simBox.back().id= dCreateBox(space,LENGTH,WIDTH,HEIGHT);
  dGeomSetBody (simBox.back().id,*(ibody[0]));
  simBox.back().color[0]= 0.0;
  simBox.back().color[1]= 0.8;
  simBox.back().color[2]= 0.0;
  simBox.back().color[3]= 0.8;

  CarList[CarList.size()-1].body= *(ibody[0]);

  // wheel bodies
  for (i=1; i<=4; i++)
  {
    *(ibody[i]) = dBodyCreate (world);
    dQuaternion q;
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
    dBodySetQuaternion (*(ibody[i]),q);
    dMassSetSphere (&m,1,RADIUS);
    dMassAdjust (&m,WMASS);
    dBodySetMass (*(ibody[i]),&m);
    simSphere.push_back();
    simSphere.back().id = dCreateSphere (space,RADIUS);
    dGeomSetBody (simSphere.back().id,*(ibody[i]));
    simSphere.back().color[0]= 0.8;
    simSphere.back().color[1]= 0.8;
    simSphere.back().color[2]= 0.8;
    simSphere.back().color[3]= 0.8;
  }
  dBodySetPosition (*(ibody[1]),x+0.4*LENGTH-0.5*RADIUS,y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
  dBodySetPosition (*(ibody[2]),x+0.4*LENGTH-0.5*RADIUS,y-WIDTH*0.5,STARTZ-HEIGHT*0.5);
  dBodySetPosition (*(ibody[3]),x-0.4*LENGTH+0.5*RADIUS,y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
  dBodySetPosition (*(ibody[4]),x-0.4*LENGTH+0.5*RADIUS,y-WIDTH*0.5,STARTZ-HEIGHT*0.5);

  for (i=0; i<4; i++)
  {
    simJoint.push_back();
    simJoint.back() = dJointCreateHinge (world,0);
    dJointAttach (simJoint.back(),*(ibody[0]),*(ibody[i+1]));
    const dReal *a = dBodyGetPosition (*(ibody[i+1]));
    dJointSetHingeAnchor (simJoint.back(),a[0],a[1],a[2]);
    // dJointSetHingeAxis1 (simJoint.back(),0,0,(i<2 ? 1 : -1));
    dJointSetHingeAxis (simJoint.back(),0,1,0);
    // dJointSetHingeParam (simJoint.back(),dParamSuspensionERP,0.8);
    // dJointSetHingeParam (simJoint.back(),dParamSuspensionCFM,1e-5);
    // dJointSetHingeParam (simJoint.back(),dParamVel2,0);
    // dJointSetHingeParam (simJoint.back(),dParamFMax2,FMAX);
    CarList[CarList.size()-1].wheels[i]= simJoint.back();
  }

  //center of mass offset body. (hang another copy of the body COMOFFSET units below it by a fixed joint)
  dBodyID b = dBodyCreate (world);
  dBodySetPosition (b,x,y,STARTZ+COMOFFSET);
  dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
  dMassAdjust (&m,CMASS/2.0);
  dBodySetMass (b,&m);
  dJointID j = dJointCreateFixed(world, 0);
  dJointAttach(j, *(ibody[0]), b);
  dJointSetFixed(j);
}
//-------------------------------------------------------------------------------------------



void setup_world (void)
{
  simBody    .clear();
  simJoint   .clear();
  simBox     .clear();
  simCapsule .clear();
  simSphere  .clear();
  CarList    .clear();
}
//-------------------------------------------------------------------------------------------



void draw_world (bool pause=false)
{
  dReal rad, len;
  dReal sides[4];
  dsSetTexture (DS_NONE);

  for (TGeomList::const_iterator itr(simSphere.begin()); itr!=simSphere.end(); ++itr)
  {
    dsSetColorAlpha (itr->color[0], itr->color[1], itr->color[2], itr->color[3]);
    dsDrawSphere (dGeomGetPosition(itr->id),dGeomGetRotation(itr->id),dGeomSphereGetRadius(itr->id));
  }
  for (TGeomList::const_iterator itr(simCapsule.begin()); itr!=simCapsule.end(); ++itr)
  {
    dsSetColorAlpha (itr->color[0], itr->color[1], itr->color[2], itr->color[3]);
    dGeomCapsuleGetParams (itr->id,&rad, &len);
    dsDrawCapsule (dGeomGetPosition(itr->id),dGeomGetRotation(itr->id),len,rad);
  }
  for (TGeomList::const_iterator itr(simBox.begin()); itr!=simBox.end(); ++itr)
  {
    dsSetColorAlpha (itr->color[0], itr->color[1], itr->color[2], itr->color[3]);
    dGeomBoxGetLengths (itr->id,sides);
    dsDrawBox (dGeomGetPosition(itr->id),dGeomGetRotation(itr->id),sides);
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
};  // end of namespace sim_objects
//-------------------------------------------------------------------------------------------
};  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // sim_objects_h
//-------------------------------------------------------------------------------------------
