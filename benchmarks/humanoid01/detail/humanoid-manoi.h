//-------------------------------------------------------------------------------------------
/*! \file
    \brief   benchmarks - create humanoid (manoi) on ODE
    \author  Akihiko Yamaguchi
    \date    Oct 8, 2008-

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
#ifndef humanoid_manoi_h
#define humanoid_manoi_h
//-------------------------------------------------------------------------------------------
#define SIM_OBJECT_DEFS
//-------------------------------------------------------------------------------------------
#include "humanoid-manoi01.h"
//-------------------------------------------------------------------------------------------
#include <iostream>
#include <list>
#include <lora/math.h>
//-------------------------------------------------------------------------------------------
#include <lora/ode.h>
#include <lora/ode_ds.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! \brief initial com-height of the body link
inline dReal get_init_body_com_height (void)
//===========================================================================================
{
  #define _l(jj)   paramCapsule[geom_table[jj].i].l
  #define _h(jj)   paramBox[geom_table[jj].i].h
  return 0.5*_h(0)+_l(9)+_l(10)+param_dhc+param_dhd+param_dhe;
  #undef _l
  #undef _h
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! \brief get height of the robot
inline dReal get_standing_height (void)
//===========================================================================================
{
  #define _l(jj)   paramCapsule[geom_table[jj].i].l
  #define _h(jj)   paramBox[geom_table[jj].i].h
  const dReal cz= get_init_body_com_height();
  return cz + 0.5*_h(0)+_h(1);
  #undef _l
  #undef _h
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! \brief create simulation objects
static void create_world (void)
//===========================================================================================
{
#ifndef FLOATING_MODE
  const dReal floating=0.0e-3;
#else
  const dReal floating=100.0e-3;
#endif

  dVector3  COM[BODY_NUM];
  dVector3  jAnchor[JOINT_NUM];
  _set_robot_param (COM, jAnchor);

  const dReal cx=0.0, cy=0.0, cz=floating+get_init_body_com_height();

  int j;
  contactgroup.create ();
#ifndef FLOATING_MODE
  world.setGravity (0,0,-GRAVITY);  // gravity [m/s^2]
#else
  world.setGravity (0,0,0.0);  // gravity [m/s^2]
#endif
  dWorldSetCFM (world.id(),1e-5);
  plane.create (space,0,0,1,0); // ground (plane)

  for(j=0; j<BODY_NUM; ++j)
  {
    const int i=geom_table[j].i;
    const dReal density(1.0);
    body[j].create (world);
    body[j].setPosition (cx+COM[j][0], cy+COM[j][1], cz+COM[j][2]);
    dMass m;
    dReal rad,len;
    switch(geom_table[j].type)
    {
      case gtSphere :
        m.setSphere (density,paramSphere[i].r);
        m.adjust (mass[j]);
        body[j].setMass (&m);
        linkSphere[i].create (space,paramSphere[i].r);
        linkSphere[i].setBody (body[j]);
        break;
      case gtCapsule :
        dMatrix3 R;
        dRSetIdentity (R);
        if(paramCapsule[i].dir==1)
          dRFromAxisAndAngle (R,0.0,1.0,0.0,0.5*M_PI);
        else if(paramCapsule[i].dir==2)
          dRFromAxisAndAngle (R,1.0,0.0,0.0,0.5*M_PI);
        body[j].setRotation (R);
        rad=paramCapsule[i].r;
        len=paramCapsule[i].l-2.0*rad;
        m.setCapsule (density,paramCapsule[i].dir,rad,len);
        m.adjust (mass[j]);
        body[j].setMass (&m);
        linkCapsule[i].create (space,rad,len);
        linkCapsule[i].setBody (body[j]);
        break;
      case gtBox :
        m.setBox (density,paramBox[i].wx,paramBox[i].wy,paramBox[i].h);
        m.adjust (mass[j]);
        body[j].setMass (&m);
        linkBox[i].create (space,paramBox[i].wx,paramBox[i].wy,paramBox[i].h);
        linkBox[i].setBody (body[j]);
        break;
      default:
        std::cerr<<"fatal!"<<std::endl;
        exit(1);
    }
  }

  for(j=0; j<JOINT_NUM; ++j)
  {
    joint[j].create (world);
    joint[j].attach (body[joint_structure[j].b[0]],body[joint_structure[j].b[1]]);
    joint[j].setAnchor (cx+jAnchor[j][0],cy+jAnchor[j][1],cz+jAnchor[j][2]);
    if(joint_structure[j].dir==1)
      joint[j].setAxis (1.0,0.0,0.0);
    else if(joint_structure[j].dir==2)
      joint[j].setAxis (0.0,1.0,0.0);
    else if(joint_structure[j].dir==3)
      joint[j].setAxis (0.0,0.0,1.0);
    else
    {
      std::cerr<<"fatal!"<<std::endl;
      exit(1);
    }
    joint[j].setParam (dParamFudgeFactor,0.8);
    joint[j].setParam (dParamHiStop, joint_range[j].Hi);
    joint[j].setParam (dParamLoStop, joint_range[j].Lo);
  }

  init_walls(space,MAP_KIND);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
static bool DRAW_COM_MODE (false);
void getCOM (dVector3 com);
//-------------------------------------------------------------------------------------------
static int AUTO_VIEWPOINT_MODE (0);
//-------------------------------------------------------------------------------------------
static bool DRAW_SEQUENCE_MODE (false);
float _CURRENT_COLOR[4];
inline void _dsSetColorAlpha (const float &red, const float &green, const float &blue, const float &Alpha)
{
  const float alpha_red(1.0f);
  if (DRAW_SEQUENCE_MODE)
  {
    _CURRENT_COLOR[0] = red;
    _CURRENT_COLOR[1] = green;
    _CURRENT_COLOR[2] = blue;
    _CURRENT_COLOR[3] = Alpha * alpha_red;
  }
  dsSetColorAlpha(red, green, blue, Alpha);
}
//-------------------------------------------------------------------------------------------
struct TGeomTable2
{
  TGeomType       type;
  union
  {
    TSphereParam    sp;
    TCylinderParam  cl;
    TCapsuleParam   cp;
    //TBoxParam       bx;
    dVector3        bx;
  };
  dVector3 pos;
  dMatrix3 Rot;
  float color[4];

  void setPosRotCol (const dVector3 p, const dMatrix3 r)
    {
      memcpy(pos,p,sizeof(dVector3));
      memcpy(Rot,r,sizeof(dMatrix3));
      memcpy(color,_CURRENT_COLOR,sizeof(color));
    };
  TGeomTable2 (const dVector3 p, const dMatrix3 r, const dReal &rad)
    {
      type= gtSphere;
      sp.r= rad;
      setPosRotCol(p,r);
    };
  TGeomTable2 (const dVector3 p, const dMatrix3 r, const dReal &len, const dReal &rad)
    {
      type= gtCapsule;
      cp.r= rad;
      cp.l= len;
      setPosRotCol(p,r);
    };
  TGeomTable2 (const dVector3 p, const dMatrix3 r, const dVector3 sides)
    {
      type= gtBox;
      memcpy(bx,sides,sizeof(dVector3));
      setPosRotCol(p,r);
    };

  void draw(void) const
    {
      dsSetColorAlpha(color[0], color[1], color[2], color[3]);
      switch(type)
      {
        case gtSphere :
          dsDrawSphere (pos,Rot,sp.r);
          break;
        case gtCapsule :
          dsDrawCapsule (pos,Rot,cp.l,cp.r);
          break;
        case gtBox :
          dsDrawBox (pos,Rot,bx);
          break;
        default:
          std::cerr<<"fatal!"<<std::endl;
          exit(1);
      }
    };
};
//-------------------------------------------------------------------------------------------
std::list<TGeomTable2>  _SEQUENCE_LIST;  // afterimage
//-------------------------------------------------------------------------------------------


//===========================================================================================
static void draw_world (bool pause=false)
//===========================================================================================
{
  SetAutoViewpointMode (AUTO_VIEWPOINT_MODE, body[BASELINK_INDEX].getPosition());
  int j;
  dsSetColor (0,0.5,1);
  dsSetTexture (DS_WOOD);
  dReal rad, len;
  dReal sides[4];
  dVector3 pos;
  dBox *blink;
  dCapsule *clink;
  dSphere *slink;
  dsSetTexture (DS_NONE);
  if (DRAW_SEQUENCE_MODE)
  {
    //dsSetColorAlpha (1.0, 1.0, 1.0, 0.7);
    dsSetColorAlpha (0.8, 0.7, 0.8, 0.8);
    for(std::list<TGeomTable2>::const_iterator itr(_SEQUENCE_LIST.begin()); itr!=_SEQUENCE_LIST.end(); ++itr)
      itr->draw();
  }
  if (DRAW_COM_MODE)
  {
    dVector3 com; getCOM(com);
    _dsSetColorAlpha (0.0, 0.5, 0.8, 0.6);
    dsDrawSphere (com,body[0].getRotation(),20.0e-3);
  }
  for(j=0; j<BODY_NUM; ++j)
  {
    if(j==1)
      _dsSetColorAlpha (0.2, 0.2, 0.7, 0.8);
    else
      _dsSetColorAlpha (0.4, 0.45, 0.6, 0.8);
    const int i=geom_table[j].i;
    switch(geom_table[j].type)
    {
      case gtSphere :
        slink=&linkSphere[i];
        dsDrawSphere (slink->getPosition(),slink->getRotation(),slink->getRadius());
        if(!pause&&DRAW_SEQUENCE_MODE) _SEQUENCE_LIST.push_back(TGeomTable2(slink->getPosition(),slink->getRotation(),slink->getRadius()));
        break;
      case gtCapsule :
        clink=&linkCapsule[i];
        clink->getParams(&rad, &len);
        dsDrawCapsule (clink->getPosition(),clink->getRotation(),len,rad);
        if(!pause&&DRAW_SEQUENCE_MODE) _SEQUENCE_LIST.push_back(TGeomTable2(clink->getPosition(),clink->getRotation(),len,rad));
        break;
      case gtBox :
        blink=&linkBox[i];
        blink->getLengths(sides);
        dsDrawBox (blink->getPosition(),blink->getRotation(),sides);
        if(!pause&&DRAW_SEQUENCE_MODE) _SEQUENCE_LIST.push_back(TGeomTable2(blink->getPosition(),blink->getRotation(),sides));
        break;
      default:
        std::cerr<<"fatal!"<<std::endl;
        exit(1);
    }
  }
  _dsSetColorAlpha (1.0, 0.2, 0.2, 0.6);
  int DISPLAY_JOINTS[] = { 1,2,3, 4,5,6, 8,9,10, 13,14,15};
  for (size_t i=0; i<sizeof(DISPLAY_JOINTS)/sizeof(int); ++i)
  {
    j=DISPLAY_JOINTS[i]; joint[j].getAnchor(pos); dsDrawSphere(pos, body[0].getRotation(), param_rj);
    if(!pause&&DRAW_SEQUENCE_MODE) _SEQUENCE_LIST.push_back(TGeomTable2(pos, body[0].getRotation(), param_rj));
  }

  for(TMaze::const_iterator itr(walls.begin());itr!=walls.end();++itr)
    itr->draw();
}
//-------------------------------------------------------------------------------------------

//! \brief start simulation - set viewpoint
static void start()
{
  // static float xyz[3] = {0.75,1.3,1.0};
  // static float hpr[3] = {-120.0,-16.0,0.0};
  static float xyz[3] = {0.35,0.30,0.30};
  static float hpr[3] = {-135.0000,-20.0000,0.0000};

  dsSetViewpoint (xyz,hpr);
  dsSetSphereQuality (3);
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
};  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // humanoid_manoi_h
//-------------------------------------------------------------------------------------------
