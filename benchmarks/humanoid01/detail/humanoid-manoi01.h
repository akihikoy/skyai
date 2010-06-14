//-------------------------------------------------------------------------------------------
/*! \file
    \brief   benchmarks - create humanoid (manoi) on ODE
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
#ifndef humanoid_manoi01_h
#define humanoid_manoi01_h
//-------------------------------------------------------------------------------------------
#include <lora/ode.h>
#include <cstring>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

static const dReal GRAVITY(9.8);
static const dReal TorqueMax(10.5*GRAVITY/100.0);  // Nm
static const int BODY_NUM(18);
static const int JOINT_NUM(17);
static const int FLESH_NUM(BODY_NUM);
#define FLOATING_BASE
//-------------------------------------------------------------------------------------------
static const int BASELINK_INDEX(0);
static const int HEADLINK_INDEX(1);
static const int LFOOT_INDEX(12);
static const int RFOOT_INDEX(17);
static const int LFOOT_JOINT_PITCH (10);
static const int LFOOT_JOINT_ROLL  (11);
static const int RFOOT_JOINT_PITCH (15);
static const int RFOOT_JOINT_ROLL  (16);
//-------------------------------------------------------------------------------------------
// [[NOTE]] BOX_LINK_NUM+CAPSULE_LINK_NUM+SPHERE_LINK_NUM must be equal to BODY_NUM
static const int BOX_LINK_NUM(4);
static const int CAPSULE_LINK_NUM(10);
static const int SPHERE_LINK_NUM(4);
//-------------------------------------------------------------------------------------------
// dynamics and collision objects
static dWorld world;
static dSimpleSpace space (0);
static dPlane plane;
static dBody body[BODY_NUM];
static dHingeJoint joint[JOINT_NUM];
static dJointGroup contactgroup;
//-------------------------------------------------------------------------------------------
}
#include "def-walls.h"
namespace loco_rabbits {
//-------------------------------------------------------------------------------------------
static dBox     linkBox     [BOX_LINK_NUM];
static dCapsule linkCapsule [CAPSULE_LINK_NUM];
static dSphere  linkSphere  [SPHERE_LINK_NUM];
//-------------------------------------------------------------------------------------------
struct TSphereParam    { dReal r; };
struct TCylinderParam  { dReal r,l; int dir; }; // dir: direction; 1:x, 2:y, 3:z
struct TCapsuleParam   { dReal r,l; int dir; }; // dir: same
struct TBoxParam       { dReal wx, wy, h; };
static TBoxParam     paramBox     [BOX_LINK_NUM];
static TCapsuleParam paramCapsule [CAPSULE_LINK_NUM];
static TSphereParam  paramSphere  [SPHERE_LINK_NUM];
//-------------------------------------------------------------------------------------------
enum TGeomType { gtSphere=0, gtCylinder, gtCapsule, gtBox };
struct TGeomTable
{
  TGeomType  type;
  int        i;
};
static TGeomTable geom_table [BODY_NUM]
  ={/*L 0*/{gtBox, -1},
    /*L 1*/{gtBox, -1},
    /*L 2*/{gtCapsule, -1},
    /*L 3*/{gtCapsule, -1},
    /*L 4*/{gtCapsule, -1},
    /*L 5*/{gtCapsule, -1},
    /*L 6*/{gtCapsule, -1},
    /*L 7*/{gtCapsule, -1},
    /*L 8*/{gtSphere, -1},
    /*L 9*/{gtCapsule, -1},
    /*L10*/{gtCapsule, -1},
    /*L11*/{gtSphere, -1},
    /*L12*/{gtBox, -1},
    /*L13*/{gtSphere, -1},
    /*L14*/{gtCapsule, -1},
    /*L15*/{gtCapsule, -1},
    /*L16*/{gtSphere, -1},
    /*L17*/{gtBox, -1}};

struct TJointStructure
{
  int b[2];
  int dir; // dir: direction; 1:x, 2:y, 3:z
};
static const TJointStructure joint_structure [JOINT_NUM]
  ={/*J 0*/{{0,1},  3},
    /*J 1*/{{0,2},  2},
    /*J 2*/{{2,3},  1},
    /*J 3*/{{3,4},  2},
    /*J 4*/{{0,5},  2},
    /*J 5*/{{5,6},  1},
    /*J 6*/{{6,7},  2},
    /*J 7*/{{0,8},  1},
    /*J 8*/{{8,9},  2},
    /*J 9*/{{9,10}, 2},
    /*J10*/{{10,11},2},
    /*J11*/{{11,12},1},
    /*J12*/{{0,13}, 1},
    /*J13*/{{13,14},2},
    /*J14*/{{14,15},2},
    /*J15*/{{15,16},2},
    /*J16*/{{16,17},1}};
//-------------------------------------------------------------------------------------------
struct TJointRange
{
  dReal Lo, Hi;
};
// static TJointRange joint_range[JOINT_NUM]
  // ={/*J 0*/{-0.5*M_PI, +0.5*M_PI        },
    // /*J 1*/{-0.5*M_PI, +0.99*M_PI       },
    // /*J 2*/{-0.99*M_PI, +1.0/12.0*M_PI  },
    // /*J 3*/{-0.5*M_PI, +0.5*M_PI        },
    // /*J 4*/{-0.5*M_PI, +0.99*M_PI       },
    // /*J 5*/{-1.0/12.0*M_PI, +0.99*M_PI  },
    // /*J 6*/{-0.5*M_PI, +0.5*M_PI        },
    // /*J 7*/{-0.5*M_PI, +1.0/9.0*M_PI    },
    // /*J 8*/{-0.5*M_PI, +0.5*M_PI        },
    // /*J 9*/{-0.78*M_PI, +1.0/18.0*M_PI  },
    // /*J10*/{-0.5*M_PI, +0.5*M_PI        },
    // /*J11*/{-1.0/7.2*M_PI, +0.4*M_PI    },
    // /*J12*/{-1.0/9.0*M_PI, +0.5*M_PI    },
    // /*J13*/{-0.5*M_PI, +0.5*M_PI        },
    // /*J14*/{-0.78*M_PI, +1.0/18.0*M_PI  },
    // /*J15*/{-0.5*M_PI, +0.5*M_PI        },
    // /*J16*/{-0.4*M_PI, +1.0/7.2*M_PI    }};
static TJointRange joint_range[JOINT_NUM]
  ={/*J 0*/{-0.5*M_PI, +0.5*M_PI        },
    /*J 1*/{-0.5*M_PI, +0.99*M_PI       },
    /*J 2*/{-0.99*M_PI, +1.0/12.0*M_PI  },
    /*J 3*/{-0.5*M_PI, +0.5*M_PI        },
    /*J 4*/{-0.5*M_PI, +0.99*M_PI       },
    /*J 5*/{-1.0/12.0*M_PI, +0.99*M_PI  },
    /*J 6*/{-0.5*M_PI, +0.5*M_PI        },
    /*J 7*/{-0.5*M_PI, +1.0/9.0*M_PI    },
    /*J 8*/{-0.5*M_PI, +0.5*M_PI        },
    /*J 9*/{-0.78*M_PI, +1.0/18.0*M_PI  },
    /*J10*/{-M_PI, +M_PI        },
    /*J11*/{-M_PI, +M_PI    },
    /*J12*/{-1.0/9.0*M_PI, +0.5*M_PI    },
    /*J13*/{-0.5*M_PI, +0.5*M_PI        },
    /*J14*/{-0.78*M_PI, +1.0/18.0*M_PI  },
    /*J15*/{-M_PI, +M_PI   },
    /*J16*/{-M_PI, +M_PI   }};
//-------------------------------------------------------------------------------------------
// static double joint_state[JOINT_STATE_DIM];
// static double base_state[BASE_STATE_DIM];
//-------------------------------------------------------------------------------------------
static const dReal param_dha     = 15.50e-3;
static const dReal param_dhb     = 11.77e-3;
static const dReal param_dhc     = 31.00e-3;
static const dReal param_dhd     = 31.44e-3;
static const dReal param_dhe     = 17.61e-3;
static const dReal param_dwya    = 23.00e-3;
static const dReal param_dwyb    = 28.20e-3;
static const dReal param_dwyc    = 23.00e-3;
static const dReal param_dwyd    =  2.50e-3;
static const dReal param_dwxa    =  8.40e-3;
static const dReal param_dwxb    = 10.50e-3;

static const dReal param_rj      = 15.00e-3;  // radius of a sphere displayed at each joint

const dReal _mass_adjust_factor (1.2/0.739700);
const dReal mass[BODY_NUM]
  ={/*L 0*/219.6e-3 *_mass_adjust_factor ,
    /*L 1*/32.30e-3 *_mass_adjust_factor ,
    /*L 2*/ 5.44e-3 *_mass_adjust_factor ,
    /*L 3*/40.16e-3 *_mass_adjust_factor ,
    /*L 4*/37.40e-3 *_mass_adjust_factor ,
    /*L 5*/ 5.44e-3 *_mass_adjust_factor ,
    /*L 6*/40.16e-3 *_mass_adjust_factor ,
    /*L 7*/37.40e-3 *_mass_adjust_factor ,
    /*L 8*/ 8.95e-3 *_mass_adjust_factor ,
    /*L 9*/67.84e-3 *_mass_adjust_factor ,
    /*L10*/40.16e-3 *_mass_adjust_factor ,
    /*L11*/ 8.95e-3 *_mass_adjust_factor ,
    /*L12*/35.00e-3 *_mass_adjust_factor ,
    /*L13*/ 8.95e-3 *_mass_adjust_factor ,
    /*L14*/67.84e-3 *_mass_adjust_factor ,
    /*L15*/40.16e-3 *_mass_adjust_factor ,
    /*L16*/ 8.95e-3 *_mass_adjust_factor ,
    /*L17*/35.00e-3 *_mass_adjust_factor };

static void _set_robot_param(dVector3 com[BODY_NUM], dVector3 janchor[JOINT_NUM])
{
  int si(0),cyi(0),cai(0), bi(0);
  for(int i(0); i<BODY_NUM; ++i)
    switch(geom_table[i].type)
    {
      case gtSphere   : geom_table[i].i= si  ; ++si  ; break;
      case gtCylinder : geom_table[i].i= cyi ; ++cyi ; break;
      case gtCapsule  : geom_table[i].i= cai ; ++cai ; break;
      case gtBox      : geom_table[i].i= bi  ; ++bi  ; break;
    }

  paramBox    [geom_table[ 0].i].wx  =  37.00e-3;
  paramBox    [geom_table[ 0].i].wy  = 105.37e-3;
  paramBox    [geom_table[ 0].i].h   =  93.15e-3;
  paramBox    [geom_table[ 1].i].wx  =  41.00e-3;
  paramBox    [geom_table[ 1].i].wy  =  21.00e-3;
  paramBox    [geom_table[ 1].i].h   =  39.40e-3;

  dReal crad(24.00e-3*0.5);
  paramCapsule[geom_table[ 2].i].r   =  crad;
  paramCapsule[geom_table[ 2].i].l   =  29.61e-3;
  paramCapsule[geom_table[ 2].i].dir =  2;
  paramCapsule[geom_table[ 3].i].r   =  crad;
  paramCapsule[geom_table[ 3].i].l   =  59.50e-3;
  paramCapsule[geom_table[ 3].i].dir =  3;
  paramCapsule[geom_table[ 4].i].r   =  crad;
  paramCapsule[geom_table[ 4].i].l   =  72.27e-3;
  paramCapsule[geom_table[ 4].i].dir =  3;

  paramCapsule[geom_table[ 5].i].r   =  crad;
  paramCapsule[geom_table[ 5].i].l   =  29.61e-3;
  paramCapsule[geom_table[ 5].i].dir =  2;
  paramCapsule[geom_table[ 6].i].r   =  crad;
  paramCapsule[geom_table[ 6].i].l   =  59.50e-3;
  paramCapsule[geom_table[ 6].i].dir =  3;
  paramCapsule[geom_table[ 7].i].r   =  crad;
  paramCapsule[geom_table[ 7].i].l   =  72.27e-3;
  paramCapsule[geom_table[ 7].i].dir =  3;

  paramSphere [geom_table[ 8].i].r   =  27.00e-3;
  paramCapsule[geom_table[ 9].i].r   =  37.15e-3*0.5;
  paramCapsule[geom_table[ 9].i].l   =  57.99e-3;
  paramCapsule[geom_table[ 9].i].dir =  3;
  paramCapsule[geom_table[10].i].r   =  37.15e-3*0.5;
  paramCapsule[geom_table[10].i].l   =  57.48e-3;
  paramCapsule[geom_table[10].i].dir =  3;
  paramSphere [geom_table[11].i].r   =  27.00e-3;
  paramBox    [geom_table[12].i].wx  = 100.00e-3;
  paramBox    [geom_table[12].i].wy  =  61.00e-3;
  paramBox    [geom_table[12].i].h   =  28.52e-3;

  paramSphere [geom_table[13].i].r   =  27.00e-3;
  paramCapsule[geom_table[14].i].r   =  37.15e-3*0.5;
  paramCapsule[geom_table[14].i].l   =  57.99e-3;
  paramCapsule[geom_table[14].i].dir =  3;
  paramCapsule[geom_table[15].i].r   =  37.15e-3*0.5;
  paramCapsule[geom_table[15].i].l   =  57.48e-3;
  paramCapsule[geom_table[15].i].dir =  3;
  paramSphere [geom_table[16].i].r   =  27.00e-3;
  paramBox    [geom_table[17].i].wx  = 100.00e-3;
  paramBox    [geom_table[17].i].wy  =  61.00e-3;
  paramBox    [geom_table[17].i].h   =  28.52e-3;

  // calculate COMs and joint anchors
  #define _l(jj)   paramCapsule[geom_table[jj].i].l
  #define _h(jj)   paramBox[geom_table[jj].i].h
  #define _wx(jj)  paramBox[geom_table[jj].i].wx
  #define _wy(jj)  paramBox[geom_table[jj].i].wy
  const dReal j10z=-0.5*_h(0)-param_dhc-_l(9)-_l(10);
  dVector3  COM[BODY_NUM]
    ={/*L 0*/{0.0,  0.0,                            0.0},
      /*L 1*/{param_dwxa-param_dwxb+0.5*_wx(1),
                    0.0,                            0.5*_h(0)+0.5*_h(1)},
      /*L 2*/{0.0,  0.5*_wy(0)+0.5*_l(2),           0.5*_h(0)-param_dha},
      /*L 3*/{0.0,  0.5*_wy(0)+_l(2),               0.5*_h(0)-param_dha-0.5*_l(3)},
      /*L 4*/{0.0,  0.5*_wy(0)+_l(2),               0.5*_h(0)-param_dha-_l(3)-0.5*_l(4)},
      /*L 5*/{0.0, -(0.5*_wy(0)+0.5*_l(2)),         0.5*_h(0)-param_dha},
      /*L 6*/{0.0, -(0.5*_wy(0)+_l(2)),             0.5*_h(0)-param_dha-0.5*_l(3)},
      /*L 7*/{0.0, -(0.5*_wy(0)+_l(2)),             0.5*_h(0)-param_dha-_l(3)-0.5*_l(4)},
      /*L 8*/{0.0,  param_dwyb,                     -0.5*_h(0)+0.5*param_dhb-0.5*param_dhc},
      /*L 9*/{0.0,  param_dwyb,                     -0.5*_h(0)-param_dhc-0.5*_l(9)},
      /*L10*/{0.0,  param_dwyb,                     -0.5*_h(0)-param_dhc-_l(9)-0.5*_l(10)},
      /*L11*/{0.0,  param_dwyb,                     j10z-0.5*param_dhd},
      /*L12*/{0.0,  param_dwyd+0.5*_wy(12),         j10z-param_dhd-param_dhe+0.5*_h(12)},
      /*L13*/{0.0, -(param_dwyb),                   -0.5*_h(0)+0.5*param_dhb-0.5*param_dhc},
      /*L14*/{0.0, -(param_dwyb),                   -0.5*_h(0)-param_dhc-0.5*_l(9)},
      /*L15*/{0.0, -(param_dwyb),                   -0.5*_h(0)-param_dhc-_l(9)-0.5*_l(10)},
      /*L16*/{0.0, -(param_dwyb),                   j10z-0.5*param_dhd},
      /*L17*/{0.0, -(param_dwyd+0.5*_wy(12)),       j10z-param_dhd-param_dhe+0.5*_h(12)}};
  dVector3  jAnchor[JOINT_NUM]
    ={/*J 0*/{param_dwxa,  0.0,                     0.5*_h(0)},
      /*J 1*/{0.0,  0.5*_wy(0),                     0.5*_h(0)-param_dha},
      /*J 2*/{0.0,  0.5*_wy(0)+_l(2),               0.5*_h(0)-param_dha},
      /*J 3*/{0.0,  0.5*_wy(0)+_l(2),               0.5*_h(0)-param_dha-_l(3)},
      /*J 4*/{0.0, -(0.5*_wy(0)),                   0.5*_h(0)-param_dha},
      /*J 5*/{0.0, -(0.5*_wy(0)+_l(2)),             0.5*_h(0)-param_dha},
      /*J 6*/{0.0, -(0.5*_wy(0)+_l(2)),             0.5*_h(0)-param_dha-_l(3)},
      /*J 7*/{0.0,  param_dwya,                     -0.5*_h(0)+param_dhb},
      /*J 8*/{0.0,  param_dwyb,                     -0.5*_h(0)-param_dhc},
      /*J 9*/{0.0,  param_dwyb,                     -0.5*_h(0)-param_dhc-_l(9)},
      /*J10*/{0.0,  param_dwyb,                     j10z},
      /*J11*/{0.0,  param_dwyc,                     j10z-param_dhd},
      /*J12*/{0.0, -(param_dwya),                   -0.5*_h(0)+param_dhb},
      /*J13*/{0.0, -(param_dwyb),                   -0.5*_h(0)-param_dhc},
      /*J14*/{0.0, -(param_dwyb),                   -0.5*_h(0)-param_dhc-_l(9)},
      /*J15*/{0.0, -(param_dwyb),                   j10z},
      /*J16*/{0.0, -(param_dwyc),                   j10z-param_dhd}};
  #undef _l
  #undef _h
  #undef _wx
  #undef _wy
  std::memcpy(com,COM,sizeof(COM));
  std::memcpy(janchor,jAnchor,sizeof(jAnchor));
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
};  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // humanoid_manoi01_h
//-------------------------------------------------------------------------------------------
