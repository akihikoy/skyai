//-------------------------------------------------------------------------------------------
/*! \file    def-walls.h
    \brief   benchmarks - define walls to make a maze on ODE
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.14, 2009-

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
#ifndef def_walls_h
#define def_walls_h
//-------------------------------------------------------------------------------------------
#include <list>
#include <lora/ode.h>
#include <lora/ode_ds.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


static dReal MAZESCALE = 5.0;
static int   MAP_KIND (-1);

class TWall
{
private:
  typedef dReal TPoint[2];
  TPoint   w1, w2;
  dBox     geom;
public:
  TWall (const dReal &_w10, const dReal &_w11, const dReal &_w20, const dReal &_w21)
    {
      w1[0]=_w10*MAZESCALE; w1[1]=_w11*MAZESCALE; w2[0]=_w20*MAZESCALE; w2[1]=_w21*MAZESCALE;
    };
  TWall (const TWall &wall)
    {w1[0]=wall.w1[0]; w1[1]=wall.w1[1]; w2[0]=wall.w2[0]; w2[1]=wall.w2[1];}

  void create (dSimpleSpace &_space)
    {
      const dReal WALL_HEIGHT = 1.0 *MAZESCALE/5.0;
      const dReal WALL_DEPTH  = 0.02 *MAZESCALE/5.0;
      dReal WX= real_sqrt(Square(w2[0]-w1[0])+Square(w2[1]-w1[1]));
      geom.create (_space,WX,WALL_DEPTH,WALL_HEIGHT);
      dMatrix3 R;
      dRFromAxisAndAngle (R, 0.0,0.0,1.0, real_atan2(w2[1]-w1[1],w2[0]-w1[0]));
      geom.setPosition (0.5*(w2[0]+w1[0]),0.5*(w2[1]+w1[1]),0.5*WALL_HEIGHT);
      geom.setRotation (R);
    };
  void draw (void) const
    {
      dsSetTexture (DS_WOOD);
      dsSetColorAlpha (0.8, 0.8, 0.0, 0.8);
      dReal sides[4];
      geom.getLengths(sides);
      dsDrawBox (geom.getPosition(),geom.getRotation(),sides);
    };

  const dReal* end1 (void) const { return w1; };
  const dReal* end2 (void) const { return w2; };
  dGeomID      id   (void) const { return geom.id(); };
};
typedef std::list<TWall>  TMaze;
TMaze walls;
//-------------------------------------------------------------------------------------------

void init_walls(dSimpleSpace &_space, int map_kind=-1)
{
  walls.clear();

  if (!IsIn(map_kind,-1,5) && map_kind!=20)
    {LERROR("in init_walls(), invalid map_kind= "<<map_kind); exit(1);}

  if (map_kind<0)  return;

  walls.push_back (TWall(-1.00l, -1.00l,  -1.00l,  1.00l));
  walls.push_back (TWall(-1.00l,  1.00l,   1.00l,  1.00l));
  walls.push_back (TWall( 1.00l,  1.00l,   1.00l, -1.00l));
  walls.push_back (TWall( 1.00l, -1.00l,  -1.00l, -1.00l));

  if (map_kind==0 || map_kind==1)
  {
    walls.push_back (TWall(-0.70l,  1.00l,  -0.70l,  0.25l));
    walls.push_back (TWall(-0.80l,  0.00l,  -0.40l,  0.00l));
    walls.push_back (TWall(-0.40l,  0.00l,  -0.40l,  0.50l));
    walls.push_back (TWall(-0.50l, -0.50l,  -0.10l, -0.15l));
    walls.push_back (TWall(-0.10l, -0.15l,  -0.10l,  0.50l));
    walls.push_back (TWall(-0.10l,  0.50l,   0.10l,  0.50l));
    if (map_kind==0)
    {
      walls.push_back (TWall( 0.10l,  0.50l,   0.10l, -1.00l));
      walls.push_back (TWall( 0.40l,  0.50l,   0.40l, -0.30l));
      walls.push_back (TWall( 0.40l, -0.30l,   0.60l,  0.20l));
      walls.push_back (TWall( 0.60l,  0.20l,   0.60l,  1.00l));
    }
    else if (map_kind==1)
    {
      walls.push_back (TWall( 0.10l,  0.50l,   0.10l, -0.00l));
      walls.push_back (TWall( 0.10l, -0.50l,   0.40l, -0.30l));
      walls.push_back (TWall( 0.40l, -0.30l,   0.60l,  0.20l));
      walls.push_back (TWall( 0.60l,  0.20l,   0.60l,  1.00l));
    }
  }
  else if (map_kind==2)
  {
    walls.push_back (TWall(-0.50l,  1.00l,  -0.50l,  0.00l));
    walls.push_back (TWall(-0.10l,  0.00l,   0.10l, -1.00l));
    walls.push_back (TWall( 0.50l,  1.00l,   0.50l,  0.00l));
  }
  else if (map_kind==3)
  {
    walls.push_back (TWall(-0.10l,  1.00l,   0.10l, -0.20l));
  }
  else if (map_kind==4)
  {
    walls.push_back (TWall(-0.20l, -1.00l,   0.20l,  0.20l));
    walls.push_back (TWall( 0.20l, -1.00l,   0.20l,  0.20l));
    walls.push_back (TWall(-0.70l,  1.00l,  -0.20l,  0.60l));
    walls.push_back (TWall( 0.30l,  1.00l,  -0.20l,  0.60l));
  }
  else if (map_kind==20)
  {
  }

  for(TMaze::iterator itr(walls.begin());itr!=walls.end();++itr)
    itr->create(_space);
}
//-------------------------------------------------------------------------------------------

#define _IGNORING_CONTACT_CODE(_data, _o1, _o2) \
  if(_o1==plane || _o2==plane){ \
    dGeomID obj=(_o1!=plane ? _o1 : _o2); \
    for(TMaze::const_iterator itr(walls.begin());itr!=walls.end();++itr) \
      if(itr->id()==obj)  return;}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // def_walls_h
//-------------------------------------------------------------------------------------------
