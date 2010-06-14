//-------------------------------------------------------------------------------------------
/*! \file    maze2d.h
    \brief   benchmarks - maze2d detailed implementation
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.26, 2009-

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
#ifndef maze2d_h
#define maze2d_h
//-------------------------------------------------------------------------------------------
#include <lora/octave.h>
#include <fstream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

inline ColumnVector CVector2( const double &x, const double &y )
{
  ColumnVector ret(2);
  ret(0) = x;
  ret(1) = y;
  return ret;
}
//-------------------------------------------------------------------------------------------

//! return if two lines (p1x,p1y)-(p2x,p2y) and (q1x,q1y)-(q2x,q2y) cross
template <class T>
inline bool AreTwoLinesCrossing (
  const T &p1x, const T &p1y, const T &p2x, const T &p2y,
  const T &q1x, const T &q1y, const T &q2x, const T &q2y )
{
  static const T zero(GetZero<T>());
  T tmp1, tmp2;
  tmp1 = (q2y-q1y)*(p1x-q1x) - (q2x-q1x)*(p1y-q1y);
  tmp2 = (q2y-q1y)*(p2x-q1x) - (q2x-q1x)*(p2y-q1y);
  if (tmp1*tmp2 > zero)  return false;
  tmp1 = (p2y-p1y)*(q1x-p1x) - (p2x-p1x)*(q1y-p1y);
  tmp2 = (p2y-p1y)*(q2x-p1x) - (p2x-p1x)*(q2y-p1y);
  if (tmp1*tmp2 > zero)  return false;
  return true;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TWall
//===========================================================================================
{
private:
  ColumnVector  w1, w2;
  ColumnVector  uw;
  double  norm;
  void calc_params (void)
    {
      norm=GetNorm(w2-w1);
      if(w2==w1) uw= CVector2(0.0,0.0);
      else       uw=(w2-w1)/norm;
    };
public:
  TWall (const ColumnVector &_w1, const ColumnVector &_w2)  : w1(_w1), w2(_w2) { calc_params(); };
  TWall (const double &_w10, const double &_w11, const double &_w20, const double &_w21)
    {w1=CVector2(_w10,_w11); w2=CVector2(_w20,_w21); calc_params();};

  bool does_cross (const ColumnVector &x1, const ColumnVector &x2) const
    { return AreTwoLinesCrossing(w1(0),w1(1), w2(0),w2(1),  x1(0),x1(1), x2(0),x2(1)); };
  bool apply_effect (const ColumnVector &x, ColumnVector &dx) const
    {
      if (!does_cross(x,x+dx))      return false;
      dx = (dx.transpose()*uw)*uw;  return true;
    };
  const ColumnVector& end1 (void) const { return w1; };
  const ColumnVector& end2 (void) const { return w2; };
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TRealWorld
//===========================================================================================
{
public:
  // configurations:
  TReal          R_WIND1, R_WIND2;
  TReal          MAX_POWER;

  int            MAP_KIND;
  TReal          WIND_FORCE1, WIND_FORCE2;  //!< force of the wind

private:
  const int           xdim_;
  TReal               time_;
  ColumnVector        state_, oldstate_;
  std::vector<TWall>  walls_;

public:

  TRealWorld (int v_xdim=2) :
      R_WIND1        (0.5l),
      R_WIND2        (1.0l),
      MAX_POWER      (0.03l),
      MAP_KIND       (0),
      WIND_FORCE1    (0.01l),
      WIND_FORCE2    (0.08l),
      xdim_      (v_xdim),
      time_      (0.0l),
      state_     (xdim_,0.0l),
      oldstate_  (xdim_,0.0l),
      walls_     ()
    {
    }

  void Init (void)
    {
      walls_.clear();
      if (MAP_KIND<0)  return;
      //! put walls
      #if 0
      walls_.push_back (TWall(-0.70l,  1.00l,  -0.70l,  0.25l));
      walls_.push_back (TWall(-0.80l,  0.00l,  -0.40l,  0.00l));
      walls_.push_back (TWall(-0.40l,  0.00l,  -0.40l,  0.50l));
      walls_.push_back (TWall(-0.50l, -0.50l,  -0.10l, -0.15l));
      walls_.push_back (TWall(-0.10l, -0.15l,  -0.10l,  0.50l));
      walls_.push_back (TWall(-0.10l,  0.50l,   0.10l,  0.50l));
      if (MAP_KIND==0)
      {
        walls_.push_back (TWall( 0.10l,  0.50l,   0.10l, -1.00l));
        walls_.push_back (TWall( 0.40l,  0.50l,   0.40l, -0.30l));
        walls_.push_back (TWall( 0.40l, -0.30l,   0.60l,  0.20l));
        walls_.push_back (TWall( 0.60l,  0.20l,   0.60l,  1.00l));
      }
      else if (MAP_KIND==1)
      {
        // walls_.push_back (TWall( 0.40l,  0.50l,   0.40l,  0.30l));
        // walls_.push_back (TWall( 0.40l,  0.00l,   0.40l, -1.00l));
        // walls_.push_back (TWall( 0.60l, -0.20l,   0.60l,  1.00l));
        walls_.push_back (TWall( 0.10l,  0.50l,   0.10l, -0.00l));
        walls_.push_back (TWall( 0.10l, -0.50l,   0.40l, -0.30l));
        walls_.push_back (TWall( 0.40l, -0.30l,   0.60l,  0.20l));
        walls_.push_back (TWall( 0.60l,  0.20l,   0.60l,  1.00l));
      }
      #endif
      if (MAP_KIND==0 || MAP_KIND==1 || MAP_KIND==10)
      {
        walls_.push_back (TWall(-0.70l,  1.00l,  -0.70l,  0.25l));
        walls_.push_back (TWall(-0.80l,  0.00l,  -0.40l,  0.00l));
        walls_.push_back (TWall(-0.40l,  0.00l,  -0.40l,  0.50l));
        walls_.push_back (TWall(-0.50l, -0.50l,  -0.10l, -0.15l));
        walls_.push_back (TWall(-0.10l, -0.15l,  -0.10l,  0.50l));
        walls_.push_back (TWall(-0.10l,  0.50l,   0.10l,  0.50l));
        if (MAP_KIND==0 || MAP_KIND==10)
        {
          walls_.push_back (TWall( 0.10l,  0.50l,   0.10l, -1.00l));
          walls_.push_back (TWall( 0.40l,  0.50l,   0.40l, -0.30l));
          walls_.push_back (TWall( 0.40l, -0.30l,   0.60l,  0.20l));
          walls_.push_back (TWall( 0.60l,  0.20l,   0.60l,  1.00l));
        }
        else if (MAP_KIND==1)
        {
          // walls_.push_back (TWall( 0.40l,  0.50l,   0.40l,  0.30l));
          // walls_.push_back (TWall( 0.40l,  0.00l,   0.40l, -1.00l));
          // walls_.push_back (TWall( 0.60l, -0.20l,   0.60l,  1.00l));
          walls_.push_back (TWall( 0.10l,  0.50l,   0.10l, -0.00l));
          walls_.push_back (TWall( 0.10l, -0.50l,   0.40l, -0.30l));
          walls_.push_back (TWall( 0.40l, -0.30l,   0.60l,  0.20l));
          walls_.push_back (TWall( 0.60l,  0.20l,   0.60l,  1.00l));
        }
        if (MAP_KIND==10)
        {
          walls_.push_back (TWall( 1.00l,  1.00l,  -1.00l,  1.00l));
          walls_.push_back (TWall(-1.00l,  1.00l,  -1.00l, -1.00l));
          walls_.push_back (TWall(-1.00l, -1.00l,   1.00l, -1.00l));
          walls_.push_back (TWall( 1.00l, -1.00l,   1.00l,  1.00l));
        }
      }
      else if (MAP_KIND==2)
      {
        walls_.push_back (TWall(-0.50l,  1.00l,  -0.50l,  0.00l));
        walls_.push_back (TWall(-0.10l,  0.00l,   0.10l, -1.00l));
        walls_.push_back (TWall( 0.50l,  1.00l,   0.50l,  0.00l));
      }
      else if (MAP_KIND==3)
      {
        walls_.push_back (TWall(-0.10l,  1.00l,   0.10l, -0.20l));
      }
      else
        {LERROR("in TRealWorld::TRealWorld(), invalid MAP_KIND= "<<MAP_KIND); lexit(df);}
    }

  void Reset (const ColumnVector &init_state)
    {
      time_=0.0l;
      state_=init_state;
      oldstate_=state_;
    }

  const ColumnVector& State (void) const {return state_;}

  //! return if s is bad state
  bool IsBadState (const ColumnVector &s) const
    {
      return !(IsIn(s(0),-1.0,1.0) && IsIn(s(1),-1.0,1.0));
    }
  bool IsBadState () const {return IsBadState(state_);}

  //! return wind effect at state s
  ColumnVector Wind (const ColumnVector &s) const
    {
      ColumnVector wnd(xdim_,0.0l);
      double d (real_sqrt(Square(s(0))+Square(s(1))));
      if (IsIn(d,(double)R_WIND1,(double)R_WIND2))
        {wnd(0)=s(0)*WIND_FORCE1/d; wnd(1)=s(1)*WIND_FORCE1/d; return wnd;}
      if (d >= R_WIND2)
        {wnd(0)=s(0)*WIND_FORCE2/d; wnd(1)=s(1)*WIND_FORCE2/d; return wnd;}
      return wnd;
    }

  //! step TimeStep from state with control input u and return step_cost
  TReal Step (const TReal &TimeStep, ColumnVector u)
    {
      oldstate_ = state_;
      if (GetNorm(u)>MAX_POWER)
        u = Normalize(u) * MAX_POWER;
      ColumnVector dx = u + Wind(state_);
      bool contact_with_wall (false);
      for (std::vector<TWall>::const_iterator itr(walls_.begin()); itr!=walls_.end(); ++itr)
        if (itr->apply_effect (state_, dx))
        {
          if (contact_with_wall)  {dx=ColumnVector(xdim_,0.0l); break;}
          itr=walls_.begin();
          contact_with_wall = true;
        }
      state_ += dx;
      time_  += TimeStep;
      TReal step_cost = GetNormSq(u) * TimeStep;
      return step_cost;
    }

  void PrintMapData (const std::string &filename_prefix) const
    {
      std::ofstream wind1   ((filename_prefix+"wind1" ).c_str());
      std::ofstream wind2   ((filename_prefix+"wind2" ).c_str());
      std::ofstream walls   ((filename_prefix+"walls" ).c_str());
      std::ofstream danger  ((filename_prefix+"danger").c_str());

      std::string zero ("0.0");
      if (WIND_FORCE1>0.0)
      {
        for (TReal t(0.0l); t<=REAL_2PI+0.05l; t+=0.05l)
          wind1 << R_WIND1*real_cos(t) << "\t" << R_WIND1*real_sin(t) <<"\t"<<zero<< std::endl;
      }
      if (WIND_FORCE2>0.0)
      {
        for (TReal t(0.0l); t<=REAL_2PI+0.05l; t+=0.05l)
          wind2 << R_WIND2*real_cos(t) << "\t" << R_WIND2*real_sin(t) <<"\t"<<zero<< std::endl;
      }
      for (std::vector<TWall>::const_iterator itr(walls_.begin()); itr!=walls_.end(); ++itr)
      {
        walls << itr->end1()(0) << "\t" << itr->end1()(1) <<"\t"<<zero<< std::endl;
        walls << itr->end2()(0) << "\t" << itr->end2()(1) <<"\t"<<zero<< std::endl << std::endl << std::endl;
      }
      {
        danger << "1.0 \t 1.0"   <<"\t"<<zero<< std::endl;
        danger << "-1.0 \t 1.0"  <<"\t"<<zero<< std::endl;
        danger << "-1.0 \t -1.0" <<"\t"<<zero<< std::endl;
        danger << "1.0 \t -1.0"  <<"\t"<<zero<< std::endl;
        danger << "1.0 \t 1.0"   <<"\t"<<zero<< std::endl;
      }

      wind1   .close();
      wind2   .close();
      walls   .close();
      danger  .close();
    }
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // maze2d_h
//-------------------------------------------------------------------------------------------
