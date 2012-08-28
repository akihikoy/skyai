//-------------------------------------------------------------------------------------------
/*! \file    cv.h
    \brief   liblora - OpenCV supplementary (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.28, 2012

    Copyright (C) 2012  Akihiko Yamaguchi

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
#ifndef loco_rabbits_cv_h
#define loco_rabbits_cv_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <cv.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

inline void RotCounterClockwise(cv::Mat &m)
{
  cv::transpose(m,m);
  cv::flip(m,m,0);
}
//-------------------------------------------------------------------------------------------

inline void RotClockwise(cv::Mat &m)
{
  cv::transpose(m,m);
  cv::flip(m,m,1);
}
//-------------------------------------------------------------------------------------------

template <typename t_elem>
inline cv::Mat_<t_elem> GetWedge (const  cv::Vec<t_elem,3> &w)
{
  cv::Mat_<t_elem> wedge(3,3);
  wedge(0,0)=0.0;    wedge(0,1)=-w(2);  wedge(0,2)=w(1);
  wedge(1,0)=w(2);   wedge(1,1)=0.0;    wedge(1,2)=-w(0);
  wedge(2,0)=-w(1);  wedge(2,1)=w(0);   wedge(2,2)=0.0;
  return wedge;
}
//-------------------------------------------------------------------------------------------

template <typename t_elem>
inline cv::Mat_<t_elem> Rodrigues (const cv::Vec<t_elem,3> &w, const TReal &epsilon=1.0e-6l)
{
  double th= norm(w);
  if(th<epsilon)  return cv::Mat_<t_elem>::eye(3,3);
  cv::Mat_<t_elem> w_wedge(3,3);
  w_wedge= GetWedge(w *(1.0/th));
  return cv::Mat_<t_elem>::eye(3,3) + w_wedge * std::sin(th) + w_wedge * w_wedge * (1.0-std::cos(th));
}
//-------------------------------------------------------------------------------------------

template <typename t_elem>
inline cv::Vec<t_elem,3> InvRodrigues (const cv::Mat_<t_elem> &R, const TReal &epsilon=1.0e-6l)
{
  double alpha= (R(0,0)+R(1,1)+R(2,2) - 1.0) / 2.0;;

  if((alpha-1.0 < epsilon) && (alpha-1.0 > -epsilon))
    return cv::Vec<t_elem,3>(0.0,0.0,0.0);
  else
  {
    cv::Vec<t_elem,3> w;
    double th = std::acos(alpha);
    double tmp= 0.5 * th / std::sin(th);
    w[0] = tmp * (R(2,1) - R(1,2));
    w[1] = tmp * (R(0,2) - R(2,0));
    w[2] = tmp * (R(1,0) - R(0,1));
    return w;
  }
}
//-------------------------------------------------------------------------------------------

template <typename t_elem>
inline cv::Mat_<t_elem> AverageRotations (const cv::Mat_<t_elem> &R1, const cv::Mat_<t_elem> &R2, const t_elem &w2)
{
  cv::Vec<t_elem,3> w= InvRodrigues(cv::Mat_<double>(R2*R1.t()));
  return Rodrigues(w2*w)*R1;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_cv_h
//-------------------------------------------------------------------------------------------
