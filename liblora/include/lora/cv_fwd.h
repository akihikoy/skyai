//-------------------------------------------------------------------------------------------
/*! \file    cv_fwd.h
    \brief   liblora - certain program (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.30, 2012

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
#ifndef loco_rabbits_cv_fwd_h
#define loco_rabbits_cv_fwd_h
//-------------------------------------------------------------------------------------------
#include <opencv/cv.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//! for Matx, Vec
template <typename t_elem, int t_rows, int t_cols>
inline t_elem*  CVBegin (cv::Matx<t_elem,t_rows,t_cols> &x)
{
  return x.val;
}
template <typename t_elem, int t_rows, int t_cols>
inline const t_elem*  CVBegin (const cv::Matx<t_elem,t_rows,t_cols> &x)
{
  return x.val;
}
template <typename t_elem, int t_rows, int t_cols>
inline t_elem*  CVEnd (cv::Matx<t_elem,t_rows,t_cols> &x)
{
  return x.val+t_rows*t_cols;
}
template <typename t_elem, int t_rows, int t_cols>
inline const t_elem*  CVEnd (const cv::Matx<t_elem,t_rows,t_cols> &x)
{
  return x.val+t_rows*t_cols;
}
//-------------------------------------------------------------------------------------------

//! for Mat_
template <typename t_elem>
inline typename cv::Mat_<t_elem>::iterator  CVBegin (cv::Mat_<t_elem> &x)
{
  return x.begin();
}
template <typename t_elem>
inline typename cv::Mat_<t_elem>::const_iterator  CVBegin (const cv::Mat_<t_elem> &x)
{
  return x.begin();
}
template <typename t_elem>
inline typename cv::Mat_<t_elem>::iterator  CVEnd (cv::Mat_<t_elem> &x)
{
  return x.end();
}
template <typename t_elem>
inline typename cv::Mat_<t_elem>::const_iterator  CVEnd (const cv::Mat_<t_elem> &x)
{
  return x.end();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_cv_fwd_h
//-------------------------------------------------------------------------------------------
