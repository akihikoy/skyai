//-------------------------------------------------------------------------------------------
/*! \file    stl_math.h
    \brief   liblora - STL extension for mathmatical use  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.13, 2010-

    Copyright (C) 2010  Akihiko Yamaguchi

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
#ifndef loco_rabbits_stl_math_h
#define loco_rabbits_stl_math_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <vector>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

template <typename T>
const std::vector<T>& operator+= (std::vector<T> &lhs, const std::vector<T> &rhs)
{
  LASSERT1op1(lhs.size(),==,rhs.size());
  typename std::vector<T>::const_iterator ritr(rhs.begin());
  for (typename std::vector<T>::iterator litr(lhs.begin()); litr!=lhs.end(); ++litr,++ritr)
    (*litr)+=(*ritr);
  return lhs;
}
//-------------------------------------------------------------------------------------------

template <typename T>
const std::vector<T>& operator-= (std::vector<T> &lhs, const std::vector<T> &rhs)
{
  LASSERT1op1(lhs.size(),==,rhs.size());
  typename std::vector<T>::const_iterator ritr(rhs.begin());
  for (typename std::vector<T>::iterator litr(lhs.begin()); litr!=lhs.end(); ++litr,++ritr)
    (*litr)-=(*ritr);
  return lhs;
}
//-------------------------------------------------------------------------------------------

template <typename T1, typename T2>
const std::vector<T1>& operator*= (std::vector<T1> &lhs, const T2 &rhs)
{
  for (typename std::vector<T1>::iterator litr(lhs.begin()); litr!=lhs.end(); ++litr)
    (*litr)*= rhs;
  return lhs;
}
//-------------------------------------------------------------------------------------------

template <typename T1, typename T2>
const std::vector<T1>& operator/= (std::vector<T1> &lhs, const T2 &rhs)
{
  for (typename std::vector<T1>::iterator litr(lhs.begin()); litr!=lhs.end(); ++litr)
    (*litr)/= rhs;
  return lhs;
}
//-------------------------------------------------------------------------------------------

template <typename T>
inline std::vector<T> operator+ (const std::vector<T> &lhs, const std::vector<T> &rhs)
{
  LASSERT1op1(lhs.size(),==,rhs.size());
  std::vector<T> res(lhs);
  res+= rhs;
  return res;
}
//-------------------------------------------------------------------------------------------

template <typename T>
inline std::vector<T> operator- (const std::vector<T> &lhs, const std::vector<T> &rhs)
{
  LASSERT1op1(lhs.size(),==,rhs.size());
  std::vector<T> res(lhs);
  res-= rhs;
  return res;
}
//-------------------------------------------------------------------------------------------

template <typename T1, typename T2>
inline std::vector<T1> operator* (const std::vector<T1> &lhs, const T2 &rhs)
{
  std::vector<T1> res(lhs);
  res*= rhs;
  return res;
}
//-------------------------------------------------------------------------------------------

template <typename T1, typename T2>
inline std::vector<T2> operator* (const T1 &lhs, const std::vector<T2> &rhs)
{
  std::vector<T2> res(rhs);
  res*= lhs;
  return res;
}
//-------------------------------------------------------------------------------------------

template <typename T1, typename T2>
inline std::vector<T1> operator/ (const std::vector<T1> &lhs, const T2 &rhs)
{
  std::vector<T1> res(lhs);
  res/= rhs;
  return res;
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_stl_math_h
//-------------------------------------------------------------------------------------------
