//-------------------------------------------------------------------------------------------
/*! \file    stl_math.h
    \brief   liblora - STL extension for mathmatical use  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.13, 2010-
    \date    Aug.25, 2010   Added GetMaxNorm
    \date    Nov.02, 2010   Modified ConstrainVector

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

    -----------------------------------------------------------------------------------------

    \note Most of the following functions can work with liboctave's vectors, such as ColumnVector.
      To do this, include lora/type_gen_oct.h

*/
//-------------------------------------------------------------------------------------------
#ifndef loco_rabbits_stl_math_h
#define loco_rabbits_stl_math_h
//-------------------------------------------------------------------------------------------
#include <lora/type_gen.h>
#include <vector>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
// extension for numerical vectors
//===========================================================================================

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


//===========================================================================================
// generic vector calculations
//===========================================================================================

//! lhs += w*rhs
//! generic definition is in stl_math.h
template <typename t_vector1, typename t_real_type, typename t_vector2>
inline const t_vector1& WeightedAdd (t_vector1 &lhs, const t_real_type &w, const t_vector2 &rhs)
{
  LASSERT1op1(GenSize(lhs),==,GenSize(rhs));
  typename TypeExt<t_vector2>::const_iterator ritr= GenBegin(rhs);
  for (typename TypeExt<t_vector1>::iterator litr=GenBegin(lhs),llast=GenEnd(lhs); litr!=llast; ++litr,++ritr)
    *litr+= w*(*ritr);
  return lhs;
}
//-------------------------------------------------------------------------------------------

template <typename t_vector>
inline typename TypeExt<t_vector>::value_type  SquareSum (const t_vector &w)
{
  typename TypeExt<t_vector>::value_type res;  SetZero(res);
  for (typename TypeExt<t_vector>::const_iterator first(GenBegin(w)),last(GenEnd(w)); first!=last; ++first)
    res += Square(*first);
  return res;
}

//!\brief return L2 norm of a vector w
template <typename t_vector>
inline typename TypeExt<t_vector>::value_type  GetNorm (const t_vector &w)
{
  return real_sqrt(SquareSum(w));
}

//!\brief return maximum norm of a vector w
template <typename t_vector>
inline typename TypeExt<t_vector>::value_type  GetMaxNorm (const t_vector &w)
{
  typename TypeExt<t_vector>::value_type res;  SetZero(res);
  for (typename TypeExt<t_vector>::const_iterator first(GenBegin(w)),last(GenEnd(w)); first!=last; ++first)
    res= std::max(res, real_fabs(*first));
  return res;
}
//-------------------------------------------------------------------------------------------

template <typename t_vector>
inline void Normalize (t_vector &vec)
{
  vec /= GetNorm(vec);
}
//-------------------------------------------------------------------------------------------

template <typename t_vector>
inline t_vector GetNormalized (const t_vector &vec)
{
  t_vector res(vec);
  Normalize(res);
  return  res;
}
//-------------------------------------------------------------------------------------------

template <typename t_vec1, typename t_vec2>
void ConstrainVector (t_vec1 &vec, const t_vec2 &min, const t_vec2 &max)
{
  LASSERT1op1(GenSize(min),==,GenSize(max));
  if (GenSize(min)==0)  return;
  if (GenSize(min)==1)
  {
    const typename TypeExt<t_vec2>::value_type min0(GenAt(min,0)), max0(GenAt(max,0));
    LASSERT1op1(min0,<,max0);
    for (typename TypeExt<t_vec1>::iterator itr(GenBegin(vec)),last(GenEnd(vec)); itr!=last; ++itr)
      *itr= ApplyRange(*itr,min0,max0);
    return;
  }
  LASSERT1op1(GenSize(vec),==,GenSize(min));
  typename TypeExt<t_vec2>::const_iterator imax(GenBegin(max)), imin(GenBegin(min));
  for (typename TypeExt<t_vec1>::iterator itr(GenBegin(vec)),last(GenEnd(vec)); itr!=last; ++itr,++imax,++imin)
    if (*imin<=*imax)  *itr= ApplyRange(*itr,*imin,*imax);
}
//-------------------------------------------------------------------------------------------

//!\brief calculate x[i]*=y[i] for all i
template <typename t_vec1, typename t_vec2>
const t_vec1& VectorElemMultAssign (t_vec1 &lhs, const t_vec2 &rhs)
{
  LASSERT1op1(GenSize(lhs),==,GenSize(rhs));
  typename TypeExt<t_vec1>::const_iterator rhs_itr(GenBegin(rhs));
  for (typename TypeExt<t_vec1>::iterator lhs_itr(GenBegin(lhs)),lhs_last(GenEnd(lhs)); lhs_itr!=lhs_last; ++lhs_itr,++rhs_itr)
    (*lhs_itr)*= (*rhs_itr);
  return lhs;
}
//!\brief calculate res[i]=x[i]*y[i] for all i
template <typename t_vec1, typename t_vec2>
inline t_vec1 VectorElemMult (const t_vec1 &lhs, const t_vec2 &rhs)
{
  t_vec1 res(lhs);
  return VectorElemMultAssign (res,rhs);
}
//-------------------------------------------------------------------------------------------

//!\brief calculate x[i]/=y[i] for all i
template <typename t_vec1, typename t_vec2>
const t_vec1& VectorElemDivAssign (t_vec1 &lhs, const t_vec2 &rhs)
{
  LASSERT1op1(GenSize(lhs),==,GenSize(rhs));
  typename TypeExt<t_vec2>::const_iterator rhs_itr(GenBegin(rhs));
  for (typename TypeExt<t_vec1>::iterator lhs_itr(GenBegin(lhs)),lhs_last(GenEnd(lhs)); lhs_itr!=lhs_last; ++lhs_itr,++rhs_itr)
    (*lhs_itr)/= (*rhs_itr);
  return lhs;
}
//!\brief calculate res[i]=x[i]/y[i] for all i
template <typename t_vec1, typename t_vec2>
inline t_vec1 VectorElemDiv (const t_vec1 &lhs, const t_vec2 &rhs)
{
  t_vec1 res(lhs);
  return VectorElemDivAssign (res,rhs);
}
//-------------------------------------------------------------------------------------------


//! need to include lora/rand.h to use
template <typename t_vec1, typename t_vec2>
void GenerateRandomVector (t_vec1 &vec, const t_vec2 &min, const t_vec2 &max)
{
  LASSERT1op1(GenSize(vec),==,GenSize(max));
  LASSERT1op1(GenSize(vec),==,GenSize(min));
  typename TypeExt<t_vec2>::const_iterator imax(GenBegin(max)), imin(GenBegin(min));
  for (typename TypeExt<t_vec1>::iterator itr(GenBegin(vec)),last(GenEnd(vec)); itr!=last; ++itr,++imax,++imin)
  {
    if (*imin==*imax) *itr= *imin;
    else              *itr= Rand(*imin,*imax);
    // *itr= (*imin+*imax)*0.5+Rand(0.01);//Rand(*imin,*imax);
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_stl_math_h
//-------------------------------------------------------------------------------------------
