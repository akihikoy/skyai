//-------------------------------------------------------------------------------------------
/*! \file    math.h
    \brief   liblora - math utility (header)
    \author  Akihiko Yamaguchi
    \date    2008-

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
#ifndef loco_rabbits_math_h
#define loco_rabbits_math_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

template< class T >
inline T Square( const T &val )
{
  return val*val;
}
//-------------------------------------------------------------------------------------------

template< class T >
inline T Cube( const T &val )
{
  return val*val*val;
}
//-------------------------------------------------------------------------------------------

template< class T >
inline T Sign (const T &val)
{
  if (val>=GetZero<T>())  return GetOne<T>();
  else return -GetOne<T>();
}
//-------------------------------------------------------------------------------------------

template <typename T>
inline T Sigmoid (const T &x)
{
  return 1.0l/(1.0l+real_exp(-x));
}
//-------------------------------------------------------------------------------------------

//!\brief summation
template <typename FwdItr>
inline typename dereferenced_type<FwdItr>::type
  Sum (FwdItr first, FwdItr last, const typename dereferenced_type<FwdItr>::type &zero=0.0l)
{
  typename elim_const_type<
      typename dereferenced_type<FwdItr>::type >::type res(zero);
  if (first==last)  return res;
  for (; first!=last; ++first)
    res += *first;
  return res;
}
//-------------------------------------------------------------------------------------------

//!\brief inner prod of two vectors ([first1,last1],[first2,-])
template <typename FwdItr1, typename FwdItr2>
inline typename dereferenced_type<FwdItr1>::type
  InnerProd (FwdItr1 first1, const FwdItr1 &last1, FwdItr2 first2, const typename dereferenced_type<FwdItr1>::type &zero=0.0l)
{
  typename elim_const_type<
      typename dereferenced_type<FwdItr1>::type >::type res(zero);
  if (first1==last1)  return res;
  for (; first1!=last1; ++first1,++first2)
    res += (*first1)*(*first2);
  return res;
}
//-------------------------------------------------------------------------------------------

//!\brief update vec1 = vec1 + step_size * vec2
template <typename FwdItr1, typename FwdItr2, typename T>
inline void VectorUpdate (FwdItr1 first1, const FwdItr1 &last1, FwdItr2 first2, const T &step_size)
{
  if (first1==last1)  return;
  for (; first1!=last1; ++first1,++first2)
    *first1 += step_size*(*first2);
}
//-------------------------------------------------------------------------------------------

//! lhs += w*rhs
template <typename T, typename _real_type>
inline const T& WeightedAdd (T &lhs, const _real_type &w, const T &rhs)
{
  lhs.WeightedAdd (w, rhs);
  return lhs;
}
#define SPECIALIZER(_type1,_type2)  \
  template <>  \
  inline const _type1& WeightedAdd (_type1 &lhs, const _type2 &w, const _type1 &rhs)  \
    {lhs += static_cast<_type1>(w*rhs); return lhs;}
SPECIALIZER(float,float)
SPECIALIZER(double,double)
SPECIALIZER(double,long double)
SPECIALIZER(long double,double)
SPECIALIZER(long double,long double)
#undef SPECIALIZER
//-------------------------------------------------------------------------------------------

//! \brief return x**y (or x^y) for int x and y
//! using the idea described in http://homepage1.nifty.com/~umetani/ims/2000/20000401002/0.html
template <class T>
inline T ipow (const T& x, const T& y)
{
  T ta, tn, tmp(1);
  T zero(0), one(1), two(2);
  ta= x;
  tn= y;
  while (tn != zero)
  {
    if ((tn % two) == one)  tmp *= ta;
    ta = ta * ta;
    tn /= 2;
  }
  return tmp;
}
//-------------------------------------------------------------------------------------------

//! return true if val satisfies left &lt;= val &lt; right
template <class T>
inline bool IsIn (const T &val, const T &left, const T &right)
{
  return (left<=val) && (val<right);
}
//-------------------------------------------------------------------------------------------

//! return min if val &lt; min, max if val &gt; max, val otherwise
template <class T>
inline T ApplyRange (const T &val, const T &min, const T &max)
{
  if (val < min)  return min;
  if (val > max)  return max;
  return val;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_math_h
//-------------------------------------------------------------------------------------------

