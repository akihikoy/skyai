//-------------------------------------------------------------------------------------------
/*! \file    stl_ext.h
    \brief   liblora - STL extension  (header)
    \author  Akihiko Yamaguchi
    \date    2008-
    \date    Oct. 12, 2009 : Specialized ConvertToStr, ConvertFromStr for containers

    Copyright (C) 2008, 2009, 2010  Akihiko Yamaguchi

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
#ifndef loco_rabbits_stl_ext_h
#define loco_rabbits_stl_ext_h
//-------------------------------------------------------------------------------------------
#include <lora/stl_fwd.h>
#include <lora/math.h>
#include <lora/string.h>
#include <vector>
#include <list>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <utility>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

/*!\brief generate vector, list (container generator)
  \note container is instantiated with std::list<T>, std::vector<T>, T={int,double,long double}  */
template <typename container>
container  ContainerGen (int size, ...);

template <typename t_fwd_iterator>
inline void PrintContainer (t_fwd_iterator first, t_fwd_iterator last, std::ostream &os=std::cout, const std::string &delim=" ")
{
  if(first==last)  return;
  os<< *first;
  for (++first; first!=last; ++first)
    os << delim << *first;
}
//-------------------------------------------------------------------------------------------

template <typename container>
inline void PrintContainer (const container &v, const std::string &prefix="", std::ostream &os=std::cout, const std::string &delim=" ")
{
  os << prefix;
  PrintContainer(v.begin(),v.end(), os, delim);
  os << std::endl;
}
//-------------------------------------------------------------------------------------------

template <typename t_fwd_iterator>
inline std::string ContainerToStr (t_fwd_iterator first, t_fwd_iterator last, const std::string &delim=" ")
{
  std::stringstream ss;
  PrintContainer(first,last, ss, delim);
  return ss.str();
}

template <class container>
inline std::string ContainerToStr (const container &v, const std::string &delim=" ")
{
  std::stringstream ss;
  PrintContainer(v.begin(),v.end(), ss, delim);
  return ss.str();
}
//-------------------------------------------------------------------------------------------

//! specialize ConvertToStr and ConvertFromStr for stl-containers (list and vector)
#define SPECIALIZER(x_type)                                                             \
  template <> inline const std::string ConvertToStr (const std::list<x_type> &val)      \
    {return NumericalContainerToString(val);}                                           \
  template <> inline const std::list<x_type> ConvertFromStr (const std::string &str)    \
    {std::list<x_type> res;  StringToNumericalContainer(str,res);  return res;}         \
  template <> inline const std::string ConvertToStr (const std::vector<x_type> &val)    \
    {return NumericalContainerToString(val);}                                           \
  template <> inline const std::vector<x_type> ConvertFromStr (const std::string &str)  \
    {std::vector<x_type> res;  StringToNumericalContainer(str,res);  return res;}
SPECIALIZER(bool           )
SPECIALIZER(signed int     )
SPECIALIZER(signed short   )
SPECIALIZER(signed long    )
SPECIALIZER(signed char    )
SPECIALIZER(unsigned int   )
SPECIALIZER(unsigned short )
SPECIALIZER(unsigned long  )
SPECIALIZER(unsigned char  )
SPECIALIZER(float          )
SPECIALIZER(double         )
SPECIALIZER(long double    )
#undef SPECIALIZER

template<class container>
inline void CSetZero (container &val)
{
  for (typename container::iterator itr(val.begin()); itr!=val.end(); ++itr)
    SetZero(*itr);
}
//-------------------------------------------------------------------------------------------

/*!\brief define SetZero for std::vector<T>
    \note this is an \b overloading of SetZero defined in common.h
      because C++ does not allow the function template partial specialization */
template <typename T> inline void SetZero (std::vector<T> &val) {CSetZero(val);}

/*!\brief define SetZero for std::list<T>
    \note this is an \b overloading of SetZero defined in common.h
      because C++ does not allow the function template partial specialization */
template <typename T> inline void SetZero (std::list<T> &val)   {CSetZero(val);}
//-------------------------------------------------------------------------------------------

/*!\todo FIXME : fix the issue of breaking strict-aliasing rules:
    If t_container is a std::list\<T\> and the result of max_element is dereferenced (by operator*),
    warning:  "cc1plus: warning: dereferencing pointer 'pretmp.xxxx' does break strict-aliasing rules"
    arises.  (Currently, this seems not a big issue)
    Note: when we use std::max_element, the same warning arises.
*/
template <typename t_container>
inline typename t_container::iterator  max_element (t_container &vec)
{
  return  std::max_element (vec.begin(), vec.end());
}
//-------------------------------------------------------------------------------------------

template <typename t_container>
inline typename t_container::const_iterator  max_element (const t_container &vec)
{
  return  std::max_element (vec.begin(), vec.end());
}
//-------------------------------------------------------------------------------------------

template <typename t_container>
inline typename t_container::iterator  min_element (t_container &vec)
{
  return  std::min_element (vec.begin(), vec.end());
}
//-------------------------------------------------------------------------------------------

template <typename t_container>
inline typename t_container::const_iterator  min_element (const t_container &vec)
{
  return  std::min_element (vec.begin(), vec.end());
}
//-------------------------------------------------------------------------------------------

template <typename t_fwd_iterator>
inline int max_element_index (t_fwd_iterator first, t_fwd_iterator last, const int notfound=-1)
{
  if (first==last)  return notfound;
  t_fwd_iterator largest(first);
  int index, ilargest; SetZero(index); SetZero(ilargest);
  for (; first!=last; ++first,++index)
    if (*largest<*first) {largest=first; ilargest=index;};
  return ilargest;
}
//-------------------------------------------------------------------------------------------

template <typename t_container>
inline int max_element_index (const t_container &vec, const int notfound=-1)
{
  return max_element_index(vec.begin(),vec.end(),notfound);
}
//-------------------------------------------------------------------------------------------

template <typename t_fwd_iterator>
inline int min_element_index (t_fwd_iterator first, t_fwd_iterator last, const int notfound=-1)
{
  if (first==last)  return notfound;
  t_fwd_iterator min(first);
  int index, imin; SetZero(index); SetZero(imin);
  for (; first!=last; ++first,++index)
    if (*min>*first) {min=first; imin=index;};
  return imin;
}
//-------------------------------------------------------------------------------------------

template <typename t_container>
inline int min_element_index (const t_container &vec, const int notfound=-1)
{
  return min_element_index(vec.begin(),vec.end(),notfound);
}
//-------------------------------------------------------------------------------------------


template< class T >
inline typename std::list<T>::iterator list_itr_at (std::list<T> &ls, int index)
{
  typename std::list<T>::iterator itr;
  int size(ls.size());
  if (index>=size)  return ls.end();
  else if (index<-size)  index= -size;
  if (index>=0) for (itr=ls.begin(); index>0; --index)  ++itr;
  else          for (itr=ls.end();   index<0; ++index)  --itr;
  return itr;
}
template< class T >
inline typename std::list<T>::const_iterator list_itr_at (const std::list<T> &ls, int index)
{
  typename std::list<T>::const_iterator itr;
  int size(ls.size());
  if (index>=size)  return ls.end();
  else if (index<-size)  index= -size;
  if (index>=0) for (itr=ls.begin(); index>0; --index)  ++itr;
  else          for (itr=ls.end();   index<0; ++index)  --itr;
  return itr;
}
//-------------------------------------------------------------------------------------------

template <typename container>
inline typename container::iterator container_itr_at (container &_ls, int index);
template <typename container>
inline typename container::const_iterator container_itr_at (const container &_ls, int index);

template <typename T>
inline typename std::list<T>::iterator container_itr_at (std::list<T> &_ls, int index)
{
  return list_itr_at (_ls, index);
}
template <typename T>
inline typename std::list<T>::const_iterator container_itr_at (const std::list<T> &_ls, int index)
{
  return list_itr_at (_ls, index);
}

template <typename T>
inline typename std::vector<T>::iterator container_itr_at (std::vector<T> &_ls, int index)
{
  return _ls.begin() + index;
}
template <typename T>
inline typename std::vector<T>::const_iterator container_itr_at (const std::vector<T> &_ls, int index)
{
  return _ls.begin() + index;
}
//-------------------------------------------------------------------------------------------

//! function object to compare two std::pair objects by their second values (use this with max_element, etc.)
template <typename t_key,typename t_value>
struct TSecondComp
{
  bool operator()(const std::pair<t_key,t_value> &lhs, const std::pair<t_key,t_value> &rhs)
    {return lhs.second < rhs.second;}
};
template <typename t_key,typename t_value>
inline TSecondComp<t_key,t_value> SecondComp(const std::pair<t_key,t_value> &ref)
{
  return TSecondComp<t_key,t_value>();
}
template <typename t_key,typename t_value>
inline TSecondComp<t_key,t_value> SecondComp(const MAP_FWD(t_key,t_value) &ref)
{
  return TSecondComp<t_key,t_value>();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_stl_ext_h
//-------------------------------------------------------------------------------------------

