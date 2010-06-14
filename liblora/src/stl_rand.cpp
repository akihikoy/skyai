//-------------------------------------------------------------------------------------------
/*! \file    stl_rand.cpp
    \brief   liblora - STL extension about random number  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.09, 2010-

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
#include <lora/stl_rand.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


/*! \brief Generate a set of random integers in [0..Nmax-1] whose size is C.  Overlap of elements is not allowed
    \note container is instantiated with std::list\<int\>, and std::vector\<int\> */
template <typename container>
void RandomIntSet (container &iset, size_t C, size_t Nmax)
{
  iset.clear();
  std::list<int> all;
  for(size_t i(0); i<Nmax; ++i) all.push_back(i);
  while(all.size()>0 && iset.size()<C)
  {
    size_t add_index= Rand(0,all.size()-1);
    std::list<int>::iterator add_itr= list_itr_at (all,add_index);
    iset.push_back(*add_itr);
    all.erase(add_itr);
  }
}
template void RandomIntSet (std::list<int> &iset, size_t C, size_t Nmax);
template void RandomIntSet (std::vector<int> &iset, size_t C, size_t Nmax);
//-------------------------------------------------------------------------------------------

/*!\brief roulette selection
  This function selects an element from {first,..,last-1} randomly, and returns the iterator (FwdItr itr) to it.
  Note that the selection probability of itr is proportional to *itr.
  \param [out]idx  if specified (i.e. not NULL), return an index of itr from first (first is index 0).
  \note FwdItr is instantiated with std::vector<T>::{,const_}iterator, T* where T is in {double,long double} */
template <typename FwdItr>
FwdItr RouletteOne (FwdItr first, FwdItr last, int *idx=NULL)
{
  if (first==last)  {if(idx)*idx=-1; return last;}
  typedef typename elim_const_type<typename dereferenced_type<FwdItr>::type>::type TValueType;
  TValueType sum; SetZero(sum);
  for (FwdItr itr=first; itr!=last; ++itr)
    sum += (*itr>0.0l)?*itr:0.0l;
  TReal p = Rand (sum);
  SetZero(sum);
  int dummy (0);
  if (idx)  *idx= 0;
  else      idx= &dummy;
  for (FwdItr itr=first; itr!=last; ++itr,++(*idx))
  {
    sum += (*itr>0.0l)?(*itr):0.0l;
    if (p < sum)  return itr;
  }
  LERROR("in RouletteOne, fatal error!");
  lexit(btfail);
  return last;
}
#define INSTANTIATOR(_type) \
  template _type RouletteOne (_type first, _type last, int *idx=NULL);
INSTANTIATOR(std::vector<double>::iterator)
INSTANTIATOR(std::vector<long double>::iterator)
INSTANTIATOR(std::vector<double>::const_iterator)
INSTANTIATOR(std::vector<long double>::const_iterator)
INSTANTIATOR(double *)
INSTANTIATOR(long double *)
INSTANTIATOR(double const*)
INSTANTIATOR(long double const*)
#undef INSTANTIATOR
//-------------------------------------------------------------------------------------------


/*!\brief roulette selection
  This function selects an element from vec randomly, and returns the index (idx) to it.
  Note that the selection probability of idx is proportional to vec[idx].
  \param [in]vec  first random iterator to an vector that support operator[]. i.e. {vec[0],vec[1],..}
  \param [in]ref  reference index vector that selects elements of vec to be used.
    e.g. if ref={0,2,3} then the result is selected from {vec[0],vec[2],vec[3]}.
  \note RandItr is instantiated with std::vector<T>::{,const_}iterator and T* where T is in {float,double,long double}
  \note index_container is instantiated with std::list<int>, std::vector<int>  */
template <typename RandItr, typename index_container>
int RouletteOne (const RandItr vec, const index_container &ref)
{
  if( ref.empty() )  return -1;
  typename index_container::const_iterator itr;
  TReal sum; SetZero(sum);
  for (itr=ref.begin(); itr!=ref.end(); ++itr)
  {
    TReal tmp = static_cast<TReal>(vec[*itr]);
    sum += (tmp>0.0l)?tmp:0.0l;
  }
  TReal p = Rand( sum );
  SetZero(sum);
  for (itr=ref.begin(); itr!=ref.end(); ++itr)
  {
    TReal tmp = static_cast<TReal>(vec[*itr]);
    sum += (tmp>0.0l)?tmp:0.0l;
    if( p < sum )
      return *itr;
  }
  return ref.back();
}
#define INSTANTIATOR1(_type1,_type2)  \
  template int RouletteOne (std::vector<_type1>::iterator       , const _type2 &);    \
  template int RouletteOne (std::vector<_type1>::const_iterator , const _type2 &);    \
  template int RouletteOne (_type1*                             , const _type2 &);    \
  template int RouletteOne (const _type1*                       , const _type2 &);
#define INSTANTIATOR2(_type1)  \
  INSTANTIATOR1(_type1, std::list<int>)      \
  INSTANTIATOR1(_type1, std::vector<int>)
INSTANTIATOR2(float)
INSTANTIATOR2(double)
INSTANTIATOR2(long double)
#undef INSTANTIATOR1
#undef INSTANTIATOR2
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

