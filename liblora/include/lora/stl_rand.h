//-------------------------------------------------------------------------------------------
/*! \file    stl_rand.h
    \brief   liblora - STL extension about random number  (header)
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
#ifndef loco_rabbits_stl_rand_h
#define loco_rabbits_stl_rand_h
//-------------------------------------------------------------------------------------------
#include <lora/rand.h>
#include <lora/stl_ext.h>
#include <list>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

/*! \brief Generate a set of random integers in [0..Nmax-1] whose size is C.  Overlap of elements is not allowed
    \note container is instantiated with std::list\<int\>, and std::vector\<int\> */
template <typename container>
void RandomIntSet (container &iset, size_t C, size_t Nmax);
//-------------------------------------------------------------------------------------------

/*! \brief Generate a subset 'sub' from a set 'set' by selecting C elements randomly */
template<typename container_set, typename container_sub>
void RandomSubset (const container_set &set, container_sub &sub, size_t C)
{
  sub.clear();
  typedef std::list<const typename container_set::value_type*> type_ptr_set;
  type_ptr_set all;
  for(typename container_set::const_iterator itr(set.begin()); itr!=set.end(); ++itr)
    all.push_back(&(*itr));
  while(all.size()>0 && sub.size()<C)
  {
    size_t add_index=Rand(0,all.size()-1);
    typename type_ptr_set::iterator add_itr = list_itr_at(all,add_index);
    sub.push_back(*(*add_itr));
    all.erase(add_itr);
  }
}
//-------------------------------------------------------------------------------------------

/*!\brief roulette selection
  This function selects an element from {first,..,last-1} randomly, and returns the iterator (FwdItr itr) to it.
  Note that the selection probability of itr is proportional to *itr.
  \param [out]idx  if specified (i.e. not NULL), return an index of itr from first (first is index 0).
  \note FwdItr is instantiated with std::vector<T>::{,const_}iterator, T* where T is in {double,long double} */
template <typename FwdItr>
FwdItr RouletteOne (FwdItr first, FwdItr last, int *idx=NULL);
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
int RouletteOne (const RandItr vec, const index_container &ref);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_stl_rand_h
//-------------------------------------------------------------------------------------------
