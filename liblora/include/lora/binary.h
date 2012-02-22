//-------------------------------------------------------------------------------------------
/*! \file    binary.h
    \brief   liblora - managing binary data (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.03, 2012

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
#ifndef loco_rabbits_binary_h
#define loco_rabbits_binary_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <string>
#include <list>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief Binary stack class */
class TBinaryStack
//===========================================================================================
{
public:

  enum TStorageType {stList=0,stBinFile};

  struct TCell
    {
      union
      {
        long I;
        long double R;
        bool B;
      };
      std::string S;
    };

  TBinaryStack() : data_type_(stList) {}

  #define DEF_PUSH(x_type,x_cell)  \
    void Push(const x_type &val)   \
      {                            \
        LASSERT(data_type_==stList);  \
        TCell c;                   \
        c.x_cell=val;              \
        ldata_.push_back(c);       \
      }
  DEF_PUSH(unsigned short  , I )
  DEF_PUSH(unsigned int    , I )
  DEF_PUSH(unsigned long   , I )
  DEF_PUSH(signed short    , I )
  DEF_PUSH(signed int      , I )
  DEF_PUSH(signed long     , I )
  DEF_PUSH(bool            , B )
  DEF_PUSH(std::string     , S )

  DEF_PUSH(float           , R )
  DEF_PUSH(double          , R )
  DEF_PUSH(long double     , R )

  DEF_PUSH(char            , I )
  #undef DEF_PUSH

  #define DEF_READ(x_type,x_cell)  \
    x_type Read##x_cell() const    \
      {                            \
        LASSERT(data_type_==stList);        \
        LASSERT(ldpointer_!=ldata_.end());  \
        return (ldpointer_++)->x_cell;      \
      }
  DEF_READ(long,I)
  DEF_READ(long double,R)
  DEF_READ(bool,B)
  DEF_READ(std::string,S)
  #undef DEF_READ

  void GoFirst() const {ldpointer_= ldata_.begin();}
  void GoLast() const {ldpointer_= ldata_.end();}
  bool IsEOD() const
    {
      LASSERT1op1(data_type_,==,stList);
      return ldpointer_==ldata_.end();
    }

  int  Size()  {return ldata_.size();}
  void Clear()  {ldata_.clear();}

private:

  TStorageType data_type_;
  std::list<TCell>  ldata_;
  mutable std::list<TCell>::const_iterator  ldpointer_;

};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_binary_h
//-------------------------------------------------------------------------------------------
