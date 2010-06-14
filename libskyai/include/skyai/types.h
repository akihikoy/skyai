//-------------------------------------------------------------------------------------------
/*! \file    types.h
    \brief   libskyai - libskyai basic type definitions
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.24, 2009-

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
#ifndef skyai_types_h
#define skyai_types_h
//-------------------------------------------------------------------------------------------
#include <lora/octave.h>
#include <lora/octave_str.h>
#include <lora/type_gen.h>
#include <lora/variable_space_oct.h>
#include <vector>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

typedef void              TVoid;

typedef int               TInt;
typedef std::vector<TInt> TIntVector;
typedef ColumnVector      TRealVector;
typedef Matrix            TRealMatrix;

typedef TInt              TDiscreteState;
typedef TInt              TDiscreteAction;
typedef TRealVector       TContinuousState;
typedef TRealVector       TContinuousAction;

typedef TReal             TSingleReward;   //!< 1-dimensional reward
typedef TReal             TValue;   //!< action-value, state-value, TD-error, etc.

typedef bool              TBool;
typedef std::string       TString;

typedef std::vector<TBool>  TBoolVector;

struct TComposite1
{
  std::vector<TInt>         DiscSet;
  std::vector<TRealVector>  ContSet;
};

typedef TComposite1       TCompositeState;
typedef TComposite1       TCompositeAction;

const std::string Composite1ToStr (const TComposite1 &val);
template <> inline const std::string ConvertToStr (const TComposite1 &val)  {return Composite1ToStr(val);}


/*!\brief continuous time (>=0)
  \attention Do not use negative time (<0). */
typedef TReal             TContinuousTime;

/*!\brief discrete time (>=0)
  \attention Do not use negative time (<0). */
typedef TInt              TDiscreteTime;


static const TContinuousTime INVALID_CONT_TIME (-1.0l);
static const TDiscreteTime   INVALID_DISC_TIME (-1);

/*!\brief tolerance of continuous time;
  if the difference of two continuous time values is less than this value, they can be regarded as the same time. */
static const TContinuousTime CONT_TIME_TOL (1.0e-8l);

//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_types_h
//-------------------------------------------------------------------------------------------
