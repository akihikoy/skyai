//-------------------------------------------------------------------------------------------
/*! \file    timed_resource.h
    \brief   libskyai - template module which stores a converted data which changes with (discrete) time
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.20, 2009-

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
#ifndef skyai_timed_resource_h
#define skyai_timed_resource_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief template module which stores the input data x which changes with (discrete) time
template <typename t_input>
class MTimedResourceXX
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface         TParent;
  typedef MTimedResourceXX         TThis;
  typedef t_input                  TInput;
  SKYAI_MODULE_NAMES(MTimedResourceXX)

  MTimedResourceXX (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      stored_time_   (-1),
      slot_reset     (*this),
      out_x          (*this),
      in_x           (*this),
      in_disc_time   (*this)
    {
      add_slot_port (slot_reset);
      add_out_port (out_x);
      add_in_port (in_x         );
      add_in_port (in_disc_time );
    }

  virtual ~MTimedResourceXX() {}

protected:

  mutable TDiscreteTime   stored_time_;
  mutable TInput          stored_x_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_OUT_PORT(out_x, const TInput&, (void), (), TThis);

  MAKE_IN_PORT(in_x, const TInput& (void), TThis);
  MAKE_IN_PORT(in_disc_time, const TDiscreteTime& (void), TThis);

  virtual void slot_reset_exec (void)
    {
      stored_time_= -1;
    }

  virtual const TInput& out_x_get (void) const
    {
      if (in_disc_time.ConnectionSize()==0)  {LERROR("in_disc_time is required.");lexit(df);}
      TDiscreteTime t= in_disc_time.GetFirst();

      if (stored_time_>=0 && stored_time_==t)  return stored_x_;
      if (in_x.ConnectionSize()==0)  {LERROR("in_x is required.");lexit(df);}
      stored_x_= in_x.GetFirst();  //!\todo FIXME: this may require a memory allocation which takes much computational cost!
      stored_time_= t;

      return stored_x_;
    }

};  // end of MTimedResourceXX
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief template module which stores a converted y from the input data x which changes with (discrete) time
template <typename t_input, typename t_output>
class MTimedResourceXY
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface         TParent;
  typedef MTimedResourceXY         TThis;
  typedef t_input                  TInput;
  typedef t_output                 TOutput;
  SKYAI_MODULE_NAMES(MTimedResourceXY)

  MTimedResourceXY (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      stored_time_   (-1),
      slot_reset     (*this),
      out_y          (*this),
      in_x           (*this),
      in_converter   (*this),
      in_disc_time   (*this)
    {
      add_slot_port (slot_reset);
      add_out_port (out_y);
      add_in_port (in_x         );
      add_in_port (in_converter );
      add_in_port (in_disc_time );
    }

  virtual ~MTimedResourceXY() {}

protected:

  mutable TDiscreteTime   stored_time_;
  mutable TOutput         stored_y_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_OUT_PORT(out_y, const TOutput&, (void), (), TThis);

  MAKE_IN_PORT(in_x, const TInput& (void), TThis);
  MAKE_IN_PORT(in_converter, void (const TInput &x, TOutput &y), TThis);
  MAKE_IN_PORT(in_disc_time, const TDiscreteTime& (void), TThis);

  virtual void slot_reset_exec (void)
    {
      stored_time_= -1;
    }

  virtual const TOutput& out_y_get (void) const
    {
      if (in_disc_time.ConnectionSize()==0)  {LERROR("in_disc_time is required.");lexit(df);}
      TDiscreteTime t= in_disc_time.GetFirst();

      if (stored_time_>=0 && stored_time_==t)  return stored_y_;
      if (in_x.ConnectionSize()==0)  {LERROR("in_x is required.");lexit(df);}
      in_converter.GetFirst (in_x.GetFirst(), stored_y_);
      stored_time_= t;

      return stored_y_;
    }

};  // end of MTimedResourceXY
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MTimedResourceXX,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MTimedResourceXY,TRealVector,TRealVector)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_timed_resource_h
//-------------------------------------------------------------------------------------------
