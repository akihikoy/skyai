//-------------------------------------------------------------------------------------------
/*! \file    mods_deprecated.h
    \brief   libskyai - deprecated modules kept for backward compatibility
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.28, 2010

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
#ifndef skyai_mods_deprecated_h
#define skyai_mods_deprecated_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <skyai/interfaces/functions.h>
#include <lora/stl_ext.h>   // for max_element_index, min_element_index
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

#define DEPRECATED_MSG(x_mod,x_alt)   \
  do{static int msg_count(20);        \
  if(msg_count) {--msg_count; LWARNING(x_mod << " is deprecated. Use " << x_alt << " instead." );}}while(0)


//===========================================================================================
/*!\brief get a max/min element value/index of an input vector */
class MMinMaxElementRv
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  // typedef t_arg1               TArgument1;
  typedef MMinMaxElementRv     TThis;
  SKYAI_MODULE_NAMES(MMinMaxElementRv)

  MMinMaxElementRv (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      in_1              (*this),
      out_max           (*this),
      out_max_value     (*this),
      out_max_index     (*this),
      out_min           (*this),
      out_min_value     (*this),
      out_min_index     (*this)
    {
      add_in_port   (in_1          );
      add_out_port  (out_max       );
      add_out_port  (out_max_value );
      add_out_port  (out_max_index );
      add_out_port  (out_min       );
      add_out_port  (out_min_value );
      add_out_port  (out_min_index );
    }

protected:

  mutable TReal  tmp_value_; //!< \warning problematic for multithread mode
  mutable TInt  tmp_index_; //!< \warning ditto

  //!\brief input the current state
  MAKE_IN_PORT(in_1, const TRealVector& (void), TThis);

  MAKE_OUT_PORT(out_max, void, (TReal &value, TInt &index), (value,index), TThis);
  MAKE_OUT_PORT(out_max_value, const TReal&, (void), (), TThis);
  MAKE_OUT_PORT(out_max_index, const TInt&, (void), (), TThis);

  MAKE_OUT_PORT(out_min, void, (TReal &value, TInt &index), (value,index), TThis);
  MAKE_OUT_PORT(out_min_value, const TReal&, (void), (), TThis);
  MAKE_OUT_PORT(out_min_index, const TInt&, (void), (), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(1, const TRealVector&, (void), ())

  #undef GET_FROM_IN_PORT

  void  out_max_get (TReal &value, TInt &index) const
    {
DEPRECATED_MSG("MMinMaxElementRv", "M{Min,Max}Element{Value,Index}");
      index= max_element_index(GenBegin(get_1()),GenEnd(get_1()));
      value= GenAt(get_1(),index);
    }
  const TReal&  out_max_value_get (void) const
    {
DEPRECATED_MSG("MMinMaxElementRv", "M{Min,Max}Element{Value,Index}");
      tmp_value_= *std::max_element(GenBegin(get_1()),GenEnd(get_1()));
      return tmp_value_;
    }
  const TInt&  out_max_index_get (void) const
    {
DEPRECATED_MSG("MMinMaxElementRv", "M{Min,Max}Element{Value,Index}");
      tmp_index_= max_element_index(GenBegin(get_1()),GenEnd(get_1()));
      return tmp_index_;
    }

  void  out_min_get (TReal &value, TInt &index) const
    {
DEPRECATED_MSG("MMinMaxElementRv", "M{Min,Max}Element{Value,Index}");
      index= min_element_index(GenBegin(get_1()),GenEnd(get_1()));
      value= GenAt(get_1(),index);
    }
  const TReal&  out_min_value_get (void) const
    {
DEPRECATED_MSG("MMinMaxElementRv", "M{Min,Max}Element{Value,Index}");
      tmp_value_= *std::min_element(GenBegin(get_1()),GenEnd(get_1()));
      return tmp_value_;
    }
  const TInt&  out_min_index_get (void) const
    {
DEPRECATED_MSG("MMinMaxElementRv", "M{Min,Max}Element{Value,Index}");
      tmp_index_= min_element_index(GenBegin(get_1()),GenEnd(get_1()));
      return tmp_index_;
    }

};  // end of MMinMaxElementRv
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief reward accumulator
    \todo TODO: replace by MSimpleAccumulator\<TReal\> */
class MRewardAccumulator
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface    TParent;
  typedef MRewardAccumulator  TThis;
  SKYAI_MODULE_NAMES(MRewardAccumulator)

  MRewardAccumulator (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      sum_              (0.0l),
      last_accessed_    (0.0l),
      slot_reset        (*this),
      slot_add          (*this),
      out_sum           (*this),
      out_last_accessed (*this)
    {
      add_slot_port (slot_reset        );
      add_slot_port (slot_add          );
      add_out_port  (out_sum           );
      add_out_port  (out_last_accessed );
    }

protected:

  TSingleReward  sum_;
  mutable TSingleReward  last_accessed_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_add, void, (const TSingleReward &r), (r), TThis);

  MAKE_OUT_PORT(out_sum, const TSingleReward&, (void), (), TThis);
  MAKE_OUT_PORT(out_last_accessed, const TSingleReward&, (void), (), TThis);


  virtual void slot_reset_exec (void)
    {
DEPRECATED_MSG("MRewardAccumulator", "MSimpleAccumulator<TReal>");
      sum_= 0.0l;
    }

  virtual void slot_add_exec (const TSingleReward &r)
    {
DEPRECATED_MSG("MRewardAccumulator", "MSimpleAccumulator<TReal>");
      sum_+= r;
    }

  virtual const TSingleReward& out_sum_get (void) const
    {
DEPRECATED_MSG("MRewardAccumulator", "MSimpleAccumulator<TReal>");
      last_accessed_= sum_;
      return sum_;
    }

  virtual const TSingleReward& out_last_accessed_get (void) const
    {
DEPRECATED_MSG("MRewardAccumulator", "MSimpleAccumulator<TReal>");
      return last_accessed_;
    }

};  // end of MRewardAccumulator
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TConstMultiplierRvConfigurations
//===========================================================================================
{
public:

  TRealVector       Factor;

  TConstMultiplierRvConfigurations(var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Factor     );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief multiplier which emits a signal with the parameter of an input signal multiplied by Factor */
class MConstMultiplierRv
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef MConstMultiplierRv   TThis;
  SKYAI_MODULE_NAMES(MConstMultiplierRv)

  MConstMultiplierRv (const std::string &v_instance_name)
    : TParent         (v_instance_name),
      conf_           (TParent::param_box_config_map()),
      slot_in         (*this),
      signal_out      (*this)
    {
      add_slot_port   (slot_in          );
      add_signal_port (signal_out       );
    }

  virtual ~MConstMultiplierRv() {}

protected:

  TConstMultiplierRvConfigurations  conf_;

  TRealVector      out_;

  MAKE_SLOT_PORT(slot_in, void, (const TRealVector &in), (in), TThis);

  MAKE_SIGNAL_PORT(signal_out, void (const TRealVector &), TThis);

  void slot_in_exec (const TRealVector &in)
    {
DEPRECATED_MSG("MConstMultiplierRv", "MConstMultiplier_TRealVector_TRealVector");
      out_.resize (GenSize(in));
      TypeExtS<TRealVector>::const_iterator iitr(GenBegin(in)), ilastitr(GenEnd(in)), fitr(GenBegin(conf_.Factor));
      if (GenSize(conf_.Factor)==GenSize(in))
      {
        for (TypeExtS<TRealVector>::iterator oitr(GenBegin(out_)); iitr!=ilastitr; ++iitr,++oitr,++fitr)
          (*oitr)= (*iitr) * (*fitr);
      }
      else if (GenSize(conf_.Factor)==1)
      {
        for (TypeExtS<TRealVector>::iterator oitr(GenBegin(out_)); iitr!=ilastitr; ++iitr,++oitr)
          (*oitr)= (*iitr) * (*fitr);
      }
      else
        {LERROR("size disagreement!"); LDBGVAR(GenSize(conf_.Factor)); LDBGVAR(GenSize(in)); lexit(df);}

      signal_out.ExecAll (out_);
    }

};  // end of MConstMultiplierRv
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TLinearFunctionRvConfigurations
//===========================================================================================
{
public:

  TRealMatrix            Factor;

  TLinearFunctionRvConfigurations(var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Factor     );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief Linear function (y = A x)
    \note this module is similar to the MConstMultiplierRv. the difference is
        this module is in/out version, MConstMultiplierRv is signal/slot version. */
class MLinearFunctionRv
    : public MFunctionSISOInterface <TRealVector, TRealVector>
//===========================================================================================
{
public:
  typedef MFunctionSISOInterface <
                        TRealVector,
                        TRealVector>     TParent;
  typedef MLinearFunctionRv              TThis;
  SKYAI_MODULE_NAMES(MLinearFunctionRv)

  MLinearFunctionRv (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map())
    {
    }

  virtual ~MLinearFunctionRv() {}

protected:

  TLinearFunctionRvConfigurations  conf_;

  override void slot_initialize_exec (void)  {}

  override void function (const TInput &x, TOutput &y) const
    {
DEPRECATED_MSG("MLinearFunctionRv", "MConstMultiplier_TRealMatrix_TRealVector");
      y= conf_.Factor * x;
    }

};  // end of MLinearFunctionRv
//-------------------------------------------------------------------------------------------


#undef DEPRECATED_MSG

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_mods_deprecated_h
//-------------------------------------------------------------------------------------------
