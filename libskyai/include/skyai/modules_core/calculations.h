//-------------------------------------------------------------------------------------------
/*! \file    calculations.h
    \brief   libskyai - modules for basic calculations
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.25, 2010

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
#ifndef skyai_calculations_h
#define skyai_calculations_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <skyai/interfaces/functions.h>
#include <lora/cast.h>
#include <lora/stl_ext.h>   // for max_element_index, min_element_index
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief simple generator of a SISO function (no config)
#define FUNCTION_SISO_GEN(x_module_name,x_input,x_output)                   \
  class x_module_name : public MFunctionSISOInterface <x_input, x_output>   \
  {                                                                         \
  public:                                                                   \
    typedef MFunctionSISOInterface <                                        \
                      x_input, x_output>   TParent;                         \
    typedef x_input                        TInput;                          \
    typedef x_output                       TOutput;                         \
    typedef x_module_name                  TThis;                           \
    SKYAI_MODULE_NAMES(x_module_name)                                       \
    x_module_name (const std::string &v_instance_name)                      \
      : TParent(v_instance_name)  {}                                        \
    virtual ~x_module_name() {}                                             \
  protected:                                                                \
    override void slot_initialize_exec (void)  {}                           \
    override void function (const TInput &x, TOutput &y) const;             \
  };
//===========================================================================================
//!\brief simple generator of a SISO function (with config)
#define FUNCTION_SISO_GEN_CFG(x_module_name,x_input,x_output,x_config)      \
  class x_module_name : public MFunctionSISOInterface <x_input, x_output>   \
  {                                                                         \
  public:                                                                   \
    typedef MFunctionSISOInterface <                                        \
                      x_input, x_output>   TParent;                         \
    typedef x_input                        TInput;                          \
    typedef x_output                       TOutput;                         \
    typedef x_module_name                  TThis;                           \
    SKYAI_MODULE_NAMES(x_module_name)                                       \
    x_module_name (const std::string &v_instance_name)                      \
      : TParent(v_instance_name),                                           \
        conf_(TParent::param_box_config_map())  {}                          \
    virtual ~x_module_name() {}                                             \
  protected:                                                                \
    x_config conf_;                                                         \
    override void slot_initialize_exec (void)  {}                           \
    override void function (const TInput &x, TOutput &y) const;             \
  };
//===========================================================================================


/*!\brief MCaster: y = lora_cast< t_output > (x)  */
template <typename t_input, typename t_output> FUNCTION_SISO_GEN (MCaster, t_input, t_output)
template <typename t_input, typename t_output>
void MCaster<t_input,t_output>::function (const TInput &x, TOutput &y) const
{
  y= lora_cast<t_output>(x);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
template <typename t_const>
struct TConstMultiplierConfigurations
{
  t_const          Factor;

  TConstMultiplierConfigurations(var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Factor     );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
/*!\brief MConstMultiplier: constant multiplier (y = Factor * x); x: input, y: output, Factor: constant factor */
template <typename t_const, typename t_input>
FUNCTION_SISO_GEN_CFG (MConstMultiplier, t_input, t_input, TConstMultiplierConfigurations<t_const>)
//-------------------------------------------------------------------------------------------
/*!\brief Specialization to Factor: real vector, x,y: real vector; y[i]=Factor[i]*x[i] for all i */
template<>void MConstMultiplier<TRealVector,TRealVector>::function (const TInput &x, TOutput &y) const;
/*!\brief Specialization to Factor: real matrix, x,y: real vector; y = Factor * x */
template<>void MConstMultiplier<TRealMatrix,TRealVector>::function (const TInput &x, TOutput &y) const;
//-------------------------------------------------------------------------------------------


/*!\brief MElemSquarer: y[i]=(x[i])^2 for all i; x: input, y: output  */
template <typename t_input> FUNCTION_SISO_GEN (MElemSquarer, t_input, t_input)
template <typename t_input> void MElemSquarer<t_input>::function (const TInput &x, TOutput &y) const
{
  GenResize(y, GenSize(x));
  typename TypeExt<TInput>::const_iterator  x_itr(GenBegin(x));
  for (typename TypeExt<TInput>::iterator y_itr(GenBegin(y)),y_last(GenEnd(y)); y_itr!=y_last; ++y_itr,++x_itr)
    (*y_itr)= Square(*x_itr);
}
//-------------------------------------------------------------------------------------------
/*!\brief Specialization to x,y: TInt; y=x^2 */
template<> void MElemSquarer<TInt>::function (const TInput &x, TOutput &y) const;
/*!\brief Specialization to x,y: TReal; y=x^2 */
template<> void MElemSquarer<TReal>::function (const TInput &x, TOutput &y) const;
//-------------------------------------------------------------------------------------------


/*!\brief MBoolAnd: y = x[0] and x[1] and ... and x[N-1]; x: input (bool vector), y: output (bool)  */
FUNCTION_SISO_GEN (MBoolAnd, TBoolVector, TBool)
/*!\brief MBoolOr: y = x[0] or x[1] or ... or x[N-1]; x: input (bool vector), y: output (bool)  */
FUNCTION_SISO_GEN (MBoolOr, TBoolVector, TBool)
//-------------------------------------------------------------------------------------------


template <typename t_vector> FUNCTION_SISO_GEN (MMinElementValue, t_vector, typename TypeExt<t_vector>::value_type)
template <typename t_vector> void MMinElementValue<t_vector>::function (const TInput &x, TOutput &y) const
{
  y= *std::min_element(GenBegin(x),GenEnd(x));
}
template <typename t_vector> FUNCTION_SISO_GEN (MMaxElementValue, t_vector, typename TypeExt<t_vector>::value_type)
template <typename t_vector> void MMaxElementValue<t_vector>::function (const TInput &x, TOutput &y) const
{
  y= *std::max_element(GenBegin(x),GenEnd(x));
}
template <typename t_vector> FUNCTION_SISO_GEN (MMinElementIndex, t_vector, TInt)
template <typename t_vector> void MMinElementIndex<t_vector>::function (const TInput &x, TOutput &y) const
{
  y= min_element_index(GenBegin(x),GenEnd(x));
}
template <typename t_vector> FUNCTION_SISO_GEN (MMaxElementIndex, t_vector, TInt)
template <typename t_vector> void MMaxElementIndex<t_vector>::function (const TInput &x, TOutput &y) const
{
  y= max_element_index(GenBegin(x),GenEnd(x));
}
//-------------------------------------------------------------------------------------------


namespace skyai_calculations_detail
{
template <typename t_vector>
inline TInt CountNonzeroElements (const t_vector &x, const typename TypeExt<t_vector>::value_type &threshold)
{
  TInt count(0);
  for (typename TypeExt<t_vector>::const_iterator itr(GenBegin(x)), last(GenEnd(x)); itr!=last; ++itr)
    if (*itr>threshold || *itr<-threshold)  ++count;
  return count;
}
}  // end of skyai_calculations_detail
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Configurations of MNonzeroElements
template <typename t_vector>
struct TNonzeroElementsConfigurations
{
  typename TypeExt<t_vector>::value_type    Threshold;

  TNonzeroElementsConfigurations (var_space::TVariableMap &mmap)
    {
      SetZero(Threshold);

      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Threshold      );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
/*!\brief MNonzeroElements: get nonzero elements of an input vector */
template <typename t_vector>
FUNCTION_SISO_GEN_CFG (MNonzeroElements,t_vector,t_vector,TNonzeroElementsConfigurations<t_vector>)
template <typename t_vector> void MNonzeroElements<t_vector>::function (const TInput &x, TOutput &y) const
{
  using namespace skyai_calculations_detail;
  GenResize(y, CountNonzeroElements(x, conf_.Threshold));
  typename TypeExt<t_vector>::iterator  nzitr(GenBegin(y));
  for (typename TypeExt<t_vector>::const_iterator itr(GenBegin(x)), last(GenEnd(x)); itr!=last; ++itr)
    if (*itr>conf_.Threshold || *itr<-conf_.Threshold)  {*nzitr= *itr; ++nzitr;}
}
//-------------------------------------------------------------------------------------------
/*!\brief MNonzeroElementsCounter: get number of nonzero elements of an input vector */
template <typename t_vector>
FUNCTION_SISO_GEN_CFG (MNonzeroElementsCounter,t_vector,TInt,TNonzeroElementsConfigurations<t_vector>)
template <typename t_vector> void MNonzeroElementsCounter<t_vector>::function (const TInput &x, TOutput &y) const
{
  using namespace skyai_calculations_detail;
  y= CountNonzeroElements(x, conf_.Threshold);
}
//-------------------------------------------------------------------------------------------


/*!\brief MVectorToScalar: convert a vector to a scalar (extract a first element) */
template <typename t_vector> FUNCTION_SISO_GEN (MVectorToScalar, t_vector, typename TypeExt<t_vector>::value_type)
template <typename t_vector> void MVectorToScalar<t_vector>::function (const TInput &x, TOutput &y) const
{
  LASSERT1op1(GenSize(x),==,1);
  y= GenAt(x,0);
}
//-------------------------------------------------------------------------------------------



#undef FUNCTION_SISO_GEN
#undef FUNCTION_SISO_GEN_CFG
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TInt ,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TInt ,TBool)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TReal,TInt )
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TReal,TBool)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TBool,TInt )
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TBool,TReal)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MConstMultiplier,TRealVector,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MConstMultiplier,TRealMatrix,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemSquarer,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemSquarer,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemSquarer,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemSquarer,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMinElementValue,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMaxElementValue,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMinElementIndex,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMaxElementIndex,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MNonzeroElements,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MNonzeroElements,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MNonzeroElementsCounter,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MNonzeroElementsCounter,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MVectorToScalar,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MVectorToScalar,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MVectorToScalar,TBoolVector)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_calculations_h
//-------------------------------------------------------------------------------------------
