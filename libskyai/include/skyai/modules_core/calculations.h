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
#include <lora/stl_math.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------



//===========================================================================================
template <typename t_const>
struct TConstantConfigurations
{
  t_const          Constant;

  TConstantConfigurations(var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Constant   );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief output a constant value */
template <typename t_const>
class MConstant
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef MConstant<t_const>   TThis;
  SKYAI_MODULE_NAMES(MConstant)

  MConstant (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      conf_             (TParent::param_box_config_map()),
      out_const         (*this)
    {
      add_out_port  (out_const );
    }

protected:

  TConstantConfigurations<t_const>  conf_;

  MAKE_OUT_PORT(out_const, const t_const&, (void), (), TThis);

  virtual const t_const& out_const_get (void) const
    {
      return conf_.Constant;
    }

};  // end of MConstant
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
    override void function (const TInput &x, TOutput &y) const;             \
  };
//===========================================================================================
//!\brief simple generator of a 2ISO function (no config)
#define FUNCTION_2ISO_GEN(x_module_name,x_input1,x_input2,x_output)         \
  class x_module_name                                                       \
    : public MFunction2ISOInterface <x_input1, x_input2, x_output>          \
  {                                                                         \
  public:                                                                   \
    typedef MFunction2ISOInterface <                                        \
            x_input1, x_input2, x_output>  TParent;                         \
    typedef x_input1                       TInput1;                         \
    typedef x_input2                       TInput2;                         \
    typedef x_output                       TOutput;                         \
    typedef x_module_name                  TThis;                           \
    SKYAI_MODULE_NAMES(x_module_name)                                       \
    x_module_name (const std::string &v_instance_name)                      \
      : TParent(v_instance_name)  {}                                        \
    virtual ~x_module_name() {}                                             \
  protected:                                                                \
    override void function (const TInput1 &x1,                              \
                            const TInput2 &x2, TOutput &y) const;           \
  };
//===========================================================================================

#define X_COMMA ,


/*!\brief MCaster: y = lora_cast< t_output > (x)  */
template <typename t_input, typename t_output> FUNCTION_SISO_GEN (MCaster, t_input, t_output)
template <typename t_input, typename t_output>
void MCaster<t_input,t_output>::function (const TInput &x, TOutput &y) const
{
  y= lora_cast<t_output>(x);
}
//-------------------------------------------------------------------------------------------


/*!\brief MVectorToScalar's Configurations */
struct TVectorToScalarConfigurations
{
  TInt    Index;

  TVectorToScalarConfigurations (var_space::TVariableMap &mmap)
    : Index (0)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Index   );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
/*!\brief MVectorToScalar: convert a vector to a scalar (extract the Index-th element; Index is 0 in default) */
template <typename t_vector, typename t_scalar> FUNCTION_SISO_GEN_CFG (MVectorToScalar, t_vector, t_scalar, TVectorToScalarConfigurations)
template <typename t_vector, typename t_scalar> void MVectorToScalar<t_vector,t_scalar>::function (const TInput &x, TOutput &y) const
{
  y= GenAt(x,conf_.Index);
}
//-------------------------------------------------------------------------------------------


/*!\brief MVectorShuffler's Configurations */
struct TVectorShufflerConfigurations
{
  TIntVector    Order;

  TVectorShufflerConfigurations (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Order   );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
/*!\brief MVectorShuffler: reconstruct a vector so that y[i]=x[Order[i]] where i=0,..,size(Order)-1  */
template <typename t_vector> FUNCTION_SISO_GEN_CFG (MVectorShuffler, t_vector, t_vector, TVectorShufflerConfigurations)
template <typename t_vector> void MVectorShuffler<t_vector>::function (const TInput &x, TOutput &y) const
{
  GenResize(y,GenSize(conf_.Order));
  typename TypeExt<TOutput>::iterator  yitr(GenBegin(y));
  for (typename TypeExt<TIntVector>::const_iterator oitr(GenBegin(conf_.Order)),olast(GenEnd(conf_.Order)); oitr!=olast; ++oitr,++yitr)
    (*yitr)= GenAt(x,*oitr);
}
//-------------------------------------------------------------------------------------------


/*!\brief MAdd: addition calculator (y = x1 + x2)
    \todo specialize for vector to make efficient */
template <typename t_input>FUNCTION_2ISO_GEN (MAdd, t_input, t_input, t_input)
template <typename t_input>void MAdd<t_input>::function (const TInput1 &x1, const TInput2 &x2, TOutput &y) const
{
  y= x1 + x2;
}
//-------------------------------------------------------------------------------------------

/*!\brief MSubtract: subtraction calculator (y = x1 - x2)
    \todo specialize for vector to make efficient */
template <typename t_input>FUNCTION_2ISO_GEN (MSubtract, t_input, t_input, t_input)
template <typename t_input>void MSubtract<t_input>::function (const TInput1 &x1, const TInput2 &x2, TOutput &y) const
{
  y= x1 - x2;
}
//-------------------------------------------------------------------------------------------

template <typename t_input1, typename t_input2> struct TMultiplyOutputType {typedef t_input1 F;};
template<>struct TMultiplyOutputType <TInt,TReal> {typedef TReal F;};
template<>struct TMultiplyOutputType <TReal,TInt> {typedef TReal F;};
template<>struct TMultiplyOutputType <TInt,TIntVector> {typedef TIntVector F;};
template<>struct TMultiplyOutputType <TIntVector,TInt> {typedef TIntVector F;};
template<>struct TMultiplyOutputType <TReal,TRealVector> {typedef TRealVector F;};
template<>struct TMultiplyOutputType <TRealVector,TReal> {typedef TRealVector F;};
template<>struct TMultiplyOutputType <TRealMatrix,TRealVector> {typedef TRealVector F;};

/*!\brief MMultiply: multiplication calculator (y = x1 * x2)
    \todo specialize for vector and matrix to make efficient */
template <typename t_input1, typename t_input2>FUNCTION_2ISO_GEN (MMultiply, t_input1, t_input2, typename TMultiplyOutputType<t_input1 X_COMMA t_input2>::F)
template <typename t_input1, typename t_input2>void MMultiply<t_input1,t_input2>::function (const TInput1 &x1, const TInput2 &x2, TOutput &y) const
{
  y= x1 * x2;
}
//-------------------------------------------------------------------------------------------

/*!\brief MDivide: division calculator (y = x1 / x2) */
template <typename t_input>FUNCTION_2ISO_GEN (MDivide, t_input, t_input, t_input)
template <typename t_input>void MDivide<t_input>::function (const TInput1 &x1, const TInput2 &x2, TOutput &y) const
{
  y= x1 / x2;
}
//-------------------------------------------------------------------------------------------


/*!\brief MElemMultiply: elemental multiplication calculator (y[i] = x1[i] * x2[i], for all i; typeof(y)==typeof(x2)) */
template <typename t_input1, typename t_input2>FUNCTION_2ISO_GEN (MElemMultiply, t_input1, t_input2, t_input2)
template <typename t_input1, typename t_input2>void MElemMultiply<t_input1,t_input2>::function (const TInput1 &x1, const TInput2 &x2, TOutput &y) const
{
  LASSERT1op1(GenSize(x1),==,GenSize(x2));
  GenResize(y,GenSize(x1));
  typename TypeExt<TInput1>::const_iterator x1_itr(GenBegin(x1));
  typename TypeExt<TInput2>::const_iterator x2_itr(GenBegin(x2));
  for (typename TypeExt<TOutput>::iterator y_itr(GenBegin(y)),y_last(GenEnd(y)); y_itr!=y_last; ++y_itr,++x1_itr,++x2_itr)
    (*y_itr)= (*x1_itr) * (*x2_itr);
}
//-------------------------------------------------------------------------------------------

/*!\brief MElemDivide: elemental division calculator (y[i] = x1[i] - x2[i], for all i; typeof(y)==typeof(x2)) */
template <typename t_input1, typename t_input2>FUNCTION_2ISO_GEN (MElemDivide, t_input1, t_input2, t_input2)
template <typename t_input1, typename t_input2>void MElemDivide<t_input1,t_input2>::function (const TInput1 &x1, const TInput2 &x2, TOutput &y) const
{
  LASSERT1op1(GenSize(x1),==,GenSize(x2));
  GenResize(y,GenSize(x1));
  typename TypeExt<TInput1>::const_iterator x1_itr(GenBegin(x1));
  typename TypeExt<TInput2>::const_iterator x2_itr(GenBegin(x2));
  for (typename TypeExt<TOutput>::iterator y_itr(GenBegin(y)),y_last(GenEnd(y)); y_itr!=y_last; ++y_itr,++x1_itr,++x2_itr)
    (*y_itr)= (*x1_itr) / (*x2_itr);
}
//-------------------------------------------------------------------------------------------


/*!\brief MConstMultiplier's Configurations */
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
template <typename t_const, typename t_input>void MConstMultiplier<t_const,t_input>::function (const TInput &x, TOutput &y) const
{
  y= conf_.Factor * x;
}
//-------------------------------------------------------------------------------------------
/*!\brief Specialization to Factor: real vector, x,y: real vector; y[i]=Factor[i]*x[i] for all i */
template<>void MConstMultiplier<TRealVector,TRealVector>::function (const TInput &x, TOutput &y) const;
/*!\brief Specialization to Factor: real matrix, x,y: real vector; y = Factor * x */
template<>void MConstMultiplier<TRealMatrix,TRealVector>::function (const TInput &x, TOutput &y) const;
//-------------------------------------------------------------------------------------------


/*!\brief MSquare: square calculator (y = x^2)  */
template <typename t_input> FUNCTION_SISO_GEN (MSquare, t_input, t_input)
template <typename t_input> void MSquare<t_input>::function (const TInput &x, TOutput &y) const
{
  y= Square(x);
}
//-------------------------------------------------------------------------------------------

/*!\brief MElemSquare: square calculator (y[i] = (x[i])^2, for all i)  */
template <typename t_input> FUNCTION_SISO_GEN (MElemSquare, t_input, t_input)
template <typename t_input> void MElemSquare<t_input>::function (const TInput &x, TOutput &y) const
{
  GenResize(y, GenSize(x));
  typename TypeExt<TInput>::const_iterator  x_itr(GenBegin(x));
  for (typename TypeExt<TInput>::iterator y_itr(GenBegin(y)),y_last(GenEnd(y)); y_itr!=y_last; ++y_itr,++x_itr)
    (*y_itr)= Square(*x_itr);
}
//-------------------------------------------------------------------------------------------


#if 0
/*!\brief MGreaterThan: greater-than calculator (y = (x1 > x2)) */
template <typename t_input>FUNCTION_2ISO_GEN (MGreaterThan, t_input, t_input, TBool)
template <typename t_input>void MGreaterThan<t_input>::function (const TInput1 &x1, const TInput2 &x2, TOutput &y) const
{
  y= (x1 > x2);
}
/*!\brief MElemGreaterThan: elemental greater-than calculator (y[i] = (x1[i] > x2[i]), for all i) */
template <typename t_input1, typename t_input2>FUNCTION_2ISO_GEN (MElemGreaterThan, t_input1, t_input2, t_input2)
template <typename t_input1, typename t_input2>void MElemGreaterThan<t_input1,t_input2>::function (const TInput1 &x1, const TInput2 &x2, TOutput &y) const
{
  LASSERT1op1(GenSize(x1),==,GenSize(x2));
  GenResize(y,GenSize(x1));
  typename TypeExt<TInput1>::const_iterator x1_itr(GenBegin(x1));
  typename TypeExt<TInput2>::const_iterator x2_itr(GenBegin(x2));
  for (typename TypeExt<TOutput>::iterator y_itr(GenBegin(y)),y_last(GenEnd(y)); y_itr!=y_last; ++y_itr,++x1_itr,++x2_itr)
    (*y_itr)= ((*x1_itr) > (*x2_itr));
}
#endif

#define INEQ_CALCULATOR_GEN(x_mod_name, x_op)                \
  template <typename t_input>                                \
  FUNCTION_2ISO_GEN (x_mod_name, t_input, t_input, TBool)    \
  template <typename t_input>                                \
  void x_mod_name<t_input>::function (const TInput1 &x1, const TInput2 &x2, TOutput &y) const  \
    {y= (x1 x_op x2);}
INEQ_CALCULATOR_GEN(MEqualTo, ==)
INEQ_CALCULATOR_GEN(MGreaterThan, >)
INEQ_CALCULATOR_GEN(MGreaterEqual, >=)
INEQ_CALCULATOR_GEN(MLessThan, <)
INEQ_CALCULATOR_GEN(MLessEqual, <=)
#undef INEQ_CALCULATOR_GEN

#define ELEM_INEQ_CALCULATOR_GEN(x_mod_name, x_op)                    \
  template <typename t_input>                                         \
  FUNCTION_2ISO_GEN (x_mod_name, t_input, t_input, TBoolVector)       \
  template <typename t_input>                                         \
  void x_mod_name<t_input>::function (const TInput1 &x1, const TInput2 &x2, TOutput &y) const  \
  {                                                                   \
    LASSERT1op1(GenSize(x1),==,GenSize(x2));                          \
    GenResize(y,GenSize(x1));                                         \
    typename TypeExt<TInput1>::const_iterator x1_itr(GenBegin(x1));   \
    typename TypeExt<TInput2>::const_iterator x2_itr(GenBegin(x2));   \
    for (typename TypeExt<TOutput>::iterator y_itr(GenBegin(y)),y_last(GenEnd(y));  \
          y_itr!=y_last; ++y_itr,++x1_itr,++x2_itr)                   \
      {(*y_itr)= ((*x1_itr) x_op (*x2_itr));}                         \
  }
ELEM_INEQ_CALCULATOR_GEN(MElemEqualTo, ==)
ELEM_INEQ_CALCULATOR_GEN(MElemGreaterThan, >)
ELEM_INEQ_CALCULATOR_GEN(MElemGreaterEqual, >=)
ELEM_INEQ_CALCULATOR_GEN(MElemLessThan, <)
ELEM_INEQ_CALCULATOR_GEN(MElemLessEqual, <=)
#undef ELEM_INEQ_CALCULATOR_GEN
//-------------------------------------------------------------------------------------------


/*!\brief MBoolVectorAnd: y = x[0] and x[1] and ... and x[N-1]; x: input (bool vector), y: output (bool)  */
FUNCTION_SISO_GEN (MBoolVectorAnd, TBoolVector, TBool)
/*!\brief MBoolVectorOr: y = x[0] or x[1] or ... or x[N-1]; x: input (bool vector), y: output (bool)  */
FUNCTION_SISO_GEN (MBoolVectorOr, TBoolVector, TBool)
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

/*!\brief MNonzeroElements' Configurations */
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



#undef FUNCTION_SISO_GEN
#undef FUNCTION_SISO_GEN_CFG
#undef FUNCTION_2ISO_GEN
#undef X_COMMA
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MConstant,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MConstant,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MConstant,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MConstant,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MConstant,TRealMatrix)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MConstant,TBool)
// SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MConstant,TBoolVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MConstant,TString)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TInt ,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TInt ,TBool)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TReal,TInt )
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TReal,TBool)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TBool,TInt )
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCaster,TBool,TReal)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MVectorToScalar,TIntVector,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MVectorToScalar,TRealVector,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MVectorToScalar,TBoolVector,TBool)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MVectorShuffler,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MVectorShuffler,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MVectorShuffler,TBoolVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MAdd,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MAdd,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MAdd,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MAdd,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSubtract,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSubtract,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSubtract,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSubtract,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMultiply,TInt,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMultiply,TInt,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMultiply,TReal,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMultiply,TReal,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMultiply,TInt,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMultiply,TIntVector,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMultiply,TReal,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMultiply,TRealVector,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMultiply,TRealMatrix,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MDivide,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MDivide,TReal)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MElemMultiply,TIntVector,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MElemMultiply,TIntVector,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MElemMultiply,TRealVector,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MElemMultiply,TRealVector,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MElemDivide,TIntVector,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MElemDivide,TIntVector,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MElemDivide,TRealVector,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MElemDivide,TRealVector,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MConstMultiplier,TInt,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MConstMultiplier,TReal,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MConstMultiplier,TRealVector,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MConstMultiplier,TRealMatrix,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSquare,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSquare,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemSquare,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemSquare,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MEqualTo,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MEqualTo,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MGreaterThan,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MGreaterThan,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MGreaterEqual,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MGreaterEqual,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MLessThan,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MLessThan,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MLessEqual,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MLessEqual,TReal)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemEqualTo,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemEqualTo,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemGreaterThan,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemGreaterThan,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemGreaterEqual,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemGreaterEqual,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemLessThan,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemLessThan,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemLessEqual,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MElemLessEqual,TRealVector)
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

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_calculations_h
//-------------------------------------------------------------------------------------------
