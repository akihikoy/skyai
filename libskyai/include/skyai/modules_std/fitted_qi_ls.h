//-------------------------------------------------------------------------------------------
/*! \file    fitted_qi_ls.h
    \brief   libskyai - Fitted Q Iteration implementation with a gradient descent method for the least square error
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Apr.13, 2010-

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
#ifndef skyai_fitted_qi_ls_h
#define skyai_fitted_qi_ls_h
//-------------------------------------------------------------------------------------------
#include <skyai/modules_std/fitted_qi_gen.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


namespace fitted_qi_detail
{
  //! type of the least square method
  enum TLSMethodType
    {
      mtBatch         =0,  //!< batch update (normal gradient descent)
      mtPointByPoint    ,  //!< update point by point (stochastic gradient descent)
    };
}
ENUM_STR_MAP_BEGIN_NS(fitted_qi_detail, TLSMethodType)
  ENUM_STR_MAP_ADD_NS(fitted_qi_detail, mtBatch         )
  ENUM_STR_MAP_ADD_NS(fitted_qi_detail, mtPointByPoint  )
ENUM_STR_MAP_END_NS  (fitted_qi_detail, TLSMethodType)
SPECIALIZE_TVARIABLE_TO_ENUM(fitted_qi_detail::TLSMethodType)


//-------------------------------------------------------------------------------------------
namespace fitted_qi_detail
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Configurations of MFittedQIterationSL
class TFittedQIterationSLConfigurations
//===========================================================================================
{
public:

  TReal                 Alpha;     //!< step-size parameter
  TReal                 AlphaMin; //!< lower bound of step-size parameter
  TReal                 AlphaDecreasingFactor;   //!< damping coefficient of the step-size parameter. larger is decreasing faster (becoming greedy). do not set a value greater than 1.

  TLSMethodType         LSMethodType;

  TReal                 SqErrGradientNormLimit;  //!< upper limit of the gradient of the square error function (==TD-error x AVF-gradient)

  TInt                  MaxNumberOfSLIteration;

  TFittedQIterationSLConfigurations (var_space::TVariableMap &mmap)
    :
      Alpha                         (0.5l),
      AlphaMin                      (DBL_TINY),
      AlphaDecreasingFactor         (0.002l),
      LSMethodType                  (mtPointByPoint),
      SqErrGradientNormLimit        (-1.0l),
      MaxNumberOfSLIteration        (3)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Alpha                           );
      ADD( AlphaMin                        );
      ADD( AlphaDecreasingFactor           );

      ADD( LSMethodType                    );
      ADD( SqErrGradientNormLimit          );

      ADD( MaxNumberOfSLIteration          );
      #undef ADD
    }

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief template reinforcement learning module using fitted Q iteration with a gradient descent method for the least square error
    \note the function approximator is NOT included in this module (use MAVFLinearDiscAction, etc.)
    \todo TODO: implement out_td_error_get function
*/
template <typename t_state, typename t_action>
class MFittedQIterationSL
    : public MFittedQIterationGen <t_state,t_action>
//===========================================================================================
{
public:
  typedef MFittedQIterationGen <t_state,t_action>  TParent;
  typedef typename TParent::TState                 TState;
  typedef typename TParent::TAction                TAction;
  typedef TActionValueFuncParamInterface           TParameter;
  typedef TActionValueFuncParamMemory              TParameterMemory;
  typedef MFittedQIterationSL                      TThis;
  SKYAI_MODULE_NAMES(MFittedQIterationSL)

  MFittedQIterationSL (const std::string &v_instance_name)
    : TParent                       (v_instance_name),
      sl_conf_                      (TParent::param_box_config_map()),
      signal_avf_add_to_parameter   (*this),
      // in_avf_parameter_ref          (*this),
      // in_avf_parameter_val          (*this),
      // in_avf_replacing_trace        (*this),
      in_avf_create_parameter       (*this),
      in_avf_zero_parameter         (*this)
    {
      this->add_signal_port (signal_avf_add_to_parameter);

      // this->add_in_port (in_avf_parameter_ref    );
      // this->add_in_port (in_avf_parameter_val    );
      // this->add_in_port (in_avf_replacing_trace  );
      this->add_in_port (in_avf_create_parameter );
      this->add_in_port (in_avf_zero_parameter   );

      this->gradient_      .SetAllocator (boost::bind(&TThis::get_avf_create_parameter,this));
      this->difference_    .SetAllocator (boost::bind(&TThis::get_avf_create_parameter,this));
    }

protected:

  TFittedQIterationSLConfigurations  sl_conf_;

  // [-- ports for a function approximator
  MAKE_SIGNAL_PORT(signal_avf_add_to_parameter, void (const TParameter &), TThis);

  // //!\brief output a reference to the parameter
  // MAKE_IN_PORT(in_avf_parameter_ref, const TParameter& (void), TThis);

  // //!\brief assign the parameter to outerparam
  // MAKE_IN_PORT(in_avf_parameter_val, void (TParameter &outerparam), TThis);

  // //!\brief apply a replacing trace to an eligibility trace eligibility_trace
  // MAKE_IN_PORT(in_avf_replacing_trace, void (TParameter &eligibility_trace), TThis);

  //!\brief create (allocate) a parameter vector that has the same size as the parameter and is initialized to be zero
  MAKE_IN_PORT(in_avf_create_parameter, TParameter* (void), TThis);

  //!\brief clear a parameter vector (set zero)
  MAKE_IN_PORT(in_avf_zero_parameter, void (TParameter &outerparam), TThis);
  // ports for a function approximator --]

  using TParent::ModuleUniqueCode        ;

  using TParent::conf_;
  using TParent::mem_;

  using TParent::get_avf_evaluate;

  using TParent::fqi_data_;

  // generate get_##x_in (e.g. get_reward)
  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  // GET_FROM_IN_PORT(avf_parameter_ref, const TParameter&, (void), ())
  // GET_FROM_IN_PORT(avf_parameter_val, void, (TParameter &outerparam), (outerparam))
  // GET_FROM_IN_PORT(avf_replacing_trace, void, (TParameter &eligibility_trace), (eligibility_trace))
  GET_FROM_IN_PORT(avf_create_parameter, TParameter*, (void), ())
  GET_FROM_IN_PORT(avf_zero_parameter, void, (TParameter &outerparam), (outerparam))

  #undef GET_FROM_IN_PORT

  // temporary variable
  TParameterMemory      gradient_;  //!< used in update
  TParameterMemory      difference_;  //!< used in update


  inline TReal get_alpha (void) const;

  // execution methods
  override void     reset_episode (); //!< execute this function to start new trial

  //! supervised learning iteration
  override void exec_supervised_learning (int number_of_used_samples);

};
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFittedQIterationSL,TContinuousState,TContinuousAction)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFittedQIterationSL,TContinuousState,TDiscreteAction)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFittedQIterationSL,TCompositeState,TCompositeAction)
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of fitted_qi_detail
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_fitted_qi_ls_h
//-------------------------------------------------------------------------------------------
