//-------------------------------------------------------------------------------------------
/*! \file    action_value_func.h
    \brief   libskyai - interface of a action value function module
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Dec.08, 2009-

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
#ifndef skyai_action_value_func_h
#define skyai_action_value_func_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <boost/function.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

/*!\brief interface class of a parameter vector of an action value function approximator
    \note Functions and operators must be performed in the context of the subclass.
        e.g. by executing operator=(rhs), the elements of rhs (subclass) should be copied to
          *this (subclass).
        For executing safely, cast the parameters of the functions/operators to the subclass
          by dynamic_cast. */
class TActionValueFuncParamInterface
{
public:

  virtual ~TActionValueFuncParamInterface(void) {}

  //!\brief assign zero (but size is not changed)
  virtual void Zero (void) = 0;

  //!\brief return the norm of the parameter
  virtual TReal Norm (void) const {LERROR("not implemented"); lexit(df);return 0.0l;}

  /*!\brief return (*this = rhs) */
  virtual const TActionValueFuncParamInterface& operator= (const TActionValueFuncParamInterface &rhs)
    {
      LERROR("subclass of TActionValueFuncParamInterface must override "
             "const TActionValueFuncParamInterface& operator= (const TActionValueFuncParamInterface&)");
      lexit(df); return *this;
    }

  /*!\brief return (*this += rhs) */
  virtual const TActionValueFuncParamInterface& operator+= (const TActionValueFuncParamInterface &rhs) = 0;

  //!\brief return (*this += weight*rhs)
  virtual const TActionValueFuncParamInterface& AddProd (const TReal &weight, const TActionValueFuncParamInterface &rhs) = 0;

  /*!\brief return (*this *= rhs) */
  virtual const TActionValueFuncParamInterface& operator*= (const TReal &rhs) = 0;

};
//-------------------------------------------------------------------------------------------

/*!\brief supplementary class to hold an allocated memory of an instance of a subclass of TActionValueFuncParamInterface
    \note as an example, see MTDGenericFuncApprox module
          defined in skyai/modules_std/td_generic_fa.h */
class TActionValueFuncParamMemory
{
public:
  // typedef TActionValueFuncParamInterface* (*TAllocator)(void);
  typedef boost::function<TActionValueFuncParamInterface*(void)>  TAllocator;

  TActionValueFuncParamMemory() : memory_(NULL), allocator_(NULL) {}
  TActionValueFuncParamMemory(TAllocator a, bool does_allocate=false) : memory_(NULL), allocator_(a)
    {if(does_allocate) Allocate();}
  ~TActionValueFuncParamMemory()  {Free();}

  TAllocator Allocator() const {return allocator_;}
  void SetAllocator(TAllocator a)  {allocator_= a;}

  void Allocate ()
    {
      if(allocator_.empty())
        {LERROR("fatal! cannot allocate memory because allocator_==NULL"); lexit(df);}
      Free();
      memory_= allocator_();
    }
  void Free()
    {
      if(memory_)  delete memory_;
      memory_= NULL;
    }
  TActionValueFuncParamInterface& operator()()
    {
      if(memory_==NULL)  Allocate();
      return *memory_;
    }
  const TActionValueFuncParamInterface& operator()() const
    {
      if(memory_==NULL)
        {LERROR("fatal! cannot allocate memory for const object"); throw;}
      return *memory_;
    }

  const TActionValueFuncParamMemory& operator=(const TActionValueFuncParamMemory &rhs)
    {
      if(memory_==NULL)  Allocate();
      (*memory_)= rhs();
      return *this;
    }

private:
  TActionValueFuncParamInterface *memory_;
  TAllocator allocator_;

};
//-------------------------------------------------------------------------------------------

/*!\brief attribute of an action value function at a state and an action
    \note This structure is used to return some values from a function (out-port).
        If a member pointer is NULL, its value is neither calculated nor assigned.
        Use TStateActionAttribute() if you need not to get the all values. */
struct TStateActionAttribute
{
  TValue *ActionValue;  // Q(s,a)
  TValue *StateValue;   // V(s)= argmax_a Q(s,a)
  TActionValueFuncParamInterface *Gradient;  // derivative of Q(s,a;w) w.r.t. parameter w

  TStateActionAttribute(void)
    : ActionValue(NULL), StateValue(NULL), Gradient(NULL)  {}

  TStateActionAttribute(TValue *q, TValue *v, TActionValueFuncParamInterface *grad)
    : ActionValue(q), StateValue(v), Gradient(grad)  {}

  bool IsNull() const {return ActionValue==NULL && StateValue==NULL && Gradient==NULL;}
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief interface module of a generic function approximator that approximates Q~f(x,a; param)
    \note parameter of the function should be a subclass of TActionValueFuncParamInterface */
template <typename t_state, typename t_action>
class MActionValueFuncInterface
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface                TParent;
  typedef t_state                         TState;
  typedef t_action                        TAction;
  typedef TActionValueFuncParamInterface  TParameter;
  typedef MActionValueFuncInterface       TThis;
  SKYAI_MODULE_NAMES(MActionValueFuncInterface)

  MActionValueFuncInterface (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_initialize        (*this),
      slot_reset             (*this),
      slot_add_to_parameter  (*this),
      out_parameter_ref      (*this),
      out_parameter_val      (*this),
      out_evaluate           (*this),
      out_greedy             (*this),
      out_select_action      (*this),
      out_replacing_trace    (*this),
      out_create_parameter   (*this),
      out_zero_parameter     (*this)
    {
      add_slot_port (slot_initialize       );
      add_slot_port (slot_reset            );
      add_slot_port (slot_add_to_parameter );

      add_out_port (out_parameter_ref    );
      add_out_port (out_parameter_val    );
      add_out_port (out_evaluate         );
      add_out_port (out_greedy           );
      add_out_port (out_select_action    );
      add_out_port (out_replacing_trace  );
      add_out_port (out_create_parameter );
      add_out_port (out_zero_parameter   );
    }

protected:

  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  //!\brief will be called at the beginning of each episode
  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  //!\brief add diff to the parameter
  MAKE_SLOT_PORT(slot_add_to_parameter, void, (const TParameter &diff), (diff), TThis);

  // i/o

  //!\brief output a reference to the parameter
  MAKE_OUT_PORT(out_parameter_ref, const TParameter&, (void), (), TThis);

  //!\brief assign the parameter to outerparam
  MAKE_OUT_PORT(out_parameter_val, void, (TParameter &outerparam), (outerparam), TThis);

  // MAKE_OUT_PORT(out_action_value, const TValue&, (const TState &x, const TAction &a), (x,a), TThis);
  // MAKE_OUT_PORT(out_gradient, void, (const TState &x, const TAction &a, TParameter &grad), (x,a,grad), TThis);
  // //!\brief calculate a greedy action at x, assign it into *greedy (if not NULL), assign its action value into q (if not NULL)
  // MAKE_OUT_PORT(out_greedy, void, (const TState &x, TAction *greedy, TValue *q), (x,greedy,q), TThis);

  MAKE_OUT_PORT(out_evaluate, void, (const TState &x, const TAction &a, TStateActionAttribute attrib), (x,a,attrib), TThis);

  //!\brief calculate a greedy action at x, assign it into *greedy (if not NULL), assign its attribute into attrib
  MAKE_OUT_PORT(out_greedy, void, (const TState &x, TAction *greedy, TStateActionAttribute attrib), (x,greedy,attrib), TThis);

  /*!\brief select an action at the current state by the current polocy
      \param [out]a  : if not NULL, selected action is stored
      \param [out]attrib  : the action value, the state value, and the gradient are stored */
  MAKE_OUT_PORT(out_select_action, void, (TAction *a, TStateActionAttribute attrib), (a,attrib), TThis);

  //!\brief apply a replacing trace to an eligibility trace eligibility_trace
  MAKE_OUT_PORT(out_replacing_trace, void, (TParameter &eligibility_trace), (eligibility_trace), TThis);


  /*!\brief create (allocate) a parameter vector that has the same size as the parameter (initialized to be zero)
      \note the obtained parameter should be freed by the user */
  MAKE_OUT_PORT(out_create_parameter, TParameter*, (void), (), TThis);

  //!\brief clear a parameter vector (set zero)
  MAKE_OUT_PORT(out_zero_parameter, void, (TParameter &outerparam), (outerparam), TThis);


  virtual void slot_initialize_exec (void) = 0;
  virtual void slot_reset_exec (void) = 0;
  virtual void slot_add_to_parameter_exec (const TParameter &diff) = 0;

  virtual const TParameter& out_parameter_ref_get (void) const = 0;
  virtual void out_parameter_val_get (TParameter &outerparam) const = 0;
  // virtual const TValue& out_action_value_get (const TState &x, const TAction &a) const = 0;
  // virtual void out_gradient_get (const TState &x, const TAction &a, TParameter &grad) const = 0;
  virtual void out_evaluate_get (const TState &x, const TAction &a, TStateActionAttribute attrib) const = 0;
  virtual void out_greedy_get (const TState &x, TAction *greedy, TStateActionAttribute attrib) const = 0;
  virtual void out_select_action_get (TAction *a, TStateActionAttribute attrib) const = 0;
  virtual void out_replacing_trace_get (TParameter &eligibility_trace) const = 0;

  virtual TParameter* out_create_parameter_get (void) const = 0;
  virtual void out_zero_parameter_get (TParameter &outerparam) const = 0;

};  // end of MActionValueFuncInterface
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_action_value_func_h
//-------------------------------------------------------------------------------------------
