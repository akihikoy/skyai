//-------------------------------------------------------------------------------------------
/*! \file    functions.h
    \brief   libskyai - interface of function modules
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
#ifndef skyai_functions_h
#define skyai_functions_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief single input, single output
template <typename t_input, typename t_output>
class MFunctionSISOInterface
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface         TParent;
  typedef MFunctionSISOInterface   TThis;
  typedef t_input                  TInput;
  typedef t_output                 TOutput;
  SKYAI_MODULE_NAMES(MFunctionSISOInterface)

  MFunctionSISOInterface (const std::string &v_instance_name)
    : TParent          (v_instance_name),
      slot_initialize  (*this),
      out_f1           (*this),
      out_f2           (*this),
      in_x             (*this),
      out_y            (*this),
      slot_x           (*this),
      signal_y         (*this)
    {
      this->add_slot_port  (slot_initialize );
      this->add_out_port   (out_f1          );
      this->add_out_port   (out_f2          );
      this->add_in_port    (in_x            );
      this->add_out_port   (out_y           );
      this->add_slot_port  (slot_x          );
      this->add_signal_port(signal_y        );
    }

  virtual ~MFunctionSISOInterface() {}

  void Initialize(void) {slot_initialize_exec();}

  void ExecFunction(const TInput &x, TOutput &y) const {function(x,y);}

protected:

  mutable TOutput tmp_y_;

  //!\brief if this slot catch a signal, this module is initialized
  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  MAKE_OUT_PORT(out_f1, const TOutput&, (const TInput &x), (x), TThis);
  MAKE_OUT_PORT(out_f2, void, (const TInput &x, TOutput &y), (x,y), TThis);

  MAKE_IN_PORT (in_x, const TInput& (void), TThis);
  //!\brief output y for in_x
  MAKE_OUT_PORT(out_y, const TOutput&, (void), (), TThis);

  MAKE_SLOT_PORT(slot_x, void, (const TInput &x), (x), TThis);
  //!\brief emit y for slot_x
  MAKE_SIGNAL_PORT(signal_y, void (const TOutput&), TThis);

  // generate get_##x_in (e.g. get_reward)
  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(x, const TInput&, (void), ())

  #undef GET_FROM_IN_PORT

  //! sometimes this is convenient to use
  const TOutput& out_f1_get (const TInput &x) const
    {
      // TOutput y;
      function (x,tmp_y_);
      return tmp_y_;
    }

  //! sometimes this is faster
  void out_f2_get (const TInput &x, TOutput &y) const
    {
      function (x,y);
    }

  const TOutput& out_y_get (void) const
    {
      function (get_x(),tmp_y_);
      return tmp_y_;
    }

  void slot_x_exec (const TInput &x)
    {
      function (x,tmp_y_);
      signal_y.ExecAll (tmp_y_);
    }

  virtual void slot_initialize_exec (void) {}

  virtual void function (const TInput &x, TOutput &y) const = 0;

};  // end of MFunctionSISOInterface
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief two input, single output
template <typename t_input1, typename t_input2, typename t_output>
class MFunction2ISOInterface
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface         TParent;
  typedef MFunction2ISOInterface   TThis;
  typedef t_input1                 TInput1;
  typedef t_input2                 TInput2;
  typedef t_output                 TOutput;
  SKYAI_MODULE_NAMES(MFunction2ISOInterface)

  MFunction2ISOInterface (const std::string &v_instance_name)
    : TParent          (v_instance_name),
      slot_initialize  (*this),
      out_f1           (*this),
      out_f2           (*this),
      in_x1            (*this),
      in_x2            (*this),
      out_y            (*this),
      slot_x           (*this),
      signal_y         (*this)
    {
      this->add_slot_port  (slot_initialize );
      this->add_out_port   (out_f1          );
      this->add_out_port   (out_f2          );
      this->add_in_port    (in_x1           );
      this->add_in_port    (in_x2           );
      this->add_out_port   (out_y           );
      this->add_slot_port  (slot_x          );
      this->add_signal_port(signal_y        );
    }

  virtual ~MFunction2ISOInterface() {}

  void Initialize(void) {slot_initialize_exec();}

  void ExecFunction(const TInput1 &x1, const TInput2 &x2, TOutput &y) const {function(x1,x2,y);}

protected:

  mutable TOutput tmp_y_;

  //!\brief if this slot catch a signal, this module is initialized
  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  MAKE_OUT_PORT(out_f1, const TOutput&, (const TInput1 &x1, const TInput2 &x2), (x1,x2), TThis);
  MAKE_OUT_PORT(out_f2, void, (const TInput1 &x1, const TInput2 &x2, TOutput &y), (x1,x2,y), TThis);

  MAKE_IN_PORT (in_x1, const TInput1& (void), TThis);
  MAKE_IN_PORT (in_x2, const TInput2& (void), TThis);
  //!\brief output y for in_x
  MAKE_OUT_PORT(out_y, const TOutput&, (void), (), TThis);

  MAKE_SLOT_PORT(slot_x, void, (const TInput1 &x1, const TInput2 &x2), (x1,x2), TThis);
  //!\brief emit y for slot_x
  MAKE_SIGNAL_PORT(signal_y, void (const TOutput&), TThis);

  // generate get_##x_in (e.g. get_reward)
  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(x1, const TInput1&, (void), ())
  GET_FROM_IN_PORT(x2, const TInput2&, (void), ())

  #undef GET_FROM_IN_PORT

  //! sometimes this is convenient to use
  const TOutput& out_f1_get (const TInput1 &x1, const TInput2 &x2) const
    {
      function (x1,x2,tmp_y_);
      return tmp_y_;
    }

  //! sometimes this is faster
  void out_f2_get (const TInput1 &x1, const TInput2 &x2, TOutput &y) const
    {
      function (x1,x2,y);
    }

  const TOutput& out_y_get (void) const
    {
      function (get_x1(),get_x2(),tmp_y_);
      return tmp_y_;
    }

  void slot_x_exec (const TInput1 &x1, const TInput2 &x2)
    {
      function (x1,x2,tmp_y_);
      signal_y.ExecAll (tmp_y_);
    }

  virtual void slot_initialize_exec (void) {}

  virtual void function (const TInput1 &x1, const TInput2 &x2, TOutput &y) const = 0;

};  // end of MFunction2ISOInterface
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief multiple input, single output
template <typename t_input, typename t_output>
class MFunctionMISOInterface
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface         TParent;
  typedef MFunctionMISOInterface   TThis;
  typedef t_input                  TInput;
  typedef t_output                 TOutput;
  typedef typename TypeToVector<TInput>::vector_type  TMultipleInput;
  SKYAI_MODULE_NAMES(MFunctionMISOInterface)

  MFunctionMISOInterface (const std::string &v_instance_name)
    : TParent          (v_instance_name),
      slot_initialize  (*this),
      out_f1           (*this),
      out_f2           (*this),
      in_x             (*this),
      out_y            (*this)
    {
      this->add_slot_port  (slot_initialize );
      this->add_out_port   (out_f1          );
      this->add_out_port   (out_f2          );
      this->add_in_port    (in_x            );
      this->add_out_port   (out_y           );
    }

  virtual ~MFunctionMISOInterface() {}

  void Initialize(void) {slot_initialize_exec();}

  void ExecFunction(const TMultipleInput &x, TOutput &y) const {function(x,y);}

protected:

  mutable TOutput tmp_y_;
  mutable TMultipleInput tmp_mx_;

  //!\brief if this slot catch a signal, this module is initialized
  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  MAKE_OUT_PORT(out_f1, const TOutput&, (const TMultipleInput &x), (x), TThis);
  MAKE_OUT_PORT(out_f2, void, (const TMultipleInput &x, TOutput &y), (x,y), TThis);

  MAKE_IN_PORT_SPECIFIC (in_x, const TInput& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);

  //!\brief output y for in_x
  MAKE_OUT_PORT(out_y, const TOutput&, (void), (), TThis);


  //! sometimes this is convenient to use
  const TOutput& out_f1_get (const TMultipleInput &x) const
    {
      // TOutput y;
      function (x,tmp_y_);
      return tmp_y_;
    }

  //! sometimes this is faster
  void out_f2_get (const TMultipleInput &x, TOutput &y) const
    {
      function (x,y);
    }

  const TOutput& out_y_get (void) const
    {
      GenResize(tmp_mx_, in_x.ConnectionSize());
      typename TypeExt<TMultipleInput>::iterator mx_itr(GenBegin(tmp_mx_));
      for (typename GET_PORT_TYPE(in_x)::TConnectedPortIterator itr(in_x.ConnectedPortBegin()),last(in_x.ConnectedPortEnd());
            itr!=last; ++itr,++mx_itr)
      {
        *mx_itr= in_x.GetCurrent(itr);
      }
      function (tmp_mx_,tmp_y_);
      return tmp_y_;
    }

  virtual void slot_initialize_exec (void) {}

  virtual void function (const TMultipleInput &x, TOutput &y) const = 0;

};  // end of MFunctionMISOInterface
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief single input, single output
template <typename t_input, typename t_feature>
class MBasisFunctionsInterface
    : public MFunctionSISOInterface <t_input, t_feature>
//===========================================================================================
{
public:
  typedef MFunctionSISOInterface <
                    t_input, t_feature>  TParent;
  typedef MBasisFunctionsInterface       TThis;
  typedef t_feature                      TFeature;
  SKYAI_MODULE_NAMES(MBasisFunctionsInterface)

  MBasisFunctionsInterface (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      out_feature_dim   (*this)
    {
      this->add_out_port (out_feature_dim);
    }

  virtual ~MBasisFunctionsInterface() {}

  const TInt& GetFeatureDim(void) const {return out_feature_dim_get();}

protected:

  //!\brief this port returns the dimensin of the feature
  MAKE_OUT_PORT(out_feature_dim, const TInt&, (void), (), TThis);

  virtual const TInt& out_feature_dim_get (void) const = 0;

};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_functions_h
//-------------------------------------------------------------------------------------------
