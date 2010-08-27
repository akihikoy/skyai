//-------------------------------------------------------------------------------------------
/*! \file    avf_wire_fitting_disc.h
    \brief   libskyai - wire-fitting function approximator module for action value function
              over continuous state-action space (Grid-base, DCOB) (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Dec.17, 2009-

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
#ifndef skyai_avf_wire_fitting_disc_h
#define skyai_avf_wire_fitting_disc_h
//-------------------------------------------------------------------------------------------
#include <skyai/modules_std/avf_wire_fitting_smpl.h>
#include <lora/small_classes.h>
#include <lora/octave_str.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

namespace avf_wire_fitting_detail
{
  enum TAVFWFGridConstraintKind
  {
    gckNone            =0,
    gckEllipseOnGrid     , //!< {des_i} are constrained inside each ellipse on the grid
    gckMinMax            , //!< the boundary constraint [Min, Max]
    gckEoGMinMax           //!< gckEllipseOnGrid + gckMinMax
  };
}
ENUM_STR_MAP_BEGIN_NS(avf_wire_fitting_detail, TAVFWFGridConstraintKind)
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail,  gckNone           )
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail,  gckEllipseOnGrid  )
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail,  gckMinMax         )
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail,  gckEoGMinMax      )
ENUM_STR_MAP_END_NS  (avf_wire_fitting_detail, TAVFWFGridConstraintKind)
SPECIALIZE_TVARIABLE_TO_ENUM(avf_wire_fitting_detail::TAVFWFGridConstraintKind)

//-------------------------------------------------------------------------------------------
namespace avf_wire_fitting_detail
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Configurations of MAVFWireFittingGridBase
class TAVFWireFittingGridBaseConfigurations
//===========================================================================================
{
public:

  TIntVector           Levels;      //!< vector of the number of partitions into which each dimension is divided
  TRealVector          Radiuses;    //!< vector of radiuse of a ellipse into which each des_i of wires is constrained

  TAVFWFGridConstraintKind  GConstraintKind;  /*!< \note TParent::param_box::ConstraintKind is ignored.
                                                  \todo remove ConstraintKind from param_box at the constructor */

  TAVFWireFittingGridBaseConfigurations(var_space::TVariableMap &mmap)
    :
      GConstraintKind  (gckEoGMinMax)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      //!\todo we need the method: param_box.Remove (WireSize);
      //!\todo we need the method: param_box.Remove (ConstraintKind);

      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Levels              );
      ADD( Radiuses            );
      ADD( GConstraintKind     );
      #undef ADD
    }

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief wire-fitting module to approximate an action value function (wires are generated from a grid action set) */
class MAVFWireFittingGridBase
    : public MAVFWireFittingSimple
//===========================================================================================
{
public:
  typedef MAVFWireFittingSimple        TParent;
  typedef MAVFWireFittingGridBase      TThis;
  SKYAI_MODULE_NAMES(MAVFWireFittingGridBase)

  MAVFWireFittingGridBase (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_grid_     (TParent::param_box_config_map())
    {
    }

protected:

  TAVFWireFittingGridBaseConfigurations conf_grid_;
  std::vector<TRealVector>  center_of_u_;

  override void slot_initialize_exec (void);
  override void slot_add_to_parameter_exec (const TParameter &diff);

};  // end of MAVFWireFittingGridBase
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of avf_wire_fitting_detail
//-------------------------------------------------------------------------------------------



namespace avf_wire_fitting_detail
{
  enum TAVFWFDcobConstraintKind
  {
    dckNone            =0,
    dckSpheres           , //!< {des_i} are constrained inside spheres centered at each basis function
    dckMinMax            , //!< the boundary constraint [Min, Max]
    dckSphMinMax           //!< dckSpheres + dckMinMax
  };
}
ENUM_STR_MAP_BEGIN_NS(avf_wire_fitting_detail, TAVFWFDcobConstraintKind)
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail,  dckNone        )
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail,  dckSpheres     )
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail,  dckMinMax      )
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail,  dckSphMinMax   )
ENUM_STR_MAP_END_NS  (avf_wire_fitting_detail, TAVFWFDcobConstraintKind)
SPECIALIZE_TVARIABLE_TO_ENUM(avf_wire_fitting_detail::TAVFWFDcobConstraintKind)

//-------------------------------------------------------------------------------------------
namespace avf_wire_fitting_detail
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Configurations of MAVFWireFittingDCOB
class TAVFWireFittingDCOBConfigurations
//===========================================================================================
{
public:

  TRealMatrix    Interval2Set;  //! each row should take a form (t_s,t_e), i.e. Nx2 matrix.  N is the number of partitions

  TAVFWFDcobConstraintKind  DConstraintKind;  //!< \note TParent::param_box::ConstraintKind is ignored.

  /*! using maximum norm as a distance instead of L-2 norm (in constraining parameters)
      \note this value should be the same as one which used in in_distance_to_nearest_bf */
  bool                       UsingMaxNorm;

  TAVFWireFittingDCOBConfigurations(var_space::TVariableMap &mmap)
    :
      DConstraintKind  (dckSpheres),
      UsingMaxNorm     (false)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      //!\todo we need the method: param_box.Remove (WireSize);
      //!\todo we need the method: param_box.Remove (ConstraintKind);
      //!\todo we need the method: param_box.Remove (ActionDim);

      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Interval2Set       );
      ADD( DConstraintKind    );
      ADD( UsingMaxNorm       );
      #undef ADD
    }

};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief wire-fitting module to approximate an action value function (wires are generated from the basis functions) */
class MAVFWireFittingDCOB
    : public MAVFWireFittingSimple
//===========================================================================================
{
public:
  typedef MAVFWireFittingSimple    TParent;
  typedef MAVFWireFittingDCOB      TThis;
  SKYAI_MODULE_NAMES(MAVFWireFittingDCOB)

  MAVFWireFittingDCOB (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_wfdcob_   (TParent::param_box_config_map()),
      in_distance_to_nearest_bf   (*this),
      in_center_state_set         (*this)
    {
      add_in_port ( in_distance_to_nearest_bf  );
      add_in_port ( in_center_state_set        );
    }

protected:

  TAVFWireFittingDCOBConfigurations conf_wfdcob_;

  TRealVector   des_i_diff;   //!< temporary variable

  //!\brief distance vector to the nearest basis function from each BF (dim == num. of BFs)
  MAKE_IN_PORT(in_distance_to_nearest_bf, const TRealVector& (void), TThis);

  //!\brief input a set of the center state of the basis functions
  MAKE_IN_PORT(in_center_state_set, const std::vector<TRealVector>& (void), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(distance_to_nearest_bf, const TRealVector&, (void), ())

  GET_FROM_IN_PORT(center_state_set, const std::vector<TRealVector>&, (void), ())

  #undef GET_FROM_IN_PORT

  override void slot_initialize_exec (void);
  override void slot_add_to_parameter_exec (const TParameter &diff);


  int get_wire_size (void) const
    {
      return get_center_state_set().size() * conf_wfdcob_.Interval2Set.rows();
    };
  int compose_action (const int s, const int &dT) const
    {
      const int NI (conf_wfdcob_.Interval2Set.rows());
      return  s * NI + dT;
    };
  void decompose_action (const int a, int &s, int &dT) const
    {
      const int NI (conf_wfdcob_.Interval2Set.rows());
      dT = a % NI;
      s  = (a-dT) / NI;
    };
  bool increment_action (int &s, int &dT) const
      //! update (s,dT) so that composeAction(s,dT) is incremented by 1 \return true if s is changed
    {
      const int NI (conf_wfdcob_.Interval2Set.rows());
      dT += 1;
      if (dT>=NI)  {dT=0; s+=1; return true;}
      return false;
    };

};  // end of MAVFWireFittingDCOB
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of avf_wire_fitting_detail
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_avf_wire_fitting_disc_h
//-------------------------------------------------------------------------------------------
