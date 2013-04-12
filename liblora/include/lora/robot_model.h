//-------------------------------------------------------------------------------------------
/*! \file    robot_model.h
    \brief   liblora - Robot model definition for ODE
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jan.31, 2013

    Copyright (C) 2013  Akihiko Yamaguchi

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
#ifndef loco_rabbits_robot_model_h
#define loco_rabbits_robot_model_h
//-------------------------------------------------------------------------------------------
#include <lora/variable_space_impl.h>
#include <lora/string.h>
#include <lora/ode.h>
#include <lora/type_gen.h>
#include <vector>
#include <valarray>
#include <boost/function.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

namespace xode
{

enum TGeometryType
{
  gtNone        =-1,
  gtBox         =0,
  gtCapsule     ,
  gtCylinder    ,
  gtPlane       ,
  gtSphere      ,
  gtTriMesh
};
//-------------------------------------------------------------------------------------------

enum TInertiaType
{
  itFromGeometryWithDensity =0,
  itFromGeometryWithMass,
  itManual
};
//-------------------------------------------------------------------------------------------

enum TJointType
{
  jtNone      =-1,
  jtBall      =0,
  jtHinge     ,
  jtSlider    ,
  jtUniversal ,
  jtHinge2    ,
  jtFixed
};
//-------------------------------------------------------------------------------------------

enum TJointVisualizeType
{
  jvtSphere      =0,
  jvtCapsule     ,
  jvtCylinder
};
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of xode
//-------------------------------------------------------------------------------------------


// Construct enum to string map:
ENUM_STR_MAP_BEGIN_NS(xode,TGeometryType)
ENUM_STR_MAP_ADD_NS(xode, gtNone      )
ENUM_STR_MAP_ADD_NS(xode, gtBox       )
ENUM_STR_MAP_ADD_NS(xode, gtCapsule   )
ENUM_STR_MAP_ADD_NS(xode, gtCylinder  )
ENUM_STR_MAP_ADD_NS(xode, gtPlane     )
ENUM_STR_MAP_ADD_NS(xode, gtSphere    )
ENUM_STR_MAP_ADD_NS(xode, gtTriMesh   )
ENUM_STR_MAP_END_NS(xode,TGeometryType)
SPECIALIZE_TVARIABLE_TO_ENUM(xode::TGeometryType)

ENUM_STR_MAP_BEGIN_NS(xode,TInertiaType)
ENUM_STR_MAP_ADD_NS(xode, itFromGeometryWithDensity  )
ENUM_STR_MAP_ADD_NS(xode, itFromGeometryWithMass     )
ENUM_STR_MAP_ADD_NS(xode, itManual                   )
ENUM_STR_MAP_END_NS(xode,TInertiaType)
SPECIALIZE_TVARIABLE_TO_ENUM(xode::TInertiaType)

ENUM_STR_MAP_BEGIN_NS(xode,TJointType)
ENUM_STR_MAP_ADD_NS(xode, jtNone       )
ENUM_STR_MAP_ADD_NS(xode, jtBall       )
ENUM_STR_MAP_ADD_NS(xode, jtHinge      )
ENUM_STR_MAP_ADD_NS(xode, jtSlider     )
ENUM_STR_MAP_ADD_NS(xode, jtUniversal  )
ENUM_STR_MAP_ADD_NS(xode, jtHinge2     )
ENUM_STR_MAP_ADD_NS(xode, jtFixed      )
ENUM_STR_MAP_END_NS(xode,TJointType)
SPECIALIZE_TVARIABLE_TO_ENUM(xode::TJointType)

ENUM_STR_MAP_BEGIN_NS(xode,TJointVisualizeType)
ENUM_STR_MAP_ADD_NS(xode, jvtSphere    )
ENUM_STR_MAP_ADD_NS(xode, jvtCapsule   )
ENUM_STR_MAP_ADD_NS(xode, jvtCylinder  )
ENUM_STR_MAP_END_NS(xode,TJointVisualizeType)
SPECIALIZE_TVARIABLE_TO_ENUM(xode::TJointVisualizeType)


//-------------------------------------------------------------------------------------------
namespace xode
{
//-------------------------------------------------------------------------------------------


inline void VectorToPosition(const std::vector<TReal> &v, dVector3 pos)
{
  LASSERT1op1(v.size(),==,3);
  pos[0]= v[0]; pos[1]= v[1]; pos[2]= v[2];
}
inline void VectorToRotation(const std::vector<TReal> &v, dMatrix3 R)
{
  LASSERT1op1(v.size(),==,9);
  #define _R(R,i,j) ((R)[(i)*4+(j)])
  dRSetIdentity(R);
  _R(R,0,0)= v[0]; _R(R,0,1)= v[1]; _R(R,0,2)= v[2];
  _R(R,1,0)= v[3]; _R(R,1,1)= v[4]; _R(R,1,2)= v[5];
  _R(R,2,0)= v[6]; _R(R,2,1)= v[7]; _R(R,2,2)= v[8];
  #undef _R
}
//-------------------------------------------------------------------------------------------

class TWorld;
//-------------------------------------------------------------------------------------------

template <typename t_param>
struct TNamedParam
{
  typedef std::map<std::string,t_param>  P;
  typedef std::vector<typename std::map<std::string,t_param>::iterator>  IdxMapper;
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*! \brief Geometry parameters

  Required parameters:
  - gtBox      : Lx, Ly, Lz, Color
  - gtCapsule  : Radius, Length, Color (Z-direction)
  - gtCylinder : Radius, Length, Color (Z-direction)
  - gtPlane    : A, B, C, D, Color (Plane Ax+By+Cz=D; norm(A,B,C) should be 1)
  - gtSphere   : Radius, Color
  - gtTriMesh  : Vertices, Indices, (Directions) {Color or Colors}
*/
class TGeometryParameters
//===========================================================================================
{
public:
  TGeometryType Type;
  TReal  Lx, Ly, Lz;
  TReal  Radius, Length;
  TReal  A, B, C, D;
  std::vector<TReal> Color;  //!< Size should be 1(brightness) or 3(RGB) or 4(RGBA)

  std::vector<TReal> Vertices;  //!< =[(x1,y1,z1), (x2,y2,z2), ...]
  std::vector<int>   Indices;  //!< =[(p11,p12,p13), (p21,p22,p23), ...]
  std::vector<int>   Directions;  /*!< =[d1={-1,1}, d2={-1,1}, ...].
                                Used to chenge the side.
                                Size should be Indices.size()/3.
                                If the size is zero, every element is assumed to be 1. */
  std::vector<TReal> Colors;  /*!< =[(r1,g1,b1,a1), (r2,g2,b2,a2), ...].
                                Size should be Indices.size()/3 * 4 */

  TGeometryParameters() :
      Type(gtNone),
      Lx(0), Ly(0), Lz(0),
      Radius(0), Length(0),
      A(0), B(0), C(0), D(0)
    {}

};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TLinkParameters
//===========================================================================================
{
public:
  TGeometryParameters Geometry;
  TInertiaType        InertiaType;
  TReal               Mass;  //!< Used when InertiaType==itFromGeometryWithMass or itManual
  TReal               Density;  //!< Used when InertiaType==itFromGeometryWithDensity
  std::vector<TReal>  CoMOffset;  /*!< =[x,y,z].  Position offset of the Center of Mass.
                                    Used when InertiaType==itManual */
  std::vector<TReal>  InertiaMatrix; /*!< =[M11,M12,M13, M21,M22,M23, M31,M32,M33]
                                      Used when InertiaType==itManual */
  bool  MoveCoMToCenter;  //!< If true, move CoM to (0,0,0).  Useful for geometory==trimesh

  std::vector<TReal>  Position;  //!< =[x,y,z]
  std::vector<TReal>  Rotation;  //!< =[R11,R12,R13, R21,R22,R23, R31,R32,R33]

  TLinkParameters() :
      InertiaType(itFromGeometryWithDensity),
      Mass(0),
      Density(0),
      MoveCoMToCenter(false)
    {}

};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TStaticLinkParameters
//===========================================================================================
{
public:
  TGeometryParameters Geometry;
  std::vector<TReal>  Position;
  std::vector<TReal>  Rotation;  //!< =[R11,R12,R13, R21,R22,R23, R31,R32,R33]

  TStaticLinkParameters()
    {}

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TAxisParameters
//===========================================================================================
{
public:
  TReal LoStop;
  TReal HiStop;
  TReal FMax;

  TAxisParameters() :
      LoStop(-dInfinity),
      HiStop(dInfinity),
      FMax(0)
    {}
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*! \brief Joint parameters

  Required parameters:
  - common : Body1, Body2
  - jtBall      : Anchor
  - jtHinge     : Anchor, Axis1, AParam1
  - jtSlider    : Axis1, AParam1
  - jtUniversal : Anchor, Axis1, Axis2, AParam1, AParam2
  - jtHinge2    : Anchor, Axis1, Axis2, AParam1, AParam2
  - jtFixed     :
*/
class TJointParameters
//===========================================================================================
{
public:
  std::string         Body1, Body2;
  TJointType          Type;
  std::vector<TReal>  Axis1, Axis2;  //!< =[x,y,z]
  std::vector<TReal>  Anchor;  //!< =[x,y,z]

  TAxisParameters  AParam1, AParam2;

  bool                Visualize;
  TJointVisualizeType VizType;
  TReal               VizRadius, VizLength;
  std::vector<TReal>  VizColor;  //!< Size should be 1(brightness) or 3(RGB) or 4(RGBA)

  TJointParameters() :
      Body1(""), Body2(""),
      Type(jtNone),
      Visualize(true),
      VizType(jvtCylinder),
      VizRadius(0.1), VizLength(0.3),
      id_(NULL)
    {}
private:
  friend class TWorld;
  dJointID  id_;
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
//! Force sensor inherits a link model which is the phisical model of the sensor
class TForceSensorParameters : public TLinkParameters
//===========================================================================================
{
public:
  std::string      Attached;  //!< Body on which the force sensor is attached

  TForceSensorParameters() :
      Attached("")
    {}
private:
  friend class TWorld;
  TJointParameters fixed_joint_;  //!< Fixed joint to attach the sensor to the body
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Surface model parameters (cf. dSurfaceParameters)
class TSurfaceParameters
//===========================================================================================
{
public:
  bool ContactMu2        ;
  bool ContactFDir1      ;
  bool ContactBounce     ;
  bool ContactSoftERP    ;
  bool ContactSoftCFM    ;
  bool ContactMotion1    ;
  bool ContactMotion2    ;
  bool ContactMotionN    ;
  bool ContactSlip1      ;
  bool ContactSlip2      ;
  bool ContactApprox0    ;
  bool ContactApprox1_1  ;
  bool ContactApprox1_2  ;
  bool ContactApprox1    ;

  TReal Mu        ;
  TReal Mu2       ;
  TReal Bounce    ;
  TReal BounceVel ;
  TReal SoftERP   ;
  TReal SoftCFM   ;
  TReal Motion1   ;
  TReal Motion2   ;
  TReal MotionN   ;
  TReal Slip1     ;
  TReal Slip2     ;

  TSurfaceParameters() :
      ContactMu2        (false),
      ContactFDir1      (false),
      ContactBounce     (true),
      ContactSoftERP    (false),
      ContactSoftCFM    (true),
      ContactMotion1    (false),
      ContactMotion2    (false),
      ContactMotionN    (false),
      ContactSlip1      (false),
      ContactSlip2      (false),
      ContactApprox0    (false),
      ContactApprox1_1  (false),
      ContactApprox1_2  (false),
      ContactApprox1    (false),
      Mu                (dInfinity),
      Mu2               (0.0),
      Bounce            (0.1),
      BounceVel         (0.1),
      SoftERP           (0.0),
      SoftCFM           (0.01),
      Motion1           (0.0),
      Motion2           (0.0),
      MotionN           (0.0),
      Slip1             (0.0),
      Slip2             (0.0)
    {}

};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Individual surface model parameters
class TIndividualSurfaceParameters
//===========================================================================================
{
public:
  // std::string Robot1         ;
  // std::string Link1          ;
  // std::string StaticObject1  ;
  // std::string StaticLink1    ;
  // std::string Robot2         ;
  // std::string Link2          ;
  // std::string StaticObject2  ;
  // std::string StaticLink2    ;
  // TSurfaceParameters Surface ;
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TJointManipulator
//===========================================================================================
{
public:
  std::string    Name;   //!< Joint name
  int            Index;  //!< 0: axis1, 1: axis2

  TJointManipulator() :
      Name  (""),
      Index (0)
    {}
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TRobotParameters
//===========================================================================================
{
public:
  TNamedParam<TLinkParameters>::P        Links;
  TNamedParam<TJointParameters>::P       Joints;
  TNamedParam<TForceSensorParameters>::P ForceSensors;

  std::string                          RootLink;
  std::vector<TReal>                   Position; //!< =[x,y,z]; Initial position of RootLink.
  std::vector<TReal>                   Rotation; //!< =[R11,R12,R13, R21,R22,R23, R31,R32,R33]; Initial rotation of RootLink.
  std::vector<TReal>                   InitialJointAngles;  //!< Initial joint angles/slider positions where the order of elements corresponds to that of JointAngleObserver.

  std::vector<TJointManipulator>       JointAngleObserver;  //!< Arrangement of a vector obtained by GetJointAngles to observe joint angles/slider positions
  std::vector<TJointManipulator>       JointAngVelObserver;  //!< Arrangement of a vector obtained by GetJointAngVels to observe joint angular velocities/slider velocities
  std::vector<TJointManipulator>       JointTorqueAdder;  //!< Arrangement of a vector input to AddToJointTorques to add torques/forces

  std::vector<std::string>             ForceObserver;  //!< Arrangement of a vector obtained by GetForces to observe force sensors

  std::vector<std::string>             LinkContactObserver;  //!< Arrangement of a vector obtained by LinkContacts to observe link contact (true/false)

  TRobotParameters() :
      RootLink ("")
    {}

private:
  friend class TWorld;
  TNamedParam<TLinkParameters>::IdxMapper links_;
  TNamedParam<TJointParameters>::IdxMapper joints_;
  TNamedParam<TForceSensorParameters>::IdxMapper force_sensors_;
  void update()
    {
      int i;
      links_.resize(Links.size()); i=0; for(TNamedParam<TLinkParameters>::P::iterator itr(Links.begin()),last(Links.end());itr!=last;++itr,++i) links_[i]=itr;
      joints_.resize(Joints.size()); i=0; for(TNamedParam<TJointParameters>::P::iterator itr(Joints.begin()),last(Joints.end());itr!=last;++itr,++i) joints_[i]=itr;
      force_sensors_.resize(ForceSensors.size()); i=0; for(TNamedParam<TForceSensorParameters>::P::iterator itr(ForceSensors.begin()),last(ForceSensors.end());itr!=last;++itr,++i) force_sensors_[i]=itr;
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TStaticObjectParameters
//===========================================================================================
{
public:
  TNamedParam<TStaticLinkParameters>::P Links;

  std::vector<TReal>  Position; //!< =[x,y,z]; Translate whole Links.
  std::vector<TReal>  Rotation; /*!< =[R11,R12,R13, R21,R22,R23, R31,R32,R33];
                                  Rotate whole Links.
                                  Note: first, rotate Links around (0,0,0), then translate Links. */

  TStaticObjectParameters()
    {}

private:
  friend class TWorld;
  TNamedParam<TStaticLinkParameters>::IdxMapper links_;
  void update()
    {
      int i;
      links_.resize(Links.size()); i=0; for(TNamedParam<TStaticLinkParameters>::P::iterator itr(Links.begin()),last(Links.end());itr!=last;++itr,++i) links_[i]=itr;
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TWorldParameters
//===========================================================================================
{
public:
  TNamedParam<TRobotParameters>::P        Robots;
  TNamedParam<TStaticObjectParameters>::P StaticObjects;

  TSurfaceParameters                   DefaultSurface;
  //TODO: implement extra-surface configurations between arbitrary two geometories

  std::vector<TReal>                   Gravity;
  TReal                                WorldCFM;
  int                                  MaxContacts;

  TReal                                TimeStep;
  bool                                 UsingQuickStep;  //!< Use quickStep to step the world. Fast, but low accuracy
  int                                  QuickStepIterationNum;  //!< Number of iteration used in quickStep

  bool                                 ConsoleMode;
  TReal                                DisplayFPS;
  std::vector<TReal>                   ViewPoint;

  TWorldParameters() :
      Gravity(3,0.0l),
      WorldCFM(1e-5l),
      MaxContacts(10),
      TimeStep(0.001l),
      UsingQuickStep(false),
      QuickStepIterationNum(20),
      ConsoleMode(false),
      DisplayFPS(50.0l)
    {
      Gravity[2]= -9.8;
      TReal view[]= {0.6667l,1.7789l,1.3300l, -104.0000l,-29.5000l,0.0000l};
      ViewPoint= std::vector<TReal>(view,view+SIZE_OF_ARRAY(view));
    }

private:
  friend class TWorld;
  TNamedParam<TRobotParameters>::IdxMapper robots_;
  TNamedParam<TStaticObjectParameters>::IdxMapper static_objects_;
  void update()
    {
      int i;
      robots_.resize(Robots.size()); i=0; for(TNamedParam<TRobotParameters>::P::iterator itr(Robots.begin()),last(Robots.end());itr!=last;++itr,++i) robots_[i]=itr;
      static_objects_.resize(StaticObjects.size()); i=0; for(TNamedParam<TStaticObjectParameters>::P::iterator itr(StaticObjects.begin()),last(StaticObjects.end());itr!=last;++itr,++i) static_objects_[i]=itr;
    }
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of xode
//-------------------------------------------------------------------------------------------

namespace var_space
{
  void Register (xode::TGeometryParameters &x, TVariableMap &mmap);
  void Register (xode::TLinkParameters &x, TVariableMap &mmap);
  void Register (xode::TStaticLinkParameters &x, TVariableMap &mmap);
  void Register (xode::TAxisParameters &x, TVariableMap &mmap);
  void Register (xode::TJointParameters &x, TVariableMap &mmap);
  void Register (xode::TForceSensorParameters &x, TVariableMap &mmap);
  void Register (xode::TSurfaceParameters &x, TVariableMap &mmap);
  void Register (xode::TJointManipulator &x, TVariableMap &mmap);
  void Register (xode::TRobotParameters &x, TVariableMap &mmap);
  void Register (xode::TStaticObjectParameters &x, TVariableMap &mmap);
  void Register (xode::TWorldParameters &x, TVariableMap &mmap);
}
//-------------------------------------------------------------------------------------------





//-------------------------------------------------------------------------------------------
namespace xode
{
//-------------------------------------------------------------------------------------------


// the following classes have a copy constructor and operator= that do nothing;
// these classes are defined in order to use std::vector of them
#define DEF_NC(x_class) \
  class TNC##x_class : public d##x_class  \
  {                                       \
  public:                                 \
    TNC##x_class() : d##x_class(){}       \
    TNC##x_class(const TNC##x_class&)     \
      : d##x_class(){}                    \
    const TNC##x_class& operator=(const TNC##x_class&) {return *this;} \
  private:                                \
  };
DEF_NC(Body)
DEF_NC(Box)
DEF_NC(Capsule)
DEF_NC(Cylinder)
DEF_NC(Plane)
DEF_NC(Sphere)
DEF_NC(BallJoint)
DEF_NC(HingeJoint)
DEF_NC(SliderJoint)
DEF_NC(UniversalJoint)
DEF_NC(Hinge2Joint)
DEF_NC(FixedJoint)
#undef DEF_NC

class TTriMeshGeom : public dGeom
{
public:
  TTriMeshGeom() : dGeom() {}

  TTriMeshGeom(const TTriMeshGeom&) : dGeom() {}
  const TTriMeshGeom& operator=(const TTriMeshGeom&) {return *this;}

  dGeomID& create() {return _id;}

  void create(dSpaceID space)
    {
      dTriMeshDataID new_tmdata= dGeomTriMeshDataCreate();
      dGeomTriMeshDataBuildSingle(new_tmdata, &vertices_[0], 3*sizeof(float), vertices_.size()/3,
                  &indices_[0], indices_.size(), 3*sizeof(dTriIndex));
      _id= dCreateTriMesh(space, new_tmdata, NULL, NULL, NULL);
    }
  void setBody(dBodyID body)  {dGeomSetBody (_id,body);}

  const std::valarray<float>& getVertices() const {return vertices_;}
  const std::valarray<dTriIndex>& getIndices() const {return indices_;}

  std::valarray<float>& setVertices() {return vertices_;}
  std::valarray<dTriIndex>& setIndices() {return indices_;}

  void setVertices(const std::vector<TReal> &array)
    {
      LASSERT1op1(array.size()%3,==,0);
      vertices_.resize(array.size());
      std::vector<TReal>::const_iterator aitr(array.begin()), alast(array.end());
      for(float *itr(&vertices_[0]);aitr!=alast;++itr,++aitr)
        *itr= *aitr;
    }
  void setIndices(const std::vector<int> &array)
    {
      LASSERT1op1(array.size()%3,==,0);
      indices_.resize(array.size());
      std::vector<int>::const_iterator aitr(array.begin()), alast(array.end());
      for(dTriIndex *itr(&indices_[0]);aitr!=alast;++itr,++aitr)
        *itr= *aitr;
    }
  void setIndices(const std::vector<int> &array, const std::vector<int> &directions)
    {
      LASSERT1op1(array.size()%3,==,0);
      LASSERT1op1(array.size()/3,==,directions.size());
      indices_.resize(array.size());
      std::vector<int>::const_iterator aitr(array.begin()), alast(array.end());
      std::vector<int>::const_iterator ditr(directions.begin());
      for(dTriIndex *itr(&indices_[0]);aitr!=alast;itr+=3,aitr+=3,++ditr)
      {
        if(*ditr>0)
        {
          itr[0]= aitr[0];
          itr[1]= aitr[1];
          itr[2]= aitr[2];
        }
        else
        {
          itr[0]= aitr[0];
          itr[1]= aitr[2];
          itr[2]= aitr[1];
        }
      }
    }

protected:

  std::valarray<float> vertices_;
  std::valarray<dTriIndex> indices_;

};
//-------------------------------------------------------------------------------------------

struct TCallbacks
{
  boost::function<void (TWorld &world, const TReal &timestep)> StartOfTimeStep;
  boost::function<void (TWorld &world, const TReal &timestep)> EndOfTimeStep;
  boost::function<void (TWorld &world, const TReal &timestep)> StartOfDrawing;
  boost::function<void (TWorld &world, const TReal &timestep)> EndOfDrawing;
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TWorld
//===========================================================================================
{
public:
  TWorld() :
      space_(0),
      time_(0.0l),
      old_fps_(0.0l),
      repaint_time_(0)
    {}

  void Clear();

  bool Create();
  void Draw() const;
  void Start();
  void Stop();
  void Step(bool pause=false);
  void NearCallback(dGeomID o1, dGeomID o2);

  bool LoadFromFile(const std::string &file_name);
  bool SaveToFile(const std::string &file_name);

  TCallbacks& SetCallbacks()  {return callbacks_;}
  const TCallbacks& Callbacks() const {return callbacks_;}

  TWorldParameters& SetParams()  {return params_;}
  const TWorldParameters& Params() const {return params_;}

  const TReal& Time() const {return time_;}

  /*! Return body index of specified robot_name and link_name.
      \note The body index is unique in TWorld object. */
  int LinkBodyIndex(const std::string &robot_name, const std::string &link_name) const
    {
      std::map<std::string, std::map<std::string, int> >::const_iterator ritr= name_to_body_.find(robot_name);
      if(ritr==name_to_body_.end())
        {LERROR("Robot \""<<robot_name<<"\" not found");  return -1;}
      std::string target_link_name;
      if(link_name!="")
      {
        target_link_name= link_name;
      }
      else
      {
        int ridx= RobotIndex(robot_name);
        if(idx_to_robotparams_[ridx]->RootLink=="")
          {LERROR("Robot \""<<robot_name<<"\": RootLink not assigned");  return -1;}
        target_link_name= idx_to_robotparams_[ridx]->RootLink;
      }
      std::map<std::string, int>::const_iterator litr= ritr->second.find(target_link_name);
      if(litr==ritr->second.end())
        {LERROR("In robot "<<robot_name<<": link \""<<target_link_name<<"\" not found"); return -1;}
      return litr->second;
    }
  //! LinkBodyIndex of RootLink
  int RootLinkBodyIndex(const std::string &robot_name) const
    {
      return LinkBodyIndex(robot_name,"");
    }

  std::vector<TNCBody>& Body() {return body_;}
  TNCBody& Body(int body_index) {return body_[body_index];}
  TNCBody& Body(const std::string &robot_name, const std::string &link_name)
    {
      int body_index= LinkBodyIndex(robot_name, link_name);
      if(body_index<0)  lexit(df);
      return body_[body_index];
    }
  dBodyID BodyID(int body_index) {return body_[body_index];}
  dBodyID BodyID(const std::string &robot_name, const std::string &link_name)
    {
      int body_index= LinkBodyIndex(robot_name, link_name);
      if(body_index<0)  return NULL;
      return body_[body_index];
    }

  //! Assign the position of body[body_index] to the result vector
  template<typename t_forward_iterator>
  void GetBodyPosition(int body_index, t_forward_iterator res_itr, t_forward_iterator res_last) const
    {
      const dReal *br(body_[body_index].getPosition());
      *res_itr= *br; ++res_itr; ++br; if(res_itr==res_last) return;
      *res_itr= *br; ++res_itr; ++br; if(res_itr==res_last) return;
      *res_itr= *br; ++res_itr; ++br;
    }
  //! Assign the linear-velocity of body[body_index] to the result vector
  template<typename t_forward_iterator>
  void GetBodyLinearVel(int body_index, t_forward_iterator res_itr, t_forward_iterator res_last) const
    {
      const dReal *br(body_[body_index].getLinearVel());
      *res_itr= *br; ++res_itr; ++br; if(res_itr==res_last) return;
      *res_itr= *br; ++res_itr; ++br; if(res_itr==res_last) return;
      *res_itr= *br; ++res_itr; ++br;
    }
  //! Assign the angular-velocity of body[body_index] to the result vector
  template<typename t_forward_iterator>
  void GetBodyAngularVel(int body_index, t_forward_iterator res_itr, t_forward_iterator res_last) const
    {
      const dReal *br(body_[body_index].getAngularVel());
      *res_itr= *br; ++res_itr; ++br; if(res_itr==res_last) return;
      *res_itr= *br; ++res_itr; ++br; if(res_itr==res_last) return;
      *res_itr= *br; ++res_itr; ++br;
    }
  //! Assign the quaternion of body[body_index], [w,x,y,z], to the result vector
  template<typename t_forward_iterator>
  void GetBodyQuaternion(int body_index, t_forward_iterator res_itr, t_forward_iterator res_last) const
    {
      const dReal *br(body_[body_index].getQuaternion());
      *res_itr= *br; ++res_itr; ++br; if(res_itr==res_last) return;
      *res_itr= *br; ++res_itr; ++br; if(res_itr==res_last) return;
      *res_itr= *br; ++res_itr; ++br; if(res_itr==res_last) return;
      *res_itr= *br; ++res_itr; ++br;
    }
  //! Assign the rotation matrix of body[body_index], [R(0,0),R(1,0),R(2,0), R(0,1),R(1,1),R(2,1), R(0,2),R(1,2),R(2,2)], to the result vector
  template<typename t_forward_iterator>
  void GetBodyRotation(int body_index, t_forward_iterator res_itr, t_forward_iterator res_last) const
    {
      const dReal *br(body_[body_index].getRotation());
      #define _R(i,j) (br[(i)*4+(j)])
      if(res_itr==res_last) return;
      *res_itr=_R(0,0); ++res_itr; if(res_itr==res_last) return;
      *res_itr=_R(1,0); ++res_itr; if(res_itr==res_last) return;
      *res_itr=_R(2,0); ++res_itr; if(res_itr==res_last) return;
      *res_itr=_R(0,1); ++res_itr; if(res_itr==res_last) return;
      *res_itr=_R(1,1); ++res_itr; if(res_itr==res_last) return;
      *res_itr=_R(2,1); ++res_itr; if(res_itr==res_last) return;
      *res_itr=_R(0,2); ++res_itr; if(res_itr==res_last) return;
      *res_itr=_R(1,2); ++res_itr; if(res_itr==res_last) return;
      *res_itr=_R(2,2);
      #undef _R
    }

  int RobotIndex(const std::string &robot_name) const
    {
      std::map<std::string, int>::const_iterator ritr= robot_idxes_.find(robot_name);
      if(ritr==robot_idxes_.end())
        {LERROR("Robot \""<<robot_name<<"\" not found");  return -1;}
      return ritr->second;
    }

  int JointAngleNum(int robot_index) const {return angle_observer_[robot_index].size();}
  template<typename t_forward_iterator>
  void GetJointAngles(int robot_index, t_forward_iterator res_itr, t_forward_iterator res_last) const
    {
      for(std::vector<boost::function<dReal()> >::const_iterator
          itr(angle_observer_[robot_index].begin()),last(angle_observer_[robot_index].end());
          itr!=last && res_itr!=res_last; ++itr,++res_itr)
        *res_itr= (*itr)();
    }
  template<typename t_container>
  void GetJointAngles(int robot_index, t_container &val) const
    {
      GenResize(val,JointAngleNum(robot_index));
      GetJointAngles(robot_index, GenBegin(val), GenEnd(val));
    }

  int JointAngVelNum(int robot_index) const {return angvel_observer_[robot_index].size();}
  template<typename t_forward_iterator>
  void GetJointAngVels(int robot_index, t_forward_iterator res_itr, t_forward_iterator res_last) const
    {
      for(std::vector<boost::function<dReal()> >::const_iterator
          itr(angvel_observer_[robot_index].begin()),last(angvel_observer_[robot_index].end());
          itr!=last && res_itr!=res_last; ++itr,++res_itr)
        *res_itr= (*itr)();
    }
  template<typename t_container>
  void GetJointAngVels(int robot_index, t_container &val) const
    {
      GenResize(val,JointAngVelNum(robot_index));
      GetJointAngVels(robot_index, GenBegin(val), GenEnd(val));
    }


  int JointTorqueInputNum(int robot_index) const {return torque_adder_[robot_index].size();}
  template<typename t_forward_iterator>
  void AddToJointTorques(int robot_index, t_forward_iterator trq_itr, t_forward_iterator trq_last) const
    {
      for(std::vector<boost::function<void(dReal)> >::const_iterator
          itr(torque_adder_[robot_index].begin()),last(torque_adder_[robot_index].end());
          itr!=last && trq_itr!=trq_last; ++itr,++trq_itr)
        (*itr)(*trq_itr);
    }

  int ForceObservationNum(int robot_index) const {return 6*force_observer_[robot_index].size();}
  template <typename t_forward_iterator>
  void GetForces(int robot_index, t_forward_iterator res_itr, t_forward_iterator res_last) const
    {
      for(std::vector<int>::const_iterator
          itr(force_observer_[robot_index].begin()),last(force_observer_[robot_index].end());
          itr!=last && res_itr!=res_last; ++itr)
      {
        const dJointFeedback &jfeedback(joint_feedback_[*itr]);
        for(int i(0);i<3 && res_itr!=res_last;++i,++res_itr)  *res_itr= jfeedback.f1[i];
        for(int i(0);i<3 && res_itr!=res_last;++i,++res_itr)  *res_itr= jfeedback.t1[i];
      }
    }
  template<typename t_container>
  void GetForces(int robot_index, t_container &val) const
    {
      GenResize(val,ForceObservationNum(robot_index));
      GetForces(robot_index, GenBegin(val), GenEnd(val));
    }

  const std::vector<bool>& LinkContacts(int robot_index) const {return current_contact_[robot_index];}

  TReal TotalMass(const std::string &robot_name) const;

protected:
  TWorldParameters  params_;

  TCallbacks   callbacks_;

  dWorld world_;
  dSimpleSpace space_;
  dJointGroup contactgroup_;

  TReal time_;
  TReal old_fps_;
  int repaint_time_;

  struct TElementSize
  {
    int Body    ;
    int GeomBX  ;
    int GeomCA  ;
    int GeomCY  ;
    int GeomPL  ;
    int GeomSP  ;
    int GeomTM  ;
    int JointBA ;
    int JointH1 ;
    int JointSL ;
    int JointUN ;
    int JointH2 ;
    int JointFX ;
    int JointFeedback;
    TElementSize() :
        Body    (0),
        GeomBX  (0),
        GeomCA  (0),
        GeomCY  (0),
        GeomPL  (0),
        GeomSP  (0),
        GeomTM  (0),
        JointBA (0),
        JointH1 (0),
        JointSL (0),
        JointUN (0),
        JointH2 (0),
        JointFX (0),
        JointFeedback(0)
      {}
  };

  // Elements:
  std::vector<TNCBody>            body_;
  std::vector<TNCBox>             geom_bx_;
  std::vector<TNCCapsule>         geom_ca_;
  std::vector<TNCCylinder>        geom_cy_;
  std::vector<TNCPlane>           geom_pl_;
  std::vector<TNCSphere>          geom_sp_;
  std::vector<TTriMeshGeom>       geom_tm_;
  std::vector<TNCBallJoint>       joint_ba_;
  std::vector<TNCHingeJoint>      joint_h1_;
  std::vector<TNCSliderJoint>     joint_sl_;
  std::vector<TNCUniversalJoint>  joint_un_;
  std::vector<TNCHinge2Joint>     joint_h2_;
  std::vector<TNCFixedJoint>      joint_fx_;
  std::vector<dJointFeedback>     joint_feedback_;

  std::map<std::string, std::map<std::string, int> >  name_to_body_;  //!< [ROBOT-NAME][LINK-NAME]=Index of body_
  std::map<std::string, std::map<std::string, int> >  name_to_jfeedback_;  //!< [ROBOT-NAME][FORCE-SENSOR-NAME]=Index of joint_feedback_
  std::map<std::string, int>  robot_idxes_;  //!< [ROBOT-NAME]
  std::vector<TRobotParameters*>                      idx_to_robotparams_;  //!< [ROBOT-INDEX]=TRobotParameters
  std::vector<std::vector<boost::function<dReal()> > >  angle_observer_;  //!< [ROBOT-INDEX][ANGLE-INDEX]=Observer
  std::vector<std::vector<boost::function<dReal()> > >  angvel_observer_;  //!< [ROBOT-INDEX][ANGVEL-INDEX]=Observer
  std::vector<std::vector<boost::function<void(dReal)> > >  torque_adder_;  //!< [ROBOT-INDEX][TORQUE-INDEX]=Adder
  std::vector<std::vector<int> >  force_observer_;  //!< [ROBOT-INDEX][FORCE-INDEX]=Index of joint_feedback_
  std::vector<std::vector<int> >  contact_observer_;  //!< [ROBOT-INDEX][CONTACT-INDEX]=Index of body_
  std::vector<std::vector<bool> >  current_contact_;  //!< [ROBOT-INDEX][CONTACT-INDEX]=Contacting (true) or not (false)

  void resize_elements();
  dGeomID create_geometry(TGeometryParameters &geom, TElementSize &idx);
  static void set_mass_from_geometry(dMass &m, const std::string &link_name, const TLinkParameters &link, const dGeomID ode_geom);
  void create_link(const std::string &link_name, TLinkParameters &link, TElementSize &idx);
  dJointID create_joint(const std::string &joint_name, TJointParameters &joint, TElementSize &idx, const std::string &robot_name);
  void create_force_sensor(TNamedParam<TForceSensorParameters>::P::iterator iforce_sensor, TElementSize &idx, const std::string &robot_name);
  void create_robot(TNamedParam<TRobotParameters>::P::iterator irobot, TElementSize &idx);
  dGeomID create_static_link(TNamedParam<TStaticLinkParameters>::P::iterator ilink, TElementSize &idx);
  void create_static_object(TNamedParam<TStaticObjectParameters>::P::iterator isobj, TElementSize &idx);

  void construct_manipulators(TNamedParam<TRobotParameters>::P::iterator irobot, const std::map<std::string, TJointParameters*> &name_to_joint);

};
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of xode
//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_robot_model_h
//-------------------------------------------------------------------------------------------
