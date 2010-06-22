//-------------------------------------------------------------------------------------------
/*! \file    octave_fwd.h
    \brief   liblora - liboctave extension (forward header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.23, 2010-
*/
//-------------------------------------------------------------------------------------------
#ifndef loco_rabbits_octave_fwd_h
#define loco_rabbits_octave_fwd_h
//-------------------------------------------------------------------------------------------
#include <octave/config.h>
#include <octave/dColVector.h>
#include <octave/dRowVector.h>
#include <octave/dMatrix.h>
#include <octave/mx-m-dm.h>
//-------------------------------------------------------------------------------------------
#ifdef PACKAGE_BUGREPORT
#undef PACKAGE_BUGREPORT
#endif
#ifdef PACKAGE_NAME
#undef PACKAGE_NAME
#endif
#ifdef PACKAGE_STRING
#undef PACKAGE_STRING
#endif
#ifdef PACKAGE_TARNAME
#undef PACKAGE_TARNAME
#endif
#ifdef PACKAGE_VERSION
#undef PACKAGE_VERSION
#endif
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

template <typename OctArray>
inline typename OctArray::element_type*  OctBegin (OctArray &ar)
{
  return ar.fortran_vec();
}
template <typename OctArray>
inline const typename OctArray::element_type*  OctBegin (const OctArray &ar)
{
  return ar.fortran_vec();
}
template <typename OctArray>
inline typename OctArray::element_type*  OctEnd (OctArray &ar)
{
  return ar.fortran_vec()+ar.length();
}
template <typename OctArray>
inline const typename OctArray::element_type*  OctEnd (const OctArray &ar)
{
  return ar.fortran_vec()+ar.length();
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_octave_fwd_h
//-------------------------------------------------------------------------------------------
