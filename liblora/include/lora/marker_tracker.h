//-------------------------------------------------------------------------------------------
/*! \file    marker_tracker.h
    \brief   liblora - marker tracker program using 3rdparty/markerdetection (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.28, 2012

    Copyright (C) 2012  Akihiko Yamaguchi

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
#ifndef loco_rabbits_marker_tracker_h
#define loco_rabbits_marker_tracker_h
//-------------------------------------------------------------------------------------------
#include <lora/cv.h>
#include <highgui.h>
//-------------------------------------------------------------------------------------------
class Buffer;
class EdgelDetector;
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace marker_tracker
{
//-------------------------------------------------------------------------------------------

struct TMarkerTrackerConfig
{
  std::string MarkerFileName;
  double MarkerDetectionThreshold;

  int   NumOfParticles;

  double Dt;
  double InitV;
  double InitW;
  double InitF1;
  double InitF2;

  double NoiseC;
  double NoiseR;
  double NoiseV;
  double NoiseW;

  double NoiseL1;
  double NoiseL2;
  double NoiseF;

  bool L1EqL2;

  double ScaleX;
  double ScaleY;
  double Width;
  double Height;
  double WeightSigma;
  double Epsilon;

  int   NumOfDisplayLines;
  bool  DisplayResult;
  std::string WindowName;

  TMarkerTrackerConfig() :
      MarkerFileName("marker.png"),
      MarkerDetectionThreshold(0.8),
      NumOfParticles(10000),
      Dt(0.01),
      InitV(0.01),
      InitW(0.01),
      // InitF1(0.01),
      // InitF2(1.0),
      InitF1(0.8),
      InitF2(InitF1),

      #define NOISE_CMN 0.2
      NoiseC(NOISE_CMN),
      NoiseR(0.5*NOISE_CMN),
      NoiseV(2.0*NOISE_CMN),
      NoiseW(1.0*NOISE_CMN),

      // NOISE_CMN(0.2),
      // NoiseC(0.2*NOISE_CMN),
      // NoiseR(0.1*NOISE_CMN),
      // NoiseV(NOISE_CMN),
      // NoiseW(0.5*NOISE_CMN),

      NoiseL1(0.001*NOISE_CMN),
      NoiseL2(0.001*NOISE_CMN),
      // NoiseF(0.01*NOISE_CMN),

      NoiseF(0.0*NOISE_CMN),
      #undef NOISE_CMN

      L1EqL2(true),

      ScaleX(500.0),
      ScaleY(500.0),
      Width(640.0),
      Height(480.0),
      WeightSigma(10.0),
      Epsilon(1.0e-6),

      NumOfDisplayLines(100),
      DisplayResult(true),
      WindowName("Marker Tracker")
    {
    }
};
//-------------------------------------------------------------------------------------------

struct TParticle
{
  cv::Vec<double,3> c;
  cv::Vec<double,9> r_;
  cv::Mat_<double>  R;  // wrapper of r_
  cv::Vec<double,3> v, w;
  double l1,l2,f;

  TParticle() :
      c(0.,0.,0.),
      r_(0.,0.,0., 0.,0.,0., 0.,0.,0.),
      R(3,3,r_.val),
      v(0.,0.,0.),
      w(0.,0.,0.),
      l1(0.),
      l2(0.),
      f(0.)
    {}

  TParticle(const TParticle &p) :
      c(p.c),
      r_(p.r_),
      R(3,3,r_.val),
      v(p.v),
      w(p.w),
      l1(p.l1),
      l2(p.l2),
      f(p.f)
    {}

  const TParticle& operator=(const TParticle &rhs)
    {
#define XEQ(x_var) x_var= rhs.x_var;
      XEQ(c)
      XEQ(r_)
      XEQ(v)
      XEQ(w)
      XEQ(l1)
      XEQ(l2)
      XEQ(f)
#undef XEQ
      return *this;
    }
};
//-------------------------------------------------------------------------------------------

struct TParticleW
{
  TParticle P;
  double W;
  double prev_sum_w_;  // temporary variable
  TParticleW() : P(), W(0.0), prev_sum_w_(0.0) {}
};
//-------------------------------------------------------------------------------------------

struct TObservation
{
  cv::Vec<double,2> p[4];
};
//-------------------------------------------------------------------------------------------

struct TExtraObservation : TObservation
{
  cv::Vec<double,2> c;  // center on image
  cv::Vec<double,2> v;  // velocity on image
};
//-------------------------------------------------------------------------------------------

class TMarkerTracker
{
public:

  TMarkerTracker() : buffer_(NULL), edgel_detector_(NULL)  {}
  ~TMarkerTracker()  {Clear();}

  bool Initialize();
  bool Step();
  void Clear();

  const TParticle& EstimatedState() const {return est_state_;}
  const TExtraObservation& EstimatedObservation() const {return est_observation_;}

private:

  TMarkerTrackerConfig conf_;

  std::vector<TParticleW>  particles_;

  TParticle          est_state_;
  TExtraObservation  est_observation_;

  cv::VideoCapture  camera_;
  cv::Mat template_image_;
  cv::Mat draw_image_;

  Buffer *buffer_;
  EdgelDetector *edgel_detector_;

  bool detect_marker(const cv::Mat &image, TObservation &o);

  void generate_particle(TParticle &p);
  void generate_particles(std::vector<TParticleW> &particles, int N);
  void transition_model(const TParticle &curr, TParticle &next, bool add_noise=true);
  void estimate_observation(const TParticle &p, TObservation &o);
  void estimate_observation(const TParticle &p, TExtraObservation &o);
  double compute_weight(const TParticle &p, const TObservation &o);
  void update_particles(std::vector<TParticleW> &particles, const TObservation &o);
  void update_particles(std::vector<TParticleW> &particles);

};
//-------------------------------------------------------------------------------------------

inline TParticle operator+(const TParticle &lhs, const TParticle &rhs)
{
  TParticle res(lhs);
#define XEQ(x_var) res.x_var+= rhs.x_var;
  XEQ(c)
  XEQ(r_)
  XEQ(v)
  XEQ(w)
  XEQ(l1)
  XEQ(l2)
  XEQ(f)
#undef XEQ
  return res;
}
//-------------------------------------------------------------------------------------------

inline TParticle operator*(const TParticle &lhs, const double &rhs)
{
  TParticle res(lhs);
#define XEQ(x_var) res.x_var*= rhs;
  XEQ(c)
  XEQ(r_)
  XEQ(v)
  XEQ(w)
  XEQ(l1)
  XEQ(l2)
  XEQ(f)
#undef XEQ
  return res;
}
//-------------------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream &lhs, const TParticle &rhs);
std::ostream& operator<<(std::ostream &lhs, const TParticleW &rhs);
std::ostream& operator<<(std::ostream &lhs, const TObservation &rhs);
std::ostream& PrintParticle(std::ostream &lhs, const TParticle &rhs);
std::ostream& PrintParticle(std::ostream &lhs, const TParticleW &rhs);
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of marker_tracker
} // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_marker_tracker_h
//-------------------------------------------------------------------------------------------
