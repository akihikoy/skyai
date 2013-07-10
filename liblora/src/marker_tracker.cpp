//-------------------------------------------------------------------------------------------
/*! \file    marker_tracker.cpp
    \brief   liblora - marker tracker program using 3rdparty/markerdetection (source)
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
#include <lora/marker_tracker.h>
#include <lora/rand.h>
#include <lora/math.h>
//-------------------------------------------------------------------------------------------
#include <iostream>
//-------------------------------------------------------------------------------------------
#include <markerdetection/edgeldetector.h>
#include <markerdetection/linesegment.h>
#include <markerdetection/buffer.h>
//-------------------------------------------------------------------------------------------
void debugDrawAll()
{
}
void debugDrawLine(int x1, int y1, int x2, int y2, int r, int g, int b, int t)
{
}
void debugDrawPoint(int x1, int y1, int r, int g, int b, int t)
{
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace marker_tracker
{

struct TTetragon
{
  Vector2f C[4];
};
//-------------------------------------------------------------------------------------------

inline Vector2f ToVector2f(const cv::Vec<double,2> &v)
{
  return Vector2f(v[0],v[1]);
}
//-------------------------------------------------------------------------------------------

// return 1:points are on a clockwise triangle, -1:counter-clockwise
// if the points are on a line, return -1 when p0 is center, 1 when p1 is center, 0 when p2 is center
int CheckClockwise(const Vector2f &p0, const Vector2f &p1, const Vector2f &p2)
{
  int dx1,dx2,dy1,dy2;
  dx1= p1.x-p0.x;
  dy1= p1.y-p0.y;
  dx2= p2.x-p0.x;
  dy2= p2.y-p0.y;

  if(dx1*dy2 > dy1*dx2 ) return 1;
  if(dx1*dy2 < dy1*dx2 ) return -1;
  if((dx1*dx2 <0) || (dy1*dy2 <0)) return -1;
  if((dx1*dx1 + dy1*dy1 < dx2*dx2 + dy2*dy2)) return 1;
  return 0;
}
//-------------------------------------------------------------------------------------------

void DrawLine(cv::Mat &image, int x1, int y1, int x2, int y2, int r, int g, int b, int t)
{
  if(std::abs(x1)+std::abs(y1) > 10000 || std::abs(x2)+std::abs(y2) > 10000)
    return;
  cv::line(image, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(r,g,b), t);
}
//-------------------------------------------------------------------------------------------

void DrawArrow(cv::Mat &image, int x1, int y1, int x2, int y2, float xn, float yn, int r, int g, int b, int t)
{
  DrawLine(image, x1, y1, x2, y2, r, g, b, t);
  DrawLine(image, x2, y2, x2 + (5.0f* ( -xn + yn )), y2 + (5.0f* ( -yn - xn )), r, g, b, t);
  DrawLine(image, x2, y2, x2 + (5.0f* ( -xn - yn )), y2 + (5.0f* ( -yn + xn )), r, g, b, t);
}
//-------------------------------------------------------------------------------------------

cv::Mat LoadTemplate(const char *filename)
{
  cv::Mat img= cv::imread(filename,0);
  if(img.cols*img.rows==0)
  {
    LERROR("failed to load image: "<<filename);
    return cv::Mat();
  }
  int tsize((img.cols<=img.rows) ? img.cols : img.rows);
  cv::resize (img, img, cv::Size(tsize,tsize), 0,0, CV_INTER_LINEAR);
  cv::threshold(img,img,0,1, cv::THRESH_BINARY|cv::THRESH_OTSU);
  return img;
}
//-------------------------------------------------------------------------------------------

void ARMarkerToClockwiseTetragon(const ARMarker &marker, TTetragon &t)
{
  if(CheckClockwise(marker.c1,marker.c2,marker.c3)==1)
  {
    t.C[0]= marker.c1;
    t.C[1]= marker.c2;
    t.C[2]= marker.c3;
    t.C[3]= marker.c4;
  }
  else
  {
    t.C[0]= marker.c1;
    t.C[1]= marker.c4;
    t.C[2]= marker.c3;
    t.C[3]= marker.c2;
  }
}
//-------------------------------------------------------------------------------------------

double CalcSimilarity(const TTetragon &marker, const cv::Mat &image, const cv::Mat &tmpl, int *direction=NULL)
{
  if(CheckClockwise(marker.C[0],marker.C[1],marker.C[2])!=1 || CheckClockwise(marker.C[0],marker.C[2],marker.C[3])!=1)
    return 0.0;
  cv::Point2f src[4];
  for(int i(0);i<4;++i)
  {
    if(std::isinf(marker.C[i].x) || std::isinf(marker.C[i].y))
      return 0.0;
    src[i]= cv::Point2f(marker.C[i].x,marker.C[i].y);
  }
  cv::Point2f dst[4];
  dst[0]= cv::Point2f(0,0);
  dst[1]= cv::Point2f(tmpl.cols,0);
  dst[2]= cv::Point2f(tmpl.cols,tmpl.rows);
  dst[3]= cv::Point2f(0,tmpl.rows);

  cv::Mat trans= cv::getPerspectiveTransform(src, dst);
  cv::Mat detected;
  cv::warpPerspective(image, detected, trans, cv::Size(tmpl.cols,tmpl.rows));

  cv::Mat tmp;
  cv::cvtColor(detected,tmp,CV_BGR2GRAY);
  detected= tmp;
  cv::threshold(detected,tmp,0,1, cv::THRESH_BINARY|cv::THRESH_OTSU);
  detected= tmp;

  cv::Mat matching;
  double similarity(0.0), s;
  for(int i(0);i<4;++i)
  {
    bitwise_xor(detected,tmpl,matching);
    s= 1-static_cast<double>(sum(matching)[0])/static_cast<double>(matching.cols*matching.rows);
    if(s>similarity)
    {
      similarity= s;
      if(direction)  *direction= i;
    }
    if(i<3)
      RotCounterClockwise(detected);
  }
  return similarity;
}
//-------------------------------------------------------------------------------------------


std::ostream& operator<<(std::ostream &lhs, const TParticle &rhs)
{
  lhs<<"C:"<<cv::Mat(rhs.C)<<" R V:"<<cv::Mat(rhs.V)<<" W:"<<cv::Mat(rhs.W)<<" L1:"<<rhs.L1<<" L2:"<<rhs.L2<<" F:"<<rhs.F;
  return lhs;
}
//-------------------------------------------------------------------------------------------

std::ostream& PrintParticle(std::ostream &lhs, const TParticle &rhs)
{
  int t(1);
  for(int i(0);i<3;++i,++t)  lhs<<" "<<rhs.C[i];
  lhs<<" #"<<t; ++t;
  for(int i(0);i<9;++i,++t)  lhs<<" "<<rhs.R.val[i];
  lhs<<" #"<<t; ++t;
  for(int i(0);i<3;++i,++t)  lhs<<" "<<rhs.V[i];
  lhs<<" #"<<t; ++t;
  for(int i(0);i<3;++i,++t)  lhs<<" "<<rhs.W[i];
  lhs<<" #"<<t; ++t;
  lhs<<" "<<rhs.L1;
  lhs<<" "<<rhs.L2;
  lhs<<" "<<rhs.F;
  return lhs;
}
//-------------------------------------------------------------------------------------------

std::ostream& PrintParticle(std::ostream &lhs, const TParticleW &rhs)
{
  PrintParticle(lhs,rhs.P) << " #" << rhs.W;
}
//-------------------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream &lhs, const TParticleW &rhs)
{
  lhs<<rhs.P<<"; W:"<<rhs.W;
  return lhs;
}
//-------------------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream &lhs, const TObservation &rhs)
{
  for(int i(0);i<4;++i)  lhs<<" "<<cv::Mat(rhs.P[i]);
  return lhs;
}
//-------------------------------------------------------------------------------------------

void PrintParticles(std::ostream &os, const std::vector<TParticleW> &particles)
{
  for(std::vector<TParticleW>::const_iterator itr(particles.begin()),last(particles.end());itr!=last;++itr)
    os<<*itr<<std::endl;
}
//-------------------------------------------------------------------------------------------


int BestParticleIdx(const std::vector<TParticleW> &particles)
{
  int i(0),id(-1);
  double maxw(-1.0);
  for(std::vector<TParticleW>::const_iterator itr(particles.begin()),last(particles.end());itr!=last;++itr,++i)
    if(itr->W > maxw)
    {
      maxw= itr->W;
      id= i;
    }
  return id;
}
//-------------------------------------------------------------------------------------------

// particles[id1].W(best) > particles[id2].W > others
void BestParticleIdx2(const std::vector<TParticleW> &particles, int &id1, int &id2)
{
  int i(0);
  id1=-1; id2=-1;
  double maxw1(-1.0),maxw2(-1.0);
  for(std::vector<TParticleW>::const_iterator itr(particles.begin()),last(particles.end());itr!=last;++itr,++i)
    if(itr->W>maxw2)
    {
      if(itr->W>maxw1)
      {
        maxw2= maxw1; id2= id1;
        maxw1= itr->W; id1= i;
      }
      else
      {
        maxw2= itr->W; id2= i;
      }
    }
}
//-------------------------------------------------------------------------------------------

TParticle AverageParticles(const std::vector<TParticleW> &particles)
{
  TParticle p;
  cv::Matx<double,3,3> R(cv::Matx<double,3,3>::eye()), eye(R);
  double weight(1.0);
  for(std::vector<TParticleW>::const_iterator itr(particles.begin()),last(particles.end());itr!=last;++itr)
  {
    p= p + itr->P * itr->W;
    R= AverageRotations(R, itr->P.R, weight);
    weight-= itr->W;
  }
  p.R= R;
  return p;
}
//-------------------------------------------------------------------------------------------

TParticle EstimateFromParticles(const std::vector<TParticleW> &particles)
{
  TParticle p;
  for(std::vector<TParticleW>::const_iterator itr(particles.begin()),last(particles.end());itr!=last;++itr)
    p= p + itr->P * itr->W;
  int id1,id2;
  BestParticleIdx2(particles,id1,id2);
  p.R= AverageRotations(particles[id1].P.R, particles[id2].P.R, 0.5);
  return p;
}
//-------------------------------------------------------------------------------------------

const TParticle& BestParticle(const std::vector<TParticleW> &particles)
{
  return particles[BestParticleIdx(particles)].P;
}
//-------------------------------------------------------------------------------------------

double SumWeights(const std::vector<TParticleW> &particles)
{
  double sumw(0.0);
  for(std::vector<TParticleW>::const_iterator itr(particles.begin()),last(particles.end());itr!=last;++itr)
    sumw+= itr->W;
  return sumw;
}
//-------------------------------------------------------------------------------------------

void NormalizeWeights(std::vector<TParticleW> &particles)
{
  double sumw(SumWeights(particles));
  for(std::vector<TParticleW>::iterator itr(particles.begin()),last(particles.end());itr!=last;++itr)
    itr->W/= sumw;
}
//-------------------------------------------------------------------------------------------

void ComputePrevSumW(std::vector<TParticleW> &particles)
{
  double psumw(0.0);
  for(std::vector<TParticleW>::iterator itr(particles.begin()),last(particles.end());itr!=last;++itr)
  {
    psumw+= itr->W;
    itr->prev_sum_w_= psumw;
  }
}
//-------------------------------------------------------------------------------------------

int SelectFromPrevSumW (const std::vector<TParticleW> &particles)
{
  TReal p (Rand(1.0l));
  int imin(0), imax(particles.size()-1), index((imin+imax)/2);

  while(true)
  {
    if(p > particles[index].prev_sum_w_)
    {
      imin= index;
      if(index >= imax)  return imax;  // index==imax
      else if(index == imax-1)  ++index;
      else  index= (index+imax)/2;
    }
    else if(index<=imin)  // index==imin
    {
      return index;
    }
    else if(p > particles[index-1].prev_sum_w_)
    {
      return index;
    }
    else  // p <= particles[index-1].prev_sum_w_
    {
      imax= index;
      index= (imin+index)/2;
    }
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// class TMarkerTracker
//-------------------------------------------------------------------------------------------

bool TMarkerTracker::detect_marker(const cv::Mat &image, TObservation &o)
{
  buffer_->setBuffer((unsigned char *)image.data, image.cols, image.rows);

  // detecting marker candidates
  edgel_detector_->setBuffer(buffer_);
  std::vector<ARMarker> markers = edgel_detector_->findMarkers();

  // detecting marker
  double s, maxs(0.0);
  int d;
  TTetragon t;
  for(std::vector<ARMarker>::const_iterator itr(markers.begin()),last(markers.end());itr!=last;++itr)
  {
    ARMarkerToClockwiseTetragon(*itr,t);
    s= CalcSimilarity(t, image, template_image_, &d);
    if(s > conf_.MarkerDetectionThreshold && s>maxs)
    {
      for(int i(0);i<4;++i)
      {
        o.P[i][0]= t.C[(i+d)%4].x;
        o.P[i][1]= t.C[(i+d)%4].y;
      }
      maxs= s;
    }
  }
  return maxs > conf_.MarkerDetectionThreshold;
}
//-------------------------------------------------------------------------------------------

void TMarkerTracker::generate_particle(TParticle &p)
{
  p.C= cv::Vec<double,3>(Rand(-conf_.InitCX,conf_.InitCX), Rand(-conf_.InitCY,conf_.InitCY), Rand(conf_.InitCZ1,conf_.InitCZ2));
  cv::Vec<double,3> axis(Rand(-1.0,1.0), Rand(-1.0,1.0), Rand(-1.0,1.0));
  cv::normalize(axis,axis);
  p.R= Rodrigues(axis*static_cast<double>(Rand(-M_PI/2.0,M_PI/2.0)));
  p.V= cv::Vec<double,3>(Rand(-conf_.InitV,conf_.InitV), Rand(-conf_.InitV,conf_.InitV), Rand(-conf_.InitV,conf_.InitV));
  p.W= cv::Vec<double,3>(Rand(-conf_.InitW,conf_.InitW), Rand(-conf_.InitW,conf_.InitW), Rand(-conf_.InitW,conf_.InitW));
  p.L1= Rand(conf_.InitL11,conf_.InitL12);
  p.L2= Rand(conf_.InitL21,conf_.InitL22);
  p.F= Rand(conf_.InitF1,conf_.InitF2);
}
//-------------------------------------------------------------------------------------------

void TMarkerTracker::generate_particles(std::vector<TParticleW> &particles, int N)
{
  particles.resize(N);
  double w(1.0/static_cast<double>(N));
  for(std::vector<TParticleW>::iterator itr(particles.begin()),last(particles.end());itr!=last;++itr)
  {
    generate_particle(itr->P);
    itr->W= w;
  }
}
//-------------------------------------------------------------------------------------------

void TMarkerTracker::transition_model(const TParticle &curr, TParticle &next, bool add_noise)
{
#define NV3  cv::Vec<double,3>(NDRand(), NDRand(), NDRand())
  next= curr;

  double sw(add_noise?1.0:0.0);
  next.C= curr.C + curr.V*conf_.Dt + sw*conf_.NoiseC*NV3;

  next.R= cv::Mat(Rodrigues(curr.W*conf_.Dt + sw*conf_.NoiseR*NV3)*curr.R);

  next.V= curr.V + sw*conf_.NoiseV*NV3;
  next.W= curr.W + sw*conf_.NoiseW*NV3;
  next.L1= curr.L1 + sw*conf_.NoiseL1*NDRand();
  next.L2= curr.L2 + sw*conf_.NoiseL2*NDRand();
  next.F= curr.F + sw*conf_.NoiseF*NDRand();

  if(next.L1<0.0)  next.L1= conf_.Epsilon;
  if(next.L2<0.0)  next.L2= conf_.Epsilon;
  if(next.F<0.0)   next.F=  conf_.Epsilon;

  if(next.C[2]<next.F)  next.C[2]= next.F+conf_.Epsilon;

  if(conf_.L1EqL2)  next.L2= next.L1;
#undef NV3
}
//-------------------------------------------------------------------------------------------

void TMarkerTracker::estimate_observation(const TParticle &p, TObservation &o)
{
  cv::Vec<double,3> lp[4], wp;
  lp[0]= cv::Vec<double,3>(-p.L1/2.0, -p.L2/2.0, 0.0);
  lp[1]= cv::Vec<double,3>(+p.L1/2.0, -p.L2/2.0, 0.0);
  lp[2]= cv::Vec<double,3>(+p.L1/2.0, +p.L2/2.0, 0.0);
  lp[3]= cv::Vec<double,3>(-p.L1/2.0, +p.L2/2.0, 0.0);
  for(int i(0);i<4;++i)
  {
    wp= cv::Mat(p.C + p.R * lp[i]);
    if(wp[2]<conf_.Epsilon)
    {
      o.P[i][0]= 0.0;
      o.P[i][1]= 0.0;
    }
    else
    {
      o.P[i][0]= conf_.ScaleX * wp[0] * p.F / wp[2] + 0.5*conf_.Width;
      o.P[i][1]= conf_.ScaleY * wp[1] * p.F / wp[2] + 0.5*conf_.Height;
    }
  }
}
//-------------------------------------------------------------------------------------------

void TMarkerTracker::estimate_observation(const TParticle &p, TExtraObservation &o)
{
  estimate_observation(p, reinterpret_cast<TObservation&>(o));
  if(p.C[2]<conf_.Epsilon)
  {
    o.C[0]= 0.0;
    o.C[1]= 0.0;
    o.V[0]= 0.0;
    o.V[1]= 0.0;
  }
  else
  {
    o.C[0]= conf_.ScaleX * p.C[0] * p.F / p.C[2] + 0.5*conf_.Width;
    o.C[1]= conf_.ScaleY * p.C[1] * p.F / p.C[2] + 0.5*conf_.Height;
    o.V[0]= conf_.ScaleX * p.V[0] * p.F / p.C[2];
    o.V[1]= conf_.ScaleY * p.V[1] * p.F / p.C[2];
  }

}
//-------------------------------------------------------------------------------------------

double TMarkerTracker::compute_weight(const TParticle &p, const TObservation &o)
{
  TObservation est;
  estimate_observation(p,est);
  double w(conf_.Epsilon);

  if(CheckClockwise(ToVector2f(est.P[0]),ToVector2f(est.P[1]),ToVector2f(est.P[2]))==1)
  {
    // for(int i(0);i<4;++i)  w+= Square((o.P[i][0]-est.P[i][0])/conf_.ScaleX)+Square((o.P[i][1]-est.P[i][1])/conf_.ScaleY);
    // w= real_exp(-0.5*conf_.WeightSigma*(w))+conf_.Epsilon;

    // bool correct(true);
    // for(int i(0);i<4 && correct;++i)
    // {
    //   double d= Square((o.P[i][0]-est.P[i][0])/conf_.ScaleX)+Square((o.P[i][1]-est.P[i][1])/conf_.ScaleY);
    //   double d2;
    //   for(int j(0);j<4 && correct;++j)
    //     if(i!=j)
    //     {
    //       d2= Square((o.P[j][0]-est.P[i][0])/conf_.ScaleX)+Square((o.P[j][1]-est.P[i][1])/conf_.ScaleY);
    //       if(d>d2) correct= false;
    //     }
    //   w+= d;
    // }
    // if(correct)  w= real_exp(-0.5*conf_.WeightSigma*(w))+conf_.Epsilon;
    // else         w= conf_.Epsilon;

    for(int i(0);i<4;++i)
    {
      double d= Square((o.P[i][0]-est.P[i][0])/conf_.ScaleX)+Square((o.P[i][1]-est.P[i][1])/conf_.ScaleY);
      double d2;
      bool correct(true);
      for(int j(0);j<4 && correct;++j)
        if(i!=j)
        {
          d2= Square((o.P[j][0]-est.P[i][0])/conf_.ScaleX)+Square((o.P[j][1]-est.P[i][1])/conf_.ScaleY);
          if(d>d2) correct= false;
        }
      if(correct)  w+= d;
      else         w+= 20.0*d;
    }
    w= real_exp(-0.5*conf_.WeightSigma*(w))+conf_.Epsilon;

  }
  return w;
}
//-------------------------------------------------------------------------------------------

void TMarkerTracker::update_particles(std::vector<TParticleW> &particles, const TObservation &o)
{
  std::vector<TParticleW> pnew(particles.size());
  std::vector<TParticleW>::iterator nitr(pnew.begin());
  for(std::vector<TParticleW>::iterator pitr(particles.begin()),plast(particles.end());pitr!=plast;++pitr,++nitr)
  {
    transition_model(pitr->P,nitr->P);
    nitr->W= compute_weight(nitr->P,o);
  }
  NormalizeWeights(pnew);
  ComputePrevSumW(pnew);
  for(std::vector<TParticleW>::iterator pitr(particles.begin()),plast(particles.end());pitr!=plast;++pitr)
  {
    int i= SelectFromPrevSumW(pnew);
    *pitr= pnew[i];
  }
  NormalizeWeights(particles);
}
//-------------------------------------------------------------------------------------------

void TMarkerTracker::update_particles(std::vector<TParticleW> &particles)
{
  TParticle next;
  for(std::vector<TParticleW>::iterator pitr(particles.begin()),plast(particles.end());pitr!=plast;++pitr)
  {
    transition_model(pitr->P,next,false);
    pitr->P= next;
  }
}
//-------------------------------------------------------------------------------------------

bool TMarkerTracker::Initialize()
{
  Clear();

  generate_particles(particles_, conf_.NumOfParticles);

  camera_.open(conf_.CameraDeviceID);
  if(!camera_.isOpened())
  {
    LERROR("no camera!");
    return false;
  }

  template_image_= LoadTemplate(conf_.MarkerFileName.c_str());

  // marker detection setup
  buffer_ = new Buffer();
  edgel_detector_ = new EdgelDetector();

  edgel_detector_->debugDrawLineSegments( false );
  edgel_detector_->debugDrawPartialMergedLineSegments( false );
  edgel_detector_->debugDrawMergedLineSegments( false );
  edgel_detector_->debugDrawExtendedLineSegments( false );
  edgel_detector_->debugDrawSectors( false );
  edgel_detector_->debugDrawSectorGrids( false );
  edgel_detector_->debugDrawEdges( false );
  edgel_detector_->debugDrawCorners( false );
  edgel_detector_->debugDrawMarkers( false );

  cv::Mat current_frame;
  camera_ >> current_frame;
  if(current_frame.cols*current_frame.rows!=0)
  {
    image_width_= current_frame.cols;
    image_height_= current_frame.rows;
    draw_image_.create(cv::Size(current_frame.cols, current_frame.rows), CV_8UC3);
  }
  else
  {
    image_width_= 0;
    image_height_= 0;
    draw_image_.create(cv::Size(10, 10), CV_8UC3);
  }

  if(conf_.DisplayResult)
  {
    cv::namedWindow(conf_.WindowName,1);
    cv_key_ = cv::waitKey (conf_.WaitKeyDelay);
  }

  return true;
}
//-------------------------------------------------------------------------------------------

bool TMarkerTracker::Step()
{
  cv::Mat current_frame;
  if(camera_.isOpened())
    camera_ >> current_frame;

  bool captured(true);
  if(current_frame.cols*current_frame.rows==0)
  {
    if(camera_.isOpened())
      LERROR("camera capture failed");
    captured= false;
  }

  if(captured)
  {
    image_width_= current_frame.cols;
    image_height_= current_frame.rows;
    cv::resize (current_frame, draw_image_, cv::Size(), 1.0,1.0, CV_INTER_LINEAR);
  }

  //cv::Smooth( draw_image_, draw_image_, CV_GAUSSIAN, 3, 3 );

  // detecting marker
  observed_= (captured ? detect_marker(draw_image_,observation_) : false);

  if(observed_ && conf_.PrintResult)
    std::cout<<"observed: "<<observation_<<std::endl;

  // update particles:
  if(observed_)
    update_particles(particles_, observation_);
  else
    update_particles(particles_);

  // estimate:
  // TParticle est_state_= AverageParticles(particles_);
  est_state_= EstimateFromParticles(particles_);
  // TParticle est_state_= BestParticle(particles_);
  if(conf_.PrintResult) std::cout<<"pest: "<<est_state_<<std::endl;
  estimate_observation(est_state_, est_observation_);
  if(conf_.PrintResult) std::cout<<"est: ("<<est_observation_.P[0][0]<<","<<est_observation_.P[0][1]<<")"<<std::endl;

  // best particle:
  TObservation best;
  TParticle pbest= BestParticle(particles_);
  if(conf_.PrintResult) std::cout<<"pbest: "<<pbest<<std::endl;
  estimate_observation(pbest, best);
  if(conf_.PrintResult) std::cout<<"best: ("<<best.P[0][0]<<","<<best.P[0][1]<<")"<<std::endl;


  if(conf_.DisplayResult && captured)
  {
    int skipper(0), nskip(particles_.size()/conf_.NumOfDisplayLines);
    for(std::vector<TParticleW>::const_iterator itr(particles_.begin()),last(particles_.end());itr!=last;++itr)
    {
      if(skipper!=0)  {--skipper; continue;}
      skipper= nskip;

      TObservation e;
      estimate_observation(itr->P, e);
      for(int i(0);i<4;++i)
        DrawLine(draw_image_, e.P[i][0], e.P[i][1], e.P[(i+1)%4][0], e.P[(i+1)%4][1], 0, (0==i?200:255), 255, 0.5);
    }

    if(observed_)
      for(int i(0);i<4;++i)
        DrawLine(draw_image_, observation_.P[i][0], observation_.P[i][1], observation_.P[(i+1)%4][0], observation_.P[(i+1)%4][1], (0==i?255:0), 255, 0, ((0==i||1==i)?10:2));

    for(int i(0);i<4;++i)
      DrawLine(draw_image_, est_observation_.P[i][0], est_observation_.P[i][1], est_observation_.P[(i+1)%4][0], est_observation_.P[(i+1)%4][1], 0, (0==i?200:0), 255, ((0==i||1==i)?10:2));
    DrawArrow(draw_image_, est_observation_.C[0], est_observation_.C[1], est_observation_.C[0]+est_observation_.V[0], est_observation_.C[1]+est_observation_.V[1], 0.1*est_observation_.V[0], 0.1*est_observation_.V[1], 0, 0, 255, 2);

    for(int i(0);i<4;++i)
      DrawLine(draw_image_, best.P[i][0], best.P[i][1], best.P[(i+1)%4][0], best.P[(i+1)%4][1], 255, 0, (0==i?200:0), ((0==i||1==i)?10:2));

    cv::imshow(conf_.WindowName, draw_image_);
    cv_key_ = cv::waitKey (conf_.WaitKeyDelay);
  }

  // static ofstream ofs_pest("res/pest.dat");
  // PrintParticle(ofs_pest,est_state_)<<endl;

  // ofstream ofs_part("res/particles.dat");
  // PrintParticles(ofs_part, particles_);

// cerr<<"P[0]:"<<particles_[0].P<<endl;
// TObservation e0;estimate_observation(particles_[0].P, e0);
// cerr<<"e0: ("<<e0.P[0][0]<<","<<e0.P[0][1]<<")"<<endl;


  return true;
}
//-------------------------------------------------------------------------------------------


void TMarkerTracker::Clear()
{
  particles_.clear();

  if(buffer_)  delete buffer_;
  buffer_= NULL;
  if(edgel_detector_)  delete edgel_detector_;
  edgel_detector_= NULL;

  cv::destroyWindow(conf_.WindowName);
  camera_.release();
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of marker_tracker
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
