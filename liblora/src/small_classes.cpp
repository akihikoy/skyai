//-------------------------------------------------------------------------------------------
/*! \file    small_classes.cpp
    \brief   liblora - class utility  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008
    \date    2009
    \date    2010

    Copyright (C) 2008, 2009, 2010  Akihiko Yamaguchi

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
#include <lora/small_classes.h>
#include <lora/type_gen.h>
#include <lora/stl_math.h>
#include <lora/rand.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TOptionParser
//===========================================================================================

/*private*/void TOptionParser::parse_from_arglist (int argc, const char *const*const argv)
{
  for (int i(0); i<argc; ++i)
  {
    if (std::strcmp(argv[i],"-help")==0)        {opt[argv[i]+1]= t_cell("help"); opt[argv[i]+1].accessed= true;}
    else if (std::strcmp(argv[i],"--help")==0)  {opt[argv[i]+2]= t_cell("help"); opt[argv[i]+2].accessed= true;}
    else if (argv[i][0]=='-')
    {
      if (i==argc-1)  {LERROR("option is not complete: "<<argv[i]); lexit(qfail);}
      if (opt.find(argv[i]+1)!=opt.end())  {LWARNING("option "<<(argv[i]+1)<<" is redefined.");}
      opt[argv[i]+1] = t_cell(argv[i+1]); ++i;
    }
    else
      Floating.push_back (argv[i]);
  }
};
//-------------------------------------------------------------------------------------------

TOptionParser::TOptionParser (int argc, const char *const*const argv)
{
  stringstream ss;
  SaveArguments (argc, argv, ss);
  cmd_line= ss.str();
  parse_from_arglist(argc-1,argv+1);
}
//-------------------------------------------------------------------------------------------

/*!\brief parse from a string
  \note option are separated by white spaces;
        strings like "x x" or 'y y' are recognized as a single word
  \note backslash \\ escapes a following letter */
TOptionParser::TOptionParser (const std::string &argline)
{
  cmd_line= argline;

  list<string> arglist;
  stringstream blockss;
  bool in_str_s(false), in_str_d(false), escaped(false);
  for (string::const_iterator sitr (argline.begin()); sitr!=argline.end(); ++sitr)
  {
    if (escaped)
    {
      blockss<< *sitr;
      escaped= false;
    }
    else if (*sitr == '\\')
      escaped= true;
    else if (*sitr == '\'')
    {
      if (in_str_d)
        blockss<< *sitr;
      else
        in_str_s= !in_str_s;
    }
    else if (*sitr == '\"')
    {
      if (in_str_s)
        blockss<< *sitr;
      else
        in_str_d= !in_str_d;
    }
    else if (in_str_s || in_str_d)
      blockss<< *sitr;
    else if (*sitr != ' ' && *sitr != '\t')
      blockss<< *sitr;
    else if (blockss.str()!="")
    {
      arglist.push_back (blockss.str());
      blockss.clear();  blockss.str("");
    }
  }
  //LDEBUG("blockss.str()= "<<blockss.str());
  if (escaped)
    {LERROR("option is terminated with backslash: "<<argline); lexit(qfail);}
  if (in_str_s)
    {LERROR("unmatched \': "<<argline); lexit(qfail);}
  if (in_str_d)
    {LERROR("unmatched \": "<<argline); lexit(qfail);}
  if (blockss.str()!="")
    arglist.push_back (blockss.str());

  int argc= arglist.size();
  const char **argv= new const char*[argc];
  const char **ptr (argv);
  for (list<string>::iterator itr(arglist.begin()); itr!=arglist.end(); ++itr,++ptr)
    *ptr= itr->c_str();
  parse_from_arglist (argc, argv);
  delete[] argv; argv=NULL;
}
//-------------------------------------------------------------------------------------------

//! \brief print unaccessed options, \return if there is no unaccessed options, return false, else true
bool TOptionParser::PrintNotAccessed (std::ostream &os, const std::string &prefix, bool including_floating)
{
  bool res(false);
  for (std::map<std::string,t_cell>::iterator itr(opt.begin()); itr!=opt.end(); ++itr)
    if (!itr->second.accessed)
      { res=true; os << prefix << "-" << itr->first << "  " << itr->second.val << std::endl; }
  if (including_floating)
  {
    if (!Floating.empty())
    {
      res = true;
      os << prefix;
      for (std::list<std::string>::iterator itr(Floating.begin()); itr!=Floating.end(); ++itr)
        os << *itr << "  ";
      os << std::endl;
    }
  }
  return res;
};
//-------------------------------------------------------------------------------------------

void TOptionParser::PrintUsed (std::ostream &os, const std::string &prefix)
{
  for (std::list <std::string>::iterator itr(used_opt.begin()); itr!=used_opt.end(); ++itr)
    { os << prefix << "-" << *itr << std::endl; }
};
//-------------------------------------------------------------------------------------------



//===========================================================================================
// class TNBase
//===========================================================================================

void TNBase::assign (ValueType val)
{
  std::fill(nbase_.begin(),nbase_.end(),0);
  if (val<0)  {decimal_=0; err_=true; return;}
  if (val<=decimal_max_)  {decimal_= val;}
  else  {decimal_= decimal_max_; err_=true;}
  for (NBaseType::iterator  idigit (nbase_.begin()); idigit!=nbase_.end(); ++idigit)
  {
    if (val<N_)
    {
      *idigit= val;
      return;
    }
    *idigit= val % N_;
    val= (val-*idigit)/N_;
  }
};
//-------------------------------------------------------------------------------------------

std::ostream& operator<< (std::ostream &lhs, const TNBase &rhs)
{
  int digits(rhs.Digits());
  if(digits==0) {lhs<<"()"; return lhs;}
  int i(digits-1);
  lhs<<"("<<rhs[i];
  for (--i; i>=0; --i)
    lhs<<", "<<rhs[i];
  lhs<<")";
  return lhs;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TNxBase
//===========================================================================================

bool TNxBase::Assign (ValueType val)
{
  std::fill(Begin(),End(),0);
  if (val<0)  {decimal_=0; return false;}
  if (val<decimal_overflow_)  {decimal_= val;}
  else  {decimal_= decimal_overflow_;}
  ConstIterator sizeitr (size.begin());
  for (Iterator  idigit (Begin()); idigit!=End(); ++idigit,++sizeitr)
  {
    if (val<*sizeitr)
    {
      *idigit= val;
      return true;
    }
    if (idigit==End()-1) {*idigit=*sizeitr;} // overflow
    else  *idigit= val % (*sizeitr);
    val= (val-*idigit) / (*sizeitr);
  }
  return false;
};
//-------------------------------------------------------------------------------------------

std::ostream& operator<< (std::ostream &lhs, const TNxBase &rhs)
{
  int digits(rhs.Digits());
  if(digits==0) {lhs<<"()"; return lhs;}
  int i(digits-1);
  lhs<<"("<<rhs[i];
  for (--i; i>=0; --i)
    lhs<<", "<<rhs[i];
  lhs<<")";
  return lhs;
}
//-------------------------------------------------------------------------------------------



//===========================================================================================
// class TBubbleSet
//===========================================================================================

TBubbleSet::TRealVector TBubbleSet::Center(int index) const
{
  if (GenSize(cmin_)==GenSize(scale_))  return VectorElemMult(bubble_set_[index].Center,scale_);
  else  return  bubble_set_[index].Center;
}
//-------------------------------------------------------------------------------------------

void TBubbleSet::SetCenterMax (const TRealVector &cmax)
{
  cmax_=cmax;
  scaled_cmax_=cmax;
  if (GenSize(scaled_cmax_)==GenSize(scale_))
    VectorElemDivAssign(scaled_cmax_,scale_);
}
void TBubbleSet::SetCenterMin (const TRealVector &cmin)
{
  cmin_=cmin;
  scaled_cmin_=cmin;
  if (GenSize(scaled_cmin_)==GenSize(scale_))
    VectorElemDivAssign(scaled_cmin_,scale_);
}
void TBubbleSet::SetScale (const TRealVector &scale)
{
  scale_=scale;
  if (GenSize(cmax_)==GenSize(scale_))  {scaled_cmax_=cmax_; VectorElemDivAssign(scaled_cmax_,scale_);}
  if (GenSize(cmin_)==GenSize(scale_))  {scaled_cmin_=cmin_; VectorElemDivAssign(scaled_cmin_,scale_);}
}
//-------------------------------------------------------------------------------------------

void TBubbleSet::GenerateRandomly (int N, const TReal &init_radius)
{
  LASSERT1op1(GenSize(scaled_cmin_),==,GenSize(scaled_cmax_));
  int dim(GenSize(scaled_cmin_));
  bubble_set_.resize(N);

  radius_= init_radius;
  radius_spd_ = 0.0l;
  radius_total_force_ = 0.0l;

  for (std::vector<TBubble>::iterator itr(bubble_set_.begin()),last(bubble_set_.end()); itr!=last; ++itr)
  {
    GenResize(itr->Center,dim);
    GenResize(itr->Velocity,dim);
    GenResize(itr->TotalForce,dim);
    GenerateRandomVector(itr->Center,scaled_cmin_,scaled_cmax_);
    SetZero(itr->Velocity);
    SetZero(itr->TotalForce);
  }
}
//-------------------------------------------------------------------------------------------

TReal TBubbleSet::Step (const TReal &time_step)
{
  // clear total force:
  SetZero(radius_total_force_);
  for (std::vector<TBubble>::iterator itr(bubble_set_.begin()),last(bubble_set_.end()); itr!=last; ++itr)
    SetZero(itr->TotalForce);

  // calculate total force:
  int i(0);
  TRealVector diff(GenSize(scaled_cmin_));
  TRealVector force(GenSize(scaled_cmin_));
  for (std::vector<TBubble>::iterator itr(bubble_set_.begin()),last(bubble_set_.end()); itr!=last; ++itr,++i)
  {
    // contact force from the other bubbles:
    for (std::vector<TBubble>::iterator itr2(bubble_set_.begin()+i); itr2!=last; ++itr2)
    {
      if (itr==itr2)  continue;
      diff= itr2->Center - itr->Center;
      TReal distance= GetNorm(diff);
      if (distance<2.0l*radius_)  // overlapping
      {
        force= diff*(spring_k_*(2.0l*radius_-distance)/distance);
        itr->TotalForce  -= force;
        itr2->TotalForce += force;
        radius_total_force_= std::max(radius_total_force_, spring_k_*(2.0l*radius_-distance));
      }
    }
    // contact force from boundaries:
    TReal f;
    TypeExt<TRealVector>::const_iterator imax(GenBegin(scaled_cmax_)), imin(GenBegin(scaled_cmin_));
    TypeExt<TRealVector>::iterator  fitr(GenBegin(itr->TotalForce));
    for (TypeExt<TRealVector>::const_iterator celem_itr(GenBegin(itr->Center)),celem_last(GenEnd(itr->Center));
        celem_itr!=celem_last; ++celem_itr,++fitr,++imax,++imin)
    {
      if (*imin==*imax)
      {
        // do nothing (zero force)
      }
      else
      {
        if ((f=(*imin+margin_ratio_*radius_)-*celem_itr) > 0.0l)
        {
          f*= spring_k_;
          *fitr += f;
          radius_total_force_= std::max(radius_total_force_, f);
        }
        if ((f=*celem_itr-(*imax-margin_ratio_*radius_)) > 0.0l)
        {
          f*= spring_k_;
          *fitr -= f;
          radius_total_force_= std::max(radius_total_force_, f);
        }
      }
    }  // end of calc: contact force from boundaries
  }  // end of calc: total force
  radius_total_force_= radius_internal_force_ - radius_total_force_;

  // integrate:
  // TReal spd_norm_sum(0.0l);
  // TReal acc_norm_sum(0.0l);
  TReal acc_norm_max(0.0l);
  radius_total_force_+= -radius_dumping_*radius_spd_;
  radius_ += time_step * radius_spd_;
  radius_spd_ += time_step * radius_total_force_ / radius_mass_;
  // spd_norm_sum+= Square(radius_spd_);
  // acc_norm_sum+= Square(radius_total_force_ / radius_mass_);
  acc_norm_max= std::max(acc_norm_max, real_fabs(radius_total_force_ / radius_mass_));
  for (std::vector<TBubble>::iterator itr(bubble_set_.begin()),last(bubble_set_.end()); itr!=last; ++itr)
  {
    WeightedAdd (itr->TotalForce, -bubble_dumping_, itr->Velocity);
    WeightedAdd (itr->Center, time_step, itr->Velocity);
    WeightedAdd (itr->Velocity, time_step/bubble_mass_, itr->TotalForce);
    // spd_norm_sum+= GetNormSq(itr->Velocity);
    // acc_norm_sum+= Square(1.0l/bubble_mass_)*GetNormSq(itr->TotalForce);
    acc_norm_max= std::max(acc_norm_max, 1.0l/bubble_mass_*GetNorm(itr->TotalForce));
  }

  // constraints:
  if (radius_<0.0l)  radius_= 0.0l;
  // for (std::vector<TBubble>::iterator itr(bubble_set_.begin()),last(bubble_set_.end()); itr!=last; ++itr)
    // ConstrainVector (itr->Center,scaled_cmin_,scaled_cmax_);

  // return real_sqrt(spd_norm_sum);
  // return real_sqrt(acc_norm_sum);
  return acc_norm_max;
}
//-------------------------------------------------------------------------------------------

void TBubbleSet::PrintCenters (std::ostream &os) const
{
  if (GenSize(cmin_)==GenSize(scale_))
  {
    for (std::vector<TBubble>::const_iterator itr(bubble_set_.begin()),last(bubble_set_.end()); itr!=last; ++itr)
      os<< GenPrint(VectorElemMult(itr->Center,scale_)) <<std::endl;
  }
  else
  {
    for (std::vector<TBubble>::const_iterator itr(bubble_set_.begin()),last(bubble_set_.end()); itr!=last; ++itr)
      os<< GenPrint(itr->Center) <<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

void TBubbleSet::PrintRadiusCenters (std::ostream &os) const
{
  if (GenSize(cmin_)==GenSize(scale_))
  {
    for (std::vector<TBubble>::const_iterator itr(bubble_set_.begin()),last(bubble_set_.end()); itr!=last; ++itr)
      os<< radius_ << " " << GenPrint(VectorElemMult(itr->Center,scale_)) <<std::endl;
  }
  else
  {
    for (std::vector<TBubble>::const_iterator itr(bubble_set_.begin()),last(bubble_set_.end()); itr!=last; ++itr)
      os<< radius_ << " " << GenPrint(itr->Center) <<std::endl;
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
