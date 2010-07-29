//-------------------------------------------------------------------------------------------
/*! \file    sys.h
    \brief   liblora - system utility  (header)
    \author  Akihiko Yamaguchi
    \date    2008-

    Copyright (C) 2008, 2010  Akihiko Yamaguchi

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

    -----------------------------------------------------------------------------------------

    \warning this library depends on system-calls

    \todo   Implement DirectoryExists and CreateDirectory using boost::filesystem, and move them into file.{h,cpp}
*/
//-------------------------------------------------------------------------------------------
#ifndef loco_rabbits_sys_h
#define loco_rabbits_sys_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <sys/time.h>  // getrusage, gettimeofday
#include <sys/resource.h> // get cpu time
#include <sys/types.h>
#include <termios.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

inline TReal GetUserTime (void)
{
  struct rusage RU;
  getrusage(RUSAGE_SELF, &RU);
  return static_cast<TReal>(RU.ru_utime.tv_sec) + static_cast<TReal>(RU.ru_utime.tv_usec)*1.0e-6;
}
//-------------------------------------------------------------------------------------------

inline double GetCurrentTime (void)
{
  struct timeval time;
  gettimeofday (&time, NULL);
  return static_cast<double>(time.tv_sec) + static_cast<double>(time.tv_usec)*1.0e-6;
}
//-------------------------------------------------------------------------------------------

/*!\brief  This class is try to call 'operator()' in every specified 'timing' (second) by sleep an interval
    \note  The interval is automatically learned  */
class TTimingAdjuster
{
public:
  TTimingAdjuster (void) : timing(0.0), update_alpha(0.2), interval(0.0) {};
  TTimingAdjuster (const double &v_timing) : timing(v_timing), update_alpha(0.2), interval(v_timing)
    {
      last=GetCurrentTime();
    };
  void Init (const double &v_timing)
    {
      timing= v_timing;
      interval= timing;
      last=GetCurrentTime();
    };
  void SetLearningRate (const double &alpha) {update_alpha= alpha;};
  const double& GetInterval (void) const {return interval;};
  void operator() (void)
    {
      if (interval>0.0)  usleep (interval*1.0e+6);
      double current= GetCurrentTime();
      double delta= timing-(current-last);
      if (delta<-interval) delta= -interval;
      interval= interval + update_alpha * delta;
      last= current;
    };
  void UpdateIntervalWithoutSleep (void)
    {
      double current= GetCurrentTime();
      double delta= timing-(current+interval-last);
      if (delta<-interval) delta= -interval;
      interval= interval + update_alpha * delta;
      last= current;
    };
private:
  double timing;
  double update_alpha;
  double last;
  double interval;
};
//-------------------------------------------------------------------------------------------

/*! \brief check the dir exists */
bool DirectoryExists (const std::string &dirname);
/*! \brief create directory */
bool CreateDirectory (const std::string &dirname, mode_t mode=0755);
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TKBHit
//===========================================================================================
{
public:
  TKBHit (void) : is_open_(false)  {Open();}
  ~TKBHit (void)  {Close();}
  void Open (void);
  void Close (void);
  int operator() (void) const;
private:
  termios old_tios_;
  termios raw_tios_;
  bool is_open_;
};
//-------------------------------------------------------------------------------------------
inline int WaitKBHit(void)
{
  TKBHit kbhit;
  return kbhit();
}
inline void WaitKBHit(int k)
{
  TKBHit kbhit;
  while(true)
  {
    int s= kbhit();
    if(s==k)  break;
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_sys_h
//-------------------------------------------------------------------------------------------

