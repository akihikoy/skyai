//-------------------------------------------------------------------------------------------
/*! \file    bioloid.h
    \brief   liblora - ROBOTIS bioloid control library (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Mar.08, 2010-

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
#ifndef bioloid_h
#define bioloid_h
//-------------------------------------------------------------------------------------------
#include <lora/serial.h>
#include <lora/string.h>
#include <lora/sys.h>
#include <iomanip>
#include <cstring>
#include <cstdlib>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace bioloid
{
//-------------------------------------------------------------------------------------------

class TBioloidWriteStream : public serial::TSerialStreamBase
{
protected:
  int command_length_;
  int sum_;

public:
  TBioloidWriteStream (void) :
      serial::TSerialStreamBase() {};

  void Init(unsigned char id)
    {
      Clear();
      sum_= 0;
      command_length_= 0;

      buffer_[0]=0xff;
      buffer_[1]=0xff;
      buffer_[2]=id;    sum_+=buffer_[2];
      buffer_[3]=0;
      buf_ptr_=buffer_+4;
    }

  TBioloidWriteStream& operator<< (unsigned char s)
    {
      if (buf_ptr_==buf_end_) oferror();
      else
      {
        *buf_ptr_= s;
        ++buf_ptr_;

        ++command_length_;
        sum_+= s;
      }
      return *this;
    }

  override serial::TSerialStreamBase& operator>> (serial::TSerialCom &serial)
    {
      ++command_length_;
      buffer_[3]=command_length_;  sum_+= command_length_;
      *buf_ptr_= 255 - (sum_ % 256);  // checksum
      ++buf_ptr_;

      serial.Write (buffer_, Bytes());
      //*dbg*/std::cerr<<"write: "<<PrintBuffer(buffer_, Bytes())<<std::endl;
      Clear();
      return *this;
    };

};
//-------------------------------------------------------------------------------------------

class TSyncWriteStream : public TBioloidWriteStream
{
public:
  TSyncWriteStream (void) :
      TBioloidWriteStream() {};

  void Init(unsigned char address, unsigned char length)
    {
      TBioloidWriteStream::Init(0xfe);
      *this<<0x83<<address<<length;
    }
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
const double DEGREE_MIN (0.0);
const double DEGREE_MAX (300.0);
const double RPM_MIN (0.0);
const double RPM_MAX (114.0);
const int    COMMAND_ANGLE_MAX (1023);
const int    COMMAND_ANGVEL_MAX (1023);
const double DEGREE_PER_COMMAND ((DEGREE_MAX-DEGREE_MIN)/static_cast<double>(COMMAND_ANGLE_MAX));
const double RPM_PER_COMMAND ((RPM_MAX-RPM_MIN)/static_cast<double>(COMMAND_ANGVEL_MAX));
const double COMMON_ANGLE_OFFSET (150.0);
//-------------------------------------------------------------------------------------------
const int    BUFFER_SIZE(2048);
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
// utilities
//-------------------------------------------------------------------------------------------
termios GetDefaultTermios (void);
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
// template utilities
//-------------------------------------------------------------------------------------------
template <typename t_value>
inline int DegreeToCommand (const t_value &v_angle);
template <typename t_value>
inline int RPMToCommand (const t_value &v_rpm);
template <typename t_value>
inline t_value CommandToDegree (int command_lo, int command_hi);
template <typename t_value>
inline t_value CommandToRPM (int command_lo, int command_hi);
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TBioloidController
//===========================================================================================
{
public:

  void Connect (const std::string &v_tty, const termios &v_ios=GetDefaultTermios());
  void Disconnect ();

  //!\brief read from where the data starts with 0xffff
  int ReadFromFFFF (unsigned char *buf, int N, int max_trial=10, const int pr_max_trial=10);

  //!\brief read a valid status packet from dynamixels
  int ReadStatusPacket (unsigned char id);

  void ReadAndEcho (int N=BUFFER_SIZE);

  //!\brief binary communicating mode
  void TossMode(void);

  //! LED of (#1,#2) changes (on,off),(off,on),(on,off),(off,on),...
  void TossTest (void);

  template <typename t_angle_fwditr, typename t_id_fwditr>
  void GoTo (t_id_fwditr id_begin, t_id_fwditr id_end, t_angle_fwditr angle_begin);

  void SetLightDetectCompare (unsigned char id, unsigned char value);

  bool EnableTorque (unsigned char id);

  int GetAngle (unsigned char id, double &angle);

  int PersistingGetAngle (unsigned char id, double &angle, int max_trial=10);

  template <typename t_angle_fwditr, typename t_id_fwditr>
  int GetAllAngles (t_id_fwditr id_begin, t_id_fwditr id_end, t_angle_fwditr angle_begin);

  /*!\brief get distance to an object via IR-sensor
      \param [in]sensor_pos  : -1:left, 0:center, 1:right  */
  int GetDistance (unsigned char id, int sensor_pos, double &distance);

protected:
  serial::TSerialCom  serial_;

  serial::TStringWriteStream   str_writes_;
  TBioloidWriteStream          biol_writes_;
  TSyncWriteStream             sync_writes_;
  unsigned char buffer_[BUFFER_SIZE];

};
//-------------------------------------------------------------------------------------------


template <typename t_angle_fwditr, typename t_id_fwditr>
void TBioloidController::GoTo (t_id_fwditr id_begin, t_id_fwditr id_end, t_angle_fwditr angle_begin)
{
  sync_writes_.Init (0x1e, 2);
  int command, hi, lo;
  for (; id_begin!=id_end; ++id_begin,++angle_begin)
  {
    command= DegreeToCommand(*angle_begin);
    lo= command % 256;
    hi= (command-lo) / 256;
    sync_writes_<<*id_begin<<lo<<hi;
  }
  sync_writes_>>serial_;
}
//-------------------------------------------------------------------------------------------

template <typename t_angle_fwditr, typename t_id_fwditr>
int TBioloidController::GetAllAngles (t_id_fwditr id_begin, t_id_fwditr id_end, t_angle_fwditr angle_begin)
{
  int err(0);
  double angle;
  for (; id_begin!=id_end; ++id_begin,++angle_begin)
    if (PersistingGetAngle(*id_begin,angle)>0)
      *angle_begin= angle;
    else
      ++err;
  return err;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// implementation of template utilities
//-------------------------------------------------------------------------------------------

template <typename t_value>
inline int DegreeToCommand (const t_value &v_angle)
{
  double angle= static_cast<double>(v_angle);
  double command= (angle+COMMON_ANGLE_OFFSET)/DEGREE_PER_COMMAND;
  int icommand(round(command));
  if (icommand>COMMAND_ANGLE_MAX)  icommand=COMMAND_ANGLE_MAX;
  else if (icommand<0)  icommand=0;
  return icommand;
}
//-------------------------------------------------------------------------------------------

template <typename t_value>
inline int RPMToCommand (const t_value &v_rpm)
{
  double rpm= static_cast<double>(v_rpm);
  double command= rpm/RPM_PER_COMMAND;
  int icommand(round(command));
  if (icommand>COMMAND_ANGVEL_MAX)  icommand=COMMAND_ANGVEL_MAX;
  else if (icommand<0)  icommand=0;
  return icommand;
}
//-------------------------------------------------------------------------------------------

template <typename t_value>
inline t_value CommandToDegree (int command_lo, int command_hi)
{
  int icommand(command_lo+256*command_hi);
  if (icommand>COMMAND_ANGLE_MAX)  icommand=COMMAND_ANGLE_MAX;
  else if (icommand<0)  icommand=0;
  return  static_cast<t_value>(icommand)*DEGREE_PER_COMMAND - COMMON_ANGLE_OFFSET;
}
//-------------------------------------------------------------------------------------------

template <typename t_value>
inline t_value CommandToRPM (int command_lo, int command_hi)
{
  int icommand(command_lo+256*command_hi);
  if (icommand>COMMAND_ANGVEL_MAX)  icommand=COMMAND_ANGVEL_MAX;
  else if (icommand<0)  icommand=0;
  return  static_cast<t_value>(icommand)*RPM_PER_COMMAND;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace bioloid
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // bioloid_h
//-------------------------------------------------------------------------------------------
