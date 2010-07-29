//-------------------------------------------------------------------------------------------
/*! \file    sys.cpp
    \brief   liblora - system utility  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-

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

    \todo   Implement DirectoryExists and CreateDirectory using boost::filesystem, and move them into file.{h,cpp}
*/
//-------------------------------------------------------------------------------------------
#include <lora/sys.h>
//-------------------------------------------------------------------------------------------
#include <sys/stat.h>  // stat, mkdir
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------

/*! \brief check the dir exists */
bool DirectoryExists (const std::string &dirname)
{
  struct stat sb;
  if (stat(dirname.c_str(),&sb)==-1)  return false;
  if (S_ISDIR(sb.st_mode))  return true;
  return false;
}
//-------------------------------------------------------------------------------------------

/*! \brief create directory */
bool CreateDirectory (const std::string &dirname, mode_t mode)
{
  if (mkdir(dirname.c_str(),mode)==-1)  return false;
  return true;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class TKBHit
//===========================================================================================

void TKBHit::Open (void)
{
  if (is_open_)  Close();
  tcgetattr(STDIN_FILENO, &old_tios_);

  raw_tios_= old_tios_;
  cfmakeraw(&raw_tios_);

  tcsetattr(STDIN_FILENO, 0, &raw_tios_);
  is_open_= true;
}
//-------------------------------------------------------------------------------------------

void TKBHit::Close (void)
{
  if (!is_open_)  return;
  tcsetattr(STDIN_FILENO, 0, &old_tios_);
  std::cout<<std::endl;
  is_open_= false;
}
//-------------------------------------------------------------------------------------------

int TKBHit::operator() (void) const
{
  return getchar();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
