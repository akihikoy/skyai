//-------------------------------------------------------------------------------------------
/*! \file    boost_filesystem_ext.h
    \brief   liblora - certain program (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Apr.18, 2014

    Copyright (C) 2014  Akihiko Yamaguchi

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
#ifndef loco_rabbits_boost_filesystem_ext_h
#define loco_rabbits_boost_filesystem_ext_h
//-------------------------------------------------------------------------------------------
#include <boost/filesystem/path.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

inline boost::filesystem::path ConcatenatePath(const boost::filesystem::path &lhs, const std::string &rhs)
{
  return boost::filesystem::path(lhs.string()+rhs);
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_boost_filesystem_ext_h
//-------------------------------------------------------------------------------------------
