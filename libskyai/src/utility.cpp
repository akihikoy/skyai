//-------------------------------------------------------------------------------------------
/*! \file    utility.cpp
    \brief   libskyai - utility functions
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jul.08, 2010

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
#include <skyai/utility.h>
#include <skyai/skyai.h>
#include <lora/string.h>
#include <lora/small_classes.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


bool ParseCmdLineOption (TAgent &agent, TOptionParser &option, std::ostream &debug_stream)
{
  std::list<std::string>  included_list;

  if (ConvertFromStr<bool>(option("available_mods","false")))
  {
    LMESSAGE("TModuleManager::ShowAllModules():");
    TModuleManager::ShowAllModules(option("show_conf"));
    return false;
  }

  if (option("path")!="")
  {
    TTokenizer tokenizer(option("path"));
    while(!tokenizer.EOL())
    {
      tokenizer.ReadSeparators();
      agent.AddPath(tokenizer.ReadNonSeparators());
    }
  }
  if (option("agent")=="")
    {LERROR("fatal! -agent option is needed."); lexit(df);}
  else
  { /*load agent files*/
    TTokenizer tokenizer(option("agent"));
    string agent_file,tmp_filename;
    while(!tokenizer.EOL())
    {
      tokenizer.ReadSeparators();
      agent_file= agent.SearchFileName(tmp_filename= tokenizer.ReadNonSeparators(), ".agent");
      if (agent_file=="" || !agent.LoadFromFile(agent_file,NULL,&included_list))
        {LERROR("failed to read "<<tmp_filename); lexit(df);}
    }
  }

  if (option("outdir")!="")  agent.SetConfig().DataDir= option("outdir");
    // NOTE: this line should be after loading agent files since DataDir can be changed by the agent files

  bool overwrite(true);

  using namespace boost::filesystem;
  path  included_dir(agent.GetDataFileName("included"),native);

  if (exists(path(agent.GetDataFileName("cmdline"),native))
      || exists(included_dir))
  {
    cerr<<agent.GetDataFileName("")<<" was already used. Will you overwrite?"<<endl;
    cerr<<"  answer Yes    : overwrite"<<endl;
    cerr<<"  answer No     : not overwrite (continue)"<<endl;
    cerr<<"  answer Cancel : stop running"<<endl;
    switch(AskYesNoCancel())
    {
      case ryncYes    :  overwrite= true; break;
      case ryncNo     :  overwrite= false; break;
      case ryncCancel :  lexit(qfail); break;
      default : lexit(abort);
    }

    if (overwrite && exists(included_dir))  // move "included" directory to "included.old##"
    {
      path  renamed_path(included_dir);
      for (int i(1); i<1000; ++i)
      {
        renamed_path= included_dir.parent_path()/("included.old"+ConvertToStr(i));
        if (!exists(renamed_path))  break;
      }
      if (exists(renamed_path))
        remove_all(renamed_path);
      rename(included_dir,renamed_path);
    }
  }

  if (overwrite)
  {
    ofstream ofs(agent.GetDataFileName("cmdline").c_str());
    ofs<< option.CommandLine();
    ofs.close();
  }

  if (overwrite)
  {
    if (exists(included_dir.parent_path()))
    {
      create_directory(included_dir);
      for (std::list<std::string>::const_iterator itr(included_list.begin()),last(included_list.end()); itr!=last; ++itr)
      {
        path from(*itr,native);
        copy_file(from, included_dir/(from.filename()));
      }
    }
  }

  if (ConvertFromStr<bool>(option("show_mods","false")))
  {
    LMESSAGE("agent's modules:");
    agent.ShowAllModules(option("show_conf"),cout);
    return false;
  }

  if (ConvertFromStr<bool>(option("show_connect","false")))
  {
    LMESSAGE("agent's connections:");
    agent.ShowAllConnections(cout);
    return false;
  }

  if (ConvertFromStr<bool>(option("dump_debug","false")))
  {
    agent.SetDebugStream (debug_stream);
    agent.SetAllModuleMode (TModuleInterface::mmDebug);
  }

  if (ConvertFromStr<bool>(option("export_dot","false")))
  {
    agent.ExportToDOT(cout);
    return false;
  }

  return true;
}
//-------------------------------------------------------------------------------------------

bool ParseCmdLineOption (TAgent &agent, TOptionParser &option, std::ofstream &debug_fstream)
{
  if (ConvertFromStr<bool>(option("dump_debug","false")))
    debug_fstream.open(agent.GetDataFileName("debug.dat").c_str());
  return ParseCmdLineOption(agent,option,static_cast<std::ostream&>(debug_fstream));
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
