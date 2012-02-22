//-------------------------------------------------------------------------------------------
/*! \file    learning_manager.h
    \brief   libskyai - learning management (e.g. number of episode) module  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.30, 2009-

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
#ifndef skyai_learning_manager_h
#define skyai_learning_manager_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <lora/rand.h>
#include <lora/variable_space_impl.h>
#include <lora/variable_literal.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TBasicLearningManagerConfigurations
//===========================================================================================
{
public:

  TInt         MaxEpisodeNumber;
  TString      RandomSeed;  //!< random seed (integer).  if "t" or "time", time() is used

  TBasicLearningManagerConfigurations (var_space::TVariableMap &mmap)
    :
      MaxEpisodeNumber    (1000),
      RandomSeed          ("time")
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( MaxEpisodeNumber    );
      ADD( RandomSeed          );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TBasicLearningManagerMemories
//===========================================================================================
{
public:

  unsigned long             RandomSeed;  //!< random seed (actually used)

  TBasicLearningManagerMemories (var_space::TVariableMap &mmap)
    :
      RandomSeed          (0)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( RandomSeed        );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief a learning managing module which provides an interface to initialize and to start learning,
        and emmits initialize, start episode signals to the other modules */
class MBasicLearningManager : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface       TParent;
  typedef MBasicLearningManager  TThis;
  SKYAI_MODULE_NAMES(MBasicLearningManager)

  MBasicLearningManager(const std::string &v_instance_name)
    : TParent                (v_instance_name),
      conf_                  (TParent::param_box_config_map()),
      mem_                   (TParent::param_box_memory_map()),
      is_initialized_        (false),
      episode_number_        (0),
      is_learning_           (false),
      slot_initialize        (*this),
      slot_start_episode     (*this),
      slot_finish_episode    (*this),
      signal_end_of_learning (*this),
      out_episode_number     (*this)
    {
      add_slot_port   (slot_initialize);
      add_slot_port   (slot_start_episode);
      add_slot_port   (slot_finish_episode);
      add_signal_port (signal_end_of_learning);
      add_out_port    (out_episode_number);
    }
  virtual ~MBasicLearningManager() {}


  bool IsLearning () const {return is_learning_;}

  void Initialize ()
    {
      slot_initialize.Exec();
    }

  void StartLearning ()
    {
      if (!is_initialized_)
      {
        LERROR(PortUniqueCode(slot_initialize)<<" is not initialized! execute Initialize before StartLearning()");
        lexit(df);
      }
      slot_start_episode.Exec();
    }

protected:

  TBasicLearningManagerConfigurations conf_;
  TBasicLearningManagerMemories       mem_;

  bool  is_initialized_;
  TInt  episode_number_;
  bool  is_learning_;

  MAKE_SLOT_PORT_SPECIFIC(slot_initialize, void, (void), (), TThis, SKYAI_CONNECTION_SIZE_MAX, "signal_initialization");

  MAKE_SLOT_PORT_SPECIFIC(slot_start_episode, void, (void), (), TThis, SKYAI_CONNECTION_SIZE_MAX, "signal_start_of_episode");

  MAKE_SLOT_PORT(slot_finish_episode, void, (void), (), TThis);

  MAKE_SIGNAL_PORT(signal_end_of_learning, void(void), TThis);

  MAKE_OUT_PORT(out_episode_number, const TInt&, (void), (), TThis);


  virtual void slot_initialize_exec (void)
    {
      if (conf_.RandomSeed=="t" || conf_.RandomSeed=="time")
        mem_.RandomSeed= static_cast<unsigned long>(time(NULL));
      else
        mem_.RandomSeed= ConvertFromStr<unsigned long>(conf_.RandomSeed);
      XSrand (mem_.RandomSeed);
      LMESSAGE("random seed = "<<mem_.RandomSeed);

      episode_number_= 0;
      // LDEBUG(InstanceName()<<"->slot_initialize is activated");
      is_initialized_= true;
    }

  virtual void slot_start_episode_exec (void)
    {
      is_learning_= true;
      LMESSAGE("episode "<<episode_number_<<"...");
    }

  virtual void slot_finish_episode_exec (void)
    {
      episode_number_++;
      is_learning_= false;
      if (episode_number_ < conf_.MaxEpisodeNumber)
      {
        // start the next episode...
        slot_start_episode.Exec();
      }
      else
      {
        signal_end_of_learning.ExecAll();
      }
    }

  virtual const TInt& out_episode_number_get (void) const
    {
      return episode_number_;
    }

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief a learning managing module which provides an interface to initialize and to start episode */
class MManualLearningManager : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface       TParent;
  typedef MManualLearningManager  TThis;
  SKYAI_MODULE_NAMES(MManualLearningManager)

  MManualLearningManager(const std::string &v_instance_name)
    : TParent                (v_instance_name),
      conf_                  (TParent::param_box_config_map()),
      mem_                   (TParent::param_box_memory_map()),
      episode_number_        (-1),
      is_in_episode_         (false),
      signal_initialization  (*this),
      signal_start_of_episode(*this),
      slot_finish_episode    (*this),
      out_episode_number     (*this)
    {
      add_signal_port (signal_initialization  );
      add_signal_port (signal_start_of_episode);
      add_slot_port   (slot_finish_episode    );
      add_out_port    (out_episode_number     );
    }
  virtual ~MManualLearningManager() {}


  bool IsLearning () const {return episode_number_ >=0 && episode_number_ < conf_.MaxEpisodeNumber;}
  bool IsInEpisode () const {return is_in_episode_;}

  void Initialize ()
    {
      if (conf_.RandomSeed=="t" || conf_.RandomSeed=="time")
        mem_.RandomSeed= static_cast<unsigned long>(time(NULL));
      else
        mem_.RandomSeed= ConvertFromStr<unsigned long>(conf_.RandomSeed);
      XSrand (mem_.RandomSeed);
      LMESSAGE("random seed = "<<mem_.RandomSeed);

      episode_number_= 0;
      is_in_episode_= false;

      signal_initialization.ExecAll();
    }

  void StartEpisode ()
    {
      if (episode_number_<0)
      {
        LERROR("before StartEpisode(), execute Initialize()");
        lexit(df);
      }
      LMESSAGE("episode "<<episode_number_<<"...");
      is_in_episode_= true;
      signal_start_of_episode.ExecAll();
    }

protected:

  TBasicLearningManagerConfigurations conf_;
  TBasicLearningManagerMemories       mem_;

  TInt  episode_number_;
  bool  is_in_episode_;

  MAKE_SIGNAL_PORT(signal_initialization, void (void), TThis);

  MAKE_SIGNAL_PORT(signal_start_of_episode, void (void), TThis);

  MAKE_SLOT_PORT(slot_finish_episode, void, (void), (), TThis);

  MAKE_OUT_PORT(out_episode_number, const TInt&, (void), (), TThis);


  virtual void slot_finish_episode_exec (void)
    {
      episode_number_++;
      is_in_episode_= false;
    }

  virtual const TInt& out_episode_number_get (void) const
    {
      return episode_number_;
    }

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TFunctionSchedulerConfigurations
//===========================================================================================
{
public:

  std::map<TInt,TString>    Schedule;  //!< map [counter]="function-id"

  TFunctionSchedulerConfigurations (var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Schedule      );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TFunctionSchedulerMemories
//===========================================================================================
{
public:

  TInt          Counter;

  TFunctionSchedulerMemories (var_space::TVariableMap &mmap)
    :
      Counter  (0)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Counter   );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief function scheduler that executes functions according to conf_.Schedule ([counter]="function-id") */
class MFunctionScheduler : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface    TParent;
  typedef MFunctionScheduler  TThis;
  SKYAI_MODULE_NAMES(MFunctionScheduler)

  MFunctionScheduler(const std::string &v_instance_name)
    : TParent      (v_instance_name),
      conf_        (TParent::param_box_config_map()),
      mem_         (TParent::param_box_memory_map()),
      slot_step    (*this),
      out_counter  (*this)
    {
      add_slot_port   (slot_step      );
      add_out_port    (out_counter    );
    }
  virtual ~MFunctionScheduler() {}


protected:

  TFunctionSchedulerConfigurations conf_;
  TFunctionSchedulerMemories       mem_;

  MAKE_SLOT_PORT(slot_step, void, (void), (), TThis);

  MAKE_OUT_PORT(out_counter, const TInt&, (void), (), TThis);


  virtual void slot_step_exec (void)
    {
      using namespace var_space;
      std::map<TInt,TString>::const_iterator
          sch_itr= conf_.Schedule.find(mem_.Counter);
      if (sch_itr!=conf_.Schedule.end())
      {
        if (!ExecuteFunction(sch_itr->second, /*literal_table=*/std::list<TLiteral>()))
          {LERROR("failed to execute function(counter="<<sch_itr->first<<"): "<<sch_itr->second); lexit(df);}
      }
      ++mem_.Counter;
    }

  virtual const TInt& out_counter_get (void) const
    {
      return mem_.Counter;
    }

};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_learning_manager_h
//-------------------------------------------------------------------------------------------
