//-------------------------------------------------------------------------------------------
/*! \file    rlbioloid.cpp
    \brief   benchmarks - motion learning task in a real robot (Bioloid, ROBOTIS)
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
#include <skyai/skyai.h>
#include <skyai/parser.h>
#include <skyai/modules_core/learning_manager.h>
#include <lora/bioloid.h>
#include <lora/small_classes.h>
#include <lora/variable_space_impl.h>
#include <fstream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{


//===========================================================================================
class TBioloidEnvironmentConfigurations
//===========================================================================================
{
public:

  TString           SerialPort;
  TIntVector        ActuatorIndexes;
  TIntVector        SensingAngleIndexes;
  TInt              DistanceSensorIndex;
  TRealVector       AngleMax;
  TRealVector       AngleMin;

  TReal             InitSleepTime;
  TReal             TimeStep;


  TBioloidEnvironmentConfigurations (var_space::TVariableMap &mmap)
    :
      SerialPort      ("/dev/ttyUSB0"),
      InitSleepTime   (2.0l),
      TimeStep        (0.1l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( SerialPort               );
      ADD( ActuatorIndexes          );
      ADD( SensingAngleIndexes      );
      ADD( DistanceSensorIndex      );
      ADD( AngleMax                 );
      ADD( AngleMin                 );
      ADD( InitSleepTime            );
      ADD( TimeStep                 );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief environment module
    \todo <b>FIXME: separate into control module, robot module, and environment module</b> */
class MBioloidEnvironment
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef MBioloidEnvironment  TThis;
  SKYAI_MODULE_NAMES(MBioloidEnvironment)

  MBioloidEnvironment (const std::string &v_instance_name)
    : TParent                     (v_instance_name),
      conf_                       (TParent::param_box_config_map()),
      // executing_                  (false),

      slot_initialize             (*this),
      slot_start_episode          (*this),
      slot_execute_command_des_q  (*this),
      slot_start_step             (*this),
      slot_finish_step            (*this),
      signal_start_of_timestep    (*this),
      signal_end_of_timestep      (*this),
      signal_system_reward        (*this),
      signal_end_of_episode       (*this),
      out_sensor_angles           (*this),
      out_sensor_distance_c       (*this),
      out_sensor_distance_diff_c  (*this)
    {
      add_slot_port   (slot_initialize            );
      add_slot_port   (slot_start_episode         );
      add_slot_port   (slot_execute_command_des_q );
      add_slot_port   (slot_start_step            );
      add_slot_port   (slot_finish_step           );
      add_signal_port (signal_start_of_timestep   );
      add_signal_port (signal_end_of_timestep     );
      add_signal_port (signal_system_reward       );
      add_signal_port (signal_end_of_episode      );
      add_out_port    (out_sensor_angles          );
      add_out_port    (out_sensor_distance_c      );
      add_out_port    (out_sensor_distance_diff_c );
    }

  void StartTimeStep ()
    {
      slot_start_step.Exec();
    }
  void FinishTimeStep ()
    {
      slot_finish_step.Exec();
    }

  const TReal& TimeStep() const {return conf_.TimeStep;}

  // bool Executing() const {return executing_;}

  void ForceFinishEpisode ()
    {
      signal_end_of_episode.ExecAll();
    }

  void Setup ()
    {
      LMESSAGE("setup robot..");
      bioloid_.Connect(conf_.SerialPort.c_str());
      bioloid_.TossMode();

      LMESSAGE("start experiment..");
    }

protected:

  TBioloidEnvironmentConfigurations conf_;

  mutable bioloid::TBioloidController  bioloid_;

  mutable TRealVector  tmp_angles_;
  mutable TRealVector  tmp_distance_c_;
  mutable TRealVector  tmp_old_distance_c_;
  mutable TRealVector  tmp_distance_diff_c_;
  mutable TRealVector  tmp_target_;

  mutable TLHBPFilters<TRealVector>  distance_lpf_;
  mutable TLHBPFilters<TRealVector>  distance_diff_lpf_;

  // bool   executing_;


  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  //! u: desired joint angle
  MAKE_SLOT_PORT(slot_execute_command_des_q, void, (const TRealVector &u), (u), TThis);

  MAKE_SLOT_PORT(slot_start_step, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_finish_step, void, (void), (), TThis);


  MAKE_SIGNAL_PORT(signal_start_of_timestep, void (const TContinuousTime &), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_timestep, void (const TContinuousTime &), TThis);

  MAKE_SIGNAL_PORT(signal_system_reward, void (const TSingleReward&), TThis);

  MAKE_SIGNAL_PORT(signal_end_of_episode, void (void), TThis);

  //! output sensor vectors
  MAKE_OUT_PORT(out_sensor_angles, const TRealVector&, (void), (), TThis);
  MAKE_OUT_PORT(out_sensor_distance_c, const TRealVector&, (void), (), TThis);
  MAKE_OUT_PORT(out_sensor_distance_diff_c, const TRealVector&, (void), (), TThis);


  virtual void slot_initialize_exec (void)
    {
      Setup();
      // executing_= true;
    }

  virtual void slot_start_episode_exec (void)
    {
      Setup();

      tmp_distance_c_.resize(1);
      tmp_old_distance_c_.resize(1);
      tmp_distance_diff_c_.resize(1);
      LMESSAGE("resetting robot..");
      TRealVector  target_angle(conf_.ActuatorIndexes.size(),0.0), observed_angle(conf_.ActuatorIndexes.size());
      double time_offset(GetCurrentTime()), time(GetCurrentTime());
      while (time-time_offset<conf_.InitSleepTime)
      {
        bioloid_.GetAllAngles (conf_.ActuatorIndexes.begin(),conf_.ActuatorIndexes.end(), GenBegin(observed_angle));
        for(TypeExt<TRealVector>::iterator itr(GenBegin(tmp_angles_)); itr!=GenEnd(tmp_angles_); ++itr)
          *itr= (*itr)/180.0*M_PI;
        constrain_angles (target_angle);
        bioloid_.GoTo (conf_.ActuatorIndexes.begin(),conf_.ActuatorIndexes.end(), GenBegin(target_angle));
        bioloid_.GetDistance(conf_.DistanceSensorIndex,0,tmp_old_distance_c_(0));
        usleep(10000);
        time= GetCurrentTime();
      }
      LMESSAGE("ok..");

      bioloid_.GetDistance(conf_.DistanceSensorIndex,0,tmp_distance_c_(0));
      tmp_distance_diff_c_(0)= 0.0;

      distance_lpf_.Initialize (TLHBPFilters<TRealVector>::LPF2,
        conf_.TimeStep, 0.5/*f*/, 0.8/*q*/, TRealVector(1,0.0), tmp_distance_c_);
      distance_diff_lpf_.Initialize (TLHBPFilters<TRealVector>::LPF2,
        conf_.TimeStep, 0.5/*f*/, 0.8/*q*/, TRealVector(1,0.0), tmp_distance_diff_c_);

      // setting initial pose...
      // if (rltcnd.INIT_POSE>=0 || rltcnd.USING_INIT_STATE_SET)
      // {
        // if (!rltcnd.USING_INIT_STATE_SET)
        // {
          // #ifndef CONT_LC
          // setWholeBodyPose (getUnitCenter(r_agent.getAgent(), rltcnd.INIT_POSE));
          // #endif
        // }
        // else
        // {
          // int ipose= (rltcnd.INIT_POSE>=0) ? rltcnd.INIT_POSE : Rand(0,rltcnd.INIT_STATE_SET.rows()-1);
          // setWholeBodyPose (rltcnd.INIT_STATE_SET.row(ipose));
        // }
      // }
    }

  virtual void slot_execute_command_des_q_exec (const TRealVector &u)
    {
      tmp_target_.resize(u.length());
      TypeExt<TRealVector>::iterator  titr(GenBegin(tmp_target_));
      for(TypeExt<TRealVector>::const_iterator uitr(GenBegin(u)); uitr!=GenEnd(u); ++uitr,++titr)
        *titr= (*uitr)*180.0/M_PI;
      constrain_angles (tmp_target_);
      bioloid_.GoTo (conf_.ActuatorIndexes.begin(),conf_.ActuatorIndexes.begin()+u.length(), GenBegin(tmp_target_));
    }

  virtual void slot_start_step_exec (void)
    {
      signal_start_of_timestep.ExecAll(conf_.TimeStep);
    }

  virtual void slot_finish_step_exec (void)
    {
      tmp_old_distance_c_(0)= distance_lpf_()(0); // tmp_distance_c_(0);
      bioloid_.GetDistance(conf_.DistanceSensorIndex,0,tmp_distance_c_(0));
      distance_lpf_ (tmp_distance_c_);  // tmp_distance_c_(0)= distance_lpf_()(0);
      tmp_distance_diff_c_(0)= -1.0*(distance_lpf_()(0)-tmp_old_distance_c_(0));
      distance_diff_lpf_ (tmp_distance_diff_c_);
std::cout<<tmp_distance_c_(0)<<"\t"<<distance_lpf_()(0)<<"\t"<<tmp_distance_diff_c_(0)<<"\t"<<distance_diff_lpf_()(0)<<std::endl;

      TSingleReward reward(0.0l);
      reward+= -0.15l*conf_.TimeStep;

      signal_system_reward.ExecAll(reward);

      signal_end_of_timestep.ExecAll(conf_.TimeStep);
    }

  virtual const TRealVector& out_sensor_angles_get () const
    {
      tmp_angles_.resize(conf_.SensingAngleIndexes.size());
      bioloid_.GetAllAngles (conf_.SensingAngleIndexes.begin(),conf_.SensingAngleIndexes.end(), GenBegin(tmp_angles_));
      for(TypeExt<TRealVector>::iterator itr(GenBegin(tmp_angles_)); itr!=GenEnd(tmp_angles_); ++itr)
        *itr= (*itr)/180.0*M_PI;
      return tmp_angles_;
    }
  virtual const TRealVector& out_sensor_distance_c_get () const
    {
      return tmp_distance_c_;
    }
  virtual const TRealVector& out_sensor_distance_diff_c_get () const
    {
      return distance_diff_lpf_();
    }

  void constrain_angles (TRealVector &target_angle)
    {
      if (conf_.AngleMax.length()>=target_angle.length() && conf_.AngleMin.length()>=target_angle.length())
      {
        TypeExt<TRealVector>::const_iterator amax(GenBegin(conf_.AngleMax)), amin(GenBegin(conf_.AngleMin));
        for (TypeExt<TRealVector>::iterator itr(GenBegin(target_angle)); itr!=GenEnd(target_angle); ++itr,++amax,++amin)
        {
          if(*itr>*amax)  *itr= *amax;
          else if (*itr<*amin)  *itr= *amin;
        }
      }
    }

};  // end of MBioloidEnvironment
//-------------------------------------------------------------------------------------------


enum TMotionLearningTaskKind
{
  mltkJump          =0,
  mltkMove          ,
  mltkStandup       ,
  mltkForwardroll   //
};
ENUM_STR_MAP_BEGIN( TMotionLearningTaskKind )
  ENUM_STR_MAP_ADD( mltkJump            )
  ENUM_STR_MAP_ADD( mltkMove            )
  ENUM_STR_MAP_ADD( mltkStandup         )
  ENUM_STR_MAP_ADD( mltkForwardroll     )
ENUM_STR_MAP_END  ( TMotionLearningTaskKind )
SPECIALIZE_TVARIABLE_TO_ENUM(TMotionLearningTaskKind)


//===========================================================================================
class TMotionLearningTaskConfigurations
//===========================================================================================
{
public:

  TMotionLearningTaskKind  TaskKind;
  TSingleReward            SumOfRmin;  //!< if sum of reward in an episode is less than this value, episode is terminated
  TContinuousTime          MaxTime;

  TMotionLearningTaskConfigurations (var_space::TVariableMap &mmap)
    :
      TaskKind             (mltkMove),
      SumOfRmin            (-40.0l),
      MaxTime              (50.0l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( TaskKind       );
      ADD( SumOfRmin      );
      ADD( MaxTime        );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief task module
class MMotionLearningTask
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef MMotionLearningTask  TThis;
  SKYAI_MODULE_NAMES(MMotionLearningTask)

  MMotionLearningTask (const std::string &v_instance_name)
    : TParent                (v_instance_name),
      conf_                  (TParent::param_box_config_map()),
      slot_initialize        (*this),
      slot_start_episode     (*this),
      slot_finish_time_step  (*this),
      signal_end_of_episode  (*this),
      signal_task_reward     (*this),
      signal_damage_reward   (*this),
      in_speed               (*this),
      in_sum_of_reward       (*this)
    {
      add_slot_port   (slot_initialize       );
      add_slot_port   (slot_start_episode    );
      add_slot_port   (slot_finish_time_step );
      add_signal_port (signal_end_of_episode );
      add_signal_port (signal_task_reward    );
      add_signal_port (signal_damage_reward  );
      add_in_port     (in_speed              );
      add_in_port     (in_sum_of_reward      );
    }

protected:

  TMotionLearningTaskConfigurations  conf_;

  TContinuousTime  time_;

  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_finish_time_step, void, (const TContinuousTime &dt), (dt), TThis);

  MAKE_SIGNAL_PORT(signal_end_of_episode, void (void), TThis);

  MAKE_SIGNAL_PORT(signal_task_reward, void (const TSingleReward&), TThis);
  MAKE_SIGNAL_PORT(signal_damage_reward, void (const TSingleReward&), TThis);

  //!\brief input the speed of the robot
  MAKE_IN_PORT(in_speed, const TRealVector& (void), TThis);

  MAKE_IN_PORT(in_sum_of_reward, const TSingleReward& (void), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(speed, const TRealVector&, (void), ())

  GET_FROM_IN_PORT(sum_of_reward, const TSingleReward&, (void), ())

  #undef GET_FROM_IN_PORT

  virtual void slot_initialize_exec (void)
    {
    }

  virtual void slot_start_episode_exec (void)
    {
      time_= 0.0l;
    }

  virtual void slot_finish_time_step_exec (const TContinuousTime &dt)
    {
      time_+=dt;

      // calculate task reward
      {
        TSingleReward  task_reward(0.0l);
        switch (conf_.TaskKind)
        {
          case mltkMove        :
            /*!Goal Reward def.*/
            // task_reward= body[BASELINK_INDEX].getLinearVel()[0];
            // task_reward= 0.01l*task_reward - 0.1l*Square(body[BASELINK_INDEX].getPosition()[1]);
            task_reward= get_speed()(0);
            break;
          default :
            LERROR("invalid TaskKind= "<<conf_.TaskKind);
            lexit(df);
        }
        signal_task_reward.ExecAll(task_reward);
      }

      // calculate damage reward
      // bool fallen_down(false);
      // {
        // if (conf_.TaskKind==mltkMove)
        // {
          // if ((fallen_down=fallenDown()))
          // {
            // TSingleReward dreward=-4.0l;
            // signal_damage_reward.ExecAll(dreward);
          // }
        // }
      // }

      // is the end of the episode?
      {
        bool is_end_of_episode(false);
        switch (conf_.TaskKind)
        {
          case mltkMove        :
            if (get_sum_of_reward() <= conf_.SumOfRmin)
              is_end_of_episode=true;
            break;
          default :
            LERROR("invalid TaskKind= "<<conf_.TaskKind);
            lexit(df);
        }
        if (conf_.MaxTime>0.0l && time_+CONT_TIME_TOL>=conf_.MaxTime)
        {
          is_end_of_episode= true;
        }
        if (is_end_of_episode)
        {
          signal_end_of_episode.ExecAll();
        }
      }
    }

};  // end of MMotionLearningTask
//-------------------------------------------------------------------------------------------



SKYAI_ADD_MODULE(MBioloidEnvironment)
SKYAI_ADD_MODULE(MMotionLearningTask)


}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------



static bool Executing(true);
// static bool ForceSetup(false);
int ExitCode(0);
const TAgent *PtrAgent(NULL);
MBioloidEnvironment  *PtrEnvironment(NULL);
string OutDir;
void SaveLearningParams ();
void FinishLearningProc ();

void sig_handler(int signo)
{
  using namespace std;
  if(!Executing)
    {std::cerr<<"quit immediately!"<<std::endl;  exit(1);}

  if(signo==SIGINT)
  {
    std::cerr<<ioscc::blue<<"interrupted."<<std::endl;
    std::cerr<<
      "  quit/QUIT:  quit with returning a failure code"<<std::endl<<
      "  squit/SQUIT:  quit with returning a success code"<<std::endl<<
      "  l/L:  save the learned valuetable"<<std::endl<<
      "  r/R:  reset the simulation and start a new episode"<<std::endl<<
      "  sr/SR:  reset the simulation (including connection setup) and start a new episode"<<std::endl;
    string str;
    while(true)
    {
      std::cerr<<"select action: "<<std::flush;
      std::cin>>str;
      if(str=="quit"||str=="QUIT")
      {
        Executing=false;
        ExitCode=1;
        FinishLearningProc();
        break;
      }
      else if(str=="squit"||str=="SQUIT")
      {
        Executing=false;
        ExitCode=0;
        FinishLearningProc();
        break;
      }
      else if(str=="l"||str=="L")
      {
        SaveLearningParams();
        std::cerr<<"reset the simulation..."<<std::endl;
        PtrEnvironment->ForceFinishEpisode();
        break;
      }
      else if(str=="r"||str=="R")
      {
        std::cerr<<"reset the simulation..."<<std::endl;
        PtrEnvironment->ForceFinishEpisode();
        break;
      }
      else if(str=="sr"||str=="SR")
      {
        std::cerr<<"reset the simulation (s)..."<<std::endl;
        // ForceSetup= true;
        PtrEnvironment->Setup();
        PtrEnvironment->ForceFinishEpisode();
        break;
      }
      else {std::cerr<<"unknown action."<<std::endl;}
    }
  }
  else if (signo==SIGQUIT)
  {
    std::cerr<<"quit..."<<std::endl;
    Executing=false; FinishLearningProc();
  }
  else  std::cerr<<"signal code "<<signo<<" is ignored."<<std::endl;

  // com::client_state::ClearServerQueue();
}
//-------------------------------------------------------------------------------------------

void ConnectionErrorHandler(void)
{
  LERROR("connection error! force reset");
  PtrEnvironment->ForceFinishEpisode();
}
//-------------------------------------------------------------------------------------------

void SaveLearningParams ()
{
  static int params_index(0);
  std::string  suffix (IntToStr(params_index));
  std::string  filename (OutDir+"rlbioloid-learning"+suffix+".agent");
  LMESSAGE("writing learned parameters to "<<filename);
  SaveAgentToFile (*PtrAgent, filename);
  params_index++;
}
//-------------------------------------------------------------------------------------------

void FinishLearningProc ()
{
  std::string  filename (OutDir+"rlbioloid-after.agent");
  LMESSAGE("writing learned parameters to "<<filename);
  SaveAgentToFile (*PtrAgent, filename);
}
//-------------------------------------------------------------------------------------------


int main (int argc, const char **argv)
{
  signal(SIGINT,sig_handler);
  signal(SIGQUIT,sig_handler);

  TOptionParser option(argc,argv);
  OutDir= option("outdir", "result/rl/");
  {ofstream ofs((OutDir+"exec").c_str()); SaveArguments(argc,argv,ofs); ofs.close();}

  // [-- setup the agent

  TAgent  agent;
  if (option("agent")=="")
    {LERROR("fatal! -agent option is needed."); lexit(df);}
  /*load agent files*/{
    TTokenizer tokenizer(option("agent"));
    string agent_file;
    while(!tokenizer.EOL())
    {
      tokenizer.ReadSeparators();
      agent_file= tokenizer.ReadNonSeparators();
      if (!LoadAgentFromFile(agent,agent_file))
        {LERROR("failed to read "<<agent_file); lexit(df);}
    }
  }

  MManualLearningManager &lmanager = agent.ModuleAs<MManualLearningManager>("lmanager");
  MBioloidEnvironment &environment = agent.ModuleAs<MBioloidEnvironment>("environment");


  if (ConvertFromStr<bool>(option("available_mods","false")))
  {
    LMESSAGE("TModuleManager::ShowAllModules():");
    TModuleManager::ShowAllModules();
    return 0;
  }
  if (ConvertFromStr<bool>(option("show_mods","false")))
  {
    LMESSAGE("agent's modules:");
    agent.ShowAllModules("",cout);
    return 0;
  }
  if (ConvertFromStr<bool>(option("dot_mod","false")))
  {
    std::ofstream mdot((OutDir+"rlbioloid-modules.dot").c_str());
    agent.ExportToDOT(mdot);
    return 0;
  }
  if (ConvertFromStr<bool>(option("show_connect","false")))
  {
    LMESSAGE("agent's connections:");
    agent.ShowAllConnections(cout);
    return 0;
  }

  std::ofstream debug;
  if (ConvertFromStr<bool>(option("dump_debug","false")))
  {
    debug.open((OutDir+"rlbioloid-debug.dat").c_str());
    agent.SetDebugStream (debug);
    agent.SetAllModuleMode (TModuleInterface::mmDebug);
  }

  SaveAgentToFile (agent, OutDir+"rlbioloid-before.agent");

  // setup the agent --]

  PtrAgent= &agent;
  PtrEnvironment= &environment;

  {
    stringstream optss;
    if (option("help")!="")
      {cerr<<"valid options:"<<endl; option.PrintUsed(); return 0;}
    if (option.PrintNotAccessed(optss))
      {cerr<<"invalid options:"<<endl<<optss.str(); return 1;}
  }

  //////////////////////////////////////////////////////
  // [-- learning

  lmanager.Initialize();

  double time(0.0), time_offset(0.0), sleep_time(0.0);
  while(Executing)
  {
    // if (ForceSetup)  environment.Setup();

    LMESSAGE("can you start learning?");
    while (!AskYesNo());
    LMESSAGE("rlbioloid: learning...");

    lmanager.StartEpisode();

    time= GetCurrentTime();
    time_offset= time;
    while(Executing)
    {
      environment.StartTimeStep();

      sleep_time= time + environment.TimeStep() - GetCurrentTime();
      if(sleep_time>0.0)  usleep(static_cast<int>(sleep_time*1.0e+6));
      else  LDEBUG("at "<<time-time_offset<<"[s]: sleep_time="<<sleep_time);

      time= GetCurrentTime();
      environment.FinishTimeStep();
      if(!lmanager.IsInEpisode())  break;
    }
    if(!lmanager.IsLearning())  break;
  }

  // learning --]
  //////////////////////////////////////////////////////

  FinishLearningProc();

  return ExitCode;
}
//-------------------------------------------------------------------------------------------



