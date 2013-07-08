//-------------------------------------------------------------------------------------------
/*! \file    libbioloid.h
    \brief   benchmarks - motion learning task of a real robot (Bioloid, ROBOTIS)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.04, 2010

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
#ifndef libbioloid_h
#define libbioloid_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <skyai/modules_core/univ_task.h>
#include <lora/bioloid.h>
#include <lora/sys.h>
#include <lora/small_classes.h>
#include <lora/variable_space_impl.h>
#include <lora/marker_tracker.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

enum TBioloidControllerKind
  {
    bckCM5=0,
    bckUSB2Dynamixel
  };
ENUM_STR_MAP_BEGIN(TBioloidControllerKind)
  ENUM_STR_MAP_ADD(bckCM5            )
  ENUM_STR_MAP_ADD(bckUSB2Dynamixel  )
ENUM_STR_MAP_END  (TBioloidControllerKind)
SPECIALIZE_TVARIABLE_TO_ENUM(TBioloidControllerKind)

//===========================================================================================
class TBioloidEnvironmentConfigurations
//===========================================================================================
{
public:

  TString                 SerialPort;
  TBioloidControllerKind  BioloidControllerKind;  //!< use bckCM5 for CM-5 (or CM-500)
  TIntVector        ActuatorIndexes;
  TIntVector        SensingAngleIndexes;
  TInt              DistanceSensorIndex;
  TRealVector       AngleMax;
  TRealVector       AngleMin;

  TBool             UsingVirtualAngles;

  TReal             InitSleepTime;
  TReal             TimeStep;
  TReal             UserPenalty;


  TBioloidEnvironmentConfigurations (var_space::TVariableMap &mmap)
    :
      SerialPort            ("/dev/ttyUSB0"),
      BioloidControllerKind (bckUSB2Dynamixel),
      DistanceSensorIndex (-1),
      UsingVirtualAngles  (false),
      InitSleepTime   (2.0l),
      TimeStep        (0.1l),
      UserPenalty     (-4.0l)
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
      ADD( UsingVirtualAngles       );
      ADD( InitSleepTime            );
      ADD( TimeStep                 );
      ADD( UserPenalty              );
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
      out_sensor_distance_diff_c  (*this),
      out_is_successful           (*this)
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
      add_out_port    (out_is_successful          );
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

  void ForcePenalty ()
    {
      signal_system_reward.ExecAll(conf_.UserPenalty);
    }

  void SetMissionSuccess (bool ms)
    {
      is_successful_= ms;
    }

  void Setup ()
    {
      LMESSAGE("setup robot..");
      switch(conf_.BioloidControllerKind)
      {
      case bckCM5:
        LMESSAGE("using CM5 controller.");
        bioloid_.Connect(conf_.SerialPort.c_str());
        bioloid_.TossMode();
        break;
      case bckUSB2Dynamixel:
        LMESSAGE("using USB2Dynamixel controller.");
        bioloid_.ConnectBS(conf_.SerialPort.c_str());
        break;
      default:
        LERROR("Invalid BioloidControllerKind: "<<static_cast<int>(conf_.BioloidControllerKind));
        lexit(df);
      }

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

  mutable TBool   is_successful_;

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

  //! whether an episode is success or not, given by an operator
  MAKE_OUT_PORT(out_is_successful, const TBool&, (void), (), TThis);


  virtual void slot_initialize_exec (void)
    {
      Setup();
      // executing_= true;
    }

  virtual void slot_start_episode_exec (void)
    {
      Setup();

      tmp_distance_c_.resize(3,0.0);
      tmp_old_distance_c_.resize(3,0.0);
      tmp_distance_diff_c_.resize(3,0.0);
      is_successful_= false;
      LMESSAGE("resetting robot..");
      TRealVector  target_angle(conf_.ActuatorIndexes.size(),0.0), observed_angle(conf_.ActuatorIndexes.size());
      double time_offset(GetCurrentTime()), time(GetCurrentTime());
      while (time-time_offset<conf_.InitSleepTime)
      {
        if(!conf_.UsingVirtualAngles)
          bioloid_.GetAllAngles (conf_.ActuatorIndexes.begin(),conf_.ActuatorIndexes.end(), GenBegin(observed_angle));
        else
          bioloid_.GetVirtualAngles (conf_.ActuatorIndexes.begin(),conf_.ActuatorIndexes.end(), GenBegin(observed_angle));

        for(TypeExt<TRealVector>::iterator itr(GenBegin(tmp_angles_)); itr!=GenEnd(tmp_angles_); ++itr)
          *itr= (*itr)/180.0*M_PI;
        constrain_angles (target_angle);
        bioloid_.GoTo (conf_.ActuatorIndexes.begin(),conf_.ActuatorIndexes.end(), GenBegin(target_angle));
        if(conf_.DistanceSensorIndex>=0)
        {
          bioloid_.GetDistance(conf_.DistanceSensorIndex, 0,tmp_old_distance_c_(0));  // center sensor
          bioloid_.GetDistance(conf_.DistanceSensorIndex,-1,tmp_old_distance_c_(1));  // left sensor
          bioloid_.GetDistance(conf_.DistanceSensorIndex,+1,tmp_old_distance_c_(2));  // right sensor
        }
        usleep(10000);
        time= GetCurrentTime();
      }
      LMESSAGE("ok..");

      if(conf_.DistanceSensorIndex>=0)
      {
        bioloid_.GetDistance(conf_.DistanceSensorIndex, 0,tmp_distance_c_(0));  // center sensor
        bioloid_.GetDistance(conf_.DistanceSensorIndex,-1,tmp_distance_c_(1));  // left sensor
        bioloid_.GetDistance(conf_.DistanceSensorIndex,+1,tmp_distance_c_(2));  // right sensor
      }
      tmp_distance_diff_c_(0)= 0.0;
      tmp_distance_diff_c_(1)= 0.0;
      tmp_distance_diff_c_(2)= 0.0;

      distance_lpf_.Initialize (TLHBPFilters<TRealVector>::LPF2,
        conf_.TimeStep, 0.5/*f*/, 0.8/*q*/, TRealVector(3,0.0), tmp_distance_c_);
      distance_diff_lpf_.Initialize (TLHBPFilters<TRealVector>::LPF2,
        conf_.TimeStep, 0.5/*f*/, 0.8/*q*/, TRealVector(3,0.0), tmp_distance_diff_c_);

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
      for(int i(0);i<3;++i)  tmp_old_distance_c_(i)= distance_lpf_()(i); // tmp_distance_c_(i);
      if(conf_.DistanceSensorIndex>=0)
      {
        bioloid_.GetDistance(conf_.DistanceSensorIndex, 0,tmp_distance_c_(0));  // center sensor
        bioloid_.GetDistance(conf_.DistanceSensorIndex,-1,tmp_distance_c_(1));  // left sensor
        bioloid_.GetDistance(conf_.DistanceSensorIndex,+1,tmp_distance_c_(2));  // right sensor
        distance_lpf_ (tmp_distance_c_);  // tmp_distance_c_(0)= distance_lpf_()(0);
        for(int i(0);i<3;++i)  tmp_distance_diff_c_(i)= -1.0*(distance_lpf_()(i)-tmp_old_distance_c_(i));
        distance_diff_lpf_ (tmp_distance_diff_c_);
std::cout<<distance_lpf_()(0)<<"\t"<<distance_lpf_()(1)<<"\t"<<distance_lpf_()(2)<<std::endl;
      }

      TSingleReward reward(0.0l);
      reward+= -0.15l*conf_.TimeStep;

      signal_system_reward.ExecAll(reward);

      signal_end_of_timestep.ExecAll(conf_.TimeStep);
    }

  virtual const TRealVector& out_sensor_angles_get () const
    {
      tmp_angles_.resize(conf_.SensingAngleIndexes.size());
      if(!conf_.UsingVirtualAngles)
        bioloid_.GetAllAngles (conf_.SensingAngleIndexes.begin(),conf_.SensingAngleIndexes.end(), GenBegin(tmp_angles_));
      else
        bioloid_.GetVirtualAngles (conf_.SensingAngleIndexes.begin(),conf_.SensingAngleIndexes.end(), GenBegin(tmp_angles_));

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
  virtual const TBool& out_is_successful_get () const
    {
      return is_successful_;
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
  TReal                    RewardGain;
  TSingleReward            SumOfRmin;  //!< if sum of reward in an episode is less than this value, episode is terminated
  TContinuousTime          MaxTime;

  TReal                    TimeRewardGain;  //!< reward given for episode success; max(0, TimeRewardGain * (1.0l-time_/TimeRewardParam))
  TReal                    TimeRewardParam; //!< reward given for episode success; max(0, TimeRewardGain * (1.0l-time_/TimeRewardParam))

  TMotionLearningTaskConfigurations (var_space::TVariableMap &mmap)
    :
      TaskKind             (mltkMove),
      RewardGain           (1.0l),
      SumOfRmin            (-40.0l),
      MaxTime              (50.0l),
      TimeRewardGain       (3.0l),
      TimeRewardParam      (20.0l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( TaskKind        );
      ADD( RewardGain      );
      ADD( SumOfRmin       );
      ADD( MaxTime         );
      ADD( TimeRewardGain  );
      ADD( TimeRewardParam );
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
      in_sum_of_reward       (*this),
      in_is_successful       (*this)
    {
      add_slot_port   (slot_initialize       );
      add_slot_port   (slot_start_episode    );
      add_slot_port   (slot_finish_time_step );
      add_signal_port (signal_end_of_episode );
      add_signal_port (signal_task_reward    );
      add_signal_port (signal_damage_reward  );
      add_in_port     (in_speed              );
      add_in_port     (in_sum_of_reward      );
      add_in_port     (in_is_successful      );
    }

protected:

  TMotionLearningTaskConfigurations  conf_;

  TContinuousTime  time_;

  bool  time_reward_emitted_;

  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_finish_time_step, void, (const TContinuousTime &dt), (dt), TThis);

  MAKE_SIGNAL_PORT(signal_end_of_episode, void (void), TThis);

  MAKE_SIGNAL_PORT(signal_task_reward, void (const TSingleReward&), TThis);
  MAKE_SIGNAL_PORT(signal_damage_reward, void (const TSingleReward&), TThis);

  //!\brief input the speed of the robot
  MAKE_IN_PORT(in_speed, const TReal& (void), TThis);

  MAKE_IN_PORT(in_sum_of_reward, const TSingleReward& (void), TThis);

  //!\brief input whether an episode is success or not, given by an operator
  MAKE_IN_PORT(in_is_successful, const TBool& (void), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(speed, const TReal&, (void), ())

  GET_FROM_IN_PORT(sum_of_reward, const TSingleReward&, (void), ())

  #undef GET_FROM_IN_PORT

  bool get_is_successful() const
    {
      if (in_is_successful.ConnectionSize()==0)  return false;
      return in_is_successful.GetFirst();
    }

  virtual void slot_initialize_exec (void)
    {
    }

  virtual void slot_start_episode_exec (void)
    {
      time_= 0.0l;
      time_reward_emitted_= false;
    }

  virtual void slot_finish_time_step_exec (const TContinuousTime &dt)
    {
      time_+=dt;
      bool is_successful(get_is_successful());

      // calculate task reward
      {
        TSingleReward  task_reward(0.0l);
        switch (conf_.TaskKind)
        {
          case mltkMove        :
            /*!Goal Reward def.*/
            // task_reward= body[BASELINK_INDEX].getLinearVel()[0];
            // task_reward= 0.01l*task_reward - 0.1l*Square(body[BASELINK_INDEX].getPosition()[1]);
            task_reward= conf_.RewardGain * get_speed();
            if (is_successful && !time_reward_emitted_ && time_ < conf_.TimeRewardParam)
            {
              TSingleReward time_reward(conf_.TimeRewardGain * (1.0l-time_/conf_.TimeRewardParam));
              task_reward+= time_reward;
              LMESSAGE("episode success: time reward ("<<time_reward<<") is added to reward");
              time_reward_emitted_= true;
            }
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
        if (is_end_of_episode || is_successful)
        {
          signal_end_of_episode.ExecAll();
        }
      }
    }

};  // end of MMotionLearningTask
//-------------------------------------------------------------------------------------------


namespace var_space{
  void Register (marker_tracker::TMarkerTrackerConfig &x, TVariableMap &mmap);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Marker tracker module
class MMarkerTracker
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface   TParent;
  typedef MMarkerTracker     TThis;
  SKYAI_MODULE_NAMES(MMarkerTracker)

  MMarkerTracker (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      thread_unobserved_count_(0),
      thread_               (NULL),
      thread_running_       (false),
      slot_initialization   (*this),
      slot_step             (*this),
      slot_finish           (*this),
      out_unobserved_count  (*this),
      out_pos               (*this),
      out_rot               (*this),
      out_vel               (*this)
    {
      mtracker_.Config().PrintResult= false;
      var_space::Register(mtracker_.Config(),TParent::param_box_config_map());

      add_slot_port (slot_initialization   );
      add_slot_port (slot_step             );
      add_slot_port (slot_finish           );
      add_out_port  (out_unobserved_count  );
      add_out_port  (out_pos               );
      add_out_port  (out_rot               );
      add_out_port  (out_vel               );
    }

  ~MMarkerTracker()
    {
      clear_thread();
    }

protected:

  marker_tracker::TMarkerTracker mtracker_;

  int thread_unobserved_count_;

  mutable TRealVector thread_pos_;
  mutable TRealMatrix thread_rot_;
  mutable TRealVector thread_vel_;

  boost::thread *thread_;
  mutable boost::mutex mutex_;
  bool thread_running_;


  MAKE_SLOT_PORT(slot_initialization, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_step, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_finish, void, (void), (), TThis);

  //!\brief output number of consecutive observation failure
  MAKE_OUT_PORT(out_unobserved_count, const TInt&, (void), (), TThis);

  //!\brief output marker position
  MAKE_OUT_PORT(out_pos, const TRealVector&, (void), (), TThis);
  //!\brief output marker rotation matrix[3x3]
  MAKE_OUT_PORT(out_rot, const TRealMatrix&, (void), (), TThis);
  //!\brief output marker velocities (of position and rotation)
  MAKE_OUT_PORT(out_vel, const TRealVector&, (void), (), TThis);


  virtual void slot_initialization_exec (void)
    {
      clear_thread();

      initialize_mtracker();

      thread_running_= true;
      thread_= new boost::thread(boost::bind(&MMarkerTracker::run,this));
    }

  virtual void slot_step_exec (void)
    {
      boost::mutex::scoped_lock lock(mutex_);
      if(thread_unobserved_count_>0)
        LMESSAGE("unobserved count: "<<thread_unobserved_count_);
    }

  virtual void slot_finish_exec (void)
    {
      clear_thread();
      mtracker_.Clear();
    }

  virtual const TInt& out_unobserved_count_get (void) const
    {
      boost::mutex::scoped_lock lock(mutex_);
      return thread_unobserved_count_;
    }

  virtual const TRealVector& out_pos_get (void) const
    {
      boost::mutex::scoped_lock lock(mutex_);
      return thread_pos_;
    }

  virtual const TRealMatrix& out_rot_get (void) const
    {
      boost::mutex::scoped_lock lock(mutex_);
      return thread_rot_;
    }

  virtual const TRealVector& out_vel_get (void) const
    {
      boost::mutex::scoped_lock lock(mutex_);
      return thread_vel_;
    }

  void initialize_mtracker()
    {
      const std::string tmp_file_name(mtracker_.Config().MarkerFileName);
      std::string  marker_file_name(Agent().SearchFileName(mtracker_.Config().MarkerFileName));
      if (marker_file_name=="")
        {LERROR("Marker file "<<mtracker_.Config().MarkerFileName<<" does not exist!"); lexit(df);}
      mtracker_.Config().MarkerFileName= marker_file_name;
      mtracker_.Initialize();
      mtracker_.Config().MarkerFileName= tmp_file_name;
      LMESSAGE("info: marker-tracker can be reset by pressing 'R' on the window");
    }

  void clear_thread()
    {
      {
        boost::mutex::scoped_lock lock(mutex_);
        thread_running_= false;
      }
      if(thread_)
      {
        thread_->join();
        delete thread_;
      }
      thread_= NULL;
    }

  void run()
    {
      bool running(thread_running_);
      int unobserved_count(0);
      while(running)
      {
        if(mtracker_.Step())
        {
          if(!mtracker_.Observed())
            ++unobserved_count;
          else if(mtracker_.EstimatedObservation().C[0]<0 || mtracker_.EstimatedObservation().C[0]>mtracker_.ImageWidth()
            || mtracker_.EstimatedObservation().C[1]<0 || mtracker_.EstimatedObservation().C[1]>mtracker_.ImageHeight())
            ++unobserved_count;
          else
            unobserved_count= 0;
        }
        else
          ++unobserved_count;

        {
          // locked copy process
          boost::mutex::scoped_lock lock(mutex_);
          running= thread_running_;
          thread_unobserved_count_= unobserved_count;

          const marker_tracker::TParticle &p(mtracker_.EstimatedState());

          thread_pos_.resize(3);
          std::copy(CVBegin(p.C),CVEnd(p.C),OctBegin(thread_pos_));

          thread_rot_.resize(3,3);
          cv::Matx<double,3,3> Rt= p.R.t();
          std::copy(CVBegin(Rt),CVEnd(Rt),OctBegin(thread_rot_));

          thread_vel_.resize(6);
          std::copy(CVBegin(p.V),CVEnd(p.V),OctBegin(thread_vel_));
          std::copy(CVBegin(p.W),CVEnd(p.W),OctBegin(thread_vel_)+3);
        }

        if((mtracker_.Key()&0xFF) =='R')
        {
          LMESSAGE("resetting marker-tracker..");
          initialize_mtracker();
          LMESSAGE("info: marker-tracker can be reset by pressing 'R' on the window");
        }

        usleep(1000);
      }
    }

};  // end of MMarkerTracker
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TBioloidUnivTaskConfigurations
//===========================================================================================
{
public:

  //! forwarding the system reward given from the environment module
  TBool   ForwardSystemReward;

  TBioloidUnivTaskConfigurations (var_space::TVariableMap &mmap)
    :
      ForwardSystemReward  (true)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( ForwardSystemReward );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TBioloidUnivTaskMemory
//===========================================================================================
{
public:

  /*! the state of the robot is stored into following variables.
      these variables are assumed to be used in user-defined functions, */
  TRealVector  BasePos;
  TRealVector  BaseVel;
  TRealMatrix  BaseRot;
  TInt         UnobservedCount;

  TBioloidUnivTaskMemory (var_space::TVariableMap &mmap)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( BasePos           );
      ADD( BaseVel           );
      ADD( BaseRot           );
      ADD( UnobservedCount   );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief universal task module for bioloid environment
class MBioloidUnivTask
    : public MUniversalContTimeTask
//===========================================================================================
{
public:
  typedef MUniversalContTimeTask  TParent;
  typedef MBioloidUnivTask        TThis;
  SKYAI_MODULE_NAMES(MBioloidUnivTask)

  MBioloidUnivTask (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      bconf_         (TParent::param_box_config_map()),
      bmem_          (TParent::param_box_memory_map()),
      slot_system_reward     (*this),
      in_base_pos            (*this),
      in_base_vel            (*this),
      in_base_rot            (*this),
      in_unobserved_count    (*this)
    {
      add_slot_port   (slot_system_reward     );
      add_in_port     (in_base_pos            );
      add_in_port     (in_base_vel            );
      add_in_port     (in_base_rot            );
      add_in_port     (in_unobserved_count    );
    }

protected:

  TBioloidUnivTaskConfigurations  bconf_;
  TBioloidUnivTaskMemory          bmem_;

  MAKE_SLOT_PORT(slot_system_reward, void, (const TSingleReward &r), (r), TThis);

  MAKE_IN_PORT(in_base_pos        , const TRealVector& (void), TThis);
  MAKE_IN_PORT(in_base_vel        , const TRealVector& (void), TThis);
  MAKE_IN_PORT(in_base_rot        , const TRealMatrix& (void), TThis);
  MAKE_IN_PORT(in_unobserved_count, const TInt& (void), TThis);

  virtual void slot_system_reward_exec (const TSingleReward &r)
    {
      if(bconf_.ForwardSystemReward)  signal_reward.ExecAll(r);
    }

  override void sense_common()
    {
      if (in_base_pos        .ConnectionSize()!=0)  bmem_.BasePos          = in_base_pos        .GetFirst();
      if (in_base_vel        .ConnectionSize()!=0)  bmem_.BaseVel          = in_base_vel        .GetFirst();
      if (in_base_rot        .ConnectionSize()!=0)  bmem_.BaseRot          = in_base_rot        .GetFirst();
      if (in_unobserved_count.ConnectionSize()!=0)  bmem_.UnobservedCount  = in_unobserved_count.GetFirst();
    }

};  // end of MBioloidUnivTask
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // libbioloid_h
//-------------------------------------------------------------------------------------------
