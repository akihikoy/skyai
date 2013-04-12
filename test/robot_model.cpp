//-------------------------------------------------------------------------------------------
/*! \file    robot_model.cpp
    \brief   skyai - certain application
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.21, 2013

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
#include <lora/robot_model.h>
#include <lora/ode_ds.h>
#include <lora/string.h>
#include <lora/stl_math.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace loco_rabbits;
using namespace xode;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
#define print(var) std::cerr<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------


TWorld world;

void SimStart()
{
  world.Start();
}

void SimLoop(int pause)
{
  world.Step(pause);
}

void StartOfTimeStep(TWorld &w, const TReal &time_step)
{
  static int ridx= w.RobotIndex("Humanoid01");
  if(ridx<0)  return;
  std::vector<TReal> angles(w.JointAngleNum(ridx));
  w.GetJointAngles(ridx,angles.begin(),angles.end());
  // PrintContainer(angles,"");

  std::vector<TReal> angvels(w.JointAngVelNum(ridx));
  w.GetJointAngVels(ridx,angvels.begin(),angvels.end());

  std::vector<TReal> targets(w.JointAngleNum(ridx),0.0);

  std::vector<TReal> torques(w.JointTorqueInputNum(ridx));
  torques= 2.5l*(targets-angles) - 0.08l*angvels;
  w.AddToJointTorques(ridx,torques.begin(),torques.end());
  // PrintContainer(torques,"");
}
void EndOfTimeStep(TWorld &w, const TReal &time_step)
{
  static int ridx= w.RobotIndex("Humanoid01");
  if(ridx<0)  return;

  const std::vector<bool> &c= w.LinkContacts(ridx);
  PrintContainer(c,"");

  // std::vector<TReal> f(w.ForceObservationNum(ridx),0.0l);
  // w.GetForces(ridx,f.begin(),f.end());
  // PrintContainer(f,"");
}

void KeyEvent(int command)
{
  if(command=='r' || command=='R')
  {
    world.Create();
  }
}

int main(int argc, char**argv)
{
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &SimStart;
  fn.step = &SimLoop;
  fn.command = &KeyEvent;
  fn.stop = 0;
  fn.path_to_textures = "textures";

  dInitODE2(0);

  world.SetCallbacks().StartOfTimeStep= &StartOfTimeStep;
  world.SetCallbacks().EndOfTimeStep= &EndOfTimeStep;
  if(argc>1)  world.LoadFromFile(argv[1]);
  world.Create();
  print(world.TotalMass("Humanoid01"));

  dsSimulationLoop (argc,argv,400,400,&fn);

  dCloseODE();

  // world.SaveToFile("result.dat");

  return 0;
}
//-------------------------------------------------------------------------------------------
