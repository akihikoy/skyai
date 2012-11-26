use_skyai(skyai)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_executable (agent_talker_mz.out agent_talker.cpp)
target_link_libraries (agent_talker_mz.out maze2d ${SKYAI_LIBRARIES})
install (TARGETS agent_talker_mz.out DESTINATION bin/skyai/agent_talker)

use_skyai(skyai ode)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_executable (agent_talker_hm.out agent_talker.cpp)
target_link_libraries (agent_talker_hm.out humanoid01 ${SKYAI_LIBRARIES})
install (TARGETS agent_talker_hm.out DESTINATION bin/skyai/agent_talker)


# for agent_talker_bio
find_package(Boost COMPONENTS
  thread
  REQUIRED)
if(NOT Boost_THREAD_FOUND)
  message(WARNING, " Boost-thread library is not found." )
endif()

use_skyai(skyai cv)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_executable (agent_talker_bio.out agent_talker.cpp)
target_link_libraries (agent_talker_bio.out bioloid ${SKYAI_LIBRARIES})
install (TARGETS agent_talker_bio.out DESTINATION bin/skyai/agent_talker)
