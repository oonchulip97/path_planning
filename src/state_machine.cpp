#include "state_machine.h"

StateMachine::StateMachine()
  : states_(KEEP_LANE)
{}

StateMachine::StateMachine(state)
  : states_(state)
{}

StateMachine:: ~StateMachine()
{}

states get_state()
{
  return state_;
}

std::vector<StateMachine> StateMachine::get_successor_states();
{
  // All states progress to KEEP_LANE state
  std::vector<StateMachine> successor_states;
  successor_states.push_back(StateMachine());

  if (states_ == KEEP_LANE)
  {
    successor_states.push_back(StateMachine(PREPARE_LANE_CHANGE_LEFT));
    successor_states.push_back(StateMachine(PREPARE_LANE_CHANGE_RIGHT));
  }
  else if (states_ == PREPARE_LANE_CHANGE_LEFT)
  {
    successor_states.push_back(StateMachine(PREPARE_LANE_CHANGE_LEFT));
    successor_states.push_back(StateMachine(LANE_CHANGE_LEFT));
  }
  else if (states_ == PREPARE_LANE_CHANGE_RIGHT)
  {
    successor_states.push_back(StateMachine(PREPARE_LANE_CHANGE_RIGHT));
    successor_states.push_back(StateMachine(LANE_CHANGE_RIGHT));
  }

  // If states are LANE_CHANGE_*,
  // next state is only KEEP_LANE
  return successor_states;
}
