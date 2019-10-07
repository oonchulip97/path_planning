#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <vector>

enum states
{
  KEEP_LANE,
  PREPARE_LANE_CHANGE_LEFT,
  PREPARE_LANE_CHANGE_RIGHT,
  LANE_CHANGE_LEFT,
  LANE_CHANGE_RIGHT
};

class StateMachine
{
public:
  /**
   * Default constructor.
   */
  StateMachine();

  /**
   * Constructor using given state.
   */
  StateMachine(states state);

  /**
   * Destructor.
   */
  ~StateMachine();

  /**
   * Getter function.
   *
   * @return current state
   */
  states get_state();

  /**
   * Return next states given current state.
   *
   * @return a vector of State Machine containing next states.
   */
  std::vector<StateMachine> get_successor_states();

private:
  states state_;  // current state of the State Machine
}

#endif  // STATE_MACHINE_H
