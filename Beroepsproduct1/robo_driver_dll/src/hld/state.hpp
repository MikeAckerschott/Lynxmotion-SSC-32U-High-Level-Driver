#ifndef STATE_HPP_
#define STATE_HPP_

#include <vector>
#include "context.hpp"

// Forward declaration of Context class
class Context;

class State
{

protected:
  /**
   * @brief the context that is used to transition between states
   */
  Context *context_;

public:
  /**
   * @brief Destructor
   */
  virtual ~State();

  /**
   * @brief Sets the context
   * @param context The context
   */
  void set_context(Context *context);

  /**
   * @brief Checks if any trigger is activated
   * @return True if the state should be exited, false if not
   */
  virtual bool checkAllTriggers() = 0;

  /**
   * @brief function that is called when the state is entered
   */
  virtual void f_entry() = 0;

  /**
   * @brief function that is called as long as the state is active
   */
  virtual void f_do() = 0;

  /**
   * @brief function that is called when the state is exited
   */
  virtual void f_exit() = 0;

private:
};

#endif /* BUTTON_HPP_ */