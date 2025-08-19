#ifndef COOKING_PLATE_PLACE_BEHAVIOR_HPP
#define COOKING_PLATE_PLACE_BEHAVIOR_HPP

#include "dhtt_cooking_plugins/behaviors/cooking_place_behavior.hpp"

namespace dhtt_cooking_plugins
{
/**
 * Special case of Place behavior for interacting with plates that sends two interact commands. This
 * may not work on a plate dispenser!
 *
 * The precondition is to already have a move base resource and a gripper (holding either the plate
 * or the object to put on the plate).
 *
 * The postcondition is releasing the gripper and retaining nothing. The plate and the object
 * interacting with will be placed back on the counter (may not work on a plate dispenser).
 *
 * As a quirk in cooking_zoo, when the agent is interacting with a plate (holding a plate, or
 * interaction location contains a plate), the plate is always picked up. This class sets the
 * pre-post conditions correctly, otherwise, 'placing' on/with a plate would incorrectly free the
 * arm resource. Likewise, 'picking' on/with a plate may be incorrectly ruled impossible because the
 * arm resource is already in use.
 */
class CookingPlatePlaceBehavior : public CookingPlaceBehavior
{
  public:
	/**
	 * First move_to the target location (to set the orientation), then send two interact commands.
	 *
	 * If placing on a deliversquare, send a NOP so it clears.
	 *
	 * In the special case that we are placing on a deliversquare that is marked with our taint,
	 * unmark it to free it up for other behaviors. Normally the pick behavior is the only one to
	 * unmark static objects, but deliversquares 'pick' themselves.
	 *
	 * @param container unused
	 */
	void do_work(dhtt::Node *container) override;

  protected:
  private:
};
} // namespace dhtt_cooking_plugins

#endif // COOKING_PLATE_PLACE_BEHAVIOR_HPP
