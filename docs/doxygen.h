/** @file doxygen.h
 * @brief Document namespaces here so they show up in doxygen
 *
 * Doxygen doesn't like to document functions in namespaces that aren't
 * documented, so it would probably be better to document the namespaces
 * in a seperate file dedicated to documenting these namespaces rather than
 * documenting them in random files where they happen to show up
 */

/**
 * @brief the master namespace of librmb
 *
 * This is where all of the librmb specific functions and classes are
 * placed for your use
 */
namespace rmb {

/**
 * @brief namespace for trajectory calculations
 *
 * This namespace contains functionality that allows you to calculate
 * trajectories based on input criteria
 */
namespace trajectory {}
} // namespace rmb

/**
 * @brief WPILib namespace
 *
 * Generally this namespace is used to overload and add to existing wpilib
 * functionality.
 */
namespace wpi {}