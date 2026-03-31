//! Statistics related wrappers.
use crate::mujoco_c::*;

/***********************************************************************************************************************
** MjWarningStat
***********************************************************************************************************************/
/// Per-warning type statistics (number of warnings, last info integer from the most recent warning).
pub type MjWarningStat = mjWarningStat;

/***********************************************************************************************************************
** MjTimerStat
***********************************************************************************************************************/
/// Per-timer statistics (duration and call count).
pub type MjTimerStat = mjTimerStat;

/***********************************************************************************************************************
** MjSolverStat
***********************************************************************************************************************/
/// Per-iteration solver statistics (improvement, gradient, lineslope, active constraint count, etc.).
pub type MjSolverStat = mjSolverStat;
