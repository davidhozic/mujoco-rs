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
/// Per-timer statistics: the duration accumulated over all calls, and the call count. The duration
/// stays zero unless an [`mjcb_time`] callback is installed; its unit is the unit that callback
/// returns.
pub type MjTimerStat = mjTimerStat;

/***********************************************************************************************************************
** MjSolverStat
***********************************************************************************************************************/
/// Per-iteration solver statistics (improvement, gradient, lineslope, active constraint count, etc.).
pub type MjSolverStat = mjSolverStat;
