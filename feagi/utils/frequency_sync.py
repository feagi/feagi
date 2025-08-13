"""Frequency synchronization utility for FEAGI.

This module provides functionality to synchronize FQ sampler frequencies with
brain burst frequency to ensure consistent data streaming rates.
"""

from typing import Any, Dict, Optional

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class FrequencySynchronizer:
    """Synchronizes FQ sampler frequencies with brain burst frequency.

    This ensures that data streaming to bridges and visualization tools matches
    the brain's actual processing frequency, preventing confusing mismatches
    between brain activity and data transmission rates.
    """

    def __init__(self, process_manager=None, burst_engine=None):
        """Initialize the frequency synchronizer.

        Args:
            process_manager: ProcessManager instance containing FQ samplers
            burst_engine: BurstEngine instance for frequency info
        """
        self.process_manager = process_manager
        self.burst_engine = burst_engine
        self.logger = logger

    def sync_all_samplers_to_brain_frequency(self) -> bool:
        """Synchronize all FQ samplers to match current brain burst frequency.

        Returns:
            True if synchronization was successful, False otherwise
        """
        if not self.process_manager or not self.burst_engine:
            self.logger.error("FrequencySynchronizer not properly initialized")
            return False

        try:
            # Get current brain frequency
            brain_frequency = self.get_brain_frequency()
            if brain_frequency is None:
                self.logger.error("Could not determine brain frequency")
                return False

            self.logger.info(
                f"🔄 Synchronizing FQ samplers to brain frequency: {brain_frequency}Hz"
            )

            # Sync visualization sampler
            viz_success = self.sync_visualization_sampler(brain_frequency)

            # Sync motor sampler
            motor_success = self.sync_motor_sampler(brain_frequency)

            if viz_success or motor_success:
                self.logger.info(
                    "✅ FQ sampler frequency synchronization completed"
                )
                return True
            else:
                self.logger.warning("⚠️  No FQ samplers were synchronized")
                return False

        except Exception as e:
            self.logger.error(f"Error during frequency synchronization: {e}")
            return False

    def sync_visualization_sampler(self, target_frequency: float) -> bool:
        """Synchronize visualization sampler to target frequency.

        Args:
            target_frequency: Target frequency in Hz

        Returns:
            True if synchronization was successful, False otherwise
        """
        try:
            viz_sampler = self.process_manager.get_viz_fq_sampler()
            if viz_sampler is None:
                self.logger.debug("No visualization FQ sampler to synchronize")
                return False

            current_freq = viz_sampler.sample_frequency
            self.logger.info(
                f"🎨 Updating visualization sampler: {current_freq}Hz → {target_frequency}Hz"
            )

            viz_sampler.set_sample_frequency(target_frequency)
            return True

        except Exception as e:
            self.logger.error(
                f"Error synchronizing visualization sampler: {e}"
            )
            return False

    def sync_motor_sampler(self, target_frequency: float) -> bool:
        """Synchronize motor sampler to target frequency.

        Args:
            target_frequency: Target frequency in Hz

        Returns:
            True if synchronization was successful, False otherwise
        """
        try:
            motor_sampler = self.process_manager.get_motor_fq_sampler()
            if motor_sampler is None:
                self.logger.debug("No motor FQ sampler to synchronize")
                return False

            current_freq = motor_sampler.sample_frequency
            self.logger.info(
                f"🚗 Updating motor sampler: {current_freq}Hz → {target_frequency}Hz"
            )

            motor_sampler.set_sample_frequency(target_frequency)
            return True

        except Exception as e:
            self.logger.error(f"Error synchronizing motor sampler: {e}")
            return False

    def get_brain_frequency(self) -> Optional[float]:
        """Get the current brain burst frequency from STATE MANAGER
        (authoritative source).

        Returns:
            Brain frequency in Hz, or None if not available
        """
        try:
            # STATE MANAGER is the single source of truth
            from feagi.core.state_manager import FeagiStateManager

            state_manager = FeagiStateManager.instance()
            frequency = state_manager.get_burst_frequency()

            if frequency and frequency > 0:
                return frequency
            else:
                self.logger.warning(
                    f"Invalid frequency from state manager: {frequency}Hz - trying burst engine fallback"
                )
                # Emergency fallback to burst engine
                if hasattr(self.burst_engine, "desired_frequency"):
                    return self.burst_engine.desired_frequency
                elif hasattr(self.burst_engine, "get_frequency_config"):
                    config = self.burst_engine.get_frequency_config()
                    return config.get("current_frequency_hz")
                else:
                    self.logger.warning(
                        "Could not determine brain frequency from any source"
                    )
                    return None

        except Exception as e:
            self.logger.error(f"Error getting brain frequency: {e}")
            return None

    def get_sampler_frequencies(self) -> Dict[str, Optional[float]]:
        """Get current frequencies of all FQ samplers.

        Returns:
            Dictionary with current sampler frequencies
        """
        frequencies = {}

        try:
            # Get visualization sampler frequency
            viz_sampler = self.process_manager.get_viz_fq_sampler()
            frequencies["visualization"] = (
                viz_sampler.sample_frequency if viz_sampler else None
            )

            # Get motor sampler frequency
            motor_sampler = self.process_manager.get_motor_fq_sampler()
            frequencies["motor"] = (
                motor_sampler.sample_frequency if motor_sampler else None
            )

            # Get brain frequency for comparison
            frequencies["brain"] = self.get_brain_frequency()

        except Exception as e:
            self.logger.error(f"Error getting sampler frequencies: {e}")

        return frequencies

    def check_frequency_mismatch(self) -> Dict[str, Any]:
        """Check for frequency mismatches between brain and samplers.

        Returns:
            Dictionary with mismatch information
        """
        frequencies = self.get_sampler_frequencies()
        brain_freq = frequencies.get("brain")

        mismatches = {
            "brain_frequency": brain_freq,
            "mismatches": [],
            "needs_sync": False,
        }

        if brain_freq is None:
            mismatches["error"] = "Cannot determine brain frequency"
            return mismatches

        # Check visualization sampler
        viz_freq = frequencies.get("visualization")
        if viz_freq is not None and abs(viz_freq - brain_freq) > 0.1:
            mismatches["mismatches"].append(
                {
                    "type": "visualization",
                    "current": viz_freq,
                    "target": brain_freq,
                    "difference": viz_freq - brain_freq,
                }
            )
            mismatches["needs_sync"] = True

        # Check motor sampler
        motor_freq = frequencies.get("motor")
        if motor_freq is not None and abs(motor_freq - brain_freq) > 0.1:
            mismatches["mismatches"].append(
                {
                    "type": "motor",
                    "current": motor_freq,
                    "target": brain_freq,
                    "difference": motor_freq - brain_freq,
                }
            )
            mismatches["needs_sync"] = True

        return mismatches


# Convenience function for easy use
def sync_fq_samplers_to_brain_frequency(process_manager, burst_engine) -> bool:
    """Convenience function to synchronize all FQ samplers to brain frequency.

    Args:
        process_manager: ProcessManager instance
        burst_engine: BurstEngine instance

    Returns:
        True if synchronization was successful, False otherwise
    """
    synchronizer = FrequencySynchronizer(process_manager, burst_engine)
    return synchronizer.sync_all_samplers_to_brain_frequency()


def check_frequency_mismatches(
    process_manager, burst_engine
) -> Dict[str, Any]:
    """Convenience function to check for frequency mismatches.

    Args:
        process_manager: ProcessManager instance
        burst_engine: BurstEngine instance

    Returns:
        Dictionary with mismatch information
    """
    synchronizer = FrequencySynchronizer(process_manager, burst_engine)
    return synchronizer.check_frequency_mismatch()
