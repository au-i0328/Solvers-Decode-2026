package org.firstinspires.ftc.teamcode.opmode_new;

/**
 * Legacy constants - parameters moved to TuningConfig.
 * This file is kept for backward compatibility but TuningConfig should be used instead.
 */
public class NewConstants {

    public enum AllianceColor {
        BLUE(1),
        RED(-1);

        private final int multiplier;

        AllianceColor(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;

    // All other constants moved to TuningConfig
    // See TuningConfig.java for tunable parameters
}
