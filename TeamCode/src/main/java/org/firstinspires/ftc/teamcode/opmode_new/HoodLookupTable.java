package org.firstinspires.ftc.teamcode.opmode_new;

import static org.firstinspires.ftc.teamcode.opmode_new.TuningConfig.*;

/**
 * Linear interpolation lookup table for hood angle based on distance.
 * 
 * HOW TO CALIBRATE:
 * 1. Set robot at known distances from goal (measure in inches)
 * 2. At each distance, manually adjust hood until the shot lands
 * 3. Record the distance and the corresponding hood angle
 * 4. Update TuningConfig.HoodLookup.CAL_DIST_* and CAL_ANGLE_* values
 *    (these are tunable via FTC Dashboard)
 * 
 * The table maps: distance (inches) → hood angle (degrees)
 * Values are linearly interpolated between entries.
 */
public class HoodLookupTable {

    // ============================================================
    // Cached arrays (loaded from TuningConfig on first use)
    // ============================================================
    private static double[] DISTANCES = null;
    private static double[] HOOD_ANGLES = null;

    private static void ensureArrays() {
        if (DISTANCES == null) {
            DISTANCES = TuningConfig.HoodLookup.getDistances();
            HOOD_ANGLES = TuningConfig.HoodLookup.getAngles();
        }
    }

    // ============================================================
    // MECHANICAL LIMITS (from TuningConfig)
    // ============================================================
    private static final double MIN_ANGLE = TuningConfig.Hood.MIN_ANGLE;
    private static final double MAX_ANGLE = TuningConfig.Hood.MAX_ANGLE;

    // ============================================================
    // LINEAR INTERPOLATION
    // ============================================================

    /**
     * Get hood angle for a given distance using linear interpolation.
     * 
     * @param distanceInches Distance from shooter to goal (inches)
     * @return Hood angle in degrees (clamped to MIN/MAX limits)
     */
    public static double getHoodAngle(double distanceInches) {
        ensureArrays();

        // Clamp to table bounds
        if (distanceInches <= DISTANCES[0]) {
            return clampAngle(HOOD_ANGLES[0]);
        }
        if (distanceInches >= DISTANCES[DISTANCES.length - 1]) {
            return clampAngle(HOOD_ANGLES[HOOD_ANGLES.length - 1]);
        }

        // Find surrounding data points and interpolate
        for (int i = 0; i < DISTANCES.length - 1; i++) {
            if (distanceInches >= DISTANCES[i] && distanceInches < DISTANCES[i + 1]) {
                // Linear interpolation:
                // angle = angle1 + (angle2 - angle1) * (distance - dist1) / (dist2 - dist1)
                double t = (distanceInches - DISTANCES[i]) / (DISTANCES[i + 1] - DISTANCES[i]);
                double angle = HOOD_ANGLES[i] + t * (HOOD_ANGLES[i + 1] - HOOD_ANGLES[i]);
                return clampAngle(angle);
            }
        }

        // Fallback
        return clampAngle(HOOD_ANGLES[HOOD_ANGLES.length / 2]);
    }

    /**
     * Get hood angle with velocity compensation.
     * 
     * @param distanceInches Distance from shooter to goal (inches)
     * @param velocityErrorTicks Error in flywheel velocity (target - actual) in ticks/sec
     * @return Hood angle in degrees with velocity compensation
     */
    public static double getHoodAngleWithCompensation(
            double distanceInches, 
            double velocityErrorTicks) {
        
        // Convert velocity error to distance offset
        // Positive error = flywheel too fast = ball goes farther = aim closer
        double distanceOffset = -velocityErrorTicks * TuningConfig.HoodLookup.VELOCITY_COMP_FACTOR;
        
        return getHoodAngle(distanceInches + distanceOffset);
    }

    /**
     * Get the servo position for the hood given an angle.
     * Maps angle range to servo position range.
     * 
     * @param angleDegrees Hood angle in degrees
     * @return Servo position (0.0 to 1.0)
     */
    public static double angleToServoPosition(double angleDegrees) {
        double angle = clampAngle(angleDegrees);
        
        // Linear map from angle range to servo position range
        double t = (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        return TuningConfig.Hood.MIN_SERVO_POS + 
               t * (TuningConfig.Hood.MAX_SERVO_POS - TuningConfig.Hood.MIN_SERVO_POS);
    }

    /**
     * Get both angle AND servo position in one call.
     * 
     * @param distanceInches Distance in inches
     * @return Array: [0] = angle (degrees), [1] = servo position
     */
    public static double[] getAngleAndServo(double distanceInches) {
        double angle = getHoodAngle(distanceInches);
        double servo = angleToServoPosition(angle);
        return new double[]{angle, servo};
    }

    private static double clampAngle(double angle) {
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    // ============================================================
    // UTILITY METHODS
    // ============================================================

    public static double[] getDistanceRange() {
        ensureArrays();
        return new double[]{DISTANCES[0], DISTANCES[DISTANCES.length - 1]};
    }

    public static double[] getAngleRange() {
        ensureArrays();
        return new double[]{HOOD_ANGLES[0], HOOD_ANGLES[HOOD_ANGLES.length - 1]};
    }

    public static boolean isInRange(double distanceInches) {
        ensureArrays();
        return distanceInches >= DISTANCES[0] && distanceInches <= DISTANCES[DISTANCES.length - 1];
    }

    public static String getExtrapolationType(double distanceInches) {
        ensureArrays();
        if (distanceInches < DISTANCES[0]) return "EXTRAPOLATE_LOW";
        if (distanceInches > DISTANCES[DISTANCES.length - 1]) return "EXTRAPOLATE_HIGH";
        return "INTERPOLATE";
    }

    /**
     * Print the calibration table for debugging.
     */
    public static String getCalibrationTable() {
        ensureArrays();
        StringBuilder sb = new StringBuilder();
        sb.append("╔══════════════════════════════════════╗\n");
        sb.append("║     HOOD CALIBRATION TABLE           ║\n");
        sb.append("╠══════════════════════════════════════╣\n");
        for (int i = 0; i < DISTANCES.length; i++) {
            sb.append(String.format("║  %6.1f in  →  %5.1f°              ║\n", 
                    DISTANCES[i], HOOD_ANGLES[i]));
        }
        sb.append("╚══════════════════════════════════════╝\n");
        sb.append("Range: ").append(String.format("%.1f - %.1f inches%n", DISTANCES[0], DISTANCES[DISTANCES.length - 1]));
        sb.append("Angle: ").append(String.format("%.1f° - %.1f°", HOOD_ANGLES[0], HOOD_ANGLES[HOOD_ANGLES.length - 1]));
        return sb.toString();
    }
}
