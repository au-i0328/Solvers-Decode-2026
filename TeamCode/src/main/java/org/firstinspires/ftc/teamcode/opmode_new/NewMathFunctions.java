package org.firstinspires.ftc.teamcode.opmode_new;

import static org.firstinspires.ftc.teamcode.opmode_new.NewConstants.*;

/**
 * Math functions for launcher calculations.
 * Contains the physics calculations for shooting based on the original code.
 */
public class NewMathFunctions {

    /**
     * Calculates the best launch parameters (Velocity, Angle) for a given distance.
     * 1. Tries to hit the Backboard (high velocity, flat shot).
     * 2. Falls back to Goal Center if Backboard is unreachable.
     * 3. Ensures the shot clears the Goal Lip Buffer.
     *
     * @param distanceMeters Horizontal distance to goal center (m).
     * @return { velocity (m/s), hoodAngle (degrees from horizontal) }
     */
    public static double[] distanceToLauncherValues(double distanceMeters) {
        double g = GRAVITY;
        double x = distanceMeters;
        double xLip = x - GOAL_LIP;

        // Physical height the ball must be at when passing the lip
        double deltaYLip = (TARGET_HEIGHT + LIP_BUFFER) - LAUNCHER_HEIGHT;

        // --- Attempt 1: Backboard Shot (Priority) ---
        double[] result = calculateBestShot(x, TARGET_HEIGHT + BACKBOARD_Y_OFFSET, xLip, deltaYLip, g);

        // --- Attempt 2: Center Goal Fallback ---
        // If Attempt 1 failed (NaN), aim for the center of the goal
        if (Double.isNaN(result[0])) {
            result = calculateBestShot(x, TARGET_HEIGHT, xLip, deltaYLip, g);
        }

        return result;
    }

    /**
     * Robust solver that finds the flattest valid shot for a specific target Y.
     */
    private static double[] calculateBestShot(double x, double targetY, double xLip, double deltaYLip, double g) {
        double deltaY = targetY - LAUNCHER_HEIGHT;
        double minPhysAngleH = 90.0 - MAX_HOOD_ANGLE; // e.g., 45 deg (from horizontal)
        double maxPhysAngleH = 90.0 - MIN_HOOD_ANGLE; // e.g., 70 deg (from horizontal)

        // 1. Line-of-Sight Check
        // We cannot shoot flatter than the direct line to the target.
        // +0.1 degrees ensures the velocity formula denominator is never zero/negative.
        double minGeomAngle = Math.toDegrees(Math.atan(deltaY / x)) + 0.1;

        // 2. Lip Clearance Check
        double minLipH = 0.0;
        if (xLip > 0) {
            // Find angle that passes through (xLip, deltaYLip) AND (x, deltaY)
            double num = (deltaY * xLip * xLip) - (deltaYLip * x * x);
            double den = (x * xLip * xLip) - (xLip * x * x);

            // If den is 0 (xLip == x), vertical shot needed (impossible)
            if (Math.abs(den) > 1e-5) {
                minLipH = Math.toDegrees(Math.atan(num / den));
            } else {
                minLipH = 89.9; // Arbitrary high angle if lip is exactly at target dist
            }
        }

        // 3. Determine the Angle Floor
        // The shot must be steeper than: Hood Min, Lip Clearance, AND Line-of-Sight
        double targetAngleH = Math.max(minPhysAngleH, Math.max(minLipH, minGeomAngle));

        // If the required floor is steeper than the hood allows, this shot is impossible
        if (targetAngleH > maxPhysAngleH) {
            return new double[]{Double.NaN, Double.NaN};
        }

        // 4. Calculate Velocity for this "Floor" Angle
        // This is the fastest, flattest legal shot possible.
        double vReq = calculateVelocity(x, deltaY, targetAngleH, g);

        if (!Double.isNaN(vReq) && vReq <= LAUNCHER_MAX_BALL_VELOCITY) {
            return new double[]{vReq, 90.0 - targetAngleH};
        }

        // 5. Velocity Limited Fallback (Max Power)
        // If flattest shot requires > Max Velocity, we find the best angle at Max Velocity
        double v = LAUNCHER_MAX_BALL_VELOCITY;
        double A = (g * x * x) / (2.0 * v * v);
        double B = -x;
        double C = deltaY + A;
        double disc = B * B - 4 * A * C;

        if (disc < 0) return new double[]{Double.NaN, Double.NaN}; // Cannot reach even at Max V

        double sqrtD = Math.sqrt(disc);
        double tan1 = (-B - sqrtD) / (2 * A); // Flatter solution
        double tan2 = (-B + sqrtD) / (2 * A); // Steeper solution

        double a1 = Math.toDegrees(Math.atan(tan1));
        double a2 = Math.toDegrees(Math.atan(tan2));

        // Prefer a1 (Flat), then a2 (Lob), ensuring they meet our Angle Floor
        if (a1 >= targetAngleH && a1 <= maxPhysAngleH) return new double[]{v, 90.0 - a1};
        if (a2 >= targetAngleH && a2 <= maxPhysAngleH) return new double[]{v, 90.0 - a2};

        return new double[]{Double.NaN, Double.NaN};
    }

    /**
     * Helper to calculate required velocity.
     * Includes check for negative denominator to prevent NaN.
     */
    private static double calculateVelocity(double x, double deltaY, double angleHoriz, double g) {
        double angleRad = Math.toRadians(angleHoriz);
        double tanTheta = Math.tan(angleRad);
        double cosTheta = Math.cos(angleRad);

        // Denominator represents the vertical distance between the projected
        // angle line and the target. It must be positive.
        double denom = 2 * cosTheta * cosTheta * (x * tanTheta - deltaY);

        if (denom <= 1e-9) return Double.NaN; // Shot aims too low or infinite velocity

        return Math.sqrt((g * x * x) / denom);
    }

    /**
     * Calculates the best hood angle for a specific velocity.
     * Prioritizes backboard target, falls back to goal center.
     * Ensures the shot clears the rim by LIP_BUFFER height.
     *
     * @param distanceMeters Horizontal distance to target.
     * @param velocityMps Current ball velocity in meters per second.
     * @return Hood angle (degrees from horizontal) or NaN.
     */
    public static double getHoodAngleFromVelocity(double distanceMeters, double velocityMps) {
        double g = GRAVITY;
        double x = distanceMeters;
        double xLip = x - GOAL_LIP;

        // The ball must be this high when it crosses the lip distance
        double deltaYLip = (TARGET_HEIGHT + LIP_BUFFER) - LAUNCHER_HEIGHT;

        // 1. Try to solve for Backboard Height
        double angle = solveForTarget(x, TARGET_HEIGHT + BACKBOARD_Y_OFFSET, xLip, deltaYLip, velocityMps, g);

        // 2. Fallback to Goal Center Height
        if (Double.isNaN(angle)) {
            angle = solveForTarget(x, TARGET_HEIGHT, xLip, deltaYLip, velocityMps, g);
        }

        return angle;
    }

    /**
     * Internal helper to solve the quadratic for a specific target height Y.
     */
    private static double solveForTarget(double x, double targetY, double xLip, double deltaYLip, double v, double g) {
        double deltaY = targetY - LAUNCHER_HEIGHT;

        // Trajectory Quadratic: A*tan^2 + B*tan + C = 0
        double A = (g * x * x) / (2.0 * v * v);
        double B = -x;
        double C = deltaY + A;

        double disc = (B * B) - (4.0 * A * C);
        if (disc < 0) return Double.NaN; // Velocity too low

        double sqrtD = Math.sqrt(disc);
        double tan1 = (-B - sqrtD) / (2.0 * A); // Flatter
        double tan2 = (-B + sqrtD) / (2.0 * A); // Steeper

        double angle1H = Math.toDegrees(Math.atan(tan1));
        double angle2H = Math.toDegrees(Math.atan(tan2));

        // Calculate Lip Constraint for this specific trajectory path
        double minLipH = 0.0;
        if (xLip > 0) {
            double num = (deltaY * xLip * xLip) - (deltaYLip * x * x);
            double den = (x * xLip * xLip) - (xLip * x * x);
            minLipH = Math.toDegrees(Math.atan(num / den));
        }

        // Select the best valid angle (Flattest first)
        double epsilon = 1e-7;
        if (isAngleValid(angle1H, minLipH, epsilon)) return 90.0 - angle1H;

        return Double.NaN;
    }

    private static boolean isAngleValid(double angleHoriz, double minLipHoriz, double epsilon) {
        double angleVert = 90.0 - angleHoriz;
        // Check Lip clearance AND Hood Mechanical Limits
        return angleHoriz >= (minLipHoriz - epsilon)
                && angleVert >= (MIN_HOOD_ANGLE - epsilon)
                && angleVert <= (MAX_HOOD_ANGLE + epsilon);
    }

    /**
     * Convert velocity in m/s to encoder ticks per second for the flywheel.
     * This conversion depends on your specific flywheel setup.
     *
     * @param velocityMps Velocity in meters per second
     * @return Velocity in ticks per second
     */
    public static double velocityToTicksPerSecond(double velocityMps) {
        // Conversion factor based on flywheel geometry
        // This is a placeholder - tune based on your actual flywheel
        // Typical factor: ~300-400 ticks per m/s for small wheels
        return velocityMps * 350.0;
    }

    /**
     * Convert encoder ticks per second to velocity in m/s.
     *
     * @param ticksPerSecond Encoder velocity in ticks per second
     * @return Velocity in meters per second
     */
    public static double ticksPerSecondToVelocity(double ticksPerSecond) {
        return ticksPerSecond / 350.0;
    }
}
