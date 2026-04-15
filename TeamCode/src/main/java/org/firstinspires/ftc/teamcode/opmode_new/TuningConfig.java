package org.firstinspires.ftc.teamcode.opmode_new;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Centralized tuning configuration for FTC Dashboard and code-based tuning.
 * All tunable parameters are here with @Config annotation for FTC Dashboard.
 * 
 * HOW TO TUNE:
 * 1. Open FTC Dashboard in browser (http://192.168.43.1:8080/dash)
 * 2. Adjust values in real-time while robot is running
 * 3. Values update immediately without restarting
 * 
 * CATEGORIES:
 * - Drive: Drivetrain and movement
 * - Vision: Limelight and AprilTag
 * - Shooter: Flywheel, hood, and shooting
 * - Alignment: Goal alignment PID
 * - Localization: Kalman filter and odometry
 * - Lock: Motor lock behavior
 */
@Config
public class TuningConfig {

    // ============================================================
    // DRIVETRAIN & MOVEMENT
    // ============================================================
    public static class Drive {
        // ----- Speeds -----
        public static double SPEED_MULTIPLIER = 1.0;        // Overall drive speed (0-1)
        public static double STRAFE_SPEED = 1.0;           // Strafe speed multiplier
        public static double TURN_SPEED = 1.0;             // Manual turn speed multiplier

        // ----- Slew Rate Limiting -----
        public static double STRAFE_SLEW_RATE = 6.7;       // Joystick/second (higher = snappier)
        public static double TURN_SLEW_RATE = 8.67;      // Joystick/second (higher = snappier)

        // ----- Input Deadzones -----
        public static double INPUT_DEADZONE = 0.08;        // Joystick deadzone (0.02-0.15)
        public static double TRIGGER_DEADZONE = 0.1;      // Trigger deadzone (0.05-0.15)

        // ----- IMU -----
        public static double IMU_ORIENTATION_DEGREES = 0.0; // Yaw offset from IMU
    }

    // ============================================================
    // MOTOR LOCK
    // ============================================================
    public static class Lock {
        public static boolean LOCK_ENABLED = true;              // Master enable for lock
        public static double LOCK_STOP_DELAY_MS = 200;         // ms to wait before locking
        public static double LOCK_START_POWER = 0.3;          // Initial lock power
        public static double LOCK_MAX_POWER = 0.85;           // Maximum lock power
        public static double LOCK_RAMP_TIME_MS = 1000;       // Time to ramp to max power
        public static double ALIGNMENT_COOLDOWN_SEC = 0.5;   // Lock disabled after align ends
    }

    // ============================================================
    // VISION & LIMELIGHT
    // ============================================================
    public static class Vision {
        // ----- Limelight Mount -----
        public static double MOUNT_ANGLE_DEGREES = 25.0;       // Camera angle from horizontal
        public static double LENS_HEIGHT_INCHES = 8.0;        // Height of camera lens

        // ----- Goal -----
        public static double GOAL_HEIGHT_INCHES = 60.0;       // Height of goal target

        // ----- AprilTag IDs (set for your game) -----
        public static int BLUE_GOAL_TAG_ID = 20;              // Blue alliance goal AprilTag
        public static int RED_GOAL_TAG_ID = 24;              // Red alliance goal AprilTag
    }

    // ============================================================
    // KALMAN FILTER & LOCALIZATION
    // ============================================================
    public static class Localization {
        // ----- Process Noise (trust in odometry) -----
        // Higher = odometry drifts faster, camera corrections stronger
        // Lower = smoother but slower to correct
        public static double PROCESS_NOISE_X = 0.5;           // X position drift (inches^2/s)
        public static double PROCESS_NOISE_Y = 0.5;          // Y position drift (inches^2/s)
        public static double PROCESS_NOISE_HEADING = 0.1;    // Heading drift (rad^2/s)

        // ----- Measurement Noise (trust in camera) -----
        // Higher = less trust in camera, odometry dominates
        // Lower = more trust in camera, corrections stronger
        public static double CAMERA_NOISE_X = 3.0;           // X measurement noise (inches)
        public static double CAMERA_NOISE_Y = 3.0;          // Y measurement noise (inches)
        public static double CAMERA_NOISE_HEADING = 0.05;   // Heading noise (radians)

        // ----- Adaptive Noise Scaling -----
        public static double CAMERA_NOISE_SCALE = 0.03;      // Noise increase per inch of distance
        public static double MIN_CAMERA_NOISE = 0.5;        // Minimum noise floor (inches)
        public static double MAX_CAMERA_NOISE = 10.0;       // Maximum noise cap (inches)

        // ----- Kalman Gains (alternative to noise tuning) -----
        public static double KALMAN_GAIN_POSITION = 0.15;     // Position correction gain
        public static double KALMAN_GAIN_HEADING = 0.15;    // Heading correction gain

        // ----- Timing -----
        public static double STALE_THRESHOLD_SEC = 0.5;       // Camera data stale after this
        public static int MAX_CONSECUTIVE_MISSES = 10;      // Misses before recovery mode

        // ----- Odometry -----
        public static double TRACK_WIDTH = 11.27;            // Inches between left/right wheels
        public static double WHEEL_BASE = 11.51;            // Inches between front/back wheels
        public static double WHEEL_DIAMETER = 4.0;        // Drive wheel diameter (inches)
        public static double DEAD_WHEEL_DIAMETER = 2.0;    // Dead wheel diameter (inches)
        public static double ENCODER_TICKS_PER_REV = 28.0; // Encoder counts per revolution

        // ----- Dead Wheel Offsets -----
        public static double LATERAL_OFFSET = 6.5;          // Lateral wheel from center (inches)
        public static double FORWARD_OFFSET = -3.0;        // Forward wheel from center (inches)
    }

    // ============================================================
    // GOAL ALIGNMENT (AIMBOT)
    // ============================================================
    public static class Alignment {
        // ----- PID Controller -----
        public static double AIMBOT_KP = 0.015;             // Proportional gain
        public static double AIMBOT_KI = 0.0;               // Integral gain
        public static double AIMBOT_KD = 0.001;            // Derivative gain
        public static double AIMBOT_KF = 0.0;              // Feedforward gain

        // ----- Limits -----
        public static double MAX_TURN_SPEED = 0.5;          // Maximum turn power (0-1)
        public static double ANGLE_TOLERANCE = 1.0;        // Degrees to be considered aligned
        public static double DEADBAND_RADIANS = 0.017;     // Deadband in radians (1 deg = 0.017 rad)

        // ----- Adaptive Gains -----
        public static boolean USE_ADAPTIVE_GAINS = true;    // Reduce gains when uncertain
        public static double UNCERTAINTY_THRESHOLD_HIGH = 5.0;  // Pos uncertainty to reduce gains
        public static double UNCERTAINTY_THRESHOLD_LOW = 2.0;  // Pos uncertainty for normal gains
    }

    // ============================================================
    // SHOOTER (FLYWHEEL)
    // ============================================================
    public static class Flywheel {
        // ----- PIDF Coefficients -----
        public static double FLYWHEEL_KP = 0.01;          // Proportional
        public static double FLYWHEEL_KI = 0.0;            // Integral
        public static double FLYWHEEL_KD = 0.0;            // Derivative
        public static double FLYWHEEL_KF = 0.00045;       // Feedforward

        // ----- Target -----
        public static double TARGET_VELOCITY = 2000.0;     // Target RPM in ticks/sec
        public static double VELOCITY_TOLERANCE = 50.0;   // Ticks/sec to be "ready"

        // ----- Limits -----
        public static double MAX_POWER = 1.0;              // Maximum motor power (0-1)
        public static double MIN_POWER = 0.0;              // Minimum motor power

        // ----- Velocity Compensation -----
        public static double VELOCITY_COMP_THRESHOLD = 20.0; // Error to start compensating
        public static double VELOCITY_PREDICTION_TIME = 0.05; // Look-ahead time (seconds)
        public static double VELOCITY_COMP_FACTOR = 0.0001;   // Meters per tick/sec

        // ----- Filtering -----
        public static int VELOCITY_HISTORY_SIZE = 5;      // Samples for moving average
    }

    // ============================================================
    // HOOD (SERVOS)
    // ============================================================
    public static class Hood {
        // ----- Physical Limits -----
        public static double MIN_ANGLE = 20.0;             // Degrees from horizontal (close)
        public static double MAX_ANGLE = 45.0;             // Degrees from horizontal (far)

        // ----- Servo Positions -----
        public static double MIN_SERVO_POS = 0.24;         // Servo position for MIN_ANGLE
        public static double MAX_SERVO_POS = 0.92;         // Servo position for MAX_ANGLE

        // ----- Servo Offsets -----
        public static double LEFT_OFFSET = 0.0;             // Left servo offset
        public static double RIGHT_OFFSET = 0.0;           // Right servo offset (for mirroring)
    }

    // ============================================================
    // LAUNCHER MATH
    // ============================================================
    public static class LauncherMath {
        // ----- Physics -----
        public static double GRAVITY = 9.81;               // m/s^2
        public static double LAUNCHER_HEIGHT = 0.33;        // meters (flywheel height)
        public static double TARGET_HEIGHT = 1.0;          // meters (goal height)
        public static double GOAL_LIP = 0.45;              // meters (goal rim height)
        public static double BACKBOARD_OFFSET = 0.1;       // meters (higher target)
        public static double LIP_BUFFER = 0.02;            // meters (safety buffer)

        // ----- Limits -----
        public static double MAX_BALL_VELOCITY = 12.0;     // m/s (max achievable)

        // ----- Conversions -----
        public static double TICKS_TO_VELOCITY = 350.0;     // ticks/sec to m/s factor
    }

    // ============================================================
    // BALL FLIGHT TIME COMPENSATION
    // ============================================================
    public static class BallFlight {
        public static double MECHANICAL_DELAY = 0.1;       // seconds (gate to ball leaving)
        public static double MIN_FLIGHT_TIME = 0.3;        // minimum seconds
        public static double MAX_FLIGHT_TIME = 1.5;        // maximum seconds
    }

    // ============================================================
    // LOOKUP TABLE HOOD CONTROL
    // ============================================================
    public static class HoodLookup {
        // Velocity compensation factor (inches offset per tick/sec of error)
        // Tune this: if shots go long when flywheel is fast, increase
        // if shots go short when flywheel is fast, decrease
        public static double VELOCITY_COMP_FACTOR = 0.00015;

        // Calibration distances (inches from goal)
        // These are indexed: 0=closest, 7=farthest
        public static double CAL_DIST_0 = 36.0;
        public static double CAL_DIST_1 = 48.0;
        public static double CAL_DIST_2 = 60.0;
        public static double CAL_DIST_3 = 72.0;
        public static double CAL_DIST_4 = 84.0;
        public static double CAL_DIST_5 = 96.0;
        public static double CAL_DIST_6 = 108.0;
        public static double CAL_DIST_7 = 120.0;

        // Calibrated hood angles (degrees from horizontal)
        public static double CAL_ANGLE_0 = 20.0;
        public static double CAL_ANGLE_1 = 24.0;
        public static double CAL_ANGLE_2 = 28.0;
        public static double CAL_ANGLE_3 = 32.0;
        public static double CAL_ANGLE_4 = 36.0;
        public static double CAL_ANGLE_5 = 40.0;
        public static double CAL_ANGLE_6 = 44.0;
        public static double CAL_ANGLE_7 = 48.0;

        // Lookup table entries (used by HoodLookupTable class)
        public static double[] getDistances() {
            return new double[]{CAL_DIST_0, CAL_DIST_1, CAL_DIST_2, CAL_DIST_3,
                    CAL_DIST_4, CAL_DIST_5, CAL_DIST_6, CAL_DIST_7};
        }

        public static double[] getAngles() {
            return new double[]{CAL_ANGLE_0, CAL_ANGLE_1, CAL_ANGLE_2, CAL_ANGLE_3,
                    CAL_ANGLE_4, CAL_ANGLE_5, CAL_ANGLE_6, CAL_ANGLE_7};
        }
    }

    // ============================================================
    // INTAKE
    // ============================================================
    public static class Intake {
        public static double FORWARD_POWER = 1.0;          // Intake forward speed
        public static double REVERSE_POWER = -1.0;         // Intake reverse speed
        public static double STOP_POWER = 0.0;              // Stop speed
    }

    // ============================================================
    // GATE (SERVO)
    // ============================================================
    public static class Gate {
        public static double OPEN_POSITION = 0.5;         // Position when open
        public static double CLOSED_POSITION = 0.0;       // Position when closed
    }

    // ============================================================
    // SHOOTING
    // ============================================================
    public static class Shooting {
        public static double SHOOT_DELAY = 2.0;            // Seconds to hold shot
    }

    // ============================================================
    // FIELD CONSTANTS
    // ============================================================
    public static class Field {
        public static double GOAL_X = -72.0;               // Goal X position (inches)
        public static double GOAL_Y = 72.0;               // Goal Y position (inches)
        public static double FIELD_WIDTH = 144.0;         // Total field width (inches)
        public static double FIELD_HEIGHT = 144.0;        // Total field height (inches)
    }

    // ============================================================
    // TELEMETRY & DEBUG
    // ============================================================
    public static class Debug {
        public static boolean STREAM_TELEMETRY = true;     // Stream to FTC Dashboard
        public static boolean SHOW_KALMAN_DATA = true;      // Show Kalman filter data
        public static boolean SHOW_VELOCITY_DATA = true;    // Show flywheel velocity data
        public static boolean SHOW_ALIGNMENT_DATA = true;   // Show alignment data
        public static boolean SHOW_ODOMETRY_DATA = false;   // Show odometry raw data
        public static boolean SHOW_PREDICTION_DATA = true;   // Show ball flight prediction
    }

    // ============================================================
    // HELPER METHODS FOR SUBSYSTEMS
    // ============================================================

    /**
     * Get alliance multiplier (-1 for RED, +1 for BLUE)
     */
    public static int getAllianceMultiplier() {
        return NewConstants.ALLIANCE_COLOR == NewConstants.AllianceColor.RED ? -1 : 1;
    }

    /**
     * Get wheel circumference in inches
     */
    public static double getWheelCircumference() {
        return Math.PI * Localization.WHEEL_DIAMETER;
    }

    /**
     * Get dead wheel circumference in inches
     */
    public static double getDeadWheelCircumference() {
        return Math.PI * Localization.DEAD_WHEEL_DIAMETER;
    }

    /**
     * Convert dead wheel ticks to inches
     */
    public static double ticksToInches(double ticks) {
        return ticks * getDeadWheelCircumference() / Localization.ENCODER_TICKS_PER_REV;
    }

    /**
     * Get max velocity in inches/sec (from ticks/sec)
     */
    public static double getMaxVelocityIPS() {
        return Flywheel.TARGET_VELOCITY / LauncherMath.TICKS_TO_VELOCITY;
    }
}
