package org.firstinspires.ftc.teamcode.opmode_new;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryEx;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
import static org.firstinspires.ftc.teamcode.opmode_new.TuningConfig.*;

/**
 * Main TeleOp for the mecanum drive robot with fixed shooter.
 * Uses Kalman-filtered pose fusion from LocalizationManager for maximum accuracy.
 *
 * CONTROL SCHEME:
 *
 * DRIVER (Gamepad 1):
 * - Left Stick: Field-centric movement (forward/back/strafe)
 * - Right Stick X: Manual rotation (when not aligning)
 * - Right Trigger (hold): Auto-align to goal (drivetrain aims at goal)
 * - Left Trigger (hold): Shoot (when aligned and ready)
 * - Left Bumper (hold): Intake reverse (outtake)
 * - Right Bumper (toggle): Intake on/off
 * - D-Pad Up: Reset pose to start position
 * - D-Pad Left: Cancel alignment
 * - D-Pad Right: Start alignment
 * - D-Pad Down: Stop flywheel
 * - Square: Stop intake
 * - Circle: Start intake
 * - Triangle (hold): Intake reverse
 * - Cross: Start flywheel
 * - PS Button: Cancel all commands
 * - Share (hold): Reset heading offset
 * - Options: Hard reset from AprilTag
 *
 * OPERATOR (Gamepad 2):
 * - D-Pad Right: +0.2m distance offset
 * - D-Pad Left: -0.2m distance offset
 * - D-Pad Up: +0.1m distance offset
 * - D-Pad Down: -0.1m distance offset
 * - Circle: +0.02m fine distance adjust
 * - Square: -0.02m fine distance adjust
 * - Triangle: +0.05m fine distance adjust
 * - Cross: -0.05m fine distance adjust
 * - Left Bumper: Camera recording off
 * - Right Bumper: Camera recording on
 * - Left Stick Button: Reset to start position
 * - Right Stick Button: Undo last AprilTag reset
 */
@Config
@TeleOp(name = "NewMecanumTeleOp", group = "TeleOp")
public class NewMecanumTeleOp extends LinearOpMode {

    // Hardware
    private NewHardware robot;
    private NewDrive drive;
    private NewOdometry odometry;
    private LocalizationManager localization;
    private Shooter shooter;
    private GoalAligner aligner;

    // Gamepad state tracking
    private boolean lastRightBumper = false;
    private boolean lastShare = false;
    private boolean lastOptions = false;
    private boolean lastCircle = false;
    private boolean lastSquare = false;
    private boolean lastTriangle = false;
    private boolean lastDpadUp = false;
    private boolean lastLeftStickButton = false;
    private boolean lastRightStickButton = false;

    // Intake toggle
    private boolean intakeActive = false;

    // Rumble
    private ElapsedTime rumbleTimer = new ElapsedTime();
    private double lastRumbleTime = 0;

    // Timer
    private ElapsedTime loopTimer = new ElapsedTime();

    // Alliance
    private AllianceSelector allianceSelector;
    private NewHardware.Alliance selectedAlliance = null;
    private boolean allianceLocked = false;

    // Telemetry
    private TelemetryEx telemetryEx;
    private double alignTurnOutput = 0;

    // AprilTag reset history
    private Pose2d lastResetPose = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FTC Dashboard telemetry
        telemetryEx = new TelemetryEx(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

        // Initialize hardware
        robot = new NewHardware();
        robot.init(hardwareMap, telemetry);

        // Initialize subsystems
        odometry = new NewOdometry();
        odometry.init(robot);

        drive = new NewDrive();
        drive.init(robot);

        localization = new LocalizationManager();
        localization.init(robot, odometry);

        aligner = new GoalAligner();
        aligner.init(robot, localization);

        shooter = new Shooter();
        shooter.init(robot, localization);

        // Initialize alliance selector
        allianceSelector = new AllianceSelector(robot);

        // Set gamepad LEDs
        setGamepadLEDs();

        // ==================== ALLIANCE SELECTION (INIT PHASE) ====================
        // Allow alliance selection during init
        allianceSelector.showAllianceSelection(telemetryEx);

        while (!isStarted()) {
            // Run alliance selection
            selectedAlliance = allianceSelector.update(gamepad1, telemetryEx);
            telemetryEx.update();
            idle();
        }

        // Lock alliance when opmode starts - NO MORE CHANGES ALLOWED
        allianceLocked = true;
        allianceSelector.lockAlliance();
        applyAllianceSettings();

        // Wait for start
        waitForStart();
        loopTimer.reset();

        // Main teleop loop
        while (opModeIsActive()) {
            // ==================== UPDATE LOCALIZATION (KALMAN FILTER) ====================
            localization.update();

            // ==================== GET INPUTS ====================
            boolean rightTriggerHeld = gamepad1.right_trigger > Drive.TRIGGER_DEADZONE;
            boolean leftTriggerHeld = gamepad1.left_trigger > Drive.TRIGGER_DEADZONE;
            boolean rightBumperPressed = gamepad1.right_bumper && !lastRightBumper;

            boolean shouldAlign = rightTriggerHeld || aligner.isAligning();

            // ==================== INTAKE CONTROL ====================
            if (gamepad1.left_bumper) {
                robot.intakeReverse();
                robot.gateClose();
            } else if (rightBumperPressed) {
                intakeActive = !intakeActive;
                if (intakeActive) {
                    robot.intakeForward();
                } else {
                    robot.intakeStop();
                }
            } else if (intakeActive) {
                robot.intakeForward();
            } else {
                robot.intakeStop();
            }

            // ==================== FLYWHEEL CONTROL ====================
            if (gamepad1.cross) {
                shooter.startFlywheel();
            } else if (gamepad1.dpad_down) {
                shooter.stopFlywheel();
            }

            // ==================== ALIGNMENT CONTROL ====================
            if (shouldAlign) {
                aligner.startAlignment();
            } else if (gamepad1.dpad_left) {
                aligner.stopAlignment();
            }

            // ==================== SHOOTING CONTROL ====================
            if (leftTriggerHeld && shooter.isReady() && aligner.isAligned()) {
                shooter.startShooting();
            }

            // ==================== OPERATOR CONTROLS ====================
            // Distance offset (d-pad)
            if (gamepad2.dpad_right) {
                Shooter.distanceOffset += 0.2;
            }
            if (gamepad2.dpad_left) {
                Shooter.distanceOffset -= 0.2;
            }
            if (gamepad2.dpad_up) {
                Shooter.distanceOffset += 0.1;
            }
            if (gamepad2.dpad_down) {
                Shooter.distanceOffset -= 0.1;
            }

            // Fine adjustments (face buttons)
            if (gamepad2.circle) {
                Shooter.distanceOffset += 0.02;
            }
            if (gamepad2.square) {
                Shooter.distanceOffset -= 0.02;
            }
            if (gamepad2.triangle) {
                Shooter.distanceOffset += 0.05;
            }
            if (gamepad2.cross) {
                Shooter.distanceOffset -= 0.05;
            }

            // Pose reset controls
            if (gamepad2.left_stick_button && !lastLeftStickButton) {
                localization.resetToStartPosition();
            }

            // Hard reset from AprilTag
            if (gamepad1.options && !lastOptions) {
                Pose2d currentPose = localization.getPose();
                if (localization.isAprilTagVisible()) {
                    localization.hardResetFromAprilTag();
                    lastResetPose = currentPose;
                }
            }

            // Undo last reset
            if (gamepad2.right_stick_button && !lastRightStickButton) {
                if (lastResetPose != null) {
                    localization.resetPose(lastResetPose);
                    lastResetPose = null;
                }
            }

            // ==================== HEADING RESET ====================
            if (gamepad1.share && !lastShare) {
                rumbleTimer.reset();
                gamepad1.rumble(100);
            }
            if (gamepad1.share && rumbleTimer.seconds() > 0.25) {
                localization.resetHeading();
                rumbleTimer.reset();
            }

            // ==================== UPDATE SUBSYSTEMS ====================
            alignTurnOutput = aligner.update();
            shooter.update(loopTimer.seconds());

            // ==================== DRIVETRAIN CONTROL ====================
            drive.setAligningActive(shouldAlign);
            drive.setAutoAlignTurn(alignTurnOutput);
            drive.setLockEnabled(Lock.LOCK_ENABLED);

            boolean userWantsToMove = Math.abs(gamepad1.left_stick_x) > Drive.INPUT_DEADZONE ||
                    Math.abs(gamepad1.left_stick_y) > Drive.INPUT_DEADZONE ||
                    Math.abs(gamepad1.right_stick_x) > Drive.INPUT_DEADZONE;
            drive.setUserWantsToMove(userWantsToMove);

            drive.drive(gamepad1, false);

            // ==================== RUMBLE FEEDBACK ====================
            if (shooter.isReady() && aligner.isAligned()) {
                if (loopTimer.seconds() - lastRumbleTime > 1.0) {
                    gamepad1.rumble(100);
                    lastRumbleTime = loopTimer.seconds();
                }
            }

            // ==================== TELEMETRY ====================
            addTelemetry();

            // Store button states
            lastRightBumper = gamepad1.right_bumper;
            lastShare = gamepad1.share;
            lastOptions = gamepad1.options;
            lastCircle = gamepad1.circle;
            lastSquare = gamepad1.square;
            lastTriangle = gamepad1.triangle;
            lastDpadUp = gamepad1.dpad_up;
            lastLeftStickButton = gamepad2.left_stick_button;
            lastRightStickButton = gamepad2.right_stick_button;

            loopTimer.reset();
        }
    }

    /**
     * Apply alliance-specific settings
     */
    private void applyAllianceSettings() {
        if (selectedAlliance == null) {
            selectedAlliance = NewHardware.Alliance.BLUE; // Default to blue
        }

        robot.setAlliance(selectedAlliance);

        // Update TuningConfig.Field based on alliance
        if (selectedAlliance == NewHardware.Alliance.RED) {
            TuningConfig.Field.GOAL_X = 72.0;  // Red goal is on positive X side
            NewConstants.ALLIANCE_COLOR = NewConstants.AllianceColor.RED;
        } else {
            TuningConfig.Field.GOAL_X = -72.0; // Blue goal is on negative X side
            NewConstants.ALLIANCE_COLOR = NewConstants.AllianceColor.BLUE;
        }

        // Reset odometry and localization to start position for selected alliance
        localization.resetToStartPosition();
    }

    /**
     * Add all telemetry data to FTC Dashboard
     */
    private void addTelemetry() {
        // ---- Loop Performance ----
        telemetryEx.addData("Loop Time", String.format("%.1fms", loopTimer.milliseconds()));

        // ---- Alliance and State ----
        telemetryEx.addData("Alliance", selectedAlliance);
        telemetryEx.addData("Robot State", getRobotState());
        telemetryEx.addData("Align State", aligner.getState());
        telemetryEx.addData("Shooter State", shooter.getState());

        // ---- Fused Pose ----
        telemetryEx.addData("--- POSE (FUSED) ---", "");
        telemetryEx.addData("Pose X", String.format("%.1f in", localization.getX()));
        telemetryEx.addData("Pose Y", String.format("%.1f in", localization.getY()));
        telemetryEx.addData("Pose H", String.format("%.1f deg", Math.toDegrees(localization.getHeading())));

        // ---- Distance and Alignment ----
        telemetryEx.addData("--- ALIGNMENT ---", "");
        telemetryEx.addData("Dist to Goal", String.format("%.1f in", localization.getDistanceToGoal()));
        telemetryEx.addData("Angle to Goal", String.format("%.1f deg", Math.toDegrees(localization.getAngleToGoal())));
        telemetryEx.addData("Align Error", String.format("%.1f deg", aligner.getAngleErrorDegrees()));
        telemetryEx.addData("Align Turn Out", String.format("%.3f", alignTurnOutput));
        telemetryEx.addData("Lock Prevented", drive.isLockPrevented() ? "YES" : "NO");

        // ---- Localization Quality ----
        if (Debug.SHOW_KALMAN_DATA) {
            telemetryEx.addData("--- KALMAN FILTER ---", "");
            telemetryEx.addData("Loc State", localization.getState());
            telemetryEx.addData("Camera Conf", String.format("%.2f", localization.getCameraConfidence()));
            telemetryEx.addData("Pos Uncert", String.format("%.2f in", localization.getPositionUncertainty()));
            telemetryEx.addData("Head Uncert", String.format("%.3f rad", localization.getHeadingUncertainty()));
            telemetryEx.addData("AprilTag", localization.isAprilTagVisible() ? "VISIBLE" : "NOT VISIBLE");
            telemetryEx.addData("LL TX", String.format("%.1f deg", localization.getLimelightTx()));
            telemetryEx.addData("LL TY", String.format("%.1f deg", localization.getLimelightTy()));
            telemetryEx.addData("LL Dist (m)", String.format("%.2f m", localization.getLimelightDistance()));
        }

        // ---- Flywheel ----
        if (Debug.SHOW_VELOCITY_DATA) {
            telemetryEx.addData("--- FLYWHEEL ---", "");
            telemetryEx.addData("Velocity", String.format("%.0f tps", robot.getFlywheelVelocity()));
            telemetryEx.addData("Target", String.format("%.0f tps", shooter.getTargetFlywheelVelocity()));
            telemetryEx.addData("Filtered Vel", String.format("%.0f tps", shooter.getFilteredFlywheelVelocity()));
            telemetryEx.addData("Vel Error", String.format("%.0f tps", shooter.getTargetFlywheelVelocity() - shooter.getFlywheelVelocity()));
            telemetryEx.addData("Vel Accel", String.format("%.0f", shooter.getVelocityAcceleration()));
            telemetryEx.addData("Flywheel Ready", shooter.isFlywheelReady() ? "YES" : "NO");
        }

        // ---- Hood and Shooting ----
        if (Debug.SHOW_PREDICTION_DATA) {
            telemetryEx.addData("--- HOOD & PREDICTION ---", "");
            telemetryEx.addData("Hood Angle", String.format("%.1f deg", shooter.getTargetHoodAngle()));
            telemetryEx.addData("Predicted Dist", String.format("%.1f in", shooter.getPredictedDistance()));
            telemetryEx.addData("Distance Offset", String.format("%.2f m", Shooter.distanceOffset));
        }

        // ---- Intake and Gate ----
        telemetryEx.addData("--- INTAKE ---", "");
        telemetryEx.addData("Intake", intakeActive ? "ON" : "OFF");
        telemetryEx.addData("Gate", shooter.isShooting() ? "OPEN" : "CLOSED");

        // ---- Velocities ----
        telemetryEx.addData("--- VELOCITIES ---", "");
        telemetryEx.addData("Vel X", String.format("%.1f in/s", localization.getVelocityX()));
        telemetryEx.addData("Vel Y", String.format("%.1f in/s", localization.getVelocityY()));
        telemetryEx.addData("Vel Mag", String.format("%.1f in/s", localization.getVelocityMagnitude()));
        telemetryEx.addData("Ang Vel", String.format("%.3f rad/s", localization.getAngularVelocity()));

        // ---- Odometry (Raw) ----
        if (Debug.SHOW_ODOMETRY_DATA) {
            telemetryEx.addData("--- ODOMETRY (RAW) ---", "");
            telemetryEx.addData("Odo X", String.format("%.1f in", odometry.getX()));
            telemetryEx.addData("Odo Y", String.format("%.1f in", odometry.getY()));
            telemetryEx.addData("Odo H", String.format("%.1f deg", Math.toDegrees(odometry.getHeading())));
            telemetryEx.addData("Fwd Ticks", robot.getForwardOdometry());
            telemetryEx.addData("Lat Ticks", robot.getLateralOdometry());
        }

        telemetryEx.update();
    }

    /**
     * Get human-readable robot state
     */
    private String getRobotState() {
        if (shooter.isShooting()) {
            return "SHOOTING";
        } else if (shooter.isReady() && aligner.isAligned()) {
            return "READY";
        } else if (aligner.isAligning()) {
            return "ALIGNING";
        } else if (shooter.isFlywheelReady()) {
            return "SPIN_UP";
        } else {
            return "INTAKE";
        }
    }

    /**
     * Set gamepad LEDs based on alliance
     */
    private void setGamepadLEDs() {
        // Driver: Green
        gamepad1.setLedColor(0, 255, 0, LED_DURATION_CONTINUOUS);
        // Operator: Alliance color
        if (selectedAlliance == NewHardware.Alliance.RED) {
            gamepad2.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(0, 0, 255, LED_DURATION_CONTINUOUS);
        }
    }
}
