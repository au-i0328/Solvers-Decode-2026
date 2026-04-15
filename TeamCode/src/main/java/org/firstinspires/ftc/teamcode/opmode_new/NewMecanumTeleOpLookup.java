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
 * TeleOp using LINEAR INTERPOLATION lookup table for hood control.
 * 
 * IDENTICAL to NewMecanumTeleOp in every way EXCEPT:
 * - Uses ShooterLookupTable instead of Shooter (physics)
 * - Hood angle comes from HoodLookupTable.getHoodAngle()
 * 
 * HOW TO USE THIS:
 * 1. During calibration, run the robot and use operator controls to adjust hood
 * 2. Record distance (from FTC Dashboard telemetry) and hood angle for each shot
 * 3. Add entries to HoodLookupTable.DISTANCES[] and HOOD_ANGLES[]
 * 4. Tune VELOCITY_COMP_FACTOR until shots are consistent
 * 
 * SWITCHING BETWEEN VERSIONS:
 * - Physics: Use NewMecanumTeleOp with Shooter
 * - Lookup Table: Use this TeleOp with ShooterLookupTable
 */
@Config
@TeleOp(name = "NewMecanumTeleOp-Lookup", group = "TeleOp")
public class NewMecanumTeleOpLookup extends LinearOpMode {

    // Hardware
    private NewHardware robot;
    private NewDrive drive;
    private NewOdometry odometry;
    private LocalizationManager localization;
    private ShooterLookupTable shooter;  // <-- DIFFERENT: LookupTable version
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
        aligner.init(localization);

        shooter = new ShooterLookupTable();  // <-- DIFFERENT: LookupTable version
        shooter.init(robot, localization);

        // Initialize alliance selector
        allianceSelector = new AllianceSelector(robot);

        // Set gamepad LEDs
        setGamepadLEDs();

        // Alliance selection screen
        allianceSelector.showAllianceSelection(telemetryEx);

        while (!isStarted()) {
            selectedAlliance = allianceSelector.update(gamepad1, telemetryEx);
            telemetryEx.update();
            idle();
        }

        // Lock alliance
        allianceLocked = true;
        allianceSelector.lockAlliance();
        applyAllianceSettings();

        // Wait for start
        waitForStart();
        loopTimer.reset();

        // Main teleop loop
        while (opModeIsActive()) {
            localization.update();

            boolean rightTriggerHeld = gamepad1.right_trigger > Drive.TRIGGER_DEADZONE;
            boolean leftTriggerHeld = gamepad1.left_trigger > Drive.TRIGGER_DEADZONE;
            boolean rightBumperPressed = gamepad1.right_bumper && !lastRightBumper;
            boolean shouldAlign = rightTriggerHeld || aligner.isAligning();

            // Intake control
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

            // Flywheel control
            if (gamepad1.cross) {
                shooter.startFlywheel();
            } else if (gamepad1.dpad_down) {
                shooter.stopFlywheel();
            }

            // Alignment control
            if (shouldAlign) {
                aligner.startAlignment();
            } else if (gamepad1.dpad_left) {
                aligner.stopAlignment();
            }

            // Shooting control
            if (leftTriggerHeld && shooter.isReady() && aligner.isAligned()) {
                shooter.startShooting();
            }

            // Operator controls - distance offset
            if (gamepad2.dpad_right) {
                ShooterLookupTable.distanceOffset += 0.2;
            }
            if (gamepad2.dpad_left) {
                ShooterLookupTable.distanceOffset -= 0.2;
            }
            if (gamepad2.dpad_up) {
                ShooterLookupTable.distanceOffset += 0.1;
            }
            if (gamepad2.dpad_down) {
                ShooterLookupTable.distanceOffset -= 0.1;
            }

            // Fine adjustments
            if (gamepad2.circle) {
                ShooterLookupTable.distanceOffset += 0.02;
            }
            if (gamepad2.square) {
                ShooterLookupTable.distanceOffset -= 0.02;
            }
            if (gamepad2.triangle) {
                ShooterLookupTable.distanceOffset += 0.05;
            }
            if (gamepad2.cross) {
                ShooterLookupTable.distanceOffset -= 0.05;
            }

            // Pose reset
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

            // Undo reset
            if (gamepad2.right_stick_button && !lastRightStickButton) {
                if (lastResetPose != null) {
                    localization.resetPose(lastResetPose);
                    lastResetPose = null;
                }
            }

            // Heading reset
            if (gamepad1.share && !lastShare) {
                rumbleTimer.reset();
                gamepad1.rumble(100);
            }
            if (gamepad1.share && rumbleTimer.seconds() > 0.25) {
                localization.resetHeading();
                rumbleTimer.reset();
            }

            // Update subsystems
            alignTurnOutput = aligner.update();
            shooter.update(loopTimer.seconds());

            // Drivetrain
            drive.setAligningActive(shouldAlign);
            drive.setAutoAlignTurn(alignTurnOutput);
            drive.setLockEnabled(Lock.LOCK_ENABLED);

            boolean userWantsToMove = Math.abs(gamepad1.left_stick_x) > Drive.INPUT_DEADZONE ||
                    Math.abs(gamepad1.left_stick_y) > Drive.INPUT_DEADZONE ||
                    Math.abs(gamepad1.right_stick_x) > Drive.INPUT_DEADZONE;
            drive.setUserWantsToMove(userWantsToMove);
            drive.drive(gamepad1, false);

            // Rumble feedback
            if (shooter.isReady() && aligner.isAligned()) {
                if (loopTimer.seconds() - lastRumbleTime > 1.0) {
                    gamepad1.rumble(100);
                    lastRumbleTime = loopTimer.seconds();
                }
            }

            addTelemetry();

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

    private void applyAllianceSettings() {
        if (selectedAlliance == null) {
            selectedAlliance = NewHardware.Alliance.BLUE;
        }

        robot.setAlliance(selectedAlliance);

        if (selectedAlliance == NewHardware.Alliance.RED) {
            TuningConfig.Field.GOAL_X = 72.0;
            NewConstants.ALLIANCE_COLOR = NewConstants.AllianceColor.RED;
        } else {
            TuningConfig.Field.GOAL_X = -72.0;
            NewConstants.ALLIANCE_COLOR = NewConstants.AllianceColor.BLUE;
        }

        localization.resetToStartPosition();
    }

    private void addTelemetry() {
        telemetryEx.addData("Loop Time", String.format("%.1fms", loopTimer.milliseconds()));
        telemetryEx.addData("Alliance", selectedAlliance);
        telemetryEx.addData("Robot State", getRobotState());
        telemetryEx.addData("Align State", aligner.getState());
        telemetryEx.addData("Shooter State", shooter.getState());

        // Pose
        telemetryEx.addData("--- POSE (FUSED) ---", "");
        telemetryEx.addData("Pose X", String.format("%.1f in", localization.getX()));
        telemetryEx.addData("Pose Y", String.format("%.1f in", localization.getY()));
        telemetryEx.addData("Pose H", String.format("%.1f deg", Math.toDegrees(localization.getHeading())));

        // Alignment
        telemetryEx.addData("--- ALIGNMENT ---", "");
        telemetryEx.addData("Dist to Goal", String.format("%.1f in", localization.getDistanceToGoal()));
        telemetryEx.addData("Angle to Goal", String.format("%.1f deg", Math.toDegrees(localization.getAngleToGoal())));
        telemetryEx.addData("Align Error", String.format("%.1f deg", aligner.getAngleErrorDegrees()));
        telemetryEx.addData("Align Turn Out", String.format("%.3f", alignTurnOutput));

        // Kalman
        if (Debug.SHOW_KALMAN_DATA) {
            telemetryEx.addData("--- KALMAN FILTER ---", "");
            telemetryEx.addData("Loc State", localization.getState());
            telemetryEx.addData("Camera Conf", String.format("%.2f", localization.getCameraConfidence()));
            telemetryEx.addData("AprilTag", localization.isAprilTagVisible() ? "VISIBLE" : "NOT VISIBLE");
        }

        // Flywheel
        if (Debug.SHOW_VELOCITY_DATA) {
            telemetryEx.addData("--- FLYWHEEL ---", "");
            telemetryEx.addData("Velocity", String.format("%.0f tps", robot.getFlywheelVelocity()));
            telemetryEx.addData("Target", String.format("%.0f tps", shooter.getTargetFlywheelVelocity()));
            telemetryEx.addData("Filtered Vel", String.format("%.0f tps", shooter.getFilteredFlywheelVelocity()));
            telemetryEx.addData("Vel Error", String.format("%.0f tps", shooter.getTargetFlywheelVelocity() - shooter.getFlywheelVelocity()));
            telemetryEx.addData("Flywheel Ready", shooter.isFlywheelReady() ? "YES" : "NO");
        }

        // Hood (LOOKUP TABLE VERSION)
        telemetryEx.addData("--- HOOD (LOOKUP TABLE) ---", "");
        telemetryEx.addData("Hood Angle", String.format("%.1f deg", shooter.getTargetHoodAngle()));
        telemetryEx.addData("Dist to Goal", String.format("%.1f in", shooter.getCurrentDistance()));
        telemetryEx.addData("Dist Offset", String.format("%.2f m", ShooterLookupTable.distanceOffset));
        telemetryEx.addData("In Range", shooter.isDistanceInRange() ? "YES" : "NO (CLAMPED)");
        telemetryEx.addData("Nearest Cal Dist", String.format("%.1f in", shooter.getNearestCalibratedDistance()));

        // Intake
        telemetryEx.addData("--- INTAKE ---", "");
        telemetryEx.addData("Intake", intakeActive ? "ON" : "OFF");
        telemetryEx.addData("Gate", shooter.isShooting() ? "OPEN" : "CLOSED");

        // Velocities
        telemetryEx.addData("--- VELOCITIES ---", "");
        telemetryEx.addData("Vel X", String.format("%.1f in/s", localization.getVelocityX()));
        telemetryEx.addData("Vel Y", String.format("%.1f in/s", localization.getVelocityY()));
        telemetryEx.addData("Vel Mag", String.format("%.1f in/s", localization.getVelocityMagnitude()));

        telemetryEx.update();
    }

    private String getRobotState() {
        if (shooter.isShooting()) return "SHOOTING";
        if (shooter.isReady() && aligner.isAligned()) return "READY";
        if (aligner.isAligning()) return "ALIGNING";
        if (shooter.isFlywheelReady()) return "SPIN_UP";
        return "INTAKE";
    }

    private void setGamepadLEDs() {
        gamepad1.setLedColor(0, 255, 0, LED_DURATION_CONTINUOUS);
        if (selectedAlliance == NewHardware.Alliance.RED) {
            gamepad2.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(0, 0, 255, LED_DURATION_CONTINUOUS);
        }
    }
}
