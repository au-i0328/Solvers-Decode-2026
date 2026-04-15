package org.firstinspires.ftc.teamcode.opmode_new;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.opmode_new.TuningConfig.*;

/**
 * Hardware class for the new mecanum drive robot.
 * Contains all hardware declarations and initialization.
 */
public class NewHardware {

    // ==================== ALLIANCE ====================
    public enum Alliance {
        RED,
        BLUE
    }
    public Alliance alliance = Alliance.BLUE;

    // ==================== DRIVETRAIN MOTORS ====================
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;

    // ==================== DEAD WHEEL ODOMETRY MOTORS ====================
    public DcMotor forwardOdometry;
    public DcMotor lateralOdometry;

    // ==================== FLYWHEEL MOTORS ====================
    public DcMotorEx flywheelLeft;
    public DcMotorEx flywheelRight;

    // ==================== INTAKE MOTOR ====================
    public DcMotor intakeMotor;

    // ==================== SERVOS ====================
    public Servo hoodLeft;
    public Servo hoodRight;
    public Servo gateServo;

    // ==================== IMU ====================
    public IMU imu;

    // ==================== LIMELIGHT ====================
    public Limelight3A limelight;
    public LimelightHelper limelightHelper;

    // ==================== HARDWARE MAP ====================
    private HardwareMap hardwareMap;

    // ==================== FLYWHEEL PID CONTROLLER ====================
    private FlywheelPID flywheelPIDLeft;
    private FlywheelPID flywheelPIDRight;

    // ==================== TIMING ====================
    private ElapsedTime loopTimer = new ElapsedTime();

    /**
     * Initialize all robot hardware
     */
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.hardwareMap = hwMap;

        initDrivetrain();
        initOdometry();
        initFlywheel();
        initIntake();
        initServos();
        initIMU();
        initLimelight(hwMap, telemetry);
    }

    /**
     * Initialize drivetrain motors (FL, FR, BL, BR)
     */
    private void initDrivetrain() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        // Set motor directions
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behavior to BRAKE
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset and configure encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, FL, FR, BL, BR);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, FL, FR, BL, BR);
    }

    /**
     * Initialize dead wheel odometry motors
     */
    private void initOdometry() {
        forwardOdometry = hardwareMap.get(DcMotor.class, "forwardOdometry");
        lateralOdometry = hardwareMap.get(DcMotor.class, "lateralOdometry");

        forwardOdometry.setDirection(DcMotorSimple.Direction.FORWARD);
        lateralOdometry.setDirection(DcMotorSimple.Direction.FORWARD);

        forwardOdometry.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lateralOdometry.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, forwardOdometry, lateralOdometry);
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, forwardOdometry, lateralOdometry);
    }

    /**
     * Initialize flywheel shooter motors with encoders
     */
    private void initFlywheel() {
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");

        flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelPIDLeft = new FlywheelPID();
        flywheelPIDRight = new FlywheelPID();
    }

    /**
     * Initialize intake motor (no encoder)
     */
    private void initIntake() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Initialize servos (hood and gate)
     */
    private void initServos() {
        hoodLeft = hardwareMap.get(Servo.class, "hoodLeft");
        hoodRight = hardwareMap.get(Servo.class, "hoodRight");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        hoodLeft.setPosition(Hood.MIN_SERVO_POS);
        hoodRight.setPosition(1.0 - Hood.MIN_SERVO_POS);
        gateServo.setPosition(Gate.CLOSED_POSITION);
    }

    /**
     * Initialize IMU
     */
    private void initIMU() {
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));
    }

    /**
     * Initialize Limelight 3A
     */
    private void initLimelight(HardwareMap hwMap, Telemetry telemetry) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        if (alliance == Alliance.RED) {
            limelight.pipelineSwitch(1);
        } else {
            limelight.pipelineSwitch(0);
        }

        limelight.start();
        limelightHelper = new LimelightHelper(this, telemetry);
    }

    /**
     * Set alliance color and update limelight pipeline
     */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        if (limelight != null) {
            limelight.pipelineSwitch(alliance == Alliance.RED ? 1 : 0);
        }
    }

    // ==================== MOTOR CONTROL ====================

    public void setMotorMode(DcMotor.RunMode mode, DcMotor... motors) {
        for (DcMotor m : motors) m.setMode(mode);
    }

    public void setMotorPower(double power, DcMotor... motors) {
        for (DcMotor m : motors) m.setPower(power);
    }

    public void setFlywheelVelocity(double targetTicksPerSecond) {
        if (flywheelPIDLeft == null || flywheelPIDRight == null) return;

        double currentVelLeft = flywheelLeft.getVelocity();
        double currentVelRight = flywheelRight.getVelocity();

        double pidLeft = flywheelPIDLeft.calculate(targetTicksPerSecond, currentVelLeft);
        double pidRight = flywheelPIDRight.calculate(targetTicksPerSecond, currentVelRight);

        flywheelLeft.setPower(Math.max(0, Math.min(Flywheel.MAX_POWER, pidLeft)));
        flywheelRight.setPower(Math.max(0, Math.min(Flywheel.MAX_POWER, pidRight)));
    }

    public double getFlywheelVelocity() {
        if (flywheelLeft == null || flywheelRight == null) return 0;
        return (flywheelLeft.getVelocity() + flywheelRight.getVelocity()) / 2.0;
    }

    public double getFlywheelLeftVelocity() {
        return flywheelLeft != null ? flywheelLeft.getVelocity() : 0;
    }

    public double getFlywheelRightVelocity() {
        return flywheelRight != null ? flywheelRight.getVelocity() : 0;
    }

    // ==================== INTAKE ====================

    public void intakeForward() {
        intakeMotor.setPower(Intake.FORWARD_POWER);
    }

    public void intakeReverse() {
        intakeMotor.setPower(Intake.REVERSE_POWER);
    }

    public void intakeStop() {
        intakeMotor.setPower(Intake.STOP_POWER);
    }

    // ==================== GATE ====================

    public void gateOpen() {
        gateServo.setPosition(Gate.OPEN_POSITION);
    }

    public void gateClose() {
        gateServo.setPosition(Gate.CLOSED_POSITION);
    }

    // ==================== HOOD ====================

    public void setHoodAngle(double angleDegrees) {
        angleDegrees = Math.max(Hood.MIN_ANGLE, Math.min(Hood.MAX_ANGLE, angleDegrees));

        double servoPos = (angleDegrees - Hood.MIN_ANGLE) /
                (Hood.MAX_ANGLE - Hood.MIN_ANGLE) *
                (Hood.MAX_SERVO_POS - Hood.MIN_SERVO_POS) + Hood.MIN_SERVO_POS;

        hoodLeft.setPosition(servoPos + Hood.LEFT_OFFSET);
        hoodRight.setPosition((1.0 - servoPos) + Hood.RIGHT_OFFSET);
    }

    // ==================== IMU ====================

    public void resetIMU() {
        imu.resetYaw();
    }

    public double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(
                com.qualcomm.robotcore.external.navigation.AngleUnit.RADIANS
        );
    }

    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(
                com.qualcomm.robotcore.external.navigation.AngleUnit.DEGREES
        );
    }

    // ==================== ODOMETRY ====================

    public int getForwardOdometry() {
        return forwardOdometry.getCurrentPosition();
    }

    public int getLateralOdometry() {
        return lateralOdometry.getCurrentPosition();
    }

    // ==================== FLYWHEEL PID ====================

    public class FlywheelPID {
        private double integral = 0;
        private double previousError = 0;
        private ElapsedTime timer = new ElapsedTime();
        private static final double MAX_INTEGRAL = 1000.0;

        public double calculate(double target, double current) {
            double dt = Math.max(0.001, timer.seconds());

            double error = target - current;
            double pTerm = Flywheel.FLYWHEEL_KP * error;

            integral += error * dt;
            integral = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integral));
            double iTerm = Flywheel.FLYWHEEL_KI * integral;

            double dTerm = Flywheel.FLYWHEEL_KD * (error - previousError) / dt;
            previousError = error;

            double fTerm = Flywheel.FLYWHEEL_KF * target;

            timer.reset();

            return pTerm + iTerm + dTerm + fTerm;
        }

        public void reset() {
            integral = 0;
            previousError = 0;
            timer.reset();
        }
    }

    // ==================== LIMELIGHT HELPER ====================

    public class LimelightHelper {
        private final NewHardware robot;
        private final Telemetry telemetry;
        private LLResult result;
        private double tx, ty, ta;
        private boolean lastValid = false;
        private com.seattlesolvers.solverslib.geometry.Pose2d[] poseHistory = 
                new com.seattlesolvers.solverslib.geometry.Pose2d[5];
        private int poseHistoryIndex = 0;

        public LimelightHelper(NewHardware robot, Telemetry telemetry) {
            this.robot = robot;
            this.telemetry = telemetry;
        }

        public void update() {
            result = limelight.getLatestResult();
            if (targetsValid()) {
                tx = result.getTx();
                ty = result.getTy();
                ta = result.getTa();
                lastValid = true;
            } else {
                lastValid = false;
            }
        }

        public boolean targetsValid() {
            return result != null && result.isValid();
        }

        public double getTx() {
            return targetsValid() ? tx : 0;
        }

        public double getTy() {
            return targetsValid() ? ty : 0;
        }

        public double getTa() {
            return targetsValid() ? ta : 0;
        }

        public int getTargetCount() {
            if (!targetsValid() || result == null) return 0;
            return result.getBotpose().length > 0 ? 1 : 0;
        }

        public double getDistanceMeters() {
            if (!targetsValid()) return 0;

            double targetHeightMeters = Vision.GOAL_HEIGHT_INCHES * 0.0254;
            double cameraHeightMeters = Vision.LENS_HEIGHT_INCHES * 0.0254;
            double mountAngleDegrees = Vision.MOUNT_ANGLE_DEGREES;

            double angleToTargetRad = Math.toRadians(mountAngleDegrees + ty);
            double heightDiff = targetHeightMeters - cameraHeightMeters;

            if (Math.abs(angleToTargetRad) < 0.001) return 0;

            return heightDiff / Math.tan(angleToTargetRad);
        }

        public double getDistanceInches() {
            return getDistanceMeters() / 0.0254;
        }

        public com.seattlesolvers.solverslib.geometry.Pose2d getAprilTagPose() {
            if (!targetsValid() || result == null) return null;

            try {
                double[] botpose = result.getBotpose();
                if (botpose != null && botpose.length >= 6) {
                    double llx = botpose[0];
                    double lly = botpose[1];
                    double yaw = Math.toRadians(botpose[4]);

                    com.seattlesolvers.solverslib.geometry.Pose2d pose = 
                            new com.seattlesolvers.solverslib.geometry.Pose2d(
                                    lly, llx,
                                    com.seattlesolvers.solverslib.geometry.Rotation2d.fromDegrees(yaw)
                            );

                    poseHistory[poseHistoryIndex] = pose;
                    poseHistoryIndex = (poseHistoryIndex + 1) % poseHistory.length;

                    return pose;
                }
            } catch (Exception e) {}

            return null;
        }

        public com.seattlesolvers.solverslib.geometry.Pose2d getAveragedAprilTagPose() {
            if (!targetsValid() || result == null) return null;

            com.seattlesolvers.solverslib.geometry.Pose2d rawPose = getAprilTagPose();
            if (rawPose == null) return null;

            int validCount = 0;
            double sumX = 0, sumY = 0, sumH = 0;

            for (int i = 0; i < poseHistory.length; i++) {
                if (poseHistory[i] != null) {
                    sumX += poseHistory[i].getX();
                    sumY += poseHistory[i].getY();
                    sumH += poseHistory[i].getHeading();
                    validCount++;
                }
            }

            if (validCount == 0) return rawPose;

            double avgH = sumH / validCount;
            while (avgH > Math.PI) avgH -= 2 * Math.PI;
            while (avgH < -Math.PI) avgH += 2 * Math.PI;

            return new com.seattlesolvers.solverslib.geometry.Pose2d(
                    sumX / validCount, sumY / validCount,
                    com.seattlesolvers.solverslib.geometry.Rotation2d.fromRadians(avgH)
            );
        }

        public double getAprilTagDistance() {
            if (!targetsValid() || result == null) return 0;

            try {
                double[] botpose = result.getBotpose();
                if (botpose != null && botpose.length >= 3) {
                    return Math.sqrt(botpose[0] * botpose[0] + botpose[1] * botpose[1]);
                }
            } catch (Exception e) {}

            return 0;
        }

        public double getTimestamp() {
            if (!targetsValid() || result == null) return 0;
            return result.getTimestamp();
        }

        public double getLatency() {
            if (!targetsValid() || result == null) return 0;
            return result.getLatency();
        }

        public void addTelemetry() {
            if (targetsValid()) {
                telemetry.addData("LL TX", "%.2f", tx);
                telemetry.addData("LL TY", "%.2f", ty);
                telemetry.addData("LL TA", "%.2f", ta);
                telemetry.addData("LL Dist", "%.2fm", getDistanceMeters());
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
        }
    }
}
