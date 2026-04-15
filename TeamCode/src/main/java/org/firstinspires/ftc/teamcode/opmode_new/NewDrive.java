package org.firstinspires.ftc.teamcode.opmode_new;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.opmode_new.TuningConfig.*;

/**
 * Mecanum Drive utility class.
 * 
 * ALIGNMENT LOCK PROTECTION: Lock is disabled whenever:
 * - Auto-alignment is active
 * - User provides any movement input
 * - Lock was just engaged (prevents locking during alignment)
 */
public class NewDrive {

    private NewHardware robot;

    public enum DriveState {
        DRIVE,
        STOPPING,
        LOCKED
    }

    public DriveState drivetrainState = DriveState.DRIVE;
    private ElapsedTime stopTimer = new ElapsedTime();
    private ElapsedTime lockTimer = new ElapsedTime();

    private boolean isAligningActive = false;
    private double autoAlignTurn = 0;

    private boolean userWantsToMove = false;

    private double previousStrafeX = 0;
    private double previousStrafeY = 0;
    private double previousTurn = 0;
    private ElapsedTime slewTimer = new ElapsedTime();

    private ElapsedTime alignmentActiveTimer = new ElapsedTime();
    private boolean wasLockedLastCycle = false;

    public void init(NewHardware robot) {
        this.robot = robot;
    }

    public void setAutoAlignTurn(double turn) {
        this.autoAlignTurn = turn;
    }

    public void setAligningActive(boolean aligning) {
        boolean wasAligning = this.isAligningActive;
        this.isAligningActive = aligning;

        if (wasAligning && !aligning) {
            alignmentActiveTimer.reset();
        }
    }

    public void setLockEnabled(boolean enabled) {
        // Lock is controlled by TuningConfig.LOCK.LOCK_ENABLED in teleop
    }

    public void setUserWantsToMove(boolean wantsToMove) {
        this.userWantsToMove = wantsToMove;
    }

    private boolean shouldEngageLock() {
        if (isAligningActive) return false;
        if (alignmentActiveTimer.seconds() < Lock.ALIGNMENT_COOLDOWN_SEC) return false;
        if (!Lock.LOCK_ENABLED) return false;
        if (userWantsToMove) return false;
        return true;
    }

    private double applySlewLimit(double input, double previous, double dt, double slewRate) {
        double delta = input - previous;
        double maxDelta = slewRate * dt;

        if (delta > maxDelta) return previous + maxDelta;
        if (delta < -maxDelta) return previous - maxDelta;
        return input;
    }

    public void drive(com.qualcomm.robotcore.hardware.Gamepad gamepad1, boolean imuResetButton) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double dt = slewTimer.seconds();
        if (dt < 0.001) dt = 0.001;

        double targetX = applySlewLimit(x, previousStrafeX, dt, Drive.STRAFE_SLEW_RATE);
        double targetY = applySlewLimit(y, previousStrafeY, dt, Drive.STRAFE_SLEW_RATE);
        double targetRx = applySlewLimit(rx, previousTurn, dt, Drive.TURN_SLEW_RATE);

        previousStrafeX = targetX;
        previousStrafeY = targetY;
        previousTurn = targetRx;

        boolean movementInput = Math.abs(x) > Drive.INPUT_DEADZONE ||
                Math.abs(y) > Drive.INPUT_DEADZONE ||
                (Math.abs(rx) > Drive.INPUT_DEADZONE && !isAligningActive);

        switch (drivetrainState) {
            case DRIVE:
                if (!movementInput) {
                    if (shouldEngageLock()) {
                        setMotorPower(0);
                        stopTimer.reset();
                        drivetrainState = DriveState.STOPPING;
                    }
                } else {
                    if (imuResetButton) {
                        robot.resetIMU();
                    }

                    double botHeading = robot.getHeadingRadians();

                    double rotX = targetX * Math.cos(-botHeading) - targetY * Math.sin(-botHeading);
                    double rotY = targetX * Math.sin(-botHeading) + targetY * Math.cos(-botHeading);

                    double finalRx = isAligningActive ? autoAlignTurn : targetRx;

                    rotX *= Drive.SPEED_MULTIPLIER;
                    rotY *= Drive.SPEED_MULTIPLIER;

                    double denominator = Math.max(
                            Math.abs(rotY) + Math.abs(rotX) + Math.abs(finalRx), 1);

                    double frontLeftPower = (rotY + rotX + finalRx) / denominator;
                    double backLeftPower = (rotY - rotX + finalRx) / denominator;
                    double frontRightPower = (rotY - rotX - finalRx) / denominator;
                    double backRightPower = (rotY + rotX - finalRx) / denominator;

                    robot.FL.setPower(frontLeftPower);
                    robot.BL.setPower(backLeftPower);
                    robot.FR.setPower(frontRightPower);
                    robot.BR.setPower(backRightPower);
                }
                break;

            case STOPPING:
                if (!shouldEngageLock()) {
                    if (movementInput) {
                        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER,
                                robot.FL, robot.FR, robot.BL, robot.BR);
                        drivetrainState = DriveState.DRIVE;
                    }
                } else if (stopTimer.milliseconds() >= Lock.LOCK_STOP_DELAY_MS) {
                    robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER,
                            robot.FL, robot.FR, robot.BL, robot.BR);
                    setMotorTarget(0);
                    robot.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION,
                            robot.FL, robot.FR, robot.BL, robot.BR);
                    setMotorPower(Lock.LOCK_START_POWER);
                    lockTimer.reset();
                    wasLockedLastCycle = true;

                    drivetrainState = DriveState.LOCKED;
                } else if (movementInput) {
                    robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER,
                            robot.FL, robot.FR, robot.BL, robot.BR);
                    drivetrainState = DriveState.DRIVE;
                }
                break;

            case LOCKED:
                double lockPower;
                if (lockTimer.milliseconds() >= Lock.LOCK_RAMP_TIME_MS) {
                    lockPower = Lock.LOCK_MAX_POWER;
                } else {
                    lockPower = Lock.LOCK_START_POWER + 
                            (Lock.LOCK_MAX_POWER - Lock.LOCK_START_POWER) * 
                            (lockTimer.milliseconds() / Lock.LOCK_RAMP_TIME_MS);
                }
                setMotorPower(lockPower);

                if (movementInput || !shouldEngageLock() || isAligningActive) {
                    setMotorPower(0);
                    robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER,
                            robot.FL, robot.FR, robot.BL, robot.BR);
                    drivetrainState = DriveState.DRIVE;
                    wasLockedLastCycle = false;
                }
                break;
        }

        slewTimer.reset();
    }

    private void setMotorPower(double power) {
        robot.FL.setPower(power);
        robot.FR.setPower(power);
        robot.BL.setPower(power);
        robot.BR.setPower(power);
    }

    private void setMotorTarget(int target) {
        robot.FL.setTargetPosition(target);
        robot.FR.setTargetPosition(target);
        robot.BL.setTargetPosition(target);
        robot.BR.setTargetPosition(target);
    }

    public void resetIMU() {
        robot.resetIMU();
    }

    public DriveState getDriveState() {
        return drivetrainState;
    }

    public boolean isLocked() {
        return drivetrainState == DriveState.LOCKED;
    }

    public boolean isAligning() {
        return isAligningActive;
    }

    public boolean isLockPrevented() {
        if (isAligningActive) return true;
        if (alignmentActiveTimer.seconds() < Lock.ALIGNMENT_COOLDOWN_SEC) return true;
        return false;
    }

    public double getLockCooldownRemaining() {
        double remaining = Lock.ALIGNMENT_COOLDOWN_SEC - alignmentActiveTimer.seconds();
        return Math.max(0, remaining);
    }

    public void forceDisengageLock() {
        if (drivetrainState == DriveState.LOCKED) {
            setMotorPower(0);
            robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER,
                    robot.FL, robot.FR, robot.BL, robot.BR);
            drivetrainState = DriveState.DRIVE;
            wasLockedLastCycle = false;
        }
    }
}
