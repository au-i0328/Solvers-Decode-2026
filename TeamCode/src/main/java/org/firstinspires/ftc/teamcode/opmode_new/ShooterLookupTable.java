package org.firstinspires.ftc.teamcode.opmode_new;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.opmode_new.TuningConfig.*;

/**
 * Shooter subsystem using LINEAR INTERPOLATION lookup table for hood control.
 * 
 * USE THIS WHEN:
 * - Physics model is difficult to calibrate
 * - You want predictable, tested behavior
 * - You have tested values from practice
 * 
 * USE PHYSICS WHEN:
 * - You need shots at untested distances
 * - You want a single source of truth
 * - You have accurate measurements of all physical constants
 * 
 * CONTROL SCHEME (same as physics version):
 * - startFlywheel(): Spin up flywheel to target RPM
 * - startShooting(): Open gate for SHOOT_DELAY seconds
 * - isReady(): Flywheel at target velocity
 * - update(): Call every loop
 */
public class ShooterLookupTable {

    private NewHardware robot;
    private LocalizationManager localization;

    public enum ShooterState {
        OFF,
        SPINNING_UP,
        READY,
        SHOOTING
    }

    public ShooterState state = ShooterState.OFF;

    private double targetFlywheelVelocity = 0;
    private double targetHoodAngle = Hood.MIN_ANGLE;

    private boolean flywheelReady = false;

    private double shootStartTime = 0;
    private double currentTime = 0;

    public static double distanceOffset = 0;

    private double lastMeasuredVelocity = 0;
    private double velocityAcceleration = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();

    private static final int VELOCITY_HISTORY_SIZE = 5;
    private double[] velocityHistory = new double[VELOCITY_HISTORY_SIZE];
    private int velocityHistoryIndex = 0;

    // Velocity compensation factor (inches offset per tick/sec of error)
    // Tune this: positive = faster flywheel → ball goes farther → aim closer
    public static double VELOCITY_COMP_FACTOR = 0.00015;

    public void init(NewHardware robot, LocalizationManager localization) {
        this.robot = robot;
        this.localization = localization;

        state = ShooterState.OFF;
        targetFlywheelVelocity = 0;
        targetHoodAngle = Hood.MIN_ANGLE;

        robot.setHoodAngle(Hood.MIN_ANGLE);
        robot.gateClose();

        for (int i = 0; i < VELOCITY_HISTORY_SIZE; i++) {
            velocityHistory[i] = 0;
        }
    }

    public void startFlywheel() {
        targetFlywheelVelocity = Flywheel.TARGET_VELOCITY;
        state = ShooterState.SPINNING_UP;
    }

    public void stopFlywheel() {
        targetFlywheelVelocity = 0;
        robot.setFlywheelVelocity(0);
        state = ShooterState.OFF;
        flywheelReady = false;
    }

    public void update(double time) {
        currentTime = time;

        updateVelocityTracking();
        calculateHoodAngle();

        switch (state) {
            case OFF:
                robot.setFlywheelVelocity(0);
                robot.intakeStop();
                robot.gateClose();
                break;

            case SPINNING_UP:
                robot.setFlywheelVelocity(targetFlywheelVelocity);

                double currentVelocity = getFilteredVelocity();
                if (Math.abs(currentVelocity - targetFlywheelVelocity) < Flywheel.VELOCITY_TOLERANCE) {
                    flywheelReady = true;
                    state = ShooterState.READY;
                }

                robot.intakeForward();
                robot.gateClose();
                break;

            case READY:
                robot.setFlywheelVelocity(targetFlywheelVelocity);

                currentVelocity = getFilteredVelocity();
                if (Math.abs(currentVelocity - targetFlywheelVelocity) > Flywheel.VELOCITY_TOLERANCE * 2) {
                    flywheelReady = false;
                    state = ShooterState.SPINNING_UP;
                }

                robot.intakeForward();
                robot.gateClose();
                break;

            case SHOOTING:
                robot.setFlywheelVelocity(targetFlywheelVelocity);
                robot.gateOpen();
                robot.intakeForward();

                if (currentTime - shootStartTime > Shooting.SHOOT_DELAY) {
                    state = ShooterState.READY;
                    robot.gateClose();
                }
                break;
        }
    }

    private void updateVelocityTracking() {
        double currentVel = robot.getFlywheelVelocity();

        velocityHistory[velocityHistoryIndex] = currentVel;
        velocityHistoryIndex = (velocityHistoryIndex + 1) % VELOCITY_HISTORY_SIZE;

        double dt = velocityTimer.seconds();
        if (dt > 0.001) {
            velocityAcceleration = (currentVel - lastMeasuredVelocity) / dt;
        }
        lastMeasuredVelocity = currentVel;
        velocityTimer.reset();
    }

    private double getFilteredVelocity() {
        double sum = 0;
        int count = 0;
        for (int i = 0; i < Flywheel.VELOCITY_HISTORY_SIZE; i++) {
            if (velocityHistory[i] > 0) {
                sum += velocityHistory[i];
                count++;
            }
        }
        return count > 0 ? sum / count : lastMeasuredVelocity;
    }

    /**
     * Calculate hood angle using lookup table.
     * Takes distance from LocalizationManager and applies velocity compensation.
     */
    private void calculateHoodAngle() {
        // Get distance to goal
        double distanceInches = localization.getDistanceToGoal();

        // Apply operator offset
        distanceInches -= distanceOffset * 39.37; // Convert meters to inches

        // Get velocity error
        double velocityError = targetFlywheelVelocity - getFilteredVelocity();

        // Use lookup table with velocity compensation
        targetHoodAngle = HoodLookupTable.getHoodAngleWithCompensation(
                distanceInches,
                velocityError,
                VELOCITY_COMP_FACTOR * 39.37 // Convert to inches
        );

        // Set servo position
        robot.setHoodAngle(targetHoodAngle);
    }

    public void startShooting() {
        if (state == ShooterState.READY || state == ShooterState.SPINNING_UP) {
            state = ShooterState.SHOOTING;
            shootStartTime = currentTime;
            robot.gateOpen();
        }
    }

    public void stopShooting() {
        robot.gateClose();
        if (state == ShooterState.SHOOTING) {
            state = ShooterState.READY;
        }
    }

    public void addDistanceOffset(double delta) {
        distanceOffset += delta;
    }

    public void resetDistanceOffset() {
        distanceOffset = 0;
    }

    public boolean isReady() {
        return state == ShooterState.READY && flywheelReady;
    }

    public boolean isFlywheelReady() {
        return flywheelReady;
    }

    public boolean isShooting() {
        return state == ShooterState.SHOOTING;
    }

    public ShooterState getState() {
        return state;
    }

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public double getFlywheelVelocity() {
        return robot.getFlywheelVelocity();
    }

    public double getFilteredFlywheelVelocity() {
        return getFilteredVelocity();
    }

    public double getVelocityAcceleration() {
        return velocityAcceleration;
    }

    public double getTargetFlywheelVelocity() {
        return targetFlywheelVelocity;
    }

    public double getCurrentDistance() {
        return localization.getDistanceToGoal();
    }

    /**
     * Get the closest calibrated distance in the table.
     * Useful for debugging - shows which table entry is being used.
     */
    public double getNearestCalibratedDistance() {
        double distance = localization.getDistanceToGoal();
        double[] range = HoodLookupTable.getDistanceRange();
        return Math.max(range[0], Math.min(range[1], distance));
    }

    /**
     * Check if current distance is within calibrated range.
     */
    public boolean isDistanceInRange() {
        return HoodLookupTable.isInRange(localization.getDistanceToGoal());
    }
}
