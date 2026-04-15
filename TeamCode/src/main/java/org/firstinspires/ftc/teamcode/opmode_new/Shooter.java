package org.firstinspires.ftc.teamcode.opmode_new;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.opmode_new.TuningConfig.*;

/**
 * Shooter subsystem for the fixed launcher.
 * Features:
 * - Ball flight time compensation (predicts robot position when ball arrives)
 * - Real-time flywheel velocity compensation (adjusts hood based on actual velocity)
 * - Shoot-on-the-move with full trajectory prediction
 */
public class Shooter {

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
    private boolean hoodReady = false;

    private double shootStartTime = 0;
    private double currentTime = 0;

    public static double distanceOffset = 0;
    public static double angleOffset = 0;

    private double lastMeasuredVelocity = 0;
    private double velocityAcceleration = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();

    private double predictedX = 0;
    private double predictedY = 0;
    private double predictedHeading = 0;

    private static final int VELOCITY_HISTORY_SIZE = 5;
    private double[] velocityHistory = new double[VELOCITY_HISTORY_SIZE];
    private int velocityHistoryIndex = 0;

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
        calculateBallFlightPrediction();

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

                calculateHoodAngle();
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

                calculateHoodAngle();
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

    private double getPredictedVelocity() {
        return getFilteredVelocity() + velocityAcceleration * Flywheel.VELOCITY_PREDICTION_TIME;
    }

    private void calculateBallFlightPrediction() {
        double currentX = localization.getX();
        double currentY = localization.getY();
        double currentHeading = localization.getHeading();

        double velX = localization.getVelocityX();
        double velY = localization.getVelocityY();
        double omega = localization.getAngularVelocity();

        double distanceMeters = localization.getDistanceToGoalMeters();
        double estimatedFlightTime = estimateFlightTime(distanceMeters, getFilteredVelocity());

        double totalTime = BallFlight.MECHANICAL_DELAY + estimatedFlightTime;

        double angularContributionX = -omega * Localization.LATERAL_OFFSET * Math.sin(currentHeading);
        double angularContributionY = omega * Localization.LATERAL_OFFSET * Math.cos(currentHeading);

        predictedX = currentX + (velX + angularContributionX) * totalTime;
        predictedY = currentY + (velY + angularContributionY) * totalTime;
        predictedHeading = currentHeading + omega * totalTime;

        while (predictedHeading > Math.PI) predictedHeading -= 2 * Math.PI;
        while (predictedHeading < -Math.PI) predictedHeading += 2 * Math.PI;
    }

    private double estimateFlightTime(double distanceMeters, double flywheelVelocityTPS) {
        double ballVelocity = ticksToVelocity(flywheelVelocityTPS);
        double hoodAngleRad = Math.toRadians(targetHoodAngle);
        double horizontalVelocity = ballVelocity * Math.sin(hoodAngleRad);

        if (horizontalVelocity < 0.1) {
            return BallFlight.MIN_FLIGHT_TIME;
        }

        double flightTime = distanceMeters / horizontalVelocity;
        return Math.max(BallFlight.MIN_FLIGHT_TIME, 
                Math.min(flightTime, BallFlight.MAX_FLIGHT_TIME));
    }

    private double ticksToVelocity(double ticksPerSecond) {
        return ticksPerSecond / LauncherMath.TICKS_TO_VELOCITY;
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

    private void calculateHoodAngle() {
        double goalX = Field.GOAL_X * localization.getAllianceMultiplier();
        double goalY = Field.GOAL_Y;

        double predictedDistanceX = goalX - predictedX;
        double predictedDistanceY = goalY - predictedY;
        double predictedDistanceInches = Math.sqrt(predictedDistanceX * predictedDistanceX + predictedDistanceY * predictedDistanceY);
        double predictedDistanceMeters = predictedDistanceInches * 0.0254;

        predictedDistanceMeters += distanceOffset;
        predictedDistanceMeters += calculateVelocityCompensation(predictedDistanceMeters);

        predictedDistanceMeters = Math.max(0.5, Math.min(predictedDistanceMeters, 10.0));

        double[] launchParams = NewMathFunctions.distanceToLauncherValues(predictedDistanceMeters);

        if (!Double.isNaN(launchParams[1])) {
            targetHoodAngle = launchParams[1] + angleOffset;
            targetHoodAngle += calculateVelocityAngleAdjustment(predictedDistanceMeters);
            targetHoodAngle = Math.max(Hood.MIN_ANGLE, Math.min(Hood.MAX_ANGLE, targetHoodAngle));
            robot.setHoodAngle(targetHoodAngle);
        }
    }

    private double calculateVelocityCompensation(double distanceMeters) {
        double currentVelocity = getFilteredVelocity();
        double targetVelocity = targetFlywheelVelocity;

        double velocityError = targetVelocity - currentVelocity;
        if (Math.abs(velocityError) < Flywheel.VELOCITY_COMP_THRESHOLD) {
            return 0;
        }

        return -velocityError * Flywheel.VELOCITY_COMP_FACTOR;
    }

    private double calculateVelocityAngleAdjustment(double distanceMeters) {
        double currentVelocity = getFilteredVelocity();
        double predictedVelocity = getPredictedVelocity();

        double currentBallVelocity = ticksToVelocity(currentVelocity);
        double currentAngle = NewMathFunctions.getHoodAngleFromVelocity(distanceMeters, currentBallVelocity);

        double targetBallVelocity = ticksToVelocity(targetFlywheelVelocity);
        double targetAngle = NewMathFunctions.getHoodAngleFromVelocity(distanceMeters, targetBallVelocity);

        if (Double.isNaN(currentAngle) || Double.isNaN(targetAngle)) {
            return 0;
        }

        double predictedBallVelocity = ticksToVelocity(predictedVelocity);
        double predictedAngle = NewMathFunctions.getHoodAngleFromVelocity(distanceMeters, predictedBallVelocity);

        if (Double.isNaN(predictedAngle)) {
            double velocityRatio = (predictedVelocity - targetVelocity) / 
                    (Math.abs(currentVelocity - targetVelocity) + 0.001);
            return (targetAngle - currentAngle) * velocityRatio;
        }

        return predictedAngle - targetAngle;
    }

    public void addDistanceOffset(double delta) {
        distanceOffset += delta;
    }

    public void resetDistanceOffset() {
        distanceOffset = 0;
    }

    public void addAngleOffset(double delta) {
        angleOffset += delta;
        angleOffset = Math.max(-5, Math.min(5, angleOffset));
    }

    public void resetAngleOffset() {
        angleOffset = 0;
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

    public double getPredictedFlywheelVelocity() {
        return getPredictedVelocity();
    }

    public double getVelocityAcceleration() {
        return velocityAcceleration;
    }

    public double getTargetFlywheelVelocity() {
        return targetFlywheelVelocity;
    }

    public boolean isShotPossible() {
        double distMeters = localization.getDistanceToGoalMeters() + distanceOffset;
        double[] launchParams = NewMathFunctions.distanceToLauncherValues(distMeters);
        return !Double.isNaN(launchParams[0]);
    }

    public double getCurrentDistance() {
        return localization.getDistanceToGoal();
    }

    public double getCurrentDistanceMeters() {
        return localization.getDistanceToGoalMeters();
    }

    public double getPredictedDistance() {
        double goalX = Field.GOAL_X * localization.getAllianceMultiplier();
        double goalY = Field.GOAL_Y;
        double dx = goalX - predictedX;
        double dy = goalY - predictedY;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
