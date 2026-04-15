package org.firstinspires.ftc.teamcode.opmode_new;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.util.MathUtils;

import static org.firstinspires.ftc.teamcode.opmode_new.TuningConfig.*;

/**
 * Odometry class for tracking robot position using IMU and 2 dead wheel encoders.
 */
public class NewOdometry {

    private NewHardware robot;

    private int prevForwardTicks = 0;
    private int prevLateralTicks = 0;

    private Pose2d currentPose = new Pose2d(0, 0, 0);

    private double velocityX = 0;
    private double velocityY = 0;
    private double angularVelocity = 0;
    private double prevHeading = 0;
    private double prevTime = 0;

    private double accelX = 0;
    private double accelY = 0;

    private double headingOffset = 0;
    private int allianceMultiplier = 1;
    private double totalDistanceDriven = 0;

    public void init(NewHardware robot) {
        this.robot = robot;

        prevForwardTicks = robot.getForwardOdometry();
        prevLateralTicks = robot.getLateralOdometry();

        if (Field.GOAL_X < 0) {
            allianceMultiplier = -1;
            currentPose = new Pose2d(-Field.GOAL_X, Field.GOAL_Y, Math.PI);
        } else {
            allianceMultiplier = 1;
            currentPose = new Pose2d(Field.GOAL_X, Field.GOAL_Y, 0);
        }

        headingOffset = 0;
        prevHeading = currentPose.getHeading();
        prevTime = System.nanoTime() / 1e9;
    }

    public Pose2d update() {
        return update(System.nanoTime() / 1e9);
    }

    public Pose2d update(double timestamp) {
        int currentForwardTicks = robot.getForwardOdometry();
        int currentLateralTicks = robot.getLateralOdometry();

        int deltaForward = currentForwardTicks - prevForwardTicks;
        int deltaLateral = currentLateralTicks - prevLateralTicks;

        prevForwardTicks = currentForwardTicks;
        prevLateralTicks = currentLateralTicks;

        double forwardInches = deadWheelTicksToInches(deltaForward);
        double lateralInches = deadWheelTicksToInches(deltaLateral);

        totalDistanceDriven += Math.sqrt(forwardInches * forwardInches + lateralInches * lateralInches);

        double rawHeading = robot.getHeadingRadians();
        double heading = rawHeading + headingOffset;
        heading *= allianceMultiplier;
        heading = MathUtils.normalizeRadians(heading, false);

        double dt = timestamp - prevTime;
        dt = Math.max(0.001, Math.min(dt, 0.1));
        prevTime = timestamp;

        double deltaHeading = MathUtils.normalizeRadians(heading - prevHeading, false);

        double cosH = Math.cos(prevHeading);
        double sinH = Math.sin(prevHeading);

        double globalDeltaX = forwardInches * cosH - lateralInches * sinH;
        double globalDeltaY = forwardInches * sinH + lateralInches * cosH;

        double newX = currentPose.getX() + globalDeltaX;
        double newY = currentPose.getY() + globalDeltaY;

        prevHeading = heading;

        currentPose = new Pose2d(newX, newY, new Rotation2d(heading));

        double alpha = 0.3;

        if (dt > 0) {
            double instantVelX = globalDeltaX / dt;
            double instantVelY = globalDeltaY / dt;
            double instantOmega = deltaHeading / dt;

            velocityX = alpha * instantVelX + (1 - alpha) * velocityX;
            velocityY = alpha * instantVelY + (1 - alpha) * velocityY;
            angularVelocity = alpha * instantOmega + (1 - alpha) * angularVelocity;
        }

        return currentPose;
    }

    public Pose2d predictPose(double currentTimestamp, double targetTimestamp) {
        double dt = targetTimestamp - currentTimestamp;
        dt = Math.max(-0.5, Math.min(dt, 0.5));

        double predictedX = currentPose.getX() + velocityX * dt;
        double predictedY = currentPose.getY() + velocityY * dt;
        double predictedHeading = MathUtils.normalizeRadians(
                currentPose.getHeading() + angularVelocity * dt, false);

        return new Pose2d(predictedX, predictedY, new Rotation2d(predictedHeading));
    }

    public void setPose(Pose2d pose) {
        this.currentPose = pose;
        prevForwardTicks = robot.getForwardOdometry();
        prevLateralTicks = robot.getLateralOdometry();
        velocityX = 0;
        velocityY = 0;
        angularVelocity = 0;
    }

    public void setPose(double x, double y, double headingRadians) {
        setPose(new Pose2d(x, y, new Rotation2d(headingRadians)));
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public double getHeading() {
        return currentPose.getRotation().getRadians();
    }

    public double getX() {
        return currentPose.getX();
    }

    public double getY() {
        return currentPose.getY();
    }

    public double getVelocityX() {
        return velocityX;
    }

    public double getVelocityY() {
        return velocityY;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public double getVelocityMagnitude() {
        return Math.sqrt(velocityX * velocityX + velocityY * velocityY);
    }

    public double getAccelerationX() {
        return accelX;
    }

    public double getAccelerationY() {
        return accelY;
    }

    public void setHeadingOffset(double offset) {
        this.headingOffset = offset;
    }

    public void resetHeadingOffset() {
        this.headingOffset = -robot.getHeadingRadians() * allianceMultiplier;
    }

    private double deadWheelTicksToInches(int ticks) {
        return ticks * (Math.PI * Localization.DEAD_WHEEL_DIAMETER) / Localization.ENCODER_TICKS_PER_REV;
    }

    public double getDistanceToGoal() {
        double goalX = Field.GOAL_X * allianceMultiplier;
        double dx = goalX - currentPose.getX();
        double dy = Field.GOAL_Y - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getAngleToGoal() {
        double goalX = Field.GOAL_X * allianceMultiplier;
        double dx = goalX - currentPose.getX();
        double dy = Field.GOAL_Y - currentPose.getY();
        return Math.atan2(dy, dx);
    }

    public double getHeadingErrorToGoal() {
        double currentHeading = getHeading();
        double desiredHeading = getAngleToGoal();
        return MathUtils.normalizeRadians(desiredHeading - currentHeading, false);
    }

    public double getTotalDistanceDriven() {
        return totalDistanceDriven;
    }

    public void resetTotalDistance() {
        totalDistanceDriven = 0;
    }
}
