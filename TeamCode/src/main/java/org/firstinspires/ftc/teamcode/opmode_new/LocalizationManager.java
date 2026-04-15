package org.firstinspires.ftc.teamcode.opmode_new;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.util.MathUtils;

import static org.firstinspires.ftc.teamcode.opmode_new.TuningConfig.*;

/**
 * Localization Manager - Central hub for robot pose estimation.
 * Fuses dead wheel odometry with Limelight 3A AprilTag pose estimates using
 * an Extended Kalman Filter for optimal accuracy.
 */
public class LocalizationManager {

    private NewHardware robot;
    private NewOdometry odometry;
    private KalmanFilter kalmanFilter;

    private double lastUpdateTime = 0;
    private double currentTime = 0;

    public enum LocState {
        ODOMETRY_ONLY,
        FUSING,
        RECOVERING
    }
    public LocState state = LocState.ODOMETRY_ONLY;

    private double lastUpdateQuality = 0;
    private double cameraConfidence = 0;
    private int consecutiveMisses = 0;

    private static final int POSE_HISTORY_SIZE = 20;
    private Pose2d[] poseHistory = new Pose2d[POSE_HISTORY_SIZE];
    private double[] poseTimestamps = new double[POSE_HISTORY_SIZE];
    private int poseHistoryIndex = 0;

    private double lastCorrectionX = 0;
    private double lastCorrectionY = 0;
    private double lastCorrectionH = 0;
    private double lastInnovationX = 0;
    private double lastInnovationY = 0;
    private double lastInnovationH = 0;
    private Pose2d lastGoodCameraPose = null;

    public void init(NewHardware robot, NewOdometry odometry) {
        this.robot = robot;
        this.odometry = odometry;

        kalmanFilter = new KalmanFilter();
        kalmanFilter.setProcessNoise(
                Localization.PROCESS_NOISE_X,
                Localization.PROCESS_NOISE_Y,
                Localization.PROCESS_NOISE_HEADING
        );
        kalmanFilter.setMeasurementNoise(
                Localization.CAMERA_NOISE_X,
                Localization.CAMERA_NOISE_Y,
                Localization.CAMERA_NOISE_HEADING
        );
        kalmanFilter.init(odometry.getPose());

        for (int i = 0; i < POSE_HISTORY_SIZE; i++) {
            poseHistory[i] = odometry.getPose();
            poseTimestamps[i] = 0;
        }

        state = LocState.ODOMETRY_ONLY;
        lastUpdateTime = System.nanoTime() / 1e9;
    }

    public void update() {
        currentTime = System.nanoTime() / 1e9;
        lastUpdateTime = currentTime;

        Pose2d odometryPose = odometry.update(currentTime);

        poseHistory[poseHistoryIndex] = odometryPose;
        poseTimestamps[poseHistoryIndex] = currentTime;
        poseHistoryIndex = (poseHistoryIndex + 1) % POSE_HISTORY_SIZE;

        double deltaX = odometryPose.getX() - kalmanFilter.getState().getX();
        double deltaY = odometryPose.getY() - kalmanFilter.getState().getY();
        double deltaH = MathUtils.normalizeRadians(
                odometryPose.getHeading() - kalmanFilter.getState().getHeading(), false);

        kalmanFilter.predict(deltaX, deltaY, deltaH, currentTime);

        robot.limelightHelper.update();
        Pose2d cameraPose = robot.limelightHelper.getAprilTagPose();
        double cameraTimestamp = robot.limelightHelper.getTimestamp();
        double distanceToTag = robot.limelightHelper.getAprilTagDistance();
        int tagCount = robot.limelightHelper.getTargetCount();

        if (cameraPose != null && cameraTimestamp > 0) {
            Pose2d matchedOdometryPose = getPoseAtTimestamp(cameraTimestamp);

            if (matchedOdometryPose != null) {
                lastInnovationX = cameraPose.getX() - matchedOdometryPose.getX();
                lastInnovationY = cameraPose.getY() - matchedOdometryPose.getY();
                lastInnovationH = MathUtils.normalizeRadians(
                        cameraPose.getHeading() - matchedOdometryPose.getHeading(), false);

                double quality = kalmanFilter.update(cameraPose, distanceToTag, tagCount, cameraTimestamp);

                lastUpdateQuality = quality;
                cameraConfidence = quality;
                lastGoodCameraPose = cameraPose;
                consecutiveMisses = 0;

                if (state == LocState.ODOMETRY_ONLY || state == LocState.RECOVERING) {
                    state = LocState.FUSING;
                }
            }
        } else {
            consecutiveMisses++;
            if (consecutiveMisses > Localization.MAX_CONSECUTIVE_MISSES) {
                if (state == LocState.FUSING) {
                    state = LocState.RECOVERING;
                }
            }
            cameraConfidence *= 0.9;
            if (cameraConfidence < 0.01) cameraConfidence = 0;
        }

        double[] gains = kalmanFilter.getKalmanGains();
        lastCorrectionX = gains[0] * lastInnovationX;
        lastCorrectionY = gains[1] * lastInnovationY;
        lastCorrectionH = gains[2] * lastInnovationH;
    }

    public Pose2d getPose() {
        return kalmanFilter.getState();
    }

    public double getX() {
        return kalmanFilter.getState().getX();
    }

    public double getY() {
        return kalmanFilter.getState().getY();
    }

    public double getHeading() {
        return kalmanFilter.getState().getHeading();
    }

    public int getAllianceMultiplier() {
        return Field.GOAL_X < 0 ? -1 : 1;
    }

    public double getDistanceToGoal() {
        double goalX = Field.GOAL_X * getAllianceMultiplier();
        double dx = goalX - getX();
        double dy = Field.GOAL_Y - getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getDistanceToGoalMeters() {
        return getDistanceToGoal() * 0.0254;
    }

    public double getAngleToGoal() {
        double goalX = Field.GOAL_X * getAllianceMultiplier();
        double dx = goalX - getX();
        double dy = Field.GOAL_Y - getY();
        return Math.atan2(dy, dx);
    }

    public double getHeadingErrorToGoal() {
        return MathUtils.normalizeRadians(getAngleToGoal() - getHeading(), false);
    }

    public double getHeadingErrorToGoalDegrees() {
        return Math.toDegrees(getHeadingErrorToGoal());
    }

    public Pose2d getOdometryPose() {
        return odometry.getPose();
    }

    public double getVelocityX() {
        return odometry.getVelocityX();
    }

    public double getVelocityY() {
        return odometry.getVelocityY();
    }

    public double getAngularVelocity() {
        return odometry.getAngularVelocity();
    }

    public double getVelocityMagnitude() {
        return odometry.getVelocityMagnitude();
    }

    public double getPositionUncertainty() {
        return kalmanFilter.getPositionUncertainty();
    }

    public double getHeadingUncertainty() {
        return kalmanFilter.getHeadingUncertainty();
    }

    public LocState getState() {
        return state;
    }

    public double getCameraConfidence() {
        return cameraConfidence;
    }

    public double getLastUpdateQuality() {
        return lastUpdateQuality;
    }

    public double[] getKalmanGains() {
        return kalmanFilter.getKalmanGains();
    }

    public void hardResetFromAprilTag() {
        Pose2d cameraPose = robot.limelightHelper.getAprilTagPose();
        if (cameraPose != null) {
            kalmanFilter.hardReset(cameraPose);
            odometry.setPose(cameraPose);
            state = LocState.FUSING;
            consecutiveMisses = 0;
            cameraConfidence = 1.0;
        }
    }

    public void resetPose(Pose2d pose) {
        kalmanFilter.reset(pose.getX(), pose.getY(), pose.getHeading());
        odometry.setPose(pose);
        state = LocState.ODOMETRY_ONLY;
        consecutiveMisses = 0;
    }

    public void resetPose(double x, double y, double heading) {
        resetPose(new Pose2d(x, y, Rotation2d.fromRadians(heading)));
    }

    public void resetToStartPosition() {
        int mult = getAllianceMultiplier();
        resetPose(Field.GOAL_X * mult, Field.GOAL_Y, mult > 0 ? 0 : Math.PI);
    }

    public void resetHeading() {
        Pose2d current = kalmanFilter.getState();
        kalmanFilter.reset(current.getX(), current.getY(), odometry.getHeading());
    }

    public boolean isAprilTagVisible() {
        return robot.limelightHelper.targetsValid();
    }

    public double getLimelightTx() {
        return robot.limelightHelper.getTx();
    }

    public double getLimelightTy() {
        return robot.limelightHelper.getTy();
    }

    public double getLimelightDistance() {
        return robot.limelightHelper.getDistanceMeters();
    }

    public double getLimelightDistanceInches() {
        return robot.limelightHelper.getDistanceInches();
    }

    private Pose2d getPoseAtTimestamp(double targetTimestamp) {
        Pose2d closestPose = null;
        double closestDelta = Double.MAX_VALUE;

        for (int i = 0; i < POSE_HISTORY_SIZE; i++) {
            if (poseTimestamps[i] > 0) {
                double delta = Math.abs(poseTimestamps[i] - targetTimestamp);
                if (delta < closestDelta) {
                    closestDelta = delta;
                    closestPose = poseHistory[i];
                }
            }
        }

        return closestDelta < 0.5 ? closestPose : null;
    }

    public void setProcessNoise(double qX, double qY, double qH) {
        Localization.PROCESS_NOISE_X = qX;
        Localization.PROCESS_NOISE_Y = qY;
        Localization.PROCESS_NOISE_HEADING = qH;
        kalmanFilter.setProcessNoise(qX, qY, qH);
    }

    public void setMeasurementNoise(double rX, double rY, double rH) {
        Localization.CAMERA_NOISE_X = rX;
        Localization.CAMERA_NOISE_Y = rY;
        Localization.CAMERA_NOISE_HEADING = rH;
        kalmanFilter.setMeasurementNoise(rX, rY, rH);
    }
}
