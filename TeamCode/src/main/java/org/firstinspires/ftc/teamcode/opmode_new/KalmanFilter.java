package org.firstinspires.ftc.teamcode.opmode_new;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.util.MathUtils;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.opmode_new.TuningConfig.*;

/**
 * Extended Kalman Filter for pose fusion.
 * Fuses dead wheel odometry with Limelight 3A AprilTag pose estimates.
 */
public class KalmanFilter {

    private double x, y, heading;
    private double[][] P = {
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
    };

    private double qX, qY, qH;
    private double rX, rY, rH;

    private double kX, kY, kH;

    private final List<Pose2d> measurementHistory = new ArrayList<>();
    private static final int MAX_HISTORY_SIZE = 5;

    private double lastUpdateTime = 0;
    private double dt = 0;

    private double lastInnovationX = 0;
    private double lastInnovationY = 0;
    private double lastInnovationH = 0;

    public void init(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;

        P[0][0] = 10.0;
        P[1][1] = 10.0;
        P[2][2] = 0.5;

        measurementHistory.clear();
        lastUpdateTime = 0;
    }

    public void init(Pose2d pose) {
        init(pose.getX(), pose.getY(), pose.getHeading());
    }

    public void setProcessNoise(double qX, double qY, double qH) {
        this.qX = qX;
        this.qY = qY;
        this.qH = qH;
    }

    public void setMeasurementNoise(double rX, double rY, double rH) {
        this.rX = rX;
        this.rY = rY;
        this.rH = rH;
    }

    public void predict(double deltaX, double deltaY, double deltaTheta, double timestamp) {
        if (lastUpdateTime > 0) {
            dt = timestamp - lastUpdateTime;
        } else {
            dt = 0.02;
        }
        dt = Math.max(0.001, Math.min(dt, 0.1));
        lastUpdateTime = timestamp;

        deltaTheta = MathUtils.normalizeRadians(deltaTheta, false);

        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double globalDeltaX = deltaX * cosH - deltaY * sinH;
        double globalDeltaY = deltaX * sinH + deltaY * cosH;

        x += globalDeltaX;
        y += globalDeltaY;
        heading = MathUtils.normalizeRadians(heading + deltaTheta, false);

        double movement = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double qScale = 1.0 + movement * 0.1;

        P[0][0] += qX * qScale * dt;
        P[1][1] += qY * qScale * dt;
        P[2][2] += qH * dt;

        P[0][0] = Math.max(P[0][0], 0.01);
        P[1][1] = Math.max(P[1][1], 0.01);
        P[2][2] = Math.max(P[2][2], 0.001);
    }

    public double update(Pose2d measurement, double distanceToTag, int tagCount, double timestamp) {
        double measuredX = measurement.getX();
        double measuredY = measurement.getY();
        double measuredH = measurement.getHeading();

        lastInnovationX = measuredX - x;
        lastInnovationY = measuredY - y;
        lastInnovationH = MathUtils.normalizeRadians(measuredH - heading, false);

        double distanceFactor = Math.max(1.0, 1.0 + (distanceToTag - 36) * Localization.CAMERA_NOISE_SCALE);
        distanceFactor = Math.min(distanceFactor, Localization.MAX_CAMERA_NOISE / Localization.MIN_CAMERA_NOISE);

        double tagConfidence = Math.min(1.0, 0.5 + 0.1 * tagCount);

        double rXAdaptive = Math.max(Localization.MIN_CAMERA_NOISE, rX * distanceFactor) / tagConfidence;
        double rYAdaptive = Math.max(Localization.MIN_CAMERA_NOISE, rY * distanceFactor) / tagConfidence;
        double rHAdaptive = Math.max(0.01, rH * (1.0 + distanceToTag * 0.001)) / tagConfidence;

        double sX = P[0][0] + rXAdaptive;
        double sY = P[1][1] + rYAdaptive;
        double sH = P[2][2] + rHAdaptive;

        kX = P[0][0] / sX;
        kY = P[1][1] / sY;
        kH = P[2][2] / sH;

        kX = Math.max(0.01, Math.min(0.5, kX));
        kY = Math.max(0.01, Math.min(0.5, kY));
        kH = Math.max(0.01, Math.min(0.5, kH));

        x += kX * lastInnovationX;
        y += kY * lastInnovationY;
        heading = MathUtils.normalizeRadians(heading + kH * lastInnovationH, false);

        double kX1 = 1.0 - kX;
        double kY1 = 1.0 - kY;
        double kH1 = 1.0 - kH;

        P[0][0] = kX1 * P[0][0] * kX1 + kX * rXAdaptive * kX;
        P[1][1] = kY1 * P[1][1] * kY1 + kY * rYAdaptive * kY;
        P[2][2] = kH1 * P[2][2] * kH1 + kH * rHAdaptive * kH;

        P[0][1] = kX1 * P[0][1] * kY1;
        P[1][0] = P[0][1];
        P[0][2] = kX1 * P[0][2] * kH1;
        P[2][0] = P[0][2];
        P[1][2] = kY1 * P[1][2] * kH1;
        P[2][1] = P[1][2];
        P[0][1] = P[1][0];
        P[0][2] = P[2][0];
        P[1][2] = P[2][1];

        measurementHistory.add(new Pose2d(measuredX, measuredY, new Rotation2d(measuredH)));
        while (measurementHistory.size() > MAX_HISTORY_SIZE) {
            measurementHistory.remove(0);
        }

        return calculateUpdateQuality(rXAdaptive, rYAdaptive, rHAdaptive, tagConfidence);
    }

    private double calculateUpdateQuality(double rX, double rY, double rH, double tagConfidence) {
        double normalizedInnovationX = (lastInnovationX * lastInnovationX) / (rX + 0.01);
        double normalizedInnovationY = (lastInnovationY * lastInnovationY) / (rY + 0.01);
        double normalizedInnovationH = (lastInnovationH * lastInnovationH) / (rH + 0.0001);

        double mahalanobisDist = normalizedInnovationX + normalizedInnovationY + normalizedInnovationH;

        if (mahalanobisDist > 7.815) {
            return 0.3;
        }

        double quality = Math.max(0.1, 1.0 - mahalanobisDist / (7.815 * 2));
        quality *= tagConfidence;

        return Math.min(1.0, quality);
    }

    public Pose2d getAveragedMeasurement() {
        if (measurementHistory.isEmpty()) {
            return new Pose2d(x, y, new Rotation2d(heading));
        }

        double sumX = 0, sumY = 0, sumH = 0;
        for (Pose2d p : measurementHistory) {
            sumX += p.getX();
            sumY += p.getY();
            sumH += p.getHeading();
        }

        int n = measurementHistory.size();
        double avgH = sumH / n;
        avgH = MathUtils.normalizeRadians(avgH, false);

        return new Pose2d(sumX / n, sumY / n, new Rotation2d(avgH));
    }

    public Pose2d getState() {
        return new Pose2d(x, y, new Rotation2d(heading));
    }

    public double getPositionUncertainty() {
        return Math.sqrt(P[0][0] + P[1][1]);
    }

    public double getHeadingUncertainty() {
        return Math.sqrt(P[2][2]);
    }

    public double[] getKalmanGains() {
        return new double[]{kX, kY, kH};
    }

    public double[][] getCovariance() {
        return P;
    }

    public double[] getInnovation() {
        return new double[]{lastInnovationX, lastInnovationY, lastInnovationH};
    }

    public void reset(double x, double y, double heading) {
        init(x, y, heading);
        measurementHistory.clear();
    }

    public void hardReset(Pose2d pose) {
        this.x = pose.getX();
        this.y = pose.getY();
        this.heading = MathUtils.normalizeRadians(pose.getHeading(), false);

        P[0][0] = 0.1;
        P[1][1] = 0.1;
        P[2][2] = 0.001;

        measurementHistory.clear();
        measurementHistory.add(pose);
    }

    public void blendWith(Pose2d otherPose, double blendFactor) {
        this.x = (1 - blendFactor) * x + blendFactor * otherPose.getX();
        this.y = (1 - blendFactor) * y + blendFactor * otherPose.getY();

        double deltaH = MathUtils.normalizeRadians(otherPose.getHeading() - heading, false);
        this.heading = MathUtils.normalizeRadians(heading + blendFactor * deltaH, false);

        P[0][0] += blendFactor * 5.0;
        P[1][1] += blendFactor * 5.0;
        P[2][2] += blendFactor * 0.5;
    }
}
