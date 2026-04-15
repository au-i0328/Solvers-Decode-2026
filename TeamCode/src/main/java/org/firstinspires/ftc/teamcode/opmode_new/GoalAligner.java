package org.firstinspires.ftc.teamcode.opmode_new;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.opmode_new.TuningConfig.*;

/**
 * Goal alignment controller.
 * Aligns the drivetrain to face the goal using fused pose from LocalizationManager.
 */
public class GoalAligner {

    private LocalizationManager localization;

    private double integral = 0;
    private double previousError = 0;
    private ElapsedTime timer = new ElapsedTime();

    public enum AlignmentState {
        OFF,
        ALIGNING,
        ALIGNED
    }

    public AlignmentState state = AlignmentState.OFF;

    private double turnOutput = 0;
    private double prevDerivative = 0;

    public void init(LocalizationManager localization) {
        this.localization = localization;
        state = AlignmentState.OFF;
    }

    public void startAlignment() {
        if (state == AlignmentState.OFF) {
            state = AlignmentState.ALIGNING;
            integral = 0;
        }
    }

    public void stopAlignment() {
        state = AlignmentState.OFF;
        turnOutput = 0;
        integral = 0;
        previousError = 0;
        prevDerivative = 0;
    }

    public double update() {
        switch (state) {
            case OFF:
                turnOutput = 0;
                break;

            case ALIGNING:
            case ALIGNED:
                double currentHeading = localization.getHeading();
                double angleToGoal = localization.getAngleToGoal();
                double error = normalizeAngle(angleToGoal - currentHeading);

                if (Alignment.USE_ADAPTIVE_GAINS) {
                    updateAdaptiveGains();
                }

                double dt = Math.max(0.001, timer.seconds());

                integral += error * dt;
                integral = Math.max(-1.0, Math.min(1.0, integral));

                double rawDerivative = (error - previousError) / dt;
                double derivative = 0.3 * rawDerivative + 0.7 * prevDerivative;
                prevDerivative = derivative;

                double pTerm = (Alignment.USE_ADAPTIVE_GAINS ? adaptiveKP : Alignment.AIMBOT_KP) * error;
                double iTerm = (Alignment.USE_ADAPTIVE_GAINS ? adaptiveKI : Alignment.AIMBOT_KI) * integral;
                double dTerm = (Alignment.USE_ADAPTIVE_GAINS ? adaptiveKD : Alignment.AIMBOT_KD) * derivative;

                turnOutput = pTerm + iTerm + dTerm;
                turnOutput = Math.max(-Alignment.MAX_TURN_SPEED, 
                        Math.min(Alignment.MAX_TURN_SPEED, turnOutput));

                if (Math.abs(error) < Alignment.DEADBAND_RADIANS) {
                    turnOutput = Math.signum(turnOutput) * Math.min(Math.abs(turnOutput), 0.05);
                    
                    if (Math.abs(error) < Alignment.DEADBAND_RADIANS * 0.5) {
                        state = AlignmentState.ALIGNED;
                    }
                } else {
                    state = AlignmentState.ALIGNING;
                }

                previousError = error;
                break;
        }

        timer.reset();
        return turnOutput;
    }

    private double adaptiveKP = Alignment.AIMBOT_KP;
    private double adaptiveKI = Alignment.AIMBOT_KI;
    private double adaptiveKD = Alignment.AIMBOT_KD;

    private void updateAdaptiveGains() {
        double positionUncertainty = localization.getPositionUncertainty();
        double headingUncertainty = localization.getHeadingUncertainty();
        double cameraConfidence = localization.getCameraConfidence();

        double baseKP = Alignment.AIMBOT_KP;
        double baseKI = Alignment.AIMBOT_KI;
        double baseKD = Alignment.AIMBOT_KD;

        double uncertaintyFactor = 1.0;

        if (positionUncertainty > Alignment.UNCERTAINTY_THRESHOLD_HIGH) {
            uncertaintyFactor *= 0.7;
        } else if (positionUncertainty > Alignment.UNCERTAINTY_THRESHOLD_LOW) {
            uncertaintyFactor *= 0.85;
        }

        if (headingUncertainty > 0.1) {
            uncertaintyFactor *= 0.6;
        } else if (headingUncertainty > 0.05) {
            uncertaintyFactor *= 0.8;
        }

        if (cameraConfidence > 0.7) {
            uncertaintyFactor *= 1.1;
        }

        adaptiveKP = baseKP * uncertaintyFactor;
        adaptiveKI = baseKI * uncertaintyFactor;
        adaptiveKD = baseKD * uncertaintyFactor;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public boolean isAligned() {
        return state == AlignmentState.ALIGNED;
    }

    public boolean isAligning() {
        return state == AlignmentState.ALIGNING || state == AlignmentState.ALIGNED;
    }

    public boolean isOff() {
        return state == AlignmentState.OFF;
    }

    public AlignmentState getState() {
        return state;
    }

    public double getTurnOutput() {
        return turnOutput;
    }

    public double getAngleError() {
        return normalizeAngle(localization.getAngleToGoal() - localization.getHeading());
    }

    public double getAngleErrorDegrees() {
        return Math.toDegrees(getAngleError());
    }

    public double getAbsAngleErrorDegrees() {
        return Math.abs(getAngleErrorDegrees());
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        prevDerivative = 0;
        timer.reset();
    }

    public double getAdaptiveKP() {
        return adaptiveKP;
    }

    public double getConfidence() {
        return localization.getCameraConfidence();
    }
}
