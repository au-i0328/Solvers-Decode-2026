package org.firstinspires.ftc.teamcode.opmode_new;

import com.seattlesolvers.solverslib.util.TelemetryEx;

/**
 * Alliance selector for pre-match setup.
 * Allows drivers to select RED or BLUE alliance during init.
 * Alliance is LOCKED once the opmode starts.
 */
public class AllianceSelector {

    private NewHardware robot;

    private boolean lastTriangle = false;
    private boolean lastCircle = false;

    private NewHardware.Alliance selectedAlliance = null;
    private boolean isLocked = false;

    /**
     * Initialize the alliance selector
     */
    public AllianceSelector(NewHardware robot) {
        this.robot = robot;
        this.selectedAlliance = NewHardware.Alliance.BLUE; // Default
    }

    /**
     * Show alliance selection screen during init
     */
    public void showAllianceSelection(TelemetryEx telemetry) {
        telemetry.addLine("╔══════════════════════════════╗");
        telemetry.addLine("║    ALLIANCE SELECTION       ║");
        telemetry.addLine("╠══════════════════════════════╣");
        telemetry.addLine("║  Triangle = RED             ║");
        telemetry.addLine("║  Circle   = BLUE            ║");
        telemetry.addLine("╚══════════════════════════════╝");
        telemetry.addData("Selected", selectedAlliance != null ? selectedAlliance : "None");
    }

    /**
     * Update alliance selection during init loop.
     * Should be called every loop while !isStarted()
     *
     * @param gamepad1 The driver gamepad
     * @param telemetry The telemetry object
     * @return The currently selected alliance (or null if not selected)
     */
    public NewHardware.Alliance update(
            com.qualcomm.robotcore.hardware.Gamepad gamepad1,
            TelemetryEx telemetry) {

        if (isLocked) {
            return selectedAlliance; // No changes allowed after lock
        }

        // Triangle = RED, Circle = BLUE
        if (gamepad1.triangle && !lastTriangle) {
            selectedAlliance = NewHardware.Alliance.RED;
            telemetry.addData("Selected", selectedAlliance);
        } else if (gamepad1.circle && !lastCircle) {
            selectedAlliance = NewHardware.Alliance.BLUE;
            telemetry.addData("Selected", selectedAlliance);
        }

        lastTriangle = gamepad1.triangle;
        lastCircle = gamepad1.circle;

        return selectedAlliance;
    }

    /**
     * Lock the alliance selection.
     * After calling this, no changes are allowed.
     */
    public void lockAlliance() {
        isLocked = true;
    }

    /**
     * Check if alliance is locked
     */
    public boolean isLocked() {
        return isLocked;
    }

    /**
     * Get the selected alliance
     */
    public NewHardware.Alliance getSelectedAlliance() {
        return selectedAlliance;
    }

    /**
     * Check if selection has been made
     */
    public boolean hasSelection() {
        return selectedAlliance != null;
    }
}
