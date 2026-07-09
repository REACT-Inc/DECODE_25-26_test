package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.System.LoggingSystem;

@Configurable
public class AimControl {
    public static double testVariable = 6.7;
    // ---------------------------------------------------------
    // TUNING VALUES
    // ---------------------------------------------------------
    // Auto-Aim PID (Simple P-Controller is usually enough for turning)
    public static double P_TURN = 0.035; // Power per degree of error
    public static double D_TURN = 0.002; // Dampening
    public static double MIN_AUTO_TURN_SPEED = 0.07; // Friction overcome

    // Aim Assist (Sticky Aim)
    public static double ASSIST_WIDTH = 10.0; // Degrees. How wide is the "sticky" zone?
    public static double MIN_STICK_SPEED = 0.3; // Slowest multiplier (30% speed at center)

    // ---------------------------------------------------------
    // VARIABLES
    // ---------------------------------------------------------
    private double lastError = 0;
    public boolean targetVisible = false;

    /**
     * Main control loop. Call this in your TeleOp Loop.
     *
     * @param driverTurnInput Your gamepad.right_stick_x
     * @param isAutoAimButton Boolean (e.g., gamepad1.left_bumper)
     * @param limelightData   Object[] {Dist, TagX, TagY, TagID}
     * @param logging loging
     * @return The final turn power to send to follower.setTeleOpMovementVectors()
     */
    public double update(double driverTurnInput, boolean isAutoAimButton, Object[] limelightData, LoggingSystem logging) {

        // 1. Parse Limelight Data
        double distance = (Double) limelightData[0];
        double tagX = (Double) limelightData[1];
        double tagY = (Double) limelightData[2];
        int tagID = (Integer) limelightData[3];

        targetVisible = (tagID != -1); // Assuming -1 means no tag found

        // 2. Logic Selection
        if (isAutoAimButton && targetVisible) {
            return calculateAutoAim(tagX);
        }
        else if (targetVisible) {
            return calculateAimAssist(driverTurnInput, tagX);
        }
        else {
            // No tag seen? Just normal driving.
            return driverTurnInput;
        }
    }

    /**
     * Mode 1: Auto Aim
     * PID loop to bring TagX to 0
     */
    private double calculateAutoAim(double currentErrorX) {
        // TagX is usually negative on one side, positive on the other.
        // We want Error = 0 - TagX.
        // Note: You might need to flip the sign (-) depending on your motor config.
        double error = -currentErrorX;
        double derivative = error - lastError;

        double turnPower = (error * P_TURN) + (derivative * D_TURN);

        // Add a minimum power (feedforward) to overcome motor friction if error is significant
        if (Math.abs(error) > 1.0) {
            turnPower += Math.signum(turnPower) * MIN_AUTO_TURN_SPEED;
        }

        lastError = error;

        // Clip range to prevent crazy spinning
        return Range.clip(turnPower, -(0.5*(currentErrorX/testVariable)), 0.5*(currentErrorX/testVariable));
    }

    /**
     * Mode 2: Aim Assist (Sticky Aim)
     * Dampens manual turn speed when the target is centered.
     */
    private double calculateAimAssist(double input, double errorX) {
        // If we are outside the "Sticky Zone", do nothing
        if (Math.abs(errorX) > ASSIST_WIDTH) {
            return input;
        }

        // Calculate a scalar (0.0 to 1.0) based on how close we are to center
        // 0 degrees error = Scale down max (hardest to turn)
        // 10 degrees error = Scale down none (normal turning)

        double errorMagnitude = Math.abs(errorX);
        double ratio = errorMagnitude / ASSIST_WIDTH; // 0.0 (center) to 1.0 (edge)

        // Linear interpolation for dampening
        // If ratio is 0 (center), scalar is MIN_STICK_SPEED
        // If ratio is 1 (edge), scalar is 1.0
        double scalar = MIN_STICK_SPEED + (ratio * (1.0 - MIN_STICK_SPEED));

        return input * scalar;
    }
}