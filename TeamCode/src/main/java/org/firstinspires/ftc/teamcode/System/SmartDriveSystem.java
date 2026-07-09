package org.firstinspires.ftc.teamcode.System;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class SmartDriveSystem {

    public static double AIM_P = 0.045;
    public static double STICKY_RANGE = 20.0; // Degrees where assist starts
    public static double MIN_STICKY_FACTOR = 0.15; // Speed at 0 degrees error (hardest to turn)

    private double startAngle = Double.NaN;
    private boolean wasAutoAiming = false;

    public double[] update(Follower follower, Pose3D botPose, Gamepad gamepad, boolean triggerPressed, boolean targetVisible) {

        Pose robotPose = follower.getPose();
        double currentHeading = Math.toDegrees(robotPose.getHeading());

        // 1. DATA GATHERING
        double forward = -gamepad.left_stick_y;
        double strafe  = -gamepad.left_stick_x;
        double driverTurn = -gamepad.right_stick_x;
        double turnOutput = 0;

        // Calculate Target Angle (Assuming Limelight gives us tx/offset)
        // For this example, we use the botPose if available, or assume target is at 0 degrees field-centric
        double targetAngleDeg = 0; // Replace with your actual target logic
        double errorToGoal = angleWrap(targetAngleDeg - currentHeading);

        // 2. AUTO-AIM LOGIC (Right Trigger)
        if (triggerPressed && targetVisible) {
            // Set starting angle the moment we pull the trigger
            if (!wasAutoAiming) {
                startAngle = currentHeading;
                wasAutoAiming = true;
            }

            // Check 90 degree constraint
            double offsetFromStart = Math.abs(angleWrap(currentHeading - startAngle));

            if (offsetFromStart < 90) {
                turnOutput = errorToGoal * AIM_P;
            } else {
                turnOutput = driverTurn; // Fallback to manual if we go past 90
            }
        }
        else {
            wasAutoAiming = false;
            startAngle = Double.NaN;

            // 3. AIM ASSIST LOGIC (Sticky Aim)
            if (targetVisible && Math.abs(driverTurn) > 0.05) {
                double absError = Math.abs(errorToGoal);
                if (absError < STICKY_RANGE) {
                    // Linear interpolation: Closer to 0 = multiplier closer to MIN_STICKY_FACTOR
                    double multiplier = MIN_STICKY_FACTOR + ((absError / STICKY_RANGE) * (1.0 - MIN_STICKY_FACTOR));
                    driverTurn *= multiplier;
                }
            }
            turnOutput = driverTurn;
        }

        return new double[]{forward, strafe, Range.clip(turnOutput, -1, 1)};
    }

    private double angleWrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }
}//package org.firstinspires.ftc.teamcode.System;
//
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.Range;// CORE PEDRO IMPORTS
//import com.pedropathing.follower.Follower;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//public class SmartDriveSystem {
//
//    // ---------------------------------------------------------
//    // GAME SETTINGS (DECODE / SHOOTER GAMES)
//    // ---------------------------------------------------------
//    // Set these to the field coordinate of the High Goal / Basket
//    // Note: Verify these coordinates with the DECODE Game Manual!
//    // Usually corners are +/- 72 inches.
//    public static final Pose RED_GOAL = new Pose(-72, -72);
//    public static final Pose BLUE_GOAL = new Pose(72, 72);
//
//    // ---------------------------------------------------------
//    // TUNING VARIABLES (PID & FEEL)
//    // ---------------------------------------------------------
//    public static double AIM_P = 0.045;   // Power per degree of error (Auto-Aim)
//    public static double AIM_D = 0.003;   // Dampening to prevent jitter
//    public static double LOCK_P = 0.03;   // Power to keep driving straight
//
//    // "Sticky Aim" - Slows down turning when crosshair is near target
//    public static double STICKY_RANGE = 15.0; // Degrees wide
//    public static double STICKY_FRICTION = 0.2; // 20% speed when perfectly aimed
//
//    // Localization Trust: How much we let Limelight fix our position (0.0 - 1.0)
//    // 0.1 = Smooth blending (Recommended), 1.0 = Instant Teleport
//    public static double CAM_TRUST = 0.1;
//
//    // ---------------------------------------------------------
//    // STATE
//    // ---------------------------------------------------------
//    private Pose activeTarget = RED_GOAL;
//    private double headingLockTarget = Double.NaN;
//    private boolean wasTurning = false;
//    private Pose lastBotPose = new Pose(0,0,0);
//
//    /**
//     * Call this in init() based on your Alliance Color
//     */
//    public void setAlliance(boolean isRed) {
//        activeTarget = isRed ? RED_GOAL : BLUE_GOAL;
//    }
//
//    /**
//     * MAIN UPDATE LOOP - Call this every cycle in TeleOp
//     * * @param follower   Your PedroPathing Follower
//     * @param botPose    Double[] {X, Y, Heading(Deg)} from Limelight
//     * @param gamepad    Driver Gamepad
//     * @param autoAim    True if driver is holding the "Aim" button
//     * @return           Double[] {DriveY, DriveX, TurnPower}
//     */
//    /**
//     * UPDATED UPDATE LOOP
//     * @param follower   Your PedroPathing Follower
//     * @param botPose    Pose3D from Limelight (can be null if not using vision)
//     * @param autoAim    True if driver is holding the "Aim" button
//     * @param forward    Y input (with slow mode applied)
//     * @param strafe     X input (with slow mode applied)
//     * @param turn       Turn input (with slow mode applied)
//     * @return           Double[] {DriveY, DriveX, TurnPower}
//     */
//    public double[] update(Follower follower, Pose3D botPose, boolean autoAim, double forward, double strafe, double turn) {
//
//        // 1. LOCALIZE: Fix drift using Limelight BotPose (Optional)
//        updatePose(follower, botPose);
//
//        // 2. TARGETING: Calculate angle to the Goal
//        Pose robotPose = follower.getPose();
//
//        // Calculate the absolute field angle to the goal
//        double targetAngleRad = Math.atan2(activeTarget.getY() - robotPose.getY(), activeTarget.getX() - robotPose.getX());
//        double targetAngleDeg = Math.toDegrees(targetAngleRad);
//
//        // 3. DRIVE LOGIC
//        // We use the passed-in variables (forward/strafe/turn) instead of reading gamepad directly
//        double driverTurn = turn;
//        double turnOutput = 0;
//
//        double currentHeading = Math.toDegrees(robotPose.getHeading());
//        double errorToGoal = angleWrap(targetAngleDeg - currentHeading);
//
//        if (autoAim) {
//            // --- MODE A: AUTO-LOCK ---
//            // Ignores stick, forces robot to turn to goal
//            turnOutput = (errorToGoal * AIM_P) - (follower.getVelocity().getTheta() * AIM_D);
//            headingLockTarget = Double.NaN;
//        }
//        else if (Math.abs(driverTurn) > 0.05) {
//            // --- MODE B: MANUAL WITH AIM ASSIST ---
//            // "Sticky Aim": If aiming near the goal, slow down the turn speed
//            if (Math.abs(errorToGoal) < STICKY_RANGE) {
//                double stickiness = STICKY_FRICTION + ((Math.abs(errorToGoal) / STICKY_RANGE) * (1.0 - STICKY_FRICTION));
//                driverTurn *= stickiness;
//            }
//            turnOutput = driverTurn;
//            wasTurning = true;
//            headingLockTarget = Double.NaN;
//        }
//        else {
//            // --- MODE C: HEADING LOCK (Keep Straight) ---
//            if (wasTurning || Double.isNaN(headingLockTarget)) {
//                headingLockTarget = currentHeading;
//                wasTurning = false;
//            }
//            double lockError = angleWrap(headingLockTarget - currentHeading);
//            turnOutput = lockError * LOCK_P;
//        }
//
//        // Return the Distance for the shooter
//        double distToGoal = Math.hypot(activeTarget.getX() - robotPose.getX(), activeTarget.getY() - robotPose.getY());
//        this.lastDistanceToGoal = distToGoal;
//
//        return new double[]{forward, strafe, Range.clip(turnOutput, -1, 1)};
//    }
//    public double lastDistanceToGoal = 0;
//
//    /**
//     * Blends Limelight data into PedroPathing for continuous re-homing
//     */
//    private void updatePose(Follower follower, Pose3D botPose) {
//        // botPose format: [X(in), Y(in), Heading(Deg), ...]
//        // Check if Limelight sees nothing (zeros)
//        /// X, Y, Z, Roll, Pitch, Yaw
//        if (botPose == null || (botPose.getPosition().x == 0 && botPose.getPosition().y == 0)) return;
//
//        // Limelight usually sends Degrees, Pedro uses Radians
//        double llX = botPose.getPosition().x;
//        double llY = botPose.getPosition().y;
//
//        // Smoothly move the robot's "perceived" position toward the camera's "actual" position
//        Pose current = follower.getPose();
//        double newX = current.getX() + (llX - current.getX()) * CAM_TRUST;
//        double newY = current.getY() + (llY - current.getY()) * CAM_TRUST;
//
//        // We do NOT update heading from camera often (Gyros are faster/smoother)
//        follower.setPose(new Pose(newX, newY, current.getHeading()));
//
//        // Save for debug
//        lastBotPose = new Pose(llX, llY, Math.toRadians(botPose.getOrientation().getYaw()));
//    }
//
//    private double angleWrap(double degrees) {
//        while (degrees > 180) degrees -= 360;
//        while (degrees <= -180) degrees += 360;
//        return degrees;
//    }
//}