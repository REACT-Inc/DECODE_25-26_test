package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.System.PropertiesSystem.PropertyCleaner;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.bylazar.gamepad.GamepadManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.System.LimelightSystem;
import org.firstinspires.ftc.teamcode.System.LoggingSystem;
import org.firstinspires.ftc.teamcode.System.PropertiesSystem;
import org.firstinspires.ftc.teamcode.System.PanelsSystem;
import org.firstinspires.ftc.teamcode.System.SmartDriveSystem;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.CompletableFuture;
import java.util.function.Supplier;

/**
 * <h1>TeleOp26 -  Robot Control System</h1>
 * <p>
 * This class handles the manual driver control period. It features:
 * <ul>
 * <li>Mecanum drivetrain with optional PedroPathing assists</li>
 * <li>Dynamic Property adjustment during initialization</li>
 * <li>Complex LED feedback system for robot states</li>
 * <li>Hardware fail-safe checks and error reporting</li>
 * </ul>
 * </p>
 */
@Configurable
@TeleOp(name="TeleOp26", group="TELEOP")
public class TeleOp26 extends LinearOpMode {

    // ------------------------------------------------------------------------------------------------
    // HARDWARE ACCESSORS
    // ------------------------------------------------------------------------------------------------
    private DcMotor FLM, FRM, BLM, BRM; // Back Right Motor
    private Servo LWS, RWS; // Right Wrist Servo (Normal Rotation)
    private static CRServo LRBS, RRBS, MIS;
    public static DcMotorEx LLM, RLM, MLM;  // Middle Launcher Motor
    private DigitalChannel Laser;
    private AnalogSensor analogSensor;

    private GoBildaPrismDriver PrismLEDDriver; // Main LED Controller

    // Indicator LEDs for the expansion/control hub ports
    private LED ledRed, ledGreen;

    private Servo GobildaPWMLed, BackLed; // Addressable PWM LED strip control

    public Gamepad pad1;
    public Gamepad pad2;

    // Store the previous state of the gamepads to detect changes
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // ------------------------------------------------------------------------------------------------
    // STATE FLAGS (Used for Logic and LED Feedback)
    // ------------------------------------------------------------------------------------------------
    public enum RobotState {DRIVING, INTAKING, TARGETING, SHOOTING, IDLE};
    RobotState currentState = RobotState.IDLE;

    ///  This handles when we leave the menu so we reload everything
    private boolean JustLeftAdjustmentMenu = false;

    public boolean[] ballsCollected = {false, false, false}; // Inventory tracking

    private FlywheelLogic flywheel = new FlywheelLogic();



    public static PanelsSystem telemetryS;
    public PropertiesSystem propertiesSystem;
    public LimelightSystem limelight;
    Object[] currentResults;
    public static LoggingSystem logging = new LoggingSystem();



    // ------------------------------------------------------------------------------------------------
    // NAVIGATION & PATHING (PedroPathing Integration)
    // ------------------------------------------------------------------------------------------------
    public static Follower follower;
    private Pose startingPose = new Pose(64, 12, Math.toRadians(180)); // Default starting position
    private Supplier<PathChain> pathChain; // Functional interface for dynamic path generation


    AimControl aimControl = new AimControl();
    double slowModeMultiplier = 0.5; // Default slow mode
    boolean slowMode = false;
    boolean automatedDrive = false;
    public enum ALLIANCE {RED, BLUE, YELLOW};
    public static ALLIANCE alliance = ALLIANCE.YELLOW;

    Pose6D pose6d = new Pose6D(logging);

    /**
     * Inits all the motors and sets them ot the ocorect settings
     */
    private void setupMotors() {

        flywheel.init(hardwareMap, telemetryS, logging, propertiesSystem);
        FLM = hardwareMap.tryGet(DcMotor.class, "FL");
        FRM = hardwareMap.tryGet(DcMotor.class, "FR");
        BLM = hardwareMap.tryGet(DcMotor.class, "BL");
        BRM = hardwareMap.tryGet(DcMotor.class, "BR");
        LLM = hardwareMap.tryGet(DcMotorEx.class, "2");
        RLM = hardwareMap.tryGet(DcMotorEx.class, "3");
        MLM = hardwareMap.tryGet(DcMotorEx.class, "MLM");

        // Apply ZeroPowerBehavior based on user-defined property
        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.DRIVE_MOTOR_ZERO_POWER_BHV)) == "BRAKE") {
            FLM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            FRM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            BLM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            BRM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            logging.log("Drive motors set To BREAK");
        } else {
            FLM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            FRM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            BLM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            BRM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            logging.log("Drive motors set To FLOAT");
        }

        // Apply Motor Directions dynamically from config file
        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.BL_MOTOR_DIRECTION)) == "FORWARD") {
            BLM.setDirection(DcMotorSimple.Direction.FORWARD);
            logging.log("BLM SET TO FORWARD");
        } else {
            BLM.setDirection(DcMotorSimple.Direction.REVERSE);
            logging.log("BLM SET TO REVERSE");
        }

        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.BR_MOTOR_DIRECTION)) == "FORWARD") {
            BRM.setDirection(DcMotorSimple.Direction.FORWARD);
            logging.log("BRM SET TO FORWARD");
        } else {
            BRM.setDirection(DcMotorSimple.Direction.REVERSE);
            logging.log("BRM SET TO REVERSE");
        }

        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.FR_MOTOR_DIRECTION)) == "FORWARD") {
            FRM.setDirection(DcMotorSimple.Direction.FORWARD);
            logging.log("FRM SET TO FORWARD");
        } else {
            FRM.setDirection(DcMotorSimple.Direction.REVERSE);
            logging.log("FRM SET TO REVERSE");
        }

        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.FL_MOTOR_DIRECTION)) == "FORWARD") {
            logging.log("FLM SET TO FORWARD");
            FLM.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            FLM.setDirection(DcMotorSimple.Direction.REVERSE);
            logging.log("FLM SET TO REVERSE");
        }
    }

    /**
     * Maps servos and sets initial directions for the dual rubber band intake.
     */
    private void setupServos(){
        LWS = hardwareMap.get(Servo.class, "LeftWristServo");
        RWS = hardwareMap.get(Servo.class, "RightWristServo");
        MIS = hardwareMap.get(CRServo.class, "MIS");
        LRBS = hardwareMap.tryGet(CRServo.class, "LeftRubberBandServo");
        RRBS = hardwareMap.tryGet(CRServo.class, "RightRubberBandServo");
        RRBS.setDirection(CRServo.Direction.REVERSE); // Counter-rotational to work in sync
        LRBS.setDirection(CRServo.Direction.FORWARD);
        GobildaPWMLed = hardwareMap.get(Servo.class, "led");
        BackLed = hardwareMap.get(Servo.class, "BackLed");
        logging.log("SERVD");
    }
    DistanceSensor secondDistance;
    ColorRangeSensor allianceSensor;
    private void setupSensors(){
        // Detects sample/alliance color
         allianceSensor = hardwareMap.get(ColorRangeSensor.class, "allianceSensor");
        ledRed = hardwareMap.get(LED.class, "red");

        ledGreen = hardwareMap.get(LED.class, "green");
        Laser = hardwareMap.get(DigitalChannel.class, "laser");

         secondDistance = hardwareMap.get(DistanceSensor.class, "cr");
        // Set the channel as an input
        Laser.setMode(DigitalChannel.Mode.INPUT);

        if(allianceSensor.red() > 100 || allianceSensor.blue() > 100) {
            if(allianceSensor.red() < allianceSensor.blue()){
                alliance = ALLIANCE.BLUE;
                logging.log("ALLIANCE IS BLUE");
                logging.log(allianceSensor.blue() + " VS " + allianceSensor.red());
            }else{
                alliance = ALLIANCE.RED;
                logging.log("ALLIANCE IS RED");
                logging.log(allianceSensor.red() + " VS " + allianceSensor.blue());

            }
            ledRed.enable((allianceSensor.red() > allianceSensor.blue()));
            ledGreen.enable((allianceSensor.red() < allianceSensor.blue()));
        } else {
            ledRed.off();
            ledGreen.off();
            logging.log("ALLIANCE IS YELLOW WHAT OH NO");
//            telemetryS.speak("OH NOOO COLRO IS NOT RESPOND");
        }

    }

    private void setupConsmetics(){
        PrismLEDDriver = hardwareMap.tryGet(GoBildaPrismDriver.class,"lights");
        // Set strip length based on LED density property
        PrismLEDDriver.setStripLength((int)Math.floor(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_COUNT_PRISM_DRIVER)));
    }
 private void setupPanels(Gamepad gamepad1, Gamepad gamepad2){
      pad1 = gamepad1;//PanelsGamepad.INSTANCE.getFirstManager().(gamepad1);
      pad2 = gamepad2;//PanelsGamepad.INSTANCE.getSecondManager().asCombinedFTCGamepad(gamepad2);


 }
    SmartDriveSystem smartDrive = new SmartDriveSystem();
 public boolean logOn = false;
    public Timer opmodeTimer;
    @Override
    public void runOpMode() throws InterruptedException {

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        /// Inits the system classes
        telemetryS = new PanelsSystem(telemetry);
        propertiesSystem = new PropertiesSystem(telemetryS);
        logging.setTelemetry(telemetryS);
        limelight = new LimelightSystem(telemetryS, hardwareMap);
        // Load persisted data
        try {
            propertiesSystem.loadProperties();
            logging.log("PROPERTiES IS LOADED");
        } catch (IOException e) {
            logging.log("PROPERTIES IS BROKEN");

        }
//        try {
////            Drawing.addLogFile(logging);
////            logging.savePose();
////            Drawing.addPosesToList(logging.loadPose());
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
        flywheel.setPropertiesFile(propertiesSystem);

        // gets the last known position
        startingPose = new Pose(
                (double)(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CURRENT_POSE_X)),
                (double)(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CURRENT_POSE_Y)), // Note: Check if Y should be here
                (double)(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CURRENT_POSE_RADIANS))
        );
        logging.log("START POSE" + startingPose.getX() + ", " + startingPose.getY() + ", " + startingPose.getHeading());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        // Path Definition: Dynamic line to field center (0,0)


        setupMotors();
        setupConsmetics();
        setupServos();
        setupPanels(gamepad1, gamepad2);
        setupSensors();

        pathChain = () -> follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, new Pose(0, 0)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, AllianceSideMirroredInstance(Math.toRadians(135)), 0.8))
                .build();
        // Initial LED Boot Sequence
        if((boolean) PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_ENABLED_PRISM_DRIVER)) == true){
            PrismLEDDriver.enableDefaultBootArtboard(true);
            PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        } else {
            PrismLEDDriver.enableDefaultBootArtboard(false);
            PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
        }

        waitForStart();
        follower.startTeleopDrive();
        while(opModeIsActive()) {
            telemetryS.addData("Alliacne Sensor OQutput", allianceSensor.red());
            telemetryS.addData("Alliacne Sensor OQutput", allianceSensor.blue());
            if(gamepad2.square && gamepad1.square){
                logOn = !logOn;
                while(gamepad1.square || gamepad2.square){}
            }
            telemetryS.addData("LOGGING PRESS SQUARE!!!", logOn);
//            if(logOn){
//                logging.solog();
//                continue;
//            }
            currentState = RobotState.IDLE;
telemetryS.addData("1", secondDistance.getDistance(DistanceUnit.INCH));
//            // Check Gamepad 1
//            checkGamepadChanges(gamepad1, previousGamepad1, "gamepad1");
//
//            // Check Gamepad 2
//            checkGamepadChanges(gamepad2, previousGamepad2, "gamepad2");
//
//            // Update previous state for the next loop cycle
//            try {
//                previousGamepad1.copy(gamepad1);
//                previousGamepad2.copy(gamepad2);
//            } catch (Exception e) {
//                // Swallow copy errors (rare)
////                logging.log("Gamepad Copy Error" + e.getMessage());
//            }

            //Adjustnement system
            ///  We start with this because this affects telemetry so we need it ontop
            if (pad1.share) {
                propertiesSystem.adjustmentMode = !propertiesSystem.adjustmentMode;
                try {
                    propertiesSystem.saveProperties();
                } catch (IOException e) {
                    telemetryS.speak("ERROR SAVING");
                    logging.log("ERROR SAVING PROPS" + e.getMessage());
                }
                while (pad1.share); // Debounce
            }

            // Adjustment Menu Logic: Freezes main OpMode to allow tuning
            if (propertiesSystem.adjustmentMode) {
                propertiesSystem.startMenu(pad1);
                JustLeftAdjustmentMenu = true;
                continue;
            }

            // Re-sync hardware after adjusting properties (e.g., changing motor direction)
            if (JustLeftAdjustmentMenu) {
                setupMotors();
                setupConsmetics();
                setupServos();
                setupSensors();

                if ((boolean) PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_ENABLED_PRISM_DRIVER)) == true) {
                    PrismLEDDriver.enableDefaultBootArtboard(true);
                    PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
                } else {
                    PrismLEDDriver.enableDefaultBootArtboard(false);
                    PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
                }
                JustLeftAdjustmentMenu = false;
            }

            //Update Zone manager
            telemetryS.addLine("--------------- " + "ZoneManager" + " ----------------");
            pose6d.updateRobotPositon(follower, logging, telemetryS);
            if(gamepad1.options){
                pose6d.mapZonesToPanels();
            }

            //Update Limelight
            telemetryS.addLine("--------------- " + "Limelight" + " ----------------");
            currentResults = limelight.loop(follower.getPose(), logging, opmodeTimer);
//            Drawing.drawDebug(limelight.performLocalizationUpdate(follower, currentResults));
            //Update  Flywheel
            telemetryS.addLine("--------------- " + "Flywheel" + " ----------------");
            flywheel.loop(pad1,pad2,telemetryS, currentResults[0], pose6d, logging);

            //Update Pose
            telemetryS.addLine("--------------- " + "Pose" + " ----------------");
            propertiesSystem.PROPERTIES.put(PropertiesSystem.Prop.CURRENT_POSE_X, (float)follower.getPose().getX());
            propertiesSystem.PROPERTIES.put(PropertiesSystem.Prop.CURRENT_POSE_Y, (float)follower.getPose().getY()); // Note: Check if Y should be here
            propertiesSystem.PROPERTIES.put(PropertiesSystem.Prop.CURRENT_POSE_RADIANS, (float)follower.getPose().getHeading());
            telemetryS.addData("Current Pose X", propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CURRENT_POSE_X));
            telemetryS.addData("Current Pose Y", propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CURRENT_POSE_Y)); // Note: Check if Y should be here
            telemetryS.addData("Current Pose Radians", propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CURRENT_POSE_RADIANS));
            // Handle entering/exiting the on-the-fly Adjustment Menu

            //Drive stuff
            telemetryS.addLine("------------- Driving Telemetry --------------");



            // DRIVE SELECTION: Manual Mecanum vs PedroPathing Assisted Drive
            if ((boolean) PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.AUTO_DRIVE_DISABLED)) == true) {
                drive(); // Manual Logic
            } else {

                //Update not sure why
                follower.update();



                    // ---------------------------------------------------------
                    // 1. GATHER DATA
                    // ---------------------------------------------------------
                    double distance = (Double) currentResults[0];
                    double tagX = (Double) currentResults[1];
                    int tagID = (Integer) currentResults[3];

                    // ---------------------------------------------------------
                    // 2. DRIVER INPUTS & MODIFIERS

                double driveY = -pad1.left_stick_y;
                double driveX  = -pad1.left_stick_x;
                double turn = -pad1.right_stick_x;
                    // Toggle Slow Mode
                    if (pad1.left_trigger > 0.1) {
                        slowMode = !slowMode;
                        while(pad1.right_trigger > 0.1){}
                    }

                    // Adjust Slow Mode Strength
                if(pad1.left_bumper) slowModeMultiplier += 0.25;
                if(pad1.right_bumper) slowModeMultiplier -= 0.25;

                    // Apply Slow Mode Scaling BEFORE Aim Assist
                    if (gamepad1.right_trigger > 0.1) {
//                        turn   *= pad1.right_trigger;
                    driveY *= pad1.right_trigger-0.5;
                    driveX *= pad1.right_trigger-0.5;
                    }
// ---------------------------------------------------------
                // 3. SMART DRIVE & AIM CONTROL
                // ---------------------------------------------------------
//                boolean autoAimActive = (pad1.right_trigger > 0.5);

                // If you have Limelight Pose3D, pass it here. If not, pass null (it will use Pedro odometry).
                // org.firstinspires.ftc.robotcore.external.navigation.Pose3D botPose = ...;

                // Pass the Follower, Camera Data, AutoAim status, and your Stick Inputs (driveY, driveX, turn)
//                double[] smartPowers = smartDrive.update(follower, null, autoAimActive, driveY, driveX, turn);

                // ---------------------------------------------------------
                // 4. DRIVE EXECUTION
                // ---------------------------------------------------------

                // Start Automated Pathing (Press A)
//                if (pad1.a && !automatedDrive) {
//                    follower.followPath(pathChain.get(), true);
//                    automatedDrive = true;
//                    currentState = RobotState.DRIVING;
//                }
//
//                // Stop Automated Pathing
//                if (automatedDrive && !pad1.a) {
//                    follower.startTeleopDrive();
//                    automatedDrive = false;
//                }
////
//                // Apply Drive Powers
//                // ---------------------------------------------------------
//// 1. GATHER DATA
//// ---------------------------------------------------------
//// Check if Limelight sees a target (ID != -1 or similar check)
//                boolean targetVisible = (tagID != -1);
//                boolean autoAimTrigger = (pad1.right_trigger > 0.5);
//
//// ---------------------------------------------------------
//// 2. SMART DRIVE CALCULATION
//// ---------------------------------------------------------
//// botPose should be passed here from your Limelight results
//                double[] drivePowers = smartDrive.update(
//                        follower,
//                        limelight.getBotPose(), // Ensure this variable is defined from your camera
//                        pad1,
//                        autoAimTrigger,
//                        targetVisible
//                );
//
//                double driveY = drivePowers[0];
//                double driveX = drivePowers[1];
//                double finalTurn = drivePowers[2];
//
//// Apply Slow Mode (Optional: can be moved inside SmartDrive later)
//                if (slowMode) {
//                    driveY *= slowModeMultiplier;
//                    driveX *= slowModeMultiplier;
//                    // We don't scale finalTurn here because SmartDrive handled the friction
//                }
//
//// ---------------------------------------------------------
//// 3. DRIVE EXECUTION
//// ---------------------------------------------------------
//                if (!automatedDrive) {
//                    if((boolean) PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CONTROLLER_DRIVE_ENABLED)) == true) {
//
//                        follower.setTeleOpDrive(driveY, driveX, finalTurn, true);
//
//                    }
//                } else {
//                    follower.update();
//                }
//
//                if(autoAimTrigger && targetVisible) {
//                    currentState = RobotState.TARGETING;
//                }
//                     ---------------------------------------------------------
//                     3. AIM CONTROL (AUTO & ASSIST)
//                     ---------------------------------------------------------
//                     Use Right Trigger for Auto-Aim Lock to avoid Bumper conflicts
                    boolean autoAimActive = (pad1.left_trigger > 0.5);

                    // AimControl modifies the 'turn' value based on Limelight data
                    // If autoAimActive -> It forces the robot to turn toward the tag
                    // If manual -> It slows down rotation if crosshair is on target (Aim Assist)
//                    double finalTurn = aimControl.update(turn, autoAimActive, currentResults, logging);

                    // ---------------------------------------------------------
                    // 4. DRIVE EXECUTION (MANUAL VS AUTOMATED)
                    // ---------------------------------------------------------

                    // Start Automated Pathing
                    if (pad1.a && !automatedDrive) {
//                        if((boolean) PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CONTROLLER_DRIVE_ENABLED)) == true) {

                            follower.followPath(pathChain.get(), true);
                            automatedDrive = true;
                            currentState = RobotState.DRIVING;
//                        }

                    }
                    telemetryS.addGraphData("DRIVE Velocity", follower.getVelocity().getMagnitude());

                    // Stop Automated Pathing
                    // (Cancelled by pressing 'A' again or if path finishes)
                    if (automatedDrive && !pad1.a /* || !follower.isBusy() */) {
                        follower.startTeleopDrive();
                        automatedDrive = false;
                    }
                    if(follower.getVelocity().getMagnitude() > 0.1){
                        currentState = RobotState.DRIVING;

                    }
                if((boolean) PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CONTROLLER_DRIVE_ENABLED)) == true) {
                follower.setMaxPower(0);
                }else{
                    follower.setMaxPower(1);
                }
                    // Apply Drive Powers
                    if (!automatedDrive) {
                        follower.setTeleOpDrive(
                                driveY,
                                driveX,
                                turn, // Using the aim-controlled turn value
                                true // Robot Centric
                        );
                    } else {
                        // Update the follower during pathing
                        follower.update();
                    }

                    // ---------------------------------------------------------
                    // 6. POSE RESET & UTILITIES
                    // ---------------------------------------------------------
                    if(pad1.left_bumper && pad1.right_bumper){
                            // Reset Pose based on Red/Blue Alliance light
                            follower.setPose(new Pose(62, AllianceSideMirroredInstance(62), Math.toRadians(0)));
                        // Haptic feedback for reset
                        pad1.rumble(ledRed.isLightOn() ? 100:0, ledRed.isLightOn() ? 0:100, 100);
                        if(follower.getPose().getY() < 0){
                            GobildaPWMLed.setPosition(0.611);
                        }
                        if(follower.getPose().getY() > 0){
                            GobildaPWMLed.setPosition(0.3);
                        }
                        while(pad1.left_bumper){};

                    }
                    telemetryS.addData("Alliance Sensed", alliance.name());

                    // ---------------------------------------------------------
                    // 7. TELEMETRY
                    // ---------------------------------------------------------
                    telemetryS.debug("Pose", follower.getPose());
//                    telemetryS.addData("Final", finalTurn);
                    telemetryS.debug("Mode", automatedDrive ? "AUTO PATH" : (autoAimActive ? "AIM LOCK" : "MANUAL"));
                    telemetryS.debug("Slow Mode", slowMode ? "ON (" + slowModeMultiplier + "x)" : "OFF");
                    telemetryS.debug("Target", "LOCKED (ID " + tagID + ")" );
if(autoAimActive){
    currentState = RobotState.TARGETING;

}

//                    CompletableFuture.runAsync(() -> {
                        try {
                            Drawing.drawForTeleop(follower, opmodeTimer, telemetryS);
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
//                    });

//                    Drawing.drawLine(follower.getPose().getX(), follower.getPose().getY(), )

//
//                if (!automatedDrive) {
//                    //Make the last parameter false for field-centric
//                    //In case the drivers want to use a "slowMode" you can scale the vectors
//
//                    //This is the normal version to use in the TeleOp
//                    if (!slowMode) follower.setTeleOpDrive(
//                            -pad1.left_stick_y,
//                            -pad1.left_stick_x,
//                            -pad1.right_stick_x,
//                            true // Robot Centric
//                    );
//
//                        //This is how it looks with slowMode on
//                    else follower.setTeleOpDrive(
//                            -pad1.left_stick_y * slowModeMultiplier,
//                            -pad1.left_stick_x * slowModeMultiplier,
//                            -pad1.right_stick_x * slowModeMultiplier,
//                            true // Robot Centric
//                    );
//                }
//                //Automated PathFollowing
//                if (pad1.a && !automatedDrive) {
//                    follower.followPath(pathChain.get(), true);
//                    automatedDrive = true;
//                }
//
//                //Stop automated following if the follower is done
//                if (automatedDrive && !pad1.a/* || !follower.isBusy())*/) {
//                    follower.startTeleopDrive();
//                    automatedDrive = false;
//                }
//
//                //Slow Mode
//                if (pad1.rightBumperWasPressed()) {
//                    slowMode = !slowMode;
//                }
//
//                //Optional way to change slow mode strength
//                if (pad1.x) {
//                    slowModeMultiplier += 0.25;
//                }
//
//                //Optional way to change slow mode strength
//                if (pad2.y) {
//                    slowModeMultiplier -= 0.25;
//                }
//                telemetryS.addLine("------------- Drive Telemetry --------------");
//
//                telemetryS.debug("position", follower.getPose());
//                telemetryS.debug("velocity", follower.getVelocity());
//                telemetryS.debug("automatedDrive", automatedDrive);
//                Drawing.drawDebug(follower);
//                if(pad1.left_bumper && pad1.right_bumper){
//                    if(pad1.share) {
//                        follower.setPose(new Pose(-62, (ledRed.isLightOn() ? -62 : 62), Math.toRadians(0)));
//                    }
//                    pad1.rumble(ledRed.isLightOn() ? 100:0, ledRed.isLightOn() ? 0:100, 100);
//
//                }
            }


            // Update Subsystems
            logging.log("POSELOG:" + follower.getPose().getX() + ", " + follower.getPose().getY() + ", " + follower.getPose().getHeading());
            if(telemetryS.getGamepad2().getDpadDown()){
                currentState = RobotState.INTAKING;
                telemetryS.addData("STATE INTAKING");
            }if(telemetryS.getGamepad2().getDpadUp()){
                currentState = RobotState.TARGETING;
                telemetryS.addData("STATE TARGETING");
            }if(telemetryS.getGamepad2().getDpadLeft()){
                currentState = RobotState.DRIVING;
                telemetryS.addData("STATE DRIVING");
            }if(telemetryS.getGamepad2().getDpadRight()){
                currentState = RobotState.SHOOTING;
                telemetryS.addData("STATE SHOOTING");
            }

            rubberBand();
//            CompletableFuture.runAsync(() -> {
                lights();
//            });
            try {
                propertiesSystem.saveProperties();
            } catch (IOException e) {
                telemetryS.addData("FAILUARE SAVING ", "OH NOOOO");
            }
            telemetryS.update();
//            CompletableFuture.runAsync(() -> {
                        Drawing.sendPacket();
//                    });

telemetryS.addData("TEST", telemetryS.getGamepad2());
//if(telemetryS.getGamepad2()){
//    follower = limelight.performLocalizationUpdate(follower, currentResults);
//}

            if(telemetryS.getGamepad2().getCircle()){
                follower = limelight.performLocalizationUpdate(follower, currentResults);
            }
        }
        try {
            logging.saveLog();
//                logging.savePose(follower.getPose(), opmodeTimer);
            logging.savePoses(Drawing.allPoses, opmodeTimer);
        } catch (Exception e) {
        }
    }

    private int upPosition = 0;
    private int oldPosition = 0;
    private int selected_color = 0;

    // LED Color Map for PWM strip
    private String[] ColorsString = {"Black", "Red", "Orange", "Yellow", "Sage", "Green", "Azure", "Blue", "Indigo", "Violet"};
    private final double[] Colors = {0, 0.227, 0.333, 0.388, 0.444, 0.5, 0.555, 0.611, 0.666, 0.722};
    private double randomNumber1 = 1000+ Math.random()*5000;
    private double randomNumber2 = 1000+Math.random()*5000;
    private double randomNumber3 = 1000+Math.random()*5000;
    private double randomNumber4 = 1000+Math.random()*5000;
    private double randomNumber5 = 1000+Math.random()*5000;
    private double randomNumber6 = 1000+Math.random()*5000;
    /**
     * Controls the GoBilda Prism Driver and PWM strip based on the robot's current flags.
     */
    private void lights() {
//        pose6d.

        Pose6D robotPoseSystem = pose6d;
        ArrayList<RobotTask> tasks = robotPoseSystem.position.getTasks();
        Pose6D.Positional robotpos = robotPoseSystem.whereIsMyRobot();

        switch (robotpos.getCurrentPositon()) {
            case LAUNCH_ZONE:
                ///  Commands for when in Launch Zone
                ///  ONLY LED COMMANDS HERE
                switch (robotpos.getCurrentState()) {
                    case FULLY:
                        BackLed.setPosition(calculateFade(0.227, 0.388, 1000));
                        break;
                    case PARTIAL:
                        BackLed.setPosition(calculateFade(0, 0.227, 1000));
                        break;
                    case APPROACHING:
                        //Doesnt apply here for now
                        break;
                }
                break;
            case LOADING_ZONE:
                ///  WHEN WE LOAD BALLS
                switch (robotpos.getCurrentState()) {
                    case FULLY:
                        BackLed.setPosition(calculateFade(0.444, .5, 1000));
                        if(flywheel.getState() == FlywheelLogic.FlywheelState.INTAKE){
                            GobildaPWMLed.setPosition(calculateFade(0.5,0.55, 1000));
                        }
                        break;
                    case PARTIAL:
                        BackLed.setPosition(calculateFade(.49, 0.5, 1000));
                        break;
                    case APPROACHING:
                        break;
                }
                break;
            case UNKNOWN:
                ///  SELF EXPLAINTORY
                BackLed.setPosition(calculateFade(0.2, 1, 5200));

                break;

            /**
             * TODOOOOOOOOO
             */
        }
        switch (currentState){

            case DRIVING:
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
                pad1.setLedColor(0,calculateFade(125,255, 500),calculateFade(125,255, 500),200);
                GobildaPWMLed.setPosition(calculateFade(0.2,0.333, 1000));

                break;
            case IDLE:
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
                pad1.setLedColor(calculateFade(0, 255, randomNumber5),calculateFade(0, 255, randomNumber6), 255, 1);
                pad2.setLedColor(255,calculateFade(0, 255, randomNumber1),calculateFade(0, 255, randomNumber2), 1);
//                GobildaPWMLed.setPosition(0.388);
                GobildaPWMLed.setPosition(calculateFade(0.555,0.66, 500));
                break;
            case INTAKING:
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
                pad2.setLedColor(calculateFade(125,255, 500),calculateFade(125,255, 500),0,200);
                GobildaPWMLed.setPosition(calculateFade(0.444,0.5, 1000));
                break;
            case SHOOTING:
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3);
                GobildaPWMLed.setPosition(calculateFade(0.555,0.66, 500));
                break;
            case TARGETING:
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_4);
                GobildaPWMLed.setPosition(calculateFade(0.18,0.25, 400));

                break;


        }
        // Cycle colors manually on Gamepad 2
        if(pad2.guide){
            GobildaPWMLed.setPosition(Colors[selected_color]);
            selected_color++;
            if(Colors.length < selected_color) selected_color = 0;
            PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
        }

        // Automated Feedback Animations
        if((boolean) PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_ENABLED_PRISM_DRIVER)) == true) {
            PrismLEDDriver.setStripLength((int)Math.floor(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_COUNT_PRISM_DRIVER)));


        } else {
            PrismLEDDriver.setStripLength(0);
        }
    }

    //IT makes the led slow fad to those values
    // Add these as class variables at the top of your OpMode
    private long fadeStartTime = System.currentTimeMillis();

    /**
     * Calculates a smooth fade between two positions based on time.
     * @param min The starting position (e.g., 0.227 for Red)
     * @param max The ending position (e.g., 0.333 for Orange)
     * @param duration Total time for one full fade cycle in milliseconds
     */
    private double calculateFade(double min, double max, double duration) {
        long elapsed = System.currentTimeMillis() - fadeStartTime;

        // Get progress through the current cycle (0.0 to 1.0)
        double progress = (elapsed % duration) /  duration;

        // Ping-pong effect: if in the second half of the duration, go backwards
        // This creates the "fade back and forth" look
        double factor;
        if (progress < 0.5) {
            factor = progress * 2; // 0.0 to 1.0
        } else {
            factor = 1.0 - ((progress - 0.5) * 2); // 1.0 to 0.0
        }

        // Linear Interpolation: result = min + (range * factor)
        return min + (factor * (max - min));
    }

    /**
     * Manages the wrist rotation and intake mechanism.
     * Uses magnetic sensors to prevent mechanical strain at limit points.
     */
    private void rubberBand() {
        if(pad2.x){
            flywheel.toggle();
            while(pad2.x){}
        }
//        telemetryS.addData("UpperSensorValue", MagnetSensorUpper.getValue());
//        telemetryS.addData("LowerSensorValue", MagnetSensorLower.getValue());

        // Intake Power Control
//        if(pad1.square){
//            LRBS.setPower(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LEFT_RUBBER_BAND_POWER));
//            RRBS.setPower(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.RIGHT_RUBBER_BAND_POWER));
//        } else {
//            LRBS.setPower(0);
//            RRBS.setPower(0);
//        }
//
//        // Direct Direction Toggle
//        if(pad1.dpad_down){
//            LRBS.setDirection(LRBS.getDirection() == CRServo.Direction.FORWARD ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
//            RRBS.setDirection(RRBS.getDirection() == CRServo.Direction.FORWARD ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
//        }

        // Increment/Decrement Wrist Goal Position
        if(pad1.a){ upPosition++; }
        if(pad1.b){ upPosition--; }
        if(pad2.a){/// a and x are the same button.......
            MIS.setPower(-100);
            logging.log("Setting power for MIS to -100");
        }else if(pad2.x) {
            MIS.setPower(-100);
        }else{
            MIS.setPower(0);
        }
//
//        LWS.setPosition(-pad2.left_stick_y/7.5+0.5);
        if(pad2.right_bumper && pad2.left_bumper){
            logging.log("Setting pos for RWS to 0.7");

            RWS.setPosition(0.7);
            LWS.setPosition(-0.7);
        }else if(pad2.right_bumper) {
            RWS.setPosition(0.5);
            LWS.setPosition(-0.5);
            logging.log("Setting pos for RWS to 0.5");

        }else if(pad2.left_bumper) {
            RWS.setPosition(0.85);
            logging.log("Setting pos for RWS to 0.86");
            LWS.setPosition(-0.85);
        }else{

            RWS.setPosition(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.WRIST_POSITION));
            logging.log("Setting pos for RWS to 0.9");
            LWS.setPosition(-propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.WRIST_POSITION));
        }
                if (pad2.triangle) {
                    logging.log("Setting power for LRBS to 1");
                    currentState = RobotState.INTAKING;

            LRBS.setPower(1);
            RRBS.setPower(1);
        } else {
            LRBS.setPower(0);
            RRBS.setPower(0);
        }




        // Physical movement based on change in target position
//        if(oldPosition != upPosition) {
            // Determine rotation direction
//            if (oldPosition > upPosition) {
//                LWS.setDirection(CRServo.Direction.FORWARD);
//                RWS.setDirection(CRServo.Direction.REVERSE);
//            } else {
//                LWS.setDirection(CRServo.Direction.REVERSE);
//                RWS.setDirection(CRServo.Direction.FORWARD);
//            }
//
//            // Safety Interlock: Stop if limits are reached
//            if (MagnetSensorLower.isPressed() || MagnetSensorUpper.isPressed()) {
//                LWS.setPower(0);
//                RWS.setPower(0);
//            } else {
//                LWS.setPower(0.5);
//                RWS.setPower(0.5);
//                sleep(250); // Temporal movement block
//                LWS.setPower(0);
//                RWS.setPower(0);
//                oldPosition = upPosition;
//            }
//        }
//        oldPosition = upPosition;
    }

    /**
     * Standard Mecanum Drive Implementation.
     * Maps stick inputs to motor powers using trigonometric sum of vectors.
     */
    private void drive(){
        // Raw Inputs
        float walk = -pad1.left_stick_y;     // Forward/Backward
        float strafe = pad1.left_stick_x;   // Left/Right
        float pivot = pad1.right_stick_x;  // Rotation
        float dampen = pad1.right_trigger;  // Variable speed reduction
        if(!pad2.right_bumper){
            dampen = 0.7F;
        }
        // Kinematics Math
//        float fl = walk + strafe + pivot;
//        float fr = walk - strafe - pivot;
//        float bl = walk + strafe + pivot;
//        float br = walk - strafe - pivot;

        float fl = walk + strafe;
        float fr = walk - strafe;
        float bl = walk - strafe;
        float br = walk + strafe;

        fl += pivot;
        fr -= pivot;
        bl += pivot;
        br -= pivot;

        // Apply Dampening Scalar (Formula: Power / (DampenWeight + Offset))
        fl /= (dampen * 1.8f) + 0.4f;
        fr /= (dampen * 1.8f) + 0.4f;
        bl /= (dampen * 1.8f) + 0.4f;
        br /= (dampen * 1.8f) + 0.4f;

        // Final Power Application if enabled in config
        if((boolean) PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CONTROLLER_DRIVE_ENABLED)) == true) {
            FLM.setPower(fl);
            FRM.setPower(fr);
            BLM.setPower(bl);
            BRM.setPower(br);
        }

        // Update driving flag for LEDs
//        isDriving = fl!=0 || fr!=0 || bl!=0 || br!=0;
    }
//    @Override
//    public void stop() {
//        communication.COMMUNICATABLES.put(AutoTeleOpCommunicationSystem.Communicates.END_POSE_X, follower.getPose().getX()+ "");
//        communication.COMMUNICATABLES.put(AutoTeleOpCommunicationSystem.Communicates.END_POSE_Y, follower.getPose().getY() + ""); // Note: Check if Y should be here
//        communication.COMMUNICATABLES.put(AutoTeleOpCommunicationSystem.Communicates.END_POSE_RADIANS, follower.getPose().getHeading() + "");
//        // Resource cleanup (if applicable)
//    }
    static class RobotTask{
        public DcMotorEx motor;
        public DcMotorEx[] motors;
        public CRServo servo;
        public double Velocity;
        public double power;

    public RobotTask(DcMotorEx motor, int power){
            this.motor = motor;
        this.servo = null;
        this.power = power;
        this.Velocity = -1;


    }
    public RobotTask(CRServo servo, double power){
        this.servo = servo;
        this.motor = null;
        this.power = power;
        this.Velocity = -1;

    }
    public RobotTask(DcMotorEx motor, double velocity){
        this.motor = motor;
        this.servo = null;
        this.power = -1;
            this.Velocity = velocity;
    }
    public RobotTask(DcMotorEx[] motors, double velocity){
        this.motor = null;
        this.servo = null;
        this.power = -1;
        this.Velocity = velocity;
        this.motors = motors;
    }


    /// Auto makes the task run but not if the input for the controller is true
    public boolean runTaskWithAutoOverride(boolean gamepadInput, LoggingSystem logging) {
        if (!gamepadInput) {
            if (motors.length > 1) {
                if (Velocity > 0) {
                    for (DcMotorEx m : motors) {
                        logging.log(m.getDeviceName() + " Is being Automatically set to Velocity " + Velocity);
                        FlywheelLogic.applyPIDF(m);
                        /* Set velocity */
                        m.setVelocity(Velocity);

                        /* Telemetry */
                    }
                }else{
                    return true;
                }
            } else if (motor != null) {
                if (Velocity != -1) {
                    logging.log(motor.getDeviceName() + " Is being Automatically set to Velocity " + Velocity);

                    FlywheelLogic.applyPIDF(motor);

                    /* Set velocity */
                    motor.setVelocity(Velocity);

                    /* Telemetry */
                } else if (power != -1) {
                    logging.log(motor.getDeviceName() + " Is being Automatically set to Power " + power);

                    motor.setPower(power);
                }

            } else {
                logging.log(servo.getDeviceName() + " Is being Automatically set to Power " + power);
                servo.setPower(power);
            }
        }
        return false;
    }

        /// Auto makes the task run but not if the input for the controller is true
        public void runTaskWithAutoOverride(boolean gamepadInput, double ComputedVelocity, LoggingSystem logging) {
            if (!gamepadInput) {
                if (motors.length > 1) {
                    if (Velocity == -1) {
                        for (DcMotorEx m : motors) {
                            logging.log(m.getDeviceName() + " Is being Automatically set to Power " + ComputedVelocity);

                            FlywheelLogic.applyPIDF(m);
                            /* Set velocity */
                            m.setVelocity(ComputedVelocity);

                            /* Telemetry */
                        }
                    }
                }

        }
    }

}
    public static String getRandomHexColor() {
        Random random = new Random();
        // Generate a random integer between 0 and 0xFFFFFF (16777215)
        int nextInt = random.nextInt(0xffffff + 1);

        // Format as a 6-digit hex string with a # prefix
        return String.format("#%06x", nextInt);
    }
static class DrawingPose{
        /// Makes a pose with a color style for drawing

        private static  Style inputStyle;
        private static  Pose pose;

    public DrawingPose(Pose pose, Style style ){
        this.inputStyle = style;
        this.pose = pose;
    }
    //incase no style is provided
    public DrawingPose(Pose pose){
        this.inputStyle = new Style(getRandomHexColor(), getRandomHexColor(), 1);
        this.pose = pose;
    }
    public Pose getAsPose(){
        return this.pose;
    }
    public Style getStyle(){
        return inputStyle;
    }
    public double getY(){
        return pose.getY();

    }
    public double getX(){
        return pose.getX();
    }
    public String toString(){
        return "X" + getX() + ", Y" + getY() + ", POSe" + pose.getX() + " " + pose.getY();
    }

}

    //Pose 6D Not really 6D but 6 Pose storage current pose etc this class is for Finding the robot positon And zones
    static class Pose6D{
        public enum RobotPosition {UNKNOWN, LAUNCH_ZONE, LOADING_ZONE, LAUNCH_LINE, IN_BASE, SECERT_TUNNEL};
        public enum PositionState {UNKNOWN, APPROACHING, PARTIAL, FULLY};
        public RobotZone allZones = new RobotZone();
        //this stores the current pose  expected next pose and historys
        public Pose robotPose;
         class RobotZone{
             class Zone{
                public ArrayList<Pose> Points = new ArrayList<>();
                public RobotPosition name;
                public Style zoneColor;
                public ArrayList<RobotTask> Tasks = new ArrayList<>();
                public Zone(RobotPosition name,Pose... poses){
                    this.name = name;
                    for(Pose pose : poses){Points.add(pose);}
                }
                public void addPoint(Pose pose){ Points.add(pose); }
                public ArrayList<Pose> getPoints(){ return Points; }
                 /// Adds that task to the zone for the robot to do when in zone
                 public Zone addTask(DcMotorEx motor, int power){
                    //Should handle pidf tuned ones aswell butT
                     Tasks.add(new RobotTask(motor, power));
                     return this;
                 }
                 public Zone addTask(DcMotorEx motor, double velocity){
                     Tasks.add(new RobotTask(motor, velocity));
                     return this;
                 }

                 public Zone addTask(DcMotorEx[] motor, double velocity){
                     Tasks.add(new RobotTask(motor, velocity));
                     return this;
                 }

                 public Zone addTask(CRServo servo, double power){
                     Tasks.add(new RobotTask(servo, power));
                     return this;

                 }
                 public void setColor(String fill){
                     zoneColor = new Style(fill,fill,1);
                 }
                 public void setColor(String fill, String outline, double width){
                     zoneColor = new Style(fill, outline, width);
                 }

                 public ArrayList<RobotTask> getTasks(){
                     return Tasks;
                 }
                 public ArrayList<DrawingPose> getPointsAsDrawingPoints(){
                     ArrayList<DrawingPose> rtn = new ArrayList<>();
                     for(Pose point : Points){
                         logging.log("POINT" + point.getX() + ": " + point.getY());
                         if(zoneColor == null){
                             rtn.add(new DrawingPose(point));
                         }else{
                             rtn.add(new DrawingPose(point, zoneColor));
                        }
                     }
                     logging.log(rtn.size()+ "");
                     logging.log(rtn.get(0).getY() + "");
                     return rtn;
                 }
            }
            public ArrayList<Zone> zones = new ArrayList<Zone>();


            public Zone addZone(RobotPosition name, Pose... poses){Zone newZone = new Zone(name, poses); zones.add(newZone); return newZone;}
            public ArrayList<Zone> getZones(){ return zones; }
            public Zone getZoneByName(RobotPosition name){
                Zone zoneFound = null;
                for(Zone z : zones){ if(z.name == name){ zoneFound=  z; } }
                return zoneFound;
            }

            /**
             *
             * @return the Positonal of it
             */
            public Positional isInZone(){
                Positional pos = new Positional(RobotPosition.UNKNOWN, PositionState.UNKNOWN);
                for(Zone z : zones){
                    boolean robot5 = isInsidePolygonWithMargin(robotPose.getX(), robotPose.getY(), 5.0, z.Points);
                    boolean robot = isInsidePolygonWithMargin(robotPose.getX(), robotPose.getY(), 0.0, z.Points);
                    if(robot5){
                        pos.setCurrentPositon(z.name);
                        pos.setCurrentState(PositionState.FULLY);
                        pos.setTasks(z.Tasks);
                    }else if(robot) {
                            pos.setCurrentPositon(z.name);
                            pos.setCurrentState(PositionState.PARTIAL);
                            pos.setTasks(z.Tasks);

                        }
                }
                return pos;
            }

        }
         class Positional{
            private RobotPosition currentPositon = RobotPosition.UNKNOWN;
            private PositionState currentState = PositionState.UNKNOWN;
            private ArrayList<RobotTask> Tasks = new ArrayList<>();

             public  Positional(RobotPosition pos, PositionState state, ArrayList<RobotTask> Tasks){
                 this.currentPositon = pos;
                 this.Tasks = Tasks;
                 this.currentState = state;
             }
             public  Positional(RobotPosition pos, PositionState state){
                 this.currentPositon = pos;
//                 this.Tasks = Tasks;
                 this.currentState = state;
             }
            public void setCurrentPositon(RobotPosition pos) { this.currentPositon = pos; }
            public void setCurrentState(PositionState state){ this.currentState = state; }
            public RobotPosition getCurrentPositon(){ return this.currentPositon; }
            public PositionState getCurrentState(){ return this.currentState; }
             public void addTask(RobotTask task){
                 Tasks.add((task));
             }
             public void setTasks(ArrayList<RobotTask> tasks){
                 this.Tasks = tasks;
             }
             public ArrayList<RobotTask> getTasks(){ return this.Tasks;}
            public void merge(Positional pos){ this.currentState = pos.currentState; this.currentPositon = pos.currentPositon; }
        }


        public Positional position = new Positional(RobotPosition.UNKNOWN, PositionState.UNKNOWN);
//        public Pose next
        public Pose6D(LoggingSystem logging){
            logging.log("Pose 6D in Creation now");
//Now we init the zones so it knows where styff on the feild is
            //Loading Zone For Blue
            allZones.addZone(RobotPosition.LOADING_ZONE,
                    new Pose(72,-72), new Pose(72,-45),
                    new Pose(45,-45), new Pose(45,-72)
            ).addTask(LRBS, 1).addTask(RRBS, 1).addTask(MIS, 0.7).setColor("#97a832");
            //Loading zone for Red
            allZones.addZone(RobotPosition.LOADING_ZONE,
                    new Pose(72,72), new Pose(72,45),
                    new Pose(45,45), new Pose(45,72)
            ).addTask(LRBS, 1).addTask(RRBS, 1).addTask(MIS, 0.7).setColor("#97a832");

            //The Launch Zone

            allZones.addZone(RobotPosition.LAUNCH_ZONE,
                    new Pose(0, 0),
                    new Pose(-72, 65),
                    new Pose( -72, -65)
            ).addTask(LRBS, 1).addTask(RRBS, 1).addTask(MIS, -1).setColor("#97a832");///todo make a addawaittask
            /// that would make it so it wont run tasks till its speeding etc honestly need to kinda figure a good way for this
            //Launch Line
            allZones.addZone(RobotPosition.LAUNCH_LINE,
                    new Pose(0, 0),
                    new Pose(-72, 77),
                    new Pose(-72, 72),
                    new Pose(5,0),
//                    new Pose(5, 0),
                    new Pose(-72, -77),
                    new Pose(-72, -72),
                    new Pose(0,0)
            ).addTask(LRBS, 1).addTask(RRBS, 1).addTask(MIS, -1).setColor("#aaaaaa");
            for(RobotZone.Zone zone : allZones.getZones()){
                for(Pose point : zone.getPoints()){
                    logging.log("Pose6DZONES:" + zone.name + ", " + point.getX() + ", " + point.getY() + ", "+ point.getHeading());
                }

            }

        }
        public Positional whereIsMyRobot(){
            return position;
        }

        public RobotPosition lastPosition;
        public PositionState lastState;
        public void updateRobotPositon(Follower follower, LoggingSystem logging, PanelsSystem telemetryS){
            this.robotPose = follower.getPose().getPose();
            /**
             * TODO MAKE IT SO WE CAN SEE of the robot heading if its facing to shoot if it is then we can asusme to shoot
             */


            //merges data from that to the position to update it
            position.merge(allZones.isInZone());
            if(lastState != position.getCurrentState()) logging.log("Robot is" + position.getCurrentState());
            if(lastPosition != position.getCurrentPositon()) logging.log("Robot is In" + position.getCurrentPositon());
            telemetryS.addData("Robot Position ", position.getCurrentPositon());
            telemetryS.addData("Robot State ", position.getCurrentState());
        }

        public void mapZonesToPanels(){
            //self explaorty
            for(RobotZone.Zone zone : allZones.getZones()){
//                ArrayList<DrawingPose> points = zone.getPointsAsDrawingPoints();
//                telemetryS.speak("" +points.size());
//                DrawingPose lastPose = points.get(0);
//                logging.log("DRAWS" + points.size());
//                logging.log("DRAW" + points.toString());
//                DrawingPose firstPose = points.get(0);
//                for(DrawingPose pose : points) {
////                    draw
//                    Drawing.drawColoredLine(firstPose.getStyle(), lastPose.getX(), lastPose.getY(), pose.getX(), pose.getY());
//                    logging.log("DRAWING POSE" + lastPose.getX() + ":" + lastPose.getY() + " THEN TO " + pose.getX() + ":"+ pose.getY());
//                    lastPose = pose;
//
////Drawing.
//                }
//                ///  Otherwise the last point doesnt connect to the first and its a incomlete shade so
//                Drawing.drawColoredLine(firstPose.getStyle(),firstPose.getX(), firstPose.getY(), lastPose.getX(), lastPose.getY());

                Pose lastPose = zone.Points.get(0);
                Pose firstPose = zone.Points.get(0);
                for(Pose pose : zone.Points){
                    Drawing.drawLine(lastPose.getX(), lastPose.getY(), pose.getX(), pose.getY(), zone.zoneColor);
                    lastPose = pose;
                }
//                Pose lastsPose = Drawing.allPoses.get(0);
//            for(Pose pose : Drawing.allPoses){
//                Drawing.drawLine(pose.getX(), pose.getY(), lastsPose.getX(), lastsPose.getY(), Drawing.robotExpectedPath);
//                lastsPose = pose;
//            }
                Drawing.drawLine(firstPose.getX(), firstPose.getY(), lastPose.getX(), lastPose.getY(), zone.zoneColor);
            }
        }




    }

//    public class ShapeCoor
    /**
     * Checks if point (px, py) is inside triangle ABC AND at least 'margin' away from edges.
     */
    public static boolean isInsideWithMargin(double px, double py,
                                             double ax, double ay,
                                             double bx, double by,
                                             double cx, double cy,
                                             double margin) {

        // 1. First, check if it's even in the triangle at all
        if (!isPointInTriangle(px, py, ax, ay, bx, by, cx, cy)) return false;

        // 2. Check distance to all three edges (A-B, B-C, C-A)
        double distAB = linePointDist(px, py, ax, ay, bx, by);
        double distBC = linePointDist(px, py, bx, by, cx, cy);
        double distCA = linePointDist(px, py, cx, cy, ax, ay);

        // 3. Return true only if it's further than the margin from every wall
        return (distAB >= margin && distBC >= margin && distCA >= margin);
    }

    // Helper to calculate distance from point P to line segment AB
    private static double linePointDist(double x, double y, double x1, double y1, double x2, double y2) {
        double A = x - x1;
        double B = y - y1;
        double C = x2 - x1;
        double D = y2 - y1;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = (len_sq != 0) ? dot / len_sq : -1;

        double xx, yy;

        if (param < 0) {
            xx = x1;
            yy = y1;
        } else if (param > 1) {
            xx = x2;
            yy = y2;
        } else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        double dx = x - xx;
        double dy = y - yy;
        return Math.sqrt(dx * dx + dy * dy);
    }
    public static boolean isPointInTriangle(double px, double py,
                                            double ax, double ay,
                                            double bx, double by,
                                            double cx, double cy) {

        // Calculate the areas (using cross product) of the 3 sub-triangles
        double d1 = sign(px, py, ax, ay, bx, by);
        double d2 = sign(px, py, bx, by, cx, cy);
        double d3 = sign(px, py, cx, cy, ax, ay);

        boolean has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        // If there are no negative areas OR no positive areas, the point is inside
        return !(has_neg && has_pos);
    }

    private static double sign(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y) {
        return (p1x - p3x) * (p2y - p3y) - (p2x - p3x) * (p1y - p3y);
    }
    /**
     * @param px, py The robot's current X and Y
     * @param x1, y1 Corner 1 of the rectangle
     * @param x2, y2 Corner 2 (opposite corner)
     * @param x3, y3 Corner 3 ( corner)
     * @param x4, y4 Corner 4 ( corner)
     * @param margin The distance within the border (e.g., 5.0)
     */
    public static boolean isInsideRectangle(double px, double py, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double margin) {
        return isInsidePolygonWithMargin(px,py, margin, new Pose(x1,y1), new Pose(x2,y2), new Pose(x3,y3), new Pose(x4,y4));
    }
    public static boolean isInsidePolygonWithMargin(double px, double py, double margin, ArrayList<Pose> vertices){
        Pose[] ary = new Pose[vertices.size()];
        int index = 0;
        for(Pose vert : vertices){
            ary[index] = vert;
            index++;
        }
        return isInsidePolygonWithMargin(px,py,margin, ary);
    }
    public static boolean isInsidePolygonWithMargin(double px, double py, double margin, Pose... vertices) {
        // 1. Check if we are inside the shape at all
        if (!isInsidePolygon(px, py, vertices)) return false;

        // 2. Check if we are too close to any of the walls
        for (int i = 0, j = vertices.length - 1; i < vertices.length; j = i++) {
            double d = linePointDist(px, py,
                    vertices[i].getX(), vertices[i].getY(),
                    vertices[j].getX(), vertices[j].getY());
            if (d < margin) return false;
        }

        return true;
    }
    /**
     * Universal Point-in-Polygon check.
     * @param px The robot's current X
     * @param py The robot's current Y
     * @param vertices An array of Poses defining the corners of the shape in order.
     * @return true if the point is inside the polygon.
     */
    public static boolean isInsidePolygon(double px, double py, Pose... vertices) {
        int numVertices = vertices.length;
        boolean inside = false;

        // Iterate through every edge of the polygon
        for (int i = 0, j = numVertices - 1; i < numVertices; j = i++) {
            double xi = vertices[i].getX(), yi = vertices[i].getY();
            double xj = vertices[j].getX(), yj = vertices[j].getY();

            // Ray casting algorithm math
            boolean intersect = ((yi > py) != (yj > py))
                    && (px < (xj - xi) * (py - yi) / (yj - yi) + xi);

            if (intersect) inside = !inside;
        }

        return inside;
    }
public static double AllianceSideMirroredInstance(double redValue){
        switch(alliance){
            case RED:
                return redValue;
            case BLUE:
                return -redValue;
            case YELLOW:
                return (Math.random() * 2) > 1 ? redValue : -redValue;
        }
        return redValue;
}


    /**
     * Checks all standard buttons for a "Rising Edge" (Pressed now, but wasn't before).
     */
    private void checkGamepadChanges(Gamepad current, Gamepad previous, String name) {

        // --- Face Buttons ---
        if (current.a && !previous.a) logging.log(name + ": A Pressed");
        if (current.b && !previous.b) logging.log(name + ": B Pressed");
        if (current.x && !previous.x) logging.log(name + ": X Pressed");
        if (current.y && !previous.y) logging.log(name + ": Y Pressed");

        // --- D-Pad ---
        if (current.dpad_up && !previous.dpad_up)       logging.log(name + ": Dpad Up");
        if (current.dpad_down && !previous.dpad_down)   logging.log(name + ": Dpad Down");
        if (current.dpad_left && !previous.dpad_left)   logging.log(name + ": Dpad Left");
        if (current.dpad_right && !previous.dpad_right) logging.log(name + ": Dpad Right");

        // --- Bumpers ---
        if (current.left_bumper && !previous.left_bumper)   logging.log(name + ": Left Bumper");
        if (current.right_bumper && !previous.right_bumper) logging.log(name + ": Right Bumper");

        // --- Triggers (Treating as buttons over 50% threshold) ---
        if (current.left_trigger > 0.5 && previous.left_trigger <= 0.5)   logging.log(name + ": Left Trigger");
        if (current.right_trigger > 0.5 && previous.right_trigger <= 0.5) logging.log(name + ": Right Trigger");

        // --- Middle Buttons ---
        if (current.start && !previous.start) logging.log(name + ": Start");
        if (current.back && !previous.back)   logging.log(name + ": Back/Select");
        if (current.guide && !previous.guide) logging.log(name + ": Guide");

        // --- Stick Buttons (Clicking the stick in) ---
        if (current.left_stick_button && !previous.left_stick_button)   logging.log(name + ": Left Stick Click");
        if (current.right_stick_button && !previous.right_stick_button) logging.log(name + ": Right Stick Click");
    }
}