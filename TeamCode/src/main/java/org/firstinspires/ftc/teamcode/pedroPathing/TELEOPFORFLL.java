package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.System.PropertiesSystem.PropertyCleaner;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.System.AutoTeleOpCommunicationSystem;
import org.firstinspires.ftc.teamcode.System.LoggingSystem;
import org.firstinspires.ftc.teamcode.System.PropertiesSystem;
import org.firstinspires.ftc.teamcode.System.PanelsSystem;

import java.util.function.Supplier;

/**
 * <h1>TeleOp26 - Comprehensive Robot Control System</h1>
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
@TeleOp(name="RUN ME FLL", group="TELEOP")
public class TELEOPFORFLL extends LinearOpMode {

    // ------------------------------------------------------------------------------------------------
    // HARDWARE ACCESSORS
    // ------------------------------------------------------------------------------------------------
    private DcMotor FLM; // Front Left Motor
    private DcMotor FRM; // Front Right Motor
    private DcMotor BLM; // Back Left Motor
    private DcMotor BRM; // Back Right Motor
    private Servo LWS; // Left Wrist Servo (Continuous Rotation)
    private Servo RWS; // Right Wrist Servo (Continuous Rotation)
    private TouchSensor MagnetSensorUpper; // Upper limit safety for wrist
    private TouchSensor MagnetSensorLower; // Lower limit safety for wrist
    private CRServo LRBS; // Left Rubber Band Servo (Intake)
    private CRServo RRBS; // Right Rubber Band Servo (Intake)
    private CRServo MIS;  // Middle Intake Servo
    private DcMotor LLM;  // Left Launcher Motor
    private DcMotor RLM;  // Right Launcher Motor
    private DcMotor MLM;  // Middle Launcher Motor

    private FlywheelLogic flywheel = new FlywheelLogic();
    private GoBildaPrismDriver PrismLEDDriver; // Main LED Controller
    private ColorRangeSensor allianceSensor;   // Detects sample/alliance color

    // Indicator LEDs for the expansion/control hub ports
    private LED ledRed;
    private LED ledGreen;

    private Servo GobildaPWMLed; // Addressable PWM LED strip control

    private int counts = 0; // Iteration counter for animation timing

    // ------------------------------------------------------------------------------------------------
    // STATE FLAGS (Used for Logic and LED Feedback)
    // ------------------------------------------------------------------------------------------------
    public boolean isDriving = false;      // True if joysticks are moved
    public boolean isIntaking = false;     // True if intake servos are active
    public boolean isShooting = false;     // True if launcher motors are firing
    public boolean midFieldPickup = false; // Toggle for specific pickup routine
    public boolean isTargeting = false;    // True during automated alignment
    public boolean styleMode = false;      // Purely aesthetic state
    public boolean isWristing = false;     // True if wrist servos are moving
    public boolean[] ballsCollected = {false, false, false}; // Inventory tracking

    private boolean JustLeftAdjustmentMenu = false; // Handles hardware re-init after tuning
    public PanelsSystem telemetryS;
    public PropertiesSystem propertiesSystem;
    public LoggingSystem logging;
    public AutoTeleOpCommunicationSystem communication;

    // ------------------------------------------------------------------------------------------------
    // NAVIGATION & PATHING (PedroPathing Integration)
    // ------------------------------------------------------------------------------------------------
    private Follower follower;
    private Pose startingPose = new Pose(64, 12, Math.toRadians(180)); // Default starting position
    private boolean automatedDrive; // Flag for when Pedro is in control
    private Supplier<PathChain> pathChain; // Functional interface for dynamic path generation

    private boolean slowMode = false;         // Drivetrain sensitivity toggle
    private double slowModeMultiplier = 0.5;  // Power reduction factor for slowMode

    /**
     * Initializes all motor hardware, performs null-checks, and sets directions/behaviors
     * based on the PropertiesSystem configuration file.
     */
    private void setupMotors() {
//        flywheel.init(hardwareMap, telemetryS);
        FLM = hardwareMap.tryGet(DcMotor.class, "FL");
        FRM = hardwareMap.tryGet(DcMotor.class, "FR");
        BLM = hardwareMap.tryGet(DcMotor.class, "BL");
        BRM = hardwareMap.tryGet(DcMotor.class, "BR");
        LLM = hardwareMap.tryGet(DcMotor.class, "LLM");
        RLM = hardwareMap.tryGet(DcMotor.class, "RLM");
        MLM = hardwareMap.tryGet(DcMotor.class, "MLM");

        // Null-Safety: Notify the driver if hardware is missing from the active configuration
//        if(FLM == null){ telemetryS.speak("Variable FLM is Null depicting that FLM doesnt exist in the config Please update Config");}
//        if(FRM == null){ telemetryS.speak("Variable FRM is Null depicting that FRM doesnt exist in the config Please update Config");}
//        if(BLM == null){ telemetryS.speak("Variable BLM is Null depicting that BLM doesnt exist in the config Please update Config");}
//        if(BRM == null){ telemetryS.speak("Variable BRM is Null depicting that BRM doesnt exist in the config Please update Config");}
//        if(LLM == null){ telemetryS.speak("Variable LLM is Null depicting that LLM doesnt exist in the config Please update Config");}
//        if(RLM == null){ telemetryS.speak("Variable RLM is Null depicting that RLM doesnt exist in the config Please update Config");}
//        if(MLM == null){ telemetryS.speak("Variable MLM is Null depicting that MLM doesnt exist in the config Please update Config");}
//        telemetryS.update();

        // Critical Failure: Terminate if no motors are found (prevents crash later)
        if(FLM == null && FRM == null && BLM == null && BRM == null && LLM == null && RLM == null && MLM == null){
//            telemetryS.speak("Ternimating OP Mode!");
//            telemetryS.update();
        }

        // Apply ZeroPowerBehavior based on user-defined property
        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.DRIVE_MOTOR_ZERO_POWER_BHV)) == "BRAKE") {
            FLM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            FRM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            BLM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            BRM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        } else {
            FLM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            FRM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            BLM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            BRM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        }

        // Apply Motor Directions dynamically from config file
//        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.BL_MOTOR_DIRECTION)) == "FORWARD") {
//            BLM.setDirection(DcMotorSimple.Direction.FORWARD);
//        } else {
            BLM.setDirection(DcMotorSimple.Direction.REVERSE);
//        }

//        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.BR_MOTOR_DIRECTION)) == "FORWARD") {
//            BRM.setDirection(DcMotorSimple.Direction.FORWARD);
//        } else {
            BRM.setDirection(DcMotorSimple.Direction.REVERSE);
//        }

//        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.FR_MOTOR_DIRECTION)) == "FORWARD") {
//            FRM.setDirection(DcMotorSimple.Direction.FORWARD);
//        } else {
//            FRM.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//
//        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.FL_MOTOR_DIRECTION)) == "FORWARD") {
//            FLM.setDirection(DcMotorSimple.Direction.FORWARD);
//        } else {
//            FLM.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
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
    }

    private void setupSensors(){
        MagnetSensorUpper = hardwareMap.tryGet(TouchSensor.class, "MagnetSensorUpper");
        MagnetSensorLower = hardwareMap.tryGet(TouchSensor.class, "MagnetSensorLower");
        allianceSensor = hardwareMap.get(ColorRangeSensor.class, "allianceSensor");
        ledRed = hardwareMap.get(LED.class, "red");
        ledGreen = hardwareMap.get(LED.class, "green");
    }

    private void setupConsmetics(){
        PrismLEDDriver = hardwareMap.tryGet(GoBildaPrismDriver.class,"lights");
        // Set strip length based on LED density property
        PrismLEDDriver.setStripLength((int)Math.floor(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_COUNT_PRISM_DRIVER)));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Core System Initialization
//        telemetryS = new PanelsSystem(telemetry);
//        propertiesSystem = new PropertiesSystem(telemetryS);
//        logging = new LoggingSystem(telemetryS);
//        communication = new AutoTeleOpCommunicationSystem();

//        // Load persisted tuning data and communications from Autonomous
//        try {
//            propertiesSystem.loadProperties();
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
//        try {
//
//            communication.loadCommunication(telemetryS);
//        } catch(IOException e) {
//            throw new RuntimeException(e);
//        }

        // Retrieve Ending Pose from Autonomous to maintain localization
//        startingPose = new Pose(
//                Double.parseDouble(communication.COMMUNICATABLES.get(AutoTeleOpCommunicationSystem.Communicates.END_POSE_X)),
//                Double.parseDouble(communication.COMMUNICATABLES.get(AutoTeleOpCommunicationSystem.Communicates.END_POSE_Y)), // Note: Check if Y should be here
//                Double.parseDouble(communication.COMMUNICATABLES.get(AutoTeleOpCommunicationSystem.Communicates.END_POSE_RADIANS))
//        );
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose);
//        follower.update();

        // Path Definition: Dynamic line to field center (0,0)


        setupMotors();
//        setupConsmetics();
//        setupServos();
////        setupSensors();
//        if(allianceSensor.red() > 400 || allianceSensor.blue() > 400) {
//            ledRed.enable((allianceSensor.red() > allianceSensor.blue()));
//            ledGreen.enable((allianceSensor.red() < allianceSensor.blue()));
//        } else {
//            ledRed.off();
//
//            ledGreen.off();
//        }
//        pathChain = () -> follower.pathBuilder()
//                .addPath(new BezierLine(follower::getPose, new Pose(0, 0)))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(ledRed.isLightOn() ? -(180+45): 180+45), 0.8))
//                .build();
//        // Initial LED Boot Sequence
//        if((boolean) PropertiesSystem.PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_ENABLED_PRISM_DRIVER)) == true){
//            PrismLEDDriver.enableDefaultBootArtboard(true);
//            PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
//        } else {
//            PrismLEDDriver.enableDefaultBootArtboard(false);
//            PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
//        }

        waitForStart();
//        follower.startTeleopDrive();

        while(opModeIsActive()) {
//            @Override
//            public void stop() {
//                communication.COMMUNICATABLES.put(AutoTeleOpCommunicationSystem.Communicates.END_POSE_X, follower.getPose().getX()+ "");
//                communication.COMMUNICATABLES.put(AutoTeleOpCommunicationSystem.Communicates.END_POSE_Y, follower.getPose().getY() + ""); // Note: Check if Y should be here
//                communication.COMMUNICATABLES.put(AutoTeleOpCommunicationSystem.Communicates.END_POSE_RADIANS, follower.getPose().getHeading() + "");
//                // Resource cleanup (if applicable)
////            }
//            try {
//                communication.saveCommunications();
//            } catch (IOException e) {
//                throw new RuntimeException(e);
//            }
//            flywheel.loop(gamepad1,gamepad2,telemetryS);
//            // Handle entering/exiting the on-the-fly Adjustment Menu
//            if (gamepad1.share) {
//                propertiesSystem.adjustmentMode = !propertiesSystem.adjustmentMode;
//                try {
//                    propertiesSystem.saveProperties();
//                } catch (IOException e) {
//                    throw new RuntimeException(e);
//                }
//                while (gamepad1.share); // Debounce
//            }
//
//            // Adjustment Menu Logic: Freezes main OpMode to allow tuning
//            if (propertiesSystem.adjustmentMode) {
//                propertiesSystem.startMenu(gamepad1);
//                JustLeftAdjustmentMenu = true;
//                continue;
//            }
//
//            // Re-sync hardware after adjusting properties (e.g., changing motor direction)
//            if (JustLeftAdjustmentMenu) {
//                setupMotors();
//                setupConsmetics();
//                setupServos();
//                setupSensors();

//                if ((boolean) PropertiesSystem.PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_ENABLED_PRISM_DRIVER)) == true) {
//                    PrismLEDDriver.enableDefaultBootArtboard(true);
//                    PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
//                } else {
//                    PrismLEDDriver.enableDefaultBootArtboard(false);
//                    PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
//                }
//                JustLeftAdjustmentMenu = false;
////            }
//
//            // Alliance Sensor Logic: Simple proximity color detection
//            telemetryS.addData("ColorBlue", allianceSensor.blue());
//            telemetryS.addData("ColorRed", allianceSensor.red());
//            telemetryS.addData("X", startingPose.getX());
//            telemetryS.addData("Y", startingPose.getY());
//            telemetryS.addData("Heading", startingPose.getHeading());
//            if(allianceSensor.red() > 400 || allianceSensor.blue() > 400) {
//                ledRed.enable((allianceSensor.red() > allianceSensor.blue()));
//                ledGreen.enable((allianceSensor.red() < allianceSensor.blue()));
//            } else {
//                ledRed.off();
//
//                ledGreen.off();
//            }
//
//            // DRIVE SELECTION: Manual Mecanum vs PedroPathing Assisted Drive
//            if ((boolean) PropertiesSystem.PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.AUTO_DRIVE_DISABLED)) == true) {
                drive(); // Manual Logic
//            } else {
//                telemetryS.addData("Touchpad", gamepad1.touchpad_finger_1_x + ", " + gamepad1.touchpad_finger_1_y);
//                follower.update();
//
//                if (!automatedDrive) {
//                    //Make the last parameter false for field-centric
//                    //In case the drivers want to use a "slowMode" you can scale the vectors
//
//                    //This is the normal version to use in the TeleOp
//                    if (!slowMode) follower.setTeleOpDrive(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x,
//                            -gamepad1.right_stick_x,
//                            true // Robot Centric
//                    );
//
//                        //This is how it looks with slowMode on
//                    else follower.setTeleOpDrive(
//                            -gamepad1.left_stick_y * slowModeMultiplier,
//                            -gamepad1.left_stick_x * slowModeMultiplier,
//                            -gamepad1.right_stick_x * slowModeMultiplier,
//                            true // Robot Centric
//                    );
//                }
//
//                //Automated PathFollowing
//                if (gamepad1.a && !automatedDrive) {
//                    follower.followPath(pathChain.get(), true);
//                    automatedDrive = true;
//                }
//
//                //Stop automated following if the follower is done
//                if (automatedDrive && !gamepad1.a/* || !follower.isBusy())*/) {
//                    follower.startTeleopDrive();
//                    automatedDrive = false;
//                }
//
//                //Slow Mode
//                if (gamepad1.rightBumperWasPressed()) {
//                    slowMode = !slowMode;
//                }
//
//                //Optional way to change slow mode strength
//                if (gamepad1.x) {
//                    slowModeMultiplier += 0.25;
//                }
//
//                //Optional way to change slow mode strength
//                if (gamepad2.y) {
//                    slowModeMultiplier -= 0.25;
//                }
//
//                telemetryS.debug("position", follower.getPose());
//                telemetryS.debug("velocity", follower.getVelocity());
//                telemetryS.debug("automatedDrive", automatedDrive);
//                Drawing.drawDebug(follower);
//            }
//
//
//            // Update Subsystems
//            lights();
//            rubberBand();
//            telemetryS.update();
        }
    }

    private int upPosition = 0;
    private int oldPosition = 0;
    private int selected_color = 0;

    // LED Color Map for PWM strip
    private String[] ColorsString = {"Black", "Red", "Orange", "Yellow", "Sage", "Green", "Azure", "Blue", "Indigo", "Violet"};
    private double[] Colors = {0, 0.227, 0.333, 0.388, 0.444, 0.5, 0.555, 0.611, 0.666, 0.722};

    /**
     * Controls the GoBilda Prism Driver and PWM strip based on the robot's current flags.
     */
    private void lights() {
        // Cycle colors manually on Gamepad 2
        if(gamepad2.guide){
            GobildaPWMLed.setPosition(Colors[selected_color]);
            selected_color++;
            if(Colors.length < selected_color) selected_color = 0;
            PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
        }

        // Automated Feedback Animations
        if((boolean) PropertiesSystem.PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_ENABLED_PRISM_DRIVER)) == true) {
            PrismLEDDriver.setStripLength((int)Math.floor(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_COUNT_PRISM_DRIVER)));

            if (isDriving) {
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
                GobildaPWMLed.setPosition(Colors[4]); // Sage
            } else if (isIntaking) {
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
                GobildaPWMLed.setPosition(Colors[5]); // Green
            } else if(isTargeting) {
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_6);
                GobildaPWMLed.setPosition(Colors[6]); // Azure
            } else if(isShooting) {
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_5);

                // Dramatic Flare Animation (Frame-based switching)
                counts++;
                switch (counts % 20) {
                    case 1: case 2: case 3: case 4: case 5:
                        GobildaPWMLed.setPosition(Colors[3]); break; // Yellow
                    case 6: case 7: case 8: case 9: case 10: case 11: case 12:
                        GobildaPWMLed.setPosition(Colors[2]); break; // Orange
                    case 13: case 15: case 16: case 18: case 19: case 20:
                        GobildaPWMLed.setPosition(Colors[1]); break; // Red
                    case 14: case 17:
                        GobildaPWMLed.setPosition(Colors[0]); break; // Black (Flicker)
                }
            }
        } else {
            PrismLEDDriver.setStripLength(0);
        }
    }

    /**
     * Manages the wrist rotation and intake mechanism.
     * Uses magnetic sensors to prevent mechanical strain at limit points.
     */
    private void rubberBand() {
        if(gamepad2.x){
            flywheel.toggle();
            while(gamepad2.x){}
        }
        telemetryS.addData("UpperSensorValue", MagnetSensorUpper.getValue());
        telemetryS.addData("LowerSensorValue", MagnetSensorLower.getValue());

        // Intake Power Control
//        if(gamepad1.square){
//            LRBS.setPower(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LEFT_RUBBER_BAND_POWER));
//            RRBS.setPower(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.RIGHT_RUBBER_BAND_POWER));
//        } else {
//            LRBS.setPower(0);
//            RRBS.setPower(0);
//        }
//
//        // Direct Direction Toggle
//        if(gamepad1.dpad_down){
//            LRBS.setDirection(LRBS.getDirection() == CRServo.Direction.FORWARD ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
//            RRBS.setDirection(RRBS.getDirection() == CRServo.Direction.FORWARD ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
//        }

        // Increment/Decrement Wrist Goal Position
        if(gamepad1.a){ upPosition++; }
        if(gamepad1.b){ upPosition--; }
        if(gamepad2.a){
            MIS.setPower(-100);

        }else if(gamepad2.x) {
            MIS.setPower(-100);
        }else{
            MIS.setPower(0);
        }
//
//        LWS.setPosition(-gamepad2.left_stick_y/7.5+0.5);
        if(gamepad2.right_bumper && gamepad2.left_bumper){

            RWS.setPosition(0.7);
            LWS.setPosition(-0.7);
        }else if(gamepad2.right_bumper) {
            RWS.setPosition(0.5);
            LWS.setPosition(-0.5);

        }else if(gamepad2.left_bumper) {
            RWS.setPosition(0.85);
            LWS.setPosition(-0.85);
        }else{

            RWS.setPosition(0.9);
            LWS.setPosition(-0.9);
        }
                if (gamepad2.triangle) {
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
        float walk = gamepad1.left_stick_y;     // Forward/Backward
        float strafe = gamepad1.left_stick_x;   // Left/Right
        float pivot = -gamepad1.right_stick_x;  // Rotation
        float dampen = gamepad1.right_trigger;  // Variable speed reduction

        // Kinematics Math
        float fl = walk + strafe + pivot;
        float fr = walk - strafe - pivot;
        float bl = walk + strafe + pivot;
        float br = walk - strafe - pivot;

        // Apply Dampening Scalar (Formula: Power / (DampenWeight + Offset))
        fl /= (dampen * 1.8f) + 0.4f;
        fr /= (dampen * 1.8f) + 0.4f;
        bl /= (dampen * 1.8f) + 0.4f;
        br /= (dampen * 1.8f) + 0.4f;

        // Final Power Application if enabled in config
//        if((boolean) PropertiesSystem.PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CONTROLLER_DRIVE_ENABLED)) == true) {
            FLM.setPower(fl);
            FRM.setPower(fr);
            BLM.setPower(bl);
            BRM.setPower(br);
//        }

        // Update driving flag for LEDs
        isDriving = fl!=0 || fr!=0 || bl!=0 || br!=0;
    }
//    @Override
//    public void stop() {
//        communication.COMMUNICATABLES.put(AutoTeleOpCommunicationSystem.Communicates.END_POSE_X, follower.getPose().getX()+ "");
//        communication.COMMUNICATABLES.put(AutoTeleOpCommunicationSystem.Communicates.END_POSE_Y, follower.getPose().getY() + ""); // Note: Check if Y should be here
//        communication.COMMUNICATABLES.put(AutoTeleOpCommunicationSystem.Communicates.END_POSE_RADIANS, follower.getPose().getHeading() + "");
//        // Resource cleanup (if applicable)
//    }
}