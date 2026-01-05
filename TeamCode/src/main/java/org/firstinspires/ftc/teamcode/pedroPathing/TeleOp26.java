package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.System.PropertiesSystem.PropertyCleaner;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.System.AutoTeleOpCommunicationSystem;
import org.firstinspires.ftc.teamcode.System.LoggingSystem;
import org.firstinspires.ftc.teamcode.System.PropertiesSystem;
import org.firstinspires.ftc.teamcode.System.TelemetrySystem;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.function.Supplier;

@Configurable
@TeleOp(name="TeleOp26", group="TELEOP")
public class TeleOp26 extends LinearOpMode{

    private DcMotor FLM;//FrontLeftMotor
    private DcMotor FRM;//FrontRightMotor
    private DcMotor BLM;//BackLeftMotor
    private DcMotor BRM;//BackRightMotor
    private CRServo LWS;//LeftWristServo;
    private CRServo RWS;//RightWristServo;
    private TouchSensor MagnetSensorUpper;
    private TouchSensor MagnetSensorLower;
    private CRServo LRBS;//LeftRubberBandServo;
    private CRServo RRBS;//RightRubberBandServo;
    private CRServo MIS;//MiddleIntakeServo;
    private DcMotor LLM; //LeftLauncherMotor
    private DcMotor RLM; //RightLauncherMotor

    private DcMotor MLM; //MiddleLauncherMotor
    private GoBildaPrismDriver PrismLEDDriver;
    private ColorRangeSensor allianceSensor;

    //For the Little led thingy you have to define green and red as two different dvices
    private LED ledRed;
    private LED ledGreen;

    private Servo GobildaPWMLed;



    private int counts = 0;

/*
 * These are for if we are doing the said action on the robot they are primarily used to control LED States on the robot
 */
    public boolean isDriving = false;
    public boolean isIntaking = false;
    public boolean isShooting = false;
    public boolean midFieldPickup = false;
    public boolean isTargeting = false;
    public boolean styleMode = false;
    public boolean isWristing = false;
    public boolean[] ballsCollected = {false,false,false};


    private boolean JustLeftAdjustmentMenu = false;
    public TelemetrySystem telemetryS;// = new telemetrySSystem(this.telemetryS);
    public PropertiesSystem propertiesSystem;// = new PropertiesSystem(telemetryS);
    public LoggingSystem logging;// = new LoggingSystem(telemetryS);
    public AutoTeleOpCommunicationSystem communication;

    //Pedro
    private Follower follower;
    private  Pose startingPose = new Pose(64, 12, Math.toRadians(180)); // Start Pose of our robot.
    //Edge of feild
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;



    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;


    private void setupMotors(){
        FLM = hardwareMap.tryGet(DcMotor.class, "FL");
        FRM = hardwareMap.tryGet(DcMotor.class, "FR");
        BLM = hardwareMap.tryGet(DcMotor.class, "BL");
        BRM = hardwareMap.tryGet(DcMotor.class, "BR");
        LLM = hardwareMap.tryGet(DcMotor.class, "LLM");
        RLM = hardwareMap.tryGet(DcMotor.class, "RLM");
        MLM = hardwareMap.tryGet(DcMotor.class, "MLM");
        if(FLM == null){ telemetryS.speak("Variable FLM is Null depicting that FLM doesnt exist in the config Please update Config");}
        if(FRM == null){ telemetryS.speak("Variable FRM is Null depicting that FRM doesnt exist in the config Please update Config");}
        if(BLM == null){ telemetryS.speak("Variable BLM is Null depicting that BLM doesnt exist in the config Please update Config");}
        if(BRM == null){ telemetryS.speak("Variable BRM is Null depicting that BRM doesnt exist in the config Please update Config");}
        if(LLM == null){ telemetryS.speak("Variable LLM is Null depicting that LLM doesnt exist in the config Please update Config");}
        if(RLM == null){ telemetryS.speak("Variable RLM is Null depicting that RLM doesnt exist in the config Please update Config");}
        if(MLM == null){ telemetryS.speak("Variable MLM is Null depicting that MLM doesnt exist in the config Please update Config");}
        telemetryS.update();
        if(FLM == null && FRM == null && BLM == null && BRM == null && LLM == null && RLM == null && MLM == null){
            telemetryS.speak("Ternimating OP Mode!");
            telemetryS.update();
//            terminateOpModeNow();
        }
//        if(propertiesSystem.PROPERTIES())
        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.DRIVE_MOTOR_ZERO_POWER_BHV)) == "BRAKE") {
            FLM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            FRM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            BLM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            BRM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }else{
            FLM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            FRM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            BLM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            BRM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        }
        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.BL_MOTOR_DIRECTION)) == "FORWARD") {
            BLM.setDirection(DcMotorSimple.Direction.FORWARD);
        }else{
            BLM.setDirection(DcMotorSimple.Direction.REVERSE);

        }
        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.BR_MOTOR_DIRECTION)) == "FORWARD") {
            BRM.setDirection(DcMotorSimple.Direction.FORWARD);
        }else{
            BRM.setDirection(DcMotorSimple.Direction.REVERSE);

        }
        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.FR_MOTOR_DIRECTION)) == "FORWARD") {
            FRM.setDirection(DcMotorSimple.Direction.FORWARD);
        }else{
            FRM.setDirection(DcMotorSimple.Direction.REVERSE);

        }
        if(PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.FL_MOTOR_DIRECTION)) == "FORWARD") {
            FLM.setDirection(DcMotorSimple.Direction.FORWARD);
        }else{
            FLM.setDirection(DcMotorSimple.Direction.REVERSE);

        }
//        LLM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
//        RLM.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
//        MLM.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    }
    private void setupServos(){
        LWS = hardwareMap.tryGet(CRServo.class, "LeftWristServo");
        RWS = hardwareMap.tryGet(CRServo.class, "RightWristServo");
        LRBS = hardwareMap.tryGet(CRServo.class, "LeftRubberBandServo");
        RRBS = hardwareMap.tryGet(CRServo.class, "RightRubberBandServo");
        RRBS.setDirection(CRServo.Direction.REVERSE);
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

        PrismLEDDriver.setStripLength((int)Math.floor(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_COUNT_PRISM_DRIVER)));
    }
    @Override
    public void runOpMode() throws InterruptedException{
        File f = new File(PropertiesSystem.DATA_DIR);
        if (!f.exists()) {
            f.mkdirs();
        }
        BufferedWriter wr = null;
        try {
            wr = new BufferedWriter(new FileWriter(AutoTeleOpCommunicationSystem.COMS_SAVE));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            wr.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        telemetryS = new TelemetrySystem(telemetry);
        propertiesSystem = new PropertiesSystem(telemetryS);
        logging = new LoggingSystem(telemetryS);
        communication = new AutoTeleOpCommunicationSystem();
        try {
            propertiesSystem.loadProperties();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try{
            communication.loadCommunication();
        }catch(IOException e){
            throw  new RuntimeException(e);
        }
        startingPose = new Pose(
                Double.parseDouble(communication.COMMUNICATABLES.get(AutoTeleOpCommunicationSystem.Communicates.END_POSE_X)),
                Double.parseDouble(communication.COMMUNICATABLES.get(AutoTeleOpCommunicationSystem.Communicates.END_POSE_X)),
                Double.parseDouble(communication.COMMUNICATABLES.get(AutoTeleOpCommunicationSystem.Communicates.END_POSE_RADIANS))
        );
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(/*new Path(*/new BezierLine(follower::getPose, new Pose(0, 0)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 1))
                .build();

        setupMotors();
        setupConsmetics();
        setupServos();
        setupSensors();
//
        if((boolean) PropertiesSystem.PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_ENABLED_PRISM_DRIVER)) == true){
            PrismLEDDriver.enableDefaultBootArtboard(true);
            PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        }else{
            PrismLEDDriver.enableDefaultBootArtboard(false);
            PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
        }
        waitForStart();
        follower.startTeleopDrive();

        while(opModeIsActive()) {
            if (gamepad1.share) {
                propertiesSystem.adjustmentMode = !propertiesSystem.adjustmentMode;
                try {
                    propertiesSystem.saveProperties();
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
                while (gamepad1.share) ;//so holding the button doesnt keep doing this
            }

//            if (gamepad2.share) {
//                logviewMode = !logviewMode;
//                while (gamepad2.share) ;
//            }

            if (propertiesSystem.adjustmentMode) {
                propertiesSystem.startMenu(gamepad1);
                JustLeftAdjustmentMenu = true;
                continue;
            }
            if (JustLeftAdjustmentMenu) {
                setupMotors();
                setupConsmetics();
                setupServos();
                setupSensors();

                if ((boolean) PropertiesSystem.PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_ENABLED_PRISM_DRIVER)) == true) {
                    PrismLEDDriver.enableDefaultBootArtboard(true);
                    PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
                } else {
                    PrismLEDDriver.enableDefaultBootArtboard(false);
                    PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
                }
                JustLeftAdjustmentMenu = false;
            }
            telemetryS.addData("ColorBlue", allianceSensor.blue());
            telemetryS.addData("ColorRed", allianceSensor.red());
            if(allianceSensor.red() > 400 || allianceSensor.blue() > 400) {
                ledRed.enable((allianceSensor.red() > allianceSensor.blue()));
                ledGreen.enable((allianceSensor.red() < allianceSensor.blue()));
            }else{
                ledRed.off();
                ledGreen.off();
            }
            if ((boolean) PropertiesSystem.PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.AUTO_DRIVE_DISABLED)) == true) {

                drive();
            } else {
                telemetryS.addData("Touchpad", gamepad1.touchpad_finger_1_x + ", " + gamepad1.touchpad_finger_1_y);
                //Call this once per loop
                follower.update();
//                telemetryS.update();

                if (!automatedDrive) {
                    //Make the last parameter false for field-centric
                    //In case the drivers want to use a "slowMode" you can scale the vectors

                    //This is the normal version to use in the TeleOp
                    if (!slowMode) follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x,
                            true // Robot Centric
                    );

                        //This is how it looks with slowMode on
                    else follower.setTeleOpDrive(
                            -gamepad1.left_stick_y * slowModeMultiplier,
                            -gamepad1.left_stick_x * slowModeMultiplier,
                            -gamepad1.right_stick_x * slowModeMultiplier,
                            true // Robot Centric
                    );
                }
                //Automated PathFollowing
                if (gamepad1.a) {
                    follower.followPath(pathChain.get(), true);
                    automatedDrive = true;
                }

                //Stop automated following if the follower is done
                if (automatedDrive){// && (!gamepad1.atRest() || gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.guide || gamepad1.right_bumper || gamepad1.b || gamepad1.back || gamepad1.circle || gamepad1.cross || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.left_bumper || gamepad1.left_stick_button || gamepad1.options || gamepad1.ps || gamepad1.right_stick_button || gamepad1.share || gamepad1.square || gamepad1.start || gamepad1.triangle)) {
                    follower.startTeleopDrive();
                    automatedDrive = false;
                }

                //Slow Mode
                if (gamepad1.right_bumper) {
                    slowMode = !slowMode;
                }

                //Optional way to change slow mode strength
                if (gamepad1.x) {
                    slowModeMultiplier += 0.25;
                }

                //Optional way to change slow mode strength
                if (gamepad1.y) {
                    slowModeMultiplier -= 0.25;
                }

                telemetryS.debug("position", follower.getPose());
                telemetryS.debug("velocity", follower.getVelocity());
                telemetryS.debug("automatedDrive", automatedDrive);
//                Drawing.drawDebug(follower);
                Drawing.drawForTeleop(follower, logging);//Custom in draw method

            }

            lights();
            rubberBand();
            telemetryS.update();
            }
    }
    private int upPosition = 0;
    private int oldPosition = 0;
    private int selected_color = 0;
    private String[] ColorsString = {"Black", "Red", "Orange", "Yellow", "Sage", "Green", "Azure", "Blue", "Indigo", "Violet"};
    private double[] Colors = {0, 0.227, 0.333, 0.388, 0.444, 0.5,0.555, 0.611, 0.666, 0.722};
    private void lights(){
        if(gamepad2.guide){
            GobildaPWMLed.setPosition(Colors[selected_color]);
            selected_color++;
            if(Colors.length < selected_color) selected_color = 0;
            PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
        }
        if((boolean) PropertiesSystem.PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_ENABLED_PRISM_DRIVER)) == true) {
            PrismLEDDriver.setStripLength((int)Math.floor(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LED_COUNT_PRISM_DRIVER)));
            if (isDriving) {
                //led style for when driving
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
                GobildaPWMLed.setPosition(Colors[4]);

//                PrismLEDDriver.
            } else if (isIntaking) {
                //ok
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
                GobildaPWMLed.setPosition(Colors[5]);

            }else if(isTargeting){
                //check if we locked on and ready to fire!
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_6);
                GobildaPWMLed.setPosition(Colors[6]);

            }else if(isShooting){
                PrismLEDDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_5);
                //enter drymatic flare!
                counts++;
                switch (counts%20){
                    case 1: GobildaPWMLed.setPosition(Colors[3]); break;
                    case 2: GobildaPWMLed.setPosition(Colors[3]); break;
                    case 3: GobildaPWMLed.setPosition(Colors[3]); break;
                    case 4: GobildaPWMLed.setPosition(Colors[3]); break;
                    case 5: GobildaPWMLed.setPosition(Colors[3]); break;
                    case 6: GobildaPWMLed.setPosition(Colors[2]); break;
                    case 7: GobildaPWMLed.setPosition(Colors[2]); break;
                    case 8: GobildaPWMLed.setPosition(Colors[2]); break;
                    case 9: GobildaPWMLed.setPosition(Colors[2]); break;
                    case 10: GobildaPWMLed.setPosition(Colors[2]); break;
                    case 11: GobildaPWMLed.setPosition(Colors[2]); break;
                    case 12: GobildaPWMLed.setPosition(Colors[2]); break;
                    case 13: GobildaPWMLed.setPosition(Colors[1]); break;
                    case 14: GobildaPWMLed.setPosition(Colors[0]); break;
                    case 15: GobildaPWMLed.setPosition(Colors[1]); break;
                    case 16: GobildaPWMLed.setPosition(Colors[1]); break;
                    case 17: GobildaPWMLed.setPosition(Colors[0]); break;
                    case 18: GobildaPWMLed.setPosition(Colors[1]); break;
                    case 19: GobildaPWMLed.setPosition(Colors[1]); break;
                    case 20: GobildaPWMLed.setPosition(Colors[1]); break;
                }



            }

        }else{
            PrismLEDDriver.setStripLength(0);
        }
    }
    private void rubberBand() {
        telemetryS.addData("UpperSensorValue", MagnetSensorUpper.getValue());
        telemetryS.addData("LowerSensorValue", MagnetSensorLower.getValue());
        telemetryS.addData("UpperSensorPressed", MagnetSensorUpper.isPressed());
        telemetryS.addData("LowerSensorPressed", MagnetSensorLower.isPressed());
        telemetryS.addData("Old", upPosition);
        telemetryS.addData("Up ", oldPosition);
        if(gamepad1.square){
            LRBS.setPower(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LEFT_RUBBER_BAND_POWER));
            RRBS.setPower(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.RIGHT_RUBBER_BAND_POWER));
        }else{
            LRBS.setPower(0);
            RRBS.setPower(0);

        }
        if(gamepad1.dpad_down){
            LRBS.setDirection(LRBS.getDirection() == CRServo.Direction.FORWARD ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
            RRBS.setDirection(RRBS.getDirection() == CRServo.Direction.FORWARD ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        }
        //inits and sets limits
       if(gamepad1.a){
           upPosition++;
       }
       if(gamepad1.b){
           upPosition--;
       }
       if(oldPosition != upPosition) {
           if (oldPosition > upPosition) {
               LWS.setDirection(CRServo.Direction.FORWARD);
               RWS.setDirection(CRServo.Direction.REVERSE);
           } else {
               LWS.setDirection(CRServo.Direction.REVERSE);
               RWS.setDirection(CRServo.Direction.FORWARD);
           }
           if (MagnetSensorLower.isPressed() || MagnetSensorUpper.isPressed()) {
               LWS.setPower(0);
               RWS.setPower(0);
           } else {
               LWS.setPower(0.5);
               RWS.setPower(0.5);
               sleep(250);
               LWS.setPower(0);
               RWS.setPower(0);
               oldPosition = upPosition;
           }
       }
        oldPosition = upPosition;
    }
    private void drive(){

        float walk = gamepad1.left_stick_y;
        float strafe = gamepad1.left_stick_x;
        float pivot = -gamepad1.right_stick_x;
        float dampen = gamepad1.right_trigger;

        float fl = walk + strafe;
        float fr = walk - strafe;
        float bl = walk + strafe;
        float br = walk - strafe;

        fl += pivot;
        fr -= pivot;
        bl += pivot;
        br -= pivot;

        fl /= (dampen * 1.8f) + 0.4f;
        fr /= (dampen * 1.8f) + 0.4f;
        bl /= (dampen * 1.8f) + 0.4f;
        br /= (dampen * 1.8f) + 0.4f;

        if((boolean) PropertiesSystem.PropertyCleaner(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.CONTROLLER_DRIVE_ENABLED)) == true) {

            FLM.setPower(fl);
            FRM.setPower(fr);
            BLM.setPower(bl);
            BRM.setPower(br);
        }
        isDriving = fl!=0 || fr!=0 || bl!=0 || br!=0;
//
    }

}