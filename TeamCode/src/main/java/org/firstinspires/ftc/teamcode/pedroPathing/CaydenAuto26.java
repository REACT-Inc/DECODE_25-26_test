package org.firstinspires.ftc.teamcode.pedroPathing;



import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;


//Astrik imports everything fro that folder
import org.firstinspires.ftc.teamcode.System.*;

import java.io.IOException;

/**
 * <h1>Caydens Auto -  PedroPathing </h1>
 * <p>
 * This is the Final Auto we used for this year
 * This auto using pedropathing moves sensing the alliance color
 * </p>
 * * <b>Hardware Requirements:</b>
 * <ul>
 * <li>PedroPathing compatible drive chassis</li>
 * <li>Flywheel shooter mechanism</li>
 * </ul>
 *
 * * <b>Notes</b>
 * <ul>
 *  <li>This class is Configurable using panels!</li>
 *  <li>Uses the Custom Pose System (PositionalPose)(Formally Pose6D)</li>
 *  <li>Uses AutoTeleopCommunications</li>
 *  <li>Uses Properties System V2.3</li>
 *  <li>Uses Drawing (Alpha Version)</li>
 * </ul>
 *
 * @author ACRobotics
 * @version 2.1
 *
 * LIST:
 * turns towards goal and shoots,
 * go to pickup point by what the april tag is
 * go back to shooting position
 *
 * TODO:
 * Still need to convert to new pose system
 * Fully comment code
 */


@Configurable
@Autonomous(name = "FrontAuto2026", group = "1")
public class CaydenAuto26 extends OpMode {

    //TODO Whys this static?
    private static Servo RWS, LWS;  /* RightWristServo, LeftWristServo */
    private DistanceSensor laser;  /* GobildaLaserDistanceSensor */



    //Init Instances of other classes
    private final FlywheelLogic shooter = new FlywheelLogic();  /* State Machine for The flywheel */
    public LimelightSystem limelight;
    public LoggingSystem logging = new LoggingSystem();  /* Logging system */

    //TODO Make this away from teleop and a whole system!
    public TeleOp26.Pose6D pose6D;


    public PanelsSystem telemetryS;// = new telemetrySSystem(this.telemetryS);
    public PropertiesSystem propertiesSystem;  /* Properties */
    public AutoTeleOpCommunicationSystem communication;  /* Communication */


    Object[] currentResults;

    private Follower follower;


    /** Tracks whether the shooting command has been issued for the current state */
    private boolean shotsTriggered = false;

    /** pathTimer: Tracks time spent in a state; opmodeTimer: Tracks total match time */
    private Timer pathTimer, opmodeTimer;

    /** Current execution step of the autonomous sequence */
    private int pathState;

    // ---------------------------------------------------------
    // FIELD COORDINATES (POSES)
    // ---------------------------------------------------------

    /** @pose Start: Robot begins against the perimeter wall facing the field center. */
    public  Pose startPose = new Pose(70, 8, Math.toRadians(180));

    /** @pose Score: Optimized position to align the shooter with the high goal. */
    public  Pose scorePose = new Pose(-7, 12, Math.toRadians(/*49.5 + 90/*/139.5));




    /** @pose PrePickup1: goes in front of pickup */
    public  Pose prePickup1Pose = new Pose(-4.45,20, Math.toRadians(90));

    /** @pose Pickup1: Targets the first spike mark artifact. */
    public  Pose pickup1Pose = new Pose(-4.5, 53, Math.toRadians(90));

    /** @pose PrePickup2: goes in front of pickup */
    public  Pose prePickup2Pose = new Pose(20.1,20.1, Math.toRadians(90));

    /** @pose Pickup2: Targets the second spike mark artifact. */
    public  Pose pickup2Pose = new Pose(20, 53.1, Math.toRadians(90));

    /** @pose PrePickup3: goes in front of pickup */
    public  Pose prePickup3Pose = new Pose(44.4,20.2, Math.toRadians(90));

    /** @pose Pickup3: Targets the third spike mark artifact. */
    public  Pose pickup3Pose = new Pose(44.5, 53.2, Math.toRadians(90));

    // ---------------------------------------------------------
    // PATH CONTAINERS
    // ---------------------------------------------------------

    private Path scorePreload;
    private PathChain prePickup1, grabPickup1, scorePickup1, prePickup2, grabPickup2, scorePickup2, prePickup3, grabPickup3, scorePickup3, returnToBase;


    /**
     * Builds the paths (Obviosly But) Lots of codes but draws the lines aswell to panels when init
     */
    public void buildPaths() {
        // PRELOAD: Initial drive to goal

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        //TODO make this whole drawing all the lines automatic system it would be cool
        Drawing.drawLine(startPose.getX(),startPose.getY(), scorePose.getX(), scorePose.getY(), Drawing.robotExpectedPath);
        // PRESEQUENCE 1: Setup for First Artifact Cycle
        prePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup1Pose.getHeading())
                .build();
        Drawing.drawLine(scorePose.getX(),scorePose.getY(), prePickup1Pose.getX(), prePickup1Pose.getY(), Drawing.robotExpectedPath);

        // SEQUENCE 1: First Artifact Cycle
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(prePickup1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        Drawing.drawLine(prePickup1Pose.getX(),prePickup1Pose.getY(), pickup1Pose.getX(), pickup1Pose.getY(), Drawing.robotExpectedPath);

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
        Drawing.drawLine(pickup1Pose.getX(),pickup1Pose.getY(), scorePose.getX(), scorePose.getY(), Drawing.robotExpectedPath);

        // PRESEQUENCE 2: Setup for Second Artifact Cycle
        prePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup2Pose.getHeading())
                .build();
        Drawing.drawLine(scorePose.getX(),scorePose.getY(), prePickup2Pose.getX(), prePickup2Pose.getY(), Drawing.robotExpectedPath);

        // SEQUENCE 2: Second Artifact Cycle
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(prePickup2Pose.getHeading(), pickup2Pose.getHeading())
                .build();
        Drawing.drawLine(prePickup2Pose.getX(),prePickup2Pose.getY(), pickup2Pose.getX(), pickup2Pose.getY(), Drawing.robotExpectedPath);

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        Drawing.drawLine(pickup2Pose.getX(),pickup2Pose.getY(), scorePose.getX(), scorePose.getY(), Drawing.robotExpectedPath);

        // PRESEQUENCE 3: Setup for Third Artifact Cycle
        prePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup3Pose.getHeading())
                .build();
        Drawing.drawLine(scorePose.getX(),scorePose.getY(), prePickup3Pose.getX(), prePickup3Pose.getY(), Drawing.robotExpectedPath);

        // SEQUENCE 3: Third Artifact Cycle
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup3Pose, pickup3Pose))
                .setLinearHeadingInterpolation(prePickup3Pose.getHeading(), pickup3Pose.getHeading())
                .build();
        Drawing.drawLine(prePickup3Pose.getX(),prePickup3Pose.getY(), pickup3Pose.getX(), pickup3Pose.getY(), Drawing.robotExpectedPath);

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
        Drawing.drawLine(pickup3Pose.getX(),pickup3Pose.getY(), scorePose.getX(), scorePose.getY(), Drawing.robotExpectedPath);

        // RETURN: Move to safety/parking zone using a Bezier Curve
        returnToBase = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, startPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), startPose.getHeading())
                .build();
        Drawing.drawLine(scorePose.getX(),scorePose.getY(), startPose.getX(), startPose.getY(), Drawing.robotExpectedPath);
Drawing.sendPacket();
    }

    /**
     * <b>Handles updateing the Path</b>
     * <p>
     * This part gets messy but handles all the telling
     * </p>
     */
    public void autonomousPathUpdate() throws InterruptedException {
        //Max power canot be higher the robot rolls otherwise
        follower.setMaxPower(0.65);
        Drawing.drawDebug(follower);
        switch (pathState) {
            case 0: // Phase: Drive to High Goal with Preload
                follower.followPath(scorePreload, true); // drives to score position and holds position
                setPathState(1);
                break;
//
            case 2: // Phase:  Firing with preload

                shooter.turnOnIntake();
                follower.setMaxPower(0.4);
                if (!follower.isBusy()) {

                    shooter.turnOnIntake();
                    follower.setMaxPower(0.4);
                        // Once shooter is clear, move to score location
                        follower.followPath(prePickup1, true);
                        setPathState(3);
                }
                break;

            case 3: // Phase: goes to pickup 1
                if (!follower.isBusy()) {
                    // INTAKE HERE
                    follower.setMaxPower(0.55);
                    shooter.turnOffIntake();
                    shooter.reverseIntake();
                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        shooter.turnOffIntake();

                        follower.followPath(scorePickup1, true); //once done intake
                        setPathState(4);
                    }
                }
                break;

            case 1: // Phase:  Firing with preload
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);
                        shotsTriggered =true;
                    } else if(shooter.getState() == FlywheelLogic.FlywheelState.SPIN_UP) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);

                    } else   if (!shooter.isBusy()) {
                        follower.setMaxPower(0.65);
                        shooter.turnOffIntake();
                        // Once shooter is clear, move to score location
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                    }
                }
                break;

            case 4: // Phase: Scoring 1 Complete -> Move to Pickup 2

                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);
                        shotsTriggered =true;
                    } else if(shooter.getState() == FlywheelLogic.FlywheelState.SPIN_UP) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);

                    } else  if (!shooter.isBusy()) {

                        shooter.turnOnIntake();
                        follower.setMaxPower(0.4);
                        // Once shooter is clear, move to score location
                        follower.followPath(prePickup2, true);
                        setPathState(5);
                    }
                }
                break;

            case 6: // Phase: Artifact 2 Intake and Firing Logic
                follower.setMaxPower(0.65);
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.65);
                    shooter.turnOffIntake();
                    shooter.reverseIntake();
                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        shooter.turnOffIntake();
                        follower.setMaxPower(0.65);
                        // INTAKE HERE
                        follower.followPath(scorePickup2, true); //once done intake
                        setPathState(7);
                    }
                }
                break;

            case 5: // Phase: Scoring 2 Complete -> Move to Pickup 3
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(11);
                    follower.setMaxPower(0.65);
                }
                break;

            case 9: // Phase: Artifact 3 Intake and Firing Logic
                if (!follower.isBusy()) {
                    // INTAKE HERE
                    follower.followPath(scorePickup2, true); //once done intake
                    setPathState(10);
                }
                break;

            case 7: // Phase: Return to Base Sequence

//                shooter.turnOnIntake();
                follower.setMaxPower(0.65);
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);
                        shotsTriggered =true;
                    } else if(shooter.getState() == FlywheelLogic.FlywheelState.SPIN_UP) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);

                    } else  if (!shooter.isBusy()) {
                        // Once shooter is clear, move to score location
                        follower.followPath(prePickup3, true);
                        setPathState(8);
                    }
                }
                break;

            case 8: // Phase: Final Parking Check
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(9);
                }
                break;

            case 10: // Phase: Return to Base Sequence
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);
                        shotsTriggered =true;
                    } else if(shooter.getState() == FlywheelLogic.FlywheelState.SPIN_UP) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);

                    } else   if (!shooter.isBusy()) {
                        follower.setMaxPower(0.5);
                        shooter.turnOffIntake();
                        // Once shooter is clear, move to score location
                        follower.followPath(returnToBase, true);
                        setPathState(-1);
                    }
                }
                break;


        }
    }

    /**
     * Manages state transitions, ensuring that timers and shot flags are
     * reset before the next state begins execution.
     * * @param pState The index of the next state in the FSM.
     */
    public void setPathState(int pState) {
        pathState = pState;

        pathTimer.resetTimer();
        shotsTriggered = false; // Prepare for the next shooting sequence
    }

    /**
     * Standard OpMode Initialization. Sets up hardware mappings and pre-builds paths
     * to prevent CPU lag during the start of the match.
     */
    @Override
    public void init() {
        telemetryS = new PanelsSystem(telemetry);
        propertiesSystem = new PropertiesSystem(telemetryS);
        logging.setTelemetry(telemetryS);
        communication = new AutoTeleOpCommunicationSystem();
        limelight = new LimelightSystem(telemetryS, hardwareMap);
        setupSensors();
if(alliance == TeleOp26.ALLIANCE.BLUE){
    /// WE MUST MIRROR ALL POSES
    startPose = startPose.mirror();
    scorePose = scorePose.mirror();

    prePickup1Pose = prePickup1Pose.mirror();
    prePickup2Pose = prePickup2Pose.mirror();
    prePickup3Pose = prePickup3Pose.mirror();

    pickup1Pose  = pickup1Pose.mirror();
    pickup2Pose = pickup2Pose.mirror();
    pickup3Pose = pickup3Pose.mirror();

}

        try {
            propertiesSystem.loadProperties();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try{
            communication.loadCommunication(telemetryS);
        }catch(IOException e){
            throw  new RuntimeException(e);
        }
//        startPose = new Pose(
//                (propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.StartPose_X)),
//                (propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.StartPose_Y)),
//                Math.toRadians((propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.StartPose_RADIANS)))
//        );
//        scorePose = new Pose(
//                (propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.ScorePose_X)),
//                (propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.ScorePose_Y)),
//                Math.toRadians((propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.ScorePose_RADIANS)))
//        );
//        pickup1Pose = new Pose(
//                (propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.Pickup1Pose_X)),
//                (propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.Pickup1Pose_Y)),
//                Math.toRadians((propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.Pickup1Pose_RADIANS)))
//        );
//        pickup2Pose = new Pose(
//                (propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.Pickup2Pose_X)),
//                (propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.Pickup2Pose_Y)),
//                Math.toRadians((propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.Pickup2Pose_RADIANS)))
//        );
//        pickup3Pose = new Pose(
//                (propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.Pickup3Pose_X)),
//                (propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.Pickup3Pose_Y)),
//                Math.toRadians((propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.Pickup3Pose_RADIANS)))
//        );
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
laser = hardwareMap.get(DistanceSensor.class, "cr");
        // Initialize follower with custom constants for drive PID tuning
        follower = Constants.createFollower(hardwareMap);
        shooter.setPropertiesFile(propertiesSystem);
        shooter.init(hardwareMap, telemetryS, logging, propertiesSystem);
        pose6D = new TeleOp26.Pose6D(logging);

        buildPaths(); // Pre-calculate path points
        follower.setStartingPose(startPose);
        Drawing.drawDebug(follower);
        telemetryS.update();
setupSensors();
    }

    @Override
    public void start() {
setupSensors();


        opmodeTimer.resetTimer();
        setPathState(0); // Kick off the state machine
    }
    private LED ledRed, ledGreen;

    ColorRangeSensor allianceSensor;

    public enum ALLIANCE {RED, BLUE, YELLOW};
    public static TeleOp26.ALLIANCE alliance = TeleOp26.ALLIANCE.YELLOW;
    private void setupSensors(){
        // Detects sample/alliance color
        allianceSensor = hardwareMap.get(ColorRangeSensor.class, "allianceSensor");
        ledRed = hardwareMap.get(LED.class, "red");
        ledGreen = hardwareMap.get(LED.class, "green");
        // Set the channel as an input

        if(allianceSensor.red() > 400 || allianceSensor.blue() > 400) {
            if(allianceSensor.red() < allianceSensor.blue()){
                alliance = TeleOp26.ALLIANCE.BLUE;
                logging.log("ALLIANCE IS BLUE");
                logging.log(allianceSensor.blue() + " VS " + allianceSensor.red());
                telemetryS.addData(allianceSensor.red() + " VS " + allianceSensor.blue(), "");
            }else{
                alliance = TeleOp26.ALLIANCE.RED;
                logging.log("ALLIANCE IS RED");
                telemetryS.addData(allianceSensor.red() + " VS " + allianceSensor.blue(), "");

            }
            ledRed.enable((allianceSensor.red() > allianceSensor.blue()));
            ledGreen.enable((allianceSensor.red() < allianceSensor.blue()));
        } else {
            ledRed.off();
            ledGreen.off();
            logging.log("ALLIANCE IS YELLOW WHAT OH NO");
            telemetryS.speak("OH NOOO COLRO IS NOT RESPOND");
        }

    }

    /**
     * Continuous execution loop. Updates the Follower's PID controllers and
     * the Shooter's motor logic every tick.
     */
    @Override
    public void loop() {

        pose6D.updateRobotPositon(follower, logging, telemetryS);
        currentResults = limelight.loop(follower.getPose(), logging, opmodeTimer);
        // Critical System Updates
        follower.update();

        LWS = hardwareMap.get(Servo.class, "LeftWristServo");
        RWS = hardwareMap.get(Servo.class, "RightWristServo");
        RWS.setPosition(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.WRIST_POSITION));
        logging.log("Setting pos for RWS to 0.9");
        LWS.setPosition(-propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.WRIST_POSITION));
//        shooter.loop(gamepad1,gamepad2,telemetryS, currentResults[0], pose6D, logging);
        try {
            autonomousPathUpdate();

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Telemetry: Crucial for field-side debugging
        telemetryS.addData("Current State", pathState);
        telemetryS.addData("Robot Pose", follower.getPose().toString());
        telemetryS.addData("Results", currentResults[0]);
        telemetryS.update();
    }

    @Override
    public void stop() {
      propertiesSystem.PROPERTIES.put(PropertiesSystem.Prop.CURRENT_POSE_X, (float) follower.getPose().getX());
       propertiesSystem.PROPERTIES.put(PropertiesSystem.Prop.CURRENT_POSE_Y, (float) follower.getPose().getY()); // Note: Check if Y should be here
            propertiesSystem.PROPERTIES.put(PropertiesSystem.Prop.CURRENT_POSE_RADIANS, (float) follower.getPose().getHeading());
        // Resource cleanup (if applicable)
        try {
            propertiesSystem.saveProperties();
        } catch (IOException e) {

        }
    }
}