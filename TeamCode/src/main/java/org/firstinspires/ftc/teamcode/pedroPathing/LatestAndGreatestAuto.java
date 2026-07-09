package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

// Imports from your Systems folder
import org.firstinspires.ftc.teamcode.System.*;

import java.io.IOException;

/**
 * <h1>CaydenAuto Combined</h1>
 * <p>
 * This OpMode combines the FSM logic of CaydenAuto26 with the 
 * geometric paths of LatestAndGreatestAuto.
 * </p>
 */

@Configurable
@Autonomous(name = "LatestAndGreatestAuto Auto Combined", group = "Production")
public class LatestAndGreatestAuto extends OpMode {

    // ---------------------------------------------------------
    // SUBSYSTEMS & INFRASTRUCTURE
    // ---------------------------------------------------------

    private Follower follower;
    private final FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // ---------------------------------------------------------
    // FIELD COORDINATES (POSES)
    // ---------------------------------------------------------

    // Updated StartPose to match the start of Path1 from the new geometry
    // Note: The new paths start at x=55.6, y=7.6 roughly. 
    public Pose startPose = new Pose(55.610-72, 7.610-72, Math.toRadians(90));

    // Kept these for reference, though the paths now hardcode coordinates
    public Pose scorePose = new Pose(62.634-72, 81.171-72, Math.toRadians(135));

    // ---------------------------------------------------------
    // PATH CONTAINERS
    // ---------------------------------------------------------

    // Changed scorePreload to PathChain to match the new path builder style
    private PathChain scorePreload;
    private PathChain prePickup1, grabPickup1, scorePickup1;
    private PathChain prePickup2, grabPickup2, scorePickup2;
    private PathChain prePickup3, grabPickup3, scorePickup3;
    private PathChain returnToBase;

    // Custom System Objects
    public LimelightSystem limelight;
    Object[] currentResults;
    public PanelsSystem telemetryS;
    public PropertiesSystem propertiesSystem;
    public LoggingSystem logging = new LoggingSystem();
    public AutoTeleOpCommunicationSystem communication;
    public TeleOp26.Pose6D pose6D;
    private DistanceSensor laser;

    /**
     * Defines the geometric trajectories for the robot using LatestAndGreatestAuto coordinates.
     */
    public void buildPaths() {

        // 1. PRELOAD: Drive to goal (Mapped from Path1)
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(55.610-72, 7.610-72),
                        new Pose(62.634-72, 81.171-72)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();
        // Drawing moved inside update or init to avoid nulls if needed, or kept here if Drawing works statically

        // 2. SAMPLE 1: Setup (Mapped from line2)
        prePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(62.634-72, 81.171-72),
                        new Pose(38.439-72, 83.707-72)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // 3. SAMPLE 1: Grab (Mapped from line3)
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(38.439-72, 83.707-72),
                        new Pose(13.659-72, 83.683-72)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // 4. SAMPLE 1: Score (Mapped from line4)
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(13.659-72, 84.683-72),
                        new Pose(62.439-72, 81.171-72)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .setReversed()
                .build();

        // 5. SAMPLE 2: Setup (Mapped from line5)
        prePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(62.439-72, 81.171-72),
                        new Pose(38.244-72, 60.098-72)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // 6. SAMPLE 2: Grab (Mapped from line6)
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(38.244-72, 60.098-72),
                        new Pose(14.049-72, 60.488-72)))
                .setTangentHeadingInterpolation()
                .build();

        // 7. SAMPLE 2: Score (Mapped from line7)
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(14.049-72, 60.488-72),
                        new Pose(62.634-72, 81.171-72)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .setReversed()
                .build();

        // 8. SAMPLE 3: Setup (Mapped from line8)
        prePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(62.634-72, 81.171-72),
                        new Pose(38.244-72, 35.707-72)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // 9. SAMPLE 3: Grab (Mapped from line9)
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(38.244-72, 35.707-72),
                        new Pose(13.268-72, 36.293-72)))
                .setTangentHeadingInterpolation()
                .build();

        // NOTE: The new paths provided (LatestAndGreatest) DO NOT have a path to score the 3rd sample.
        // They go straight to parking/ascent (line10).
        // To prevent a crash, we set scorePickup3 to be the same as scorePickup2, 
        // but the logic below maps the "Return To Base" state to line10.
        scorePickup3 = scorePickup2;

        // 10. PARK / ASCENT (Mapped from line10)
        returnToBase = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(13.268, 36.293),
                        new Pose(38.244, 33.951)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

    /**
     * <b>State Machine Controller</b>
     */
    public void autonomousPathUpdate() throws InterruptedException {
        follower.setMaxPower(0.8);

        Drawing.drawDebug(follower);
        switch (pathState) {
            case 0: // Phase: Drive to High Goal with Preload
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1: // Phase: Firing with preload
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);
                        shotsTriggered =true;

                    } else if(shooter.getState() == FlywheelLogic.FlywheelState.SPIN_UP) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);

                    } else if(!shooter.isBusy()) {
                            follower.followPath(prePickup1, true); // Go to pre-pickup
                            setPathState(2);

                    }
                }
                break;

            case 2: // Phase: Move from Pre-Pickup 1 to Grab 1
                if (!follower.isBusy()) {
                    shooter.turnOnIntake();
                    follower.setMaxPower(0.5);
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;

            case 3: // Phase: Intake logic then Score
                if (!follower.isBusy()) {
                    // TODO: Insert Intake Logic Here (e.g., intake.grab())
                    // Assuming intake is instant or handled by subsystem, move to score:
                    follower.setMaxPower(0.8);
                    shooter.turnOffIntake();
                    follower.followPath(scorePickup1, true);
                    setPathState(4);
                }
                break;

            case 4: // Phase: Score Sample 1
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);
                        shotsTriggered =true;
                    } else if(shooter.getState() == FlywheelLogic.FlywheelState.SPIN_UP) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);

                    } else  if (!shooter.isBusy()) {
                        follower.followPath(prePickup2, true);
                        setPathState(5);
                    }
                }
                break;

            case 5: // Phase: Move from Pre-Pickup 2 to Grab 2
                if (!follower.isBusy()) {

                    shooter.turnOnIntake();
                    follower.setMaxPower(0.5);
                    follower.followPath(grabPickup2, true);
                    setPathState(6);
                }
                break;

            case 6: // Phase: Intake 2 then Score
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    shooter.turnOffIntake();
                    // TODO: Insert Intake Logic Here
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;

            case 7: // Phase: Score Sample 2
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);
                        shotsTriggered =true;
                    } else  if(shooter.getState() == FlywheelLogic.FlywheelState.SPIN_UP) {
                        shooter.fireShots(3, telemetryS, laser, propertiesSystem, pathTimer);

                    } else if (!shooter.isBusy()) {
                        follower.followPath(prePickup3, true);
                        setPathState(8);
                    }
                }
                break;

            case 8: // Phase: Move from Pre-Pickup 3 to Grab 3
                if (!follower.isBusy()) {

                    shooter.turnOnIntake();
                    follower.setMaxPower(0.5);
                    follower.followPath(grabPickup3, true);
                    setPathState(9); // Note: Original code skipped to scoring or parking here
                }
                break;

            // SPECIAL CASE: The provided paths do not have a "Score 3rd Sample" path.
            // They have a "Park" path immediately after grabbing 3.
            // This logic assumes we grab 3, then go to Park/Ascent.
            case 9: // Phase: Grab 3 Complete -> Go to Park (Ascent)
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    shooter.turnOffIntake();
                    // If you intended to shoot the 3rd one, you need to add a path for it.
                    // Currently mapping this to returnToBase (line10)
                    follower.followPath(returnToBase, true);
                    setPathState(-1); // End State
                }
                break;
                
            /* Original Case 10 removed/merged because line10 is the final path provided 
               in LatestAndGreatestAuto.
            */
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        shotsTriggered = false;
    }

    @Override
    public void init() {
        telemetryS = new PanelsSystem(telemetry);
        propertiesSystem = new PropertiesSystem(telemetryS);
        logging.setTelemetry(telemetryS);
        communication = new AutoTeleOpCommunicationSystem();
        limelight = new LimelightSystem(telemetryS, hardwareMap);

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

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap, telemetryS, logging, propertiesSystem);
        pose6D = new TeleOp26.Pose6D(logging);
        laser = hardwareMap.get(DistanceSensor.class, "cr");

        buildPaths();

        // IMPORTANT: Set start pose to match the beginning of Path1
        follower.setStartingPose(startPose);
        Drawing.drawDebug(follower);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        pose6D.updateRobotPositon(follower, logging, telemetryS);

        currentResults = limelight.loop(follower.getPose(), logging, opmodeTimer);

        follower.update();
        shooter.loop(gamepad1, gamepad2, telemetryS, currentResults[0], pose6D, logging);

        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetryS.addData("Current State", pathState);
        telemetryS.addData("Robot Pose", follower.getPose().toString());
        telemetryS.addData("Results", currentResults[0]);
        telemetryS.update();
    }

    @Override
    public void stop() {
        propertiesSystem.PROPERTIES.put(PropertiesSystem.Prop.CURRENT_POSE_X, (float) follower.getPose().getX());
        propertiesSystem.PROPERTIES.put(PropertiesSystem.Prop.CURRENT_POSE_Y, (float) follower.getPose().getY());
        propertiesSystem.PROPERTIES.put(PropertiesSystem.Prop.CURRENT_POSE_RADIANS, (float) follower.getPose().getHeading());
        try {
            propertiesSystem.saveProperties();
        } catch (IOException e) {
            // Handle save error
        }
    }
}