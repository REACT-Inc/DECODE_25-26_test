
package org.firstinspires.ftc.teamcode.System;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class LimelightSystem{

    public double CalculatedDistance = 70.0;
    public double TagX = 0.0;
    public double TagY = 0.0;
    public Pose3D oldPose3D;
    public double lastUpdate = 0;
    public LLResult oldResult;
    public int TagID = 0;
    private Limelight3A limelight;
    public HardwareMap hardwareMap;
public     PanelsSystem telemetry;
public Pose3D getBotPose(){
    return oldPose3D;
}

//    @Override
//    public void init() {
//         Get a reference to the sensor
//    }

    public LimelightSystem(PanelsSystem telemetry, HardwareMap hardwareMap){
        // Set the location of the robot - this should be the place you are starting the robot from
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
this.telemetry = telemetry;
        limelight.pipelineSwitch(1);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

    }

    // takes the telemetry and the robot pos to do stuff
    //Returns a arrray pf values for limelight
    public Object[] loop(Pose robotPosition, LoggingSystem logging, Timer opmodeTimer) {



        telemetry.addData("X coordinate (IN)", robotPosition.getX());
        telemetry.addData("Y coordinate (IN)", robotPosition.getY());
        telemetry.addData("Heading angle (DEGREES)", robotPosition.getHeading());
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());

        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {//if this is addded then the lines jump all aroundannoying
            // Access general information
            oldPose3D = result.getBotpose();
            oldResult = result;
            TagX = result.getTx();
            TagY = result.getTy();
            lastUpdate = opmodeTimer.getElapsedTimeSeconds();



            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                double x = (fr.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch);
                double z = (fr.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch);

                TagID = fr.getFiducialId();
                CalculatedDistance = Math.sqrt(x*x + z*z);
            }

        }
        if(oldResult != null) {
            double captureLatency = oldResult.getCaptureLatency();
            double targetingLatency = oldResult.getTargetingLatency();
            double parseLatency = oldResult.getParseLatency();
            telemetry.addData("TIME SENSE LAST UPDATE", opmodeTimer.getElapsedTimeSeconds() - lastUpdate);
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);

            telemetry.addData("tx", oldResult.getTx());
            telemetry.addData("txnc", oldResult.getTxNC());
            telemetry.addData("ty", oldResult.getTy());
            telemetry.addData("ta", oldResult.getTa());
            telemetry.addData("tl", oldResult.getTargetingLatency());
            telemetry.addData("v", oldResult.isValid());
            telemetry.addData("tync", oldResult.getTyNC());
            telemetry.addData("TEST pose", oldPose3D.toString());
            telemetry.addData("TEST y",oldPose3D.getPosition().y);
            telemetry.addData("TEST x", oldPose3D.getPosition().x);
            telemetry.addData("TEST z", oldPose3D.getPosition().z);
            telemetry.addData("TEST p", oldPose3D.getOrientation().getPitch());
            telemetry.addData("TEST r", oldPose3D.getOrientation().getRoll());
            telemetry.addData("TEST y", oldPose3D.getOrientation().getYaw());
            telemetry.addData("DIS", CalculatedDistance);
        }else{
            telemetry.addData("Limelight", "No data available");

        }
        Object[] data = {CalculatedDistance, TagX, TagY, TagID};
        logging.log("DATA FOR LIMELIGHT IS DIS " + CalculatedDistance);
        return data;
    }


        // --- SETTINGS ---
        // How much we trust the camera vs encoders per frame (0.01 to 0.1)
        private static final double TRUST_FACTOR = 0.05;

        // Physical offset of the camera from the center of the robot (in inches)
        // Example: Camera is 5 inches forward and 3 inches left of center
        private static final double CAM_X_OFFSET = 10.0;
        private static final double CAM_Y_OFFSET = 0.0;

        private static final Map<Integer, Pose> TAG_MAP = new HashMap<>();
        static {
            TAG_MAP.put(20, new Pose(-56, 56, 135));
            TAG_MAP.put(12, new Pose(-72, -24, 0));
            TAG_MAP.put(14, new Pose(72, 24, Math.toRadians(180)));
            TAG_MAP.put(15, new Pose(72, 48, Math.toRadians(180)));
        }

        /**
         * Unified Method: Calculates Global Position AND smooths it into the Follower.
         */
        public Follower performLocalizationUpdate(Follower follower, Object[] llData) {
            double dist = (Double) llData[0];
            double tagX_Deg = (Double) llData[1];
            int tagID = (Integer) llData[3];

            if (tagID == -1 || !TAG_MAP.containsKey(tagID)) {

                follower.setPose(new Pose(0,0));
                return follower;
            }

            // 1. Get current robot state
            Pose currentPose = follower.getPose();
            double heading = currentPose.getHeading();
            Pose tagPose = TAG_MAP.get(tagID);

            // 2. Calculate "Unadjusted" Position (Where the CAMERA is on the field)
            double absAngleToTag = heading + Math.toRadians(tagX_Deg);
            double camFieldX = tagPose.getX() - (dist * Math.cos(absAngleToTag));
            double camFieldY = tagPose.getY() - (dist * Math.sin(absAngleToTag));

            // 3. APPLY CAMERA OFFSET
            // We rotate the camera's physical offset by the robot's heading
            // to find where the ROBOT center is relative to the camera.
            double robotX = camFieldX - (CAM_X_OFFSET * Math.cos(heading) - CAM_Y_OFFSET * Math.sin(heading));
            double robotY = camFieldY - (CAM_X_OFFSET * Math.sin(heading) + CAM_Y_OFFSET * Math.cos(heading));

            // 4. SMOOTH BLENDING (Confidence Filter)
            // Check if the jump is sane (less than 15 inches) to avoid "glitch-teleporting"
            double driftDist = Math.hypot(robotX - currentPose.getX(), robotY - currentPose.getY());

            if (driftDist < 15000.0) {
                // Linear Interpolation (LERP) for smoothness
                double newX = currentPose.getX() + (robotX - currentPose.getX()) * TRUST_FACTOR;
                double newY = currentPose.getY() + (robotY - currentPose.getY()) * TRUST_FACTOR;

                follower.setPose(new Pose(newX, newY, heading));
            }
            return follower;
        }
    }
