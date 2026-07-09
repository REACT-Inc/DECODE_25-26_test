package org.firstinspires.ftc.teamcode;

import com.acrobotics.v1.Simplify.SimpleOpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Sensor: Limelight3AMY", group = "Sensor")
@Configurable
public class sensorlimelight3A extends SimpleOpMode {

    private Limelight3A limelight;

    // ── Tuning constants ──────────────────────────────────────────────────────
    // Limelight botpose origin is field-center. Pedro uses a corner origin.
    // FTC field is 144 in × 144 in (3.6576 m × 3.6576 m).
    // Half-field in inches = 72 in.
    private static final double FIELD_HALF_IN = 72.0;

    // Minimum number of AprilTags that must contribute to a botpose result
    // before we trust it enough to reset odometry. 2+ tags = much more reliable.
    private static final int MIN_TAG_COUNT_FOR_RESET = 1;

    // How close (inches) the Limelight pose must be to current pose before we
    // accept it — prevents wild jumps from bad single-frame estimates.
    // Set to a large number (e.g. 999) to disable the sanity check.
    private static final double MAX_JUMP_INCHES = 999;
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        // Pipeline 0 — make sure this is your AprilTag / MegaTag2 pipeline
        limelight.pipelineSwitch(0);
        limelight.start();

        console.written(">", "Robot Ready. Press Play.");
    }

    @Override
    public void onInit_loop() {}

    @Override
    public void onStart() {}

    @Override
    public void onStart_loop() {

        // ── Limelight status ─────────────────────────────────────────────────
        LLStatus status = limelight.getStatus();
        console.written("LL Name",   status.getName());
        console.written("LL Temp",   String.format("%.1f C", status.getTemp()));
        console.written("LL CPU",    String.format("%.1f %%", status.getCpu()));
        console.written("LL FPS",    String.format("%d", (int) status.getFps()));
        console.written("Pipeline",  String.format("Idx:%d  Type:%s",
                status.getPipelineIndex(), status.getPipelineType()));

        // ── Current Pedro pose (for reference & jump-check) ──────────────────
        Pose currentPose = follower.getPose();
        console.written("Pedro X",  String.format("%.2f", currentPose.getX()));
        console.written("Pedro Y",  String.format("%.2f", currentPose.getY()));
        console.written("Pedro Hdg (°)", String.format("%.1f", Math.toDegrees(currentPose.getHeading())));

        // ── Limelight result ─────────────────────────────────────────────────
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            // Nothing reliable — zero out the telemetry rows so they don't flicker
            logEmptyResult();
            return;
        }

        // Latency info
        double captureLatency  = result.getCaptureLatency();
        double targetingLatency = result.getTargetingLatency();
        double parseLatency    = result.getParseLatency();
        console.written("LL Total Latency (ms)", String.format("%.1f", captureLatency + targetingLatency));
        console.written("Parse Latency (ms)",    String.format("%.1f", parseLatency));
        console.written("tx",   String.format("%.2f", result.getTx()));
        console.written("ty",   String.format("%.2f", result.getTy()));
        console.written("ta",   String.format("%.3f", result.getTa()));
        console.written("txnc", String.format("%.2f", result.getTxNC()));
        console.written("tync", String.format("%.2f", result.getTyNC()));

        // ── Fiducial results (distance telemetry) ────────────────────────────
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        int tagCount = fiducialResults.size();
        console.written("Visible Tags", tagCount);

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            // Camera-to-target in meters; convert to inches for readability
            double camX_in = fr.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch;
            double camZ_in = fr.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch;
            double dist_in = Math.sqrt(camX_in * camX_in + camZ_in * camZ_in);
            console.written("Tag " + fr.getFiducialId() + " dist", String.format("%.2f", dist_in));
            console.written("Tag " + fr.getFiducialId() + " family",    fr.getFamily());
        }

        // ── Pose conversion & odometry reset ─────────────────────────────────
        Pose3D botpose = result.getBotpose(); // MegaTag2 recommended on LL3A

        if (botpose == null) {
            console.written("BotPose", "null — check pipeline type");
            return;
        }

        // Limelight botpose is in METERS, field-center origin.
        // Step 1: convert meters → inches
        double llX_in = botpose.getPosition().toUnit(DistanceUnit.INCH).x;
        double llY_in = botpose.getPosition().toUnit(DistanceUnit.INCH).y;

        // Step 2: shift origin from field-center to field-corner (Pedro convention)
        double pedroX = llX_in - FIELD_HALF_IN;   // [-72, 72] → [0, 144]
        double pedroY = llY_in - FIELD_HALF_IN;

        // Step 3: clamp to legal field bounds
        pedroX = Math.min(144.0, Math.max(0.0, pedroX));
        pedroY = Math.min(144.0, Math.max(0.0, pedroY));

        // Step 4: heading — getYaw(AngleUnit.RADIANS) is explicit and safe
        double headingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        console.written("LL X",    String.format("%.2f", pedroX));
        console.written("LL Y",    String.format("%.2f", pedroY));
        console.written("LL Hdg",   String.format("%.1f", Math.toDegrees(headingRad)));

        // ── Sanity / confidence gate before resetting odometry ───────────────
        boolean enoughTags = tagCount >= MIN_TAG_COUNT_FOR_RESET;

        double dx = pedroX - currentPose.getX();
        double dy = pedroY - currentPose.getY();
        double jumpDist = Math.sqrt(dx * dx + dy * dy);
        boolean jumpReasonable = jumpDist < MAX_JUMP_INCHES;

        console.written("Tag Gate Pass",  enoughTags);
        console.written("Jump",      String.format("%.2f", jumpDist));
        console.written("Jump Gate Pass", jumpReasonable);

        if (enoughTags && jumpReasonable) {
            follower.setPose(new Pose(pedroX, pedroY, headingRad));
            console.written("Pose Reset", "YES");
        } else {
            console.written("Pose Reset", enoughTags ? "BLOCKED (jump too large)" : "BLOCKED (need ≥"+ MIN_TAG_COUNT_FOR_RESET +" tags)");
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private void logEmptyResult() {
        console.written("LL Total Latency (ms)", 0);
        console.written("Parse Latency (ms)",    0);
        console.written("tx",   0);  console.written("ty",   0);
        console.written("ta",   0);  console.written("txnc", 0);
        console.written("tync", 0);  console.written("Visible Tags", 0);
        console.written("BotPose",   "no result");
        console.written("Pose Reset","NO");
    }
}