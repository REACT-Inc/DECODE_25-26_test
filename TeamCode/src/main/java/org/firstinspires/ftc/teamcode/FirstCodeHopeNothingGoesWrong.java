 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
 import com.qualcomm.robotcore.hardware.HardwareDevice;
 import com.qualcomm.robotcore.hardware.IMU;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

 import com.qualcomm.hardware.limelightvision.Limelight3A;
 import com.qualcomm.hardware.limelightvision.LLStatus;
 import com.qualcomm.hardware.limelightvision.LLResult;
 import com.qualcomm.hardware.limelightvision.LLResultTypes;

 import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

 import com.google.gson.JsonArray;
 import com.google.gson.JsonObject;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.hardware.TouchSensor;

 import java.util.ArrayList;
 import java.util.HashMap;
 import java.util.List;
 import java.util.Map;

 @TeleOp(name="OLD", group="TELEOP")
 public class FirstCodeHopeNothingGoesWrong extends LinearOpMode {
     private Limelight3A limelight;
     /*
     Need code that works todo

     */
     ArrayList<String> telemetryLog = new ArrayList<String>(1);


 private long lastTime = 0;
 private double lastRPM = 0.0;
 private int lastPos = 0;

 // GoBilda Yellow Jacket built-in encoder count:
 // 28 ticks/rev on the motor shaft
 private static final double TICKS_PER_REV = 28;

     // ---- CONFIG ----
     // Set these to match your robot & field
     final double CAMERA_HEIGHT = 0.27;   // meters (example: 27cm from floor)
     final double TAG_HEIGHT = 1.22;      // meters (example: FTC center goal tag height)
     final double CAMERA_PITCH_DEG = 20;  // upward tilt of your camera in degrees

     // Persistent light toggle + motor so state survives loop iterations
     private boolean lightToggle = false;
     private DcMotor lightMotor = null;
     private DcMotor launcherMotor = null;
     public void Shooter(){
         double targetRPM = 250;   // your chosen shooting RPM
 double k = 0.0008;        // gain — may adjust
 double shooterPower = 1.0;

 int pos = launcherMotor.getCurrentPosition();
 long now = System.currentTimeMillis();

 if (lastTime == 0) {
     lastTime = now;
     lastPos = pos;
 }
     // telemetry.addData("TEST", l;
 if (now - lastTime >= 200) {   // update every 50ms
     int delta = pos - lastPos;

     double revPerSec = (delta / TICKS_PER_REV) / 0.2;
     double currentRPM = revPerSec * 60.0;
     lastRPM = currentRPM;
     telemetry.addData("Shooter RPM", currentRPM);

     // -----------------------
     // CLOSED-LOOP CONTROL
     // -----------------------
     double error = targetRPM - currentRPM;

     // proportional adjuster
     shooterPower += error * k;

     // clamp
     shooterPower = Math.min(1.0, Math.max(0.0, shooterPower));

     // if(gamepad1.x){ launcherMotor.setPower(shooterPower); }

     lastPos = pos;
     lastTime = now;
 }else{
         telemetry.addData("Shooter RPM", lastRPM);

 }

 // telemetry.update();

     }
     public void Lights() {
         // lightMotor is initialized in runOpMode once
         if (lightMotor == null) return;

         if (gamepad1.options) {
             // toggle only on the button press edge
             // simple debouncing: wait a tiny bit so you don't flip multiple times in one loop
             lightToggle = !lightToggle;
             sleep(75);
         }

         lightMotor.setPower(lightToggle ? 1.0 : 0.0);
         telemetry.addData("lightToggle", lightToggle);
     }

     public void runOpMode() throws InterruptedException {
         // Declare our motors
         updTelemetry();
         waitForStart();
         num();
     }
     public void num(){
         // Make sure your ID's match your configuration
         DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FL");
         DcMotor backLeftMotor = hardwareMap.dcMotor.get("BL");
         DcMotor frontRightMotor = hardwareMap.dcMotor.get("FR");
         DcMotor backRightMotor = hardwareMap.dcMotor.get("BR");
         launcherMotor = hardwareMap.dcMotor.get("LM");
         DcMotor backMotor = hardwareMap.dcMotor.get("BM");
         CRServo middleServo = hardwareMap.crservo.get("middleservo");
         CRServo frontServo = hardwareMap.crservo.get("frontservo");

         // light motor (initialized once)
         lightMotor = hardwareMap.dcMotor.get("L");

         boolean debugToggle = false;
         int oldLauncherSpeed = 0;

         // Reverse the right side motors if needed
         frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         backMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
         // Retrieve the IMU from the hardware map
         IMU imu = hardwareMap.get(IMU.class, "imu");
         IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                 RevHubOrientationOnRobot.LogoFacingDirection.UP,
                 RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
         imu.initialize(parameters);

         waitForStart();

         if (isStopRequested()) return;

         limelight = hardwareMap.get(Limelight3A.class, "Limelight");

         telemetry.setMsTransmissionInterval(11);

         limelight.pipelineSwitch(0);

         /*
          * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
          */
         limelight.start();

         telemetry.addData(">", "Robot Ready.  Press Play.");
         // telemetry.update();

         while (opModeIsActive()) {

             // Lights handling (now persistent)
             Lights();

             // Limelight status telemetry
             LLStatus status = limelight.getStatus();
             telemetry.addData("Name", "%s", status.getName());
             telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                     status.getTemp(), status.getCpu(), (int) status.getFps());
             telemetry.addData("Pipeline", "Index: %d, Type: %s",
                     status.getPipelineIndex(), status.getPipelineType());

             // Legacy direct result usage (still useful)
             LLResult result = limelight.getLatestResult();
             if (result != null && result.isValid()) {
                 // Access general information
                 Pose3D botpose = result.getBotpose();
                 double captureLatency = result.getCaptureLatency();
                 double targetingLatency = result.getTargetingLatency();
                 double parseLatency = result.getParseLatency();
                 telemetry.addData("LL Latency", captureLatency + targetingLatency);
                 telemetry.addData("Parse Latency", parseLatency);
                 telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                 telemetry.addData("tx", result.getTx());
                 telemetry.addData("txnc", result.getTxNC());
                 telemetry.addData("ty", result.getTy());
                 telemetry.addData("tync", result.getTyNC());

                 telemetry.addData("Botpose", botpose.toString());

                 // Access barcode/classifier/detector/fiducial/color results as before
                 List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                 for (LLResultTypes.BarcodeResult br : barcodeResults) {
                     telemetry.addData("Barcode", "Data: %s", br.getData());
                 }

                 List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                 for (LLResultTypes.ClassifierResult cr : classifierResults) {
                     telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                 }

                 List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                 for (LLResultTypes.DetectorResult dr : detectorResults) {
                     telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                 }

                 List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                 for (LLResultTypes.FiducialResult fr : fiducialResults) {
                     telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                             fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                 }

                 List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                 for (LLResultTypes.ColorResult cr : colorResults) {
                     telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                 }
             } else {
                 telemetry.addData("Limelight", "No data available (result invalid)");
             }

             // ---------------------------
             // NEW: Gson-based remote Limelight JSON access + distance calc
             // (Requires LimelightFTC.getLatestResults(name) that returns com.google.gson.JsonObject)
             // ---------------------------
             // JsonObject llJson = LimelightFTC.getLatestResults("limelight");
 // JsonObject ll = LimelightFTC.getLatestResults("Limelight");

 //if (ll != null) {

     //if (ll.has("fiducialID")) {

         /*int id = ll.get("fiducialID").getAsInt();
         double ty = ll.get("ty").getAsDouble();
         double tx = ll.get("tx").getAsDouble();
         double fx = ll.get("fX").getAsDouble();
         double fy = ll.get("fY").getAsDouble();

         double totalAngleRad = Math.toRadians(ty + CAMERA_PITCH_DEG);
         double distanceMeters = (TAG_HEIGHT - CAMERA_HEIGHT) / Math.tan(totalAngleRad);

         telemetry.addData("Tag ID", id);
         telemetry.addData("tx", tx);
         telemetry.addData("ty", ty);
         telemetry.addData("Distance (m)", distanceMeters);
         */
//     } else {
//         telemetry.addLine("No fiducials in JSON");
//     }
//
// } else {
//     telemetry.addLine("No Limelight JSON");
// }

             // if (llJson != null && llJson.has("Results")) {
             //     JsonObject results = llJson.getAsJsonObject("Results");

             //     // Some firmwares use "Fiducial" array; if not, try "Fiducials" or "tags" fallback
             //     JsonArray tags = null;
             //     if (results.has("Fiducial")) {
             //         tags = results.getAsJsonArray("Fiducial");
             //     } else if (results.has("Fiducials")) {
             //         tags = results.getAsJsonArray("Fiducials");
             //     } else if (results.has("tags")) { // fallback common name
             //         tags = results.getAsJsonArray("tags");
             //     }

             //     if (tags != null && tags.size() > 0) {
             //         JsonObject tag = tags.get(0).getAsJsonObject();

             //         // Many firmwares put the vertical offset in "ty" or "targetY"; try both gracefully
             //         double ty = 0.0;
             //         if (tag.has("ty")) {
             //             ty = tag.get("ty").getAsDouble();
             //         } else if (tag.has("targetY")) {
             //             ty = tag.get("targetY").getAsDouble();
             //         } else if (tag.has("targetYDegrees")) {
             //             ty = tag.get("targetYDegrees").getAsDouble();
             //         }

             //         double totalAngleRad = Math.toRadians(ty + CAMERA_PITCH_DEG);
             //         double distanceMeters = (TAG_HEIGHT - CAMERA_HEIGHT) / Math.tan(totalAngleRad);

             //         // Try to get id (safe fallback)
             //         int tagId = -1;
             //         if (tag.has("id")) tagId = tag.get("id").getAsInt();
             //         else if (tag.has("fiducialId")) tagId = tag.get("fiducialId").getAsInt();

             //         telemetry.addData("Tag ID", tagId);
             //         telemetry.addData("ty", ty);
             //         telemetry.addData("Distance (m)", distanceMeters);
             //     } else {
             //         telemetry.addLine("No tags seen in JSON");
             //     }

             // } else {
             //     telemetry.addLine("No Limelight JSON / Results field");
             // }/

             // ---------------------------
             // DRIVE + actuators (unchanged logic, only cleaned small issues)
             // ---------------------------
             boolean fastMode = gamepad1.left_bumper;
             telemetry.addData("Fastmode", fastMode);
             telemetry.addData("debug", debugToggle);

             if (gamepad1.options) {
                 imu.resetYaw();
                 debugToggle = !debugToggle;
             }

             // launcher & manipulators (kept your original power choices)
             launcherMotor.setPower(gamepad2.x ? 100 : gamepad2.dpad_down ? -0.2 : gamepad2.dpad_up ? 0.70 : gamepad2.dpad_right ? 0.63 : gamepad2.dpad_left ? 0.57 : 0);
             backMotor.setPower(gamepad2.b ? 1 : gamepad2.a ? -0.5 : gamepad2.dpad_down ? 0.6 : 0);
             middleServo.setPower(gamepad2.a || gamepad2.b ? 100 : gamepad2.dpad_down ? -0.3 : 0);
             frontServo.setPower(gamepad2.a || gamepad2.b ? -100 : gamepad2.right_trigger > 0.1f ? 100 : gamepad2.dpad_down ? -1 : 0);

             telemetry.addData("launcherMotorPower", launcherMotor.getPower());
             telemetry.addData("backMotorPower", backMotor.getPower());
             telemetry.addData("middleServoPower", middleServo.getPower());
             telemetry.addData("frontServoPower", frontServo.getPower());

             // if (gamepad1.dpad_down) {
                 telemetry.addData("Launcher encoder", launcherMotor.getCurrentPosition());
                 telemetry.addData("oldLauncherMotorspeed", oldLauncherSpeed);
                 telemetry.addData("motor Speeds", launcherMotor.getCurrentPosition() - oldLauncherSpeed);
             // }
             Shooter();
             // Stop limelight polling only when exiting (you were calling limelight.stop() inside main loop — move to after loop)
             // NOTE: we WILL stop it once we break the loop - but don't stop inside the active loop
             // (so comment was removed here)

             // Drive math (kept the same)
             float walk = -gamepad1.left_stick_y;
             float strafe = gamepad1.left_stick_x;
             float pivot = gamepad1.right_stick_x;
             float dampen = gamepad1.right_trigger;
             float boost = gamepad1.left_trigger + 1;

             telemetry.addData("dampen", dampen);
             telemetry.addData("boost", boost);

             float fl = walk + strafe;
             float fr = walk - strafe;
             float bl = walk - strafe;
             float br = walk + strafe;

             float bst_mult = 0.25f;
             float dmpn_mult = 1;

             fl += pivot;
             fr -= pivot;
             bl += pivot;
             br -= pivot;

             fl /= (dampen * 0.8f) + 0.4f;
             fr /= (dampen * 0.8f) + 0.4f;
             bl /= (dampen * 0.8f) + 0.4f;
             br /= (dampen * 0.8f) + 0.4f;

             fl *= boost * bst_mult;
             fr *= boost * bst_mult;
             bl *= boost * bst_mult;
             br *= boost * bst_mult;

             telemetry.addData("fl", fl);
             telemetry.addData("fr", fr);
             telemetry.addData("bl", bl);
             telemetry.addData("br", br);

             frontLeftMotor.setPower(fl);
             frontRightMotor.setPower(fr);
             backLeftMotor.setPower(bl);
             backRightMotor.setPower(br);

             telemetry.addData("frontLeftMotorPower", frontLeftMotor.getPower());
             telemetry.addData("frontRightMotorPower", frontRightMotor.getPower());
             telemetry.addData("backLeftMotorPower", backLeftMotor.getPower());
             telemetry.addData("backRightMotorPower", backRightMotor.getPower());

             telemetry.update();
             if(oldLauncherSpeed == 0){
             oldLauncherSpeed = launcherMotor.getCurrentPosition();
 }        }

         // clean up when opmode finishes
         limelight.stop();
     }


     private DcMotor createDcMotor(String configurationName){
         DcMotor newMotor = null;
         try{
             newMotor = hardwareMap.get(DcMotor.class, configurationName);

//             newMotor = hardwareMap.dcMotor.get(configurationName);
         }catch (Exception e){
             addDataNewTelemetry("DcMotor Creation Error!", e.getMessage());
             addDataNewTelemetry("WARNING", " it looks like DC Motor: " + configurationName + "Doesnt exist? Its supposed to be in port Please check config");
         }
         return newMotor;
     }
     private Servo createServo(String configurationName){
         Servo newServo = null;
         try{
             newServo = hardwareMap.get(Servo.class, configurationName);

//             newMotor = hardwareMap.dcMotor.get(configurationName);
         }catch (Exception e){
             addDataNewTelemetry("Servo Creation Error!", e.getMessage());
             addDataNewTelemetry("WARNING", " it looks like SERVO: " + configurationName + "Doesnt exist? Its supposed to be in port Please check config");
         }
         return newServo ;
     }
     public void addDataNewTelemetry(String arg1, String arg2){
         telemetryLog.add("ADDDATASPLITMEUP" + arg1 +"SPLITMEUP" +  arg2);
     }
     public void updTelemetry(){
         for(String tely : telemetryLog){
             String[] values = tely.split("SPLITMEUP");
             if(values[0] == "ADDDATA") {
                 telemetry.addData(values[1], values[2]);
             }
         }
         telemetry.update();
     }
 }
