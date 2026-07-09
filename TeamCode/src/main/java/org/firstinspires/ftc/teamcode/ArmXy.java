//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@TeleOp(name = "AutoFactoryTest")
//public class ArmXy extends LinearOpMode {
//
//    // Example custom Enum for testing
//    public enum RobotAction { IDLE, PICKUP, SCORE }
//
//    @Override
//    public void runOpMode() {
//
//        // --- 1. REGISTER KNOWN TYPES ---
//        // Tell the factory what "Pose" and "Direction" actually mean in Java
//        RobotVariable.registerClass("Pose", Pose.class);
//        RobotVariable.registerClass("Direction", DcMotorSimple.Direction.class);
//        RobotVariable.registerClass("Action", RobotAction.class);
//
//        // --- 2. PREPARE FILE ---
//        InternalFile file = new InternalFile("RobotData.txt");
//        FileContent content = new FileContent(file);
//
//        // (Optional) Create a dummy file for this test
//        content.writeToFile(
//                "%Pose%| startPose: 10.5, 20.0, 90.0;\n" +
//                        "%Direction%| driveDir: REVERSE;\n" +
//                        "%Action%| armAction: PICKUP;\n"
//        );
////        RobotVariable.mapToFile(content, new Pose(1,2,3));
//
//        // --- 3. AUTO-LOAD ---
//        telemetry.addData("Status", "Auto-Building Objects...");
//        telemetry.update();
//
//        RobotVariable factory = new RobotVariable();
//        content.loadToFactory(factory);
//
//        // --- 4. RETRIEVE OBJECTS ---
//        // The factory has already done "new Pose(10.5, 20, 90)" for us!
//        Pose myPose = factory.get("startPose");
//
//        // It has also done "Direction.REVERSE" for us!
//        DcMotorSimple.Direction myDir = factory.get("driveDir");
//
//        RobotAction myAction = factory.get("armAction");
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            telemetry.addData("Loaded Pose", myPose.toString());
//            telemetry.addData("Loaded Direction", myDir);
//            telemetry.addData("Loaded Action", myAction);
//            telemetry.update();
//        }
//    }
//}