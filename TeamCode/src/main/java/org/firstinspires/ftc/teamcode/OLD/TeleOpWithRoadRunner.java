// package org.firstinspires.ftc.teamcode;
// 
// import com.acmerobotics.roadrunner.geometry.Pose2d;
// import com.acmerobotics.roadrunner.trajectory.Trajectory;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.ElapsedTime;
// 
// import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
// 
// @TeleOp(name = "TeleOp with RoadRunner", group = "TeleOp")
// public class TeleOpWithRoadRunner extends LinearOpMode {
// 
//     private SampleMecanumDrive drive;
//     private ElapsedTime runtime = new ElapsedTime();
// 
//     @Override
//     public void runOpMode() throws InterruptedException {
//         // Initialize the drive system
//         drive = new SampleMecanumDrive(hardwareMap);
// 
//         // Set the initial pose (starting position)
//         drive.setPoseEstimate(new Pose2d(0, 0, 0));
// 
//         waitForStart();
// 
//         while (opModeIsActive()) {
//             // Normal teleop drive control
//             drive.setWeightedDrivePower(
//                     new Pose2d(
//                             -gamepad1.left_stick_y,  // Forward/backward
//                             -gamepad1.left_stick_x,  // Strafe
//                             -gamepad1.right_stick_x  // Rotate
//                     )
//             );
// 
//             // Update the drive system
//             drive.update();
// 
//             // Example: Automatically move to a position when a button is pressed
//             if (gamepad1.a) {
//                 moveToPosition(24, 24, Math.toRadians(90)); // Move to (24, 24) and face 90 degrees
//             }
// 
//             // Add more autonomous movements as needed
//             if (gamepad1.b) {
//                 moveToPosition(0, 0, Math.toRadians(0)); // Return to the starting position
//             }
// 
//             // Telemetry for debugging
//             telemetry.addData("Status", "Run Time: " + runtime.toString());
//             telemetry.addData("Pose Estimate", drive.getPoseEstimate());
//             telemetry.update();
//         }
//     }
// 
//     // Helper method to move the robot to a specific position
//     private void moveToPosition(double x, double y, double heading) {
//         // Create a trajectory to the target position
//         Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
//                 .lineToLinearHeading(new Pose2d(x, y, heading))
//                 .build();
// 
//         // Follow the trajectory
//         drive.followTrajectory(trajectory);
// 
//         // Wait until the trajectory is complete
//         while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
//             drive.update();
//         }
//     }
// }
// 