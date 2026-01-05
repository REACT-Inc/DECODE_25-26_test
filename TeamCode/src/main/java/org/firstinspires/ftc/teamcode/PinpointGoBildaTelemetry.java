// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
// import com.qualcomm.robotcore.hardware.I2cAddr;
// import com.gobilda.robotics.PinpointOdometryComputer;
// 
// @Autonomous(name = "GoBilda Pinpoint Telemetry", group = "Odometry")
// public class PinpointGoBildaTelemetry extends LinearOpMode {
// 
//     private PinpointOdometryComputer odo;
// 
//     @Override
//     public void runOpMode() throws InterruptedException {
// 
//         // Get the I2C device from hardware map, name must match config
//         odo = hardwareMap.get(PintpointOdometryComputer.class, "odo");
//         odo.setI2cAddress(I2cAddr.create8bit(0x28)); // default PinPoint address
//         odo.engage(); // Start communication
// 
//         telemetry.addLine("Pinpoint Odometry Initialized");
//         telemetry.addLine("Waiting for start...");
//         telemetry.update();
// 
//         waitForStart();
// 
//         while (opModeIsActive()) {
// 
//             // Pinpoint outputs 12 bytes:
//             // 0-1: X (mm), 2-3: Y (mm), 4-5: Heading (deg * 100), big-endian
//             byte[] data = odo.read(0, 6); // 6 bytes is enough for X,Y,Heading
// 
//             // Convert bytes to signed integers
//             int xRaw = (data[0] << 8) | (data[1] & 0xFF);
//             int yRaw = (data[2] << 8) | (data[3] & 0xFF);
//             int headingRaw = (data[4] << 8) | (data[5] & 0xFF);
// 
//             // Convert to real units
//             double xInches = xRaw / 25.4;          // mm → inches
//             double yInches = yRaw / 25.4;          // mm → inches
//             double headingDeg = headingRaw / 100.0; // heading in degrees
// 
//             telemetry.addData("X (in)", "%.2f", xInches);
//             telemetry.addData("Y (in)", "%.2f", yInches);
//             telemetry.addData("Heading (deg)", "%.2f", headingDeg);
//             telemetry.update();
//         }
//     }
// }
// 