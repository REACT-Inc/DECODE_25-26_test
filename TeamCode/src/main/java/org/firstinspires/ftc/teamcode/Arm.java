//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import org.firstinspires.ftc.teamcode.System.File.ConfigScanner;
////import org.firstinspires.ftc.teamcode.System.File.FileContent;
////import org.firstinspires.ftc.teamcode.System.File.InternalFile;
//
//@TeleOp(name = "AutoConfigSystem", group = "System")
//public class Arm extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//        // --- 1. SETUP PHASE ---
//        telemetry.addData("System", "Initializing Config Scanner...");
//        telemetry.update();
//
//        // Initialize the new Config Scanner
//        ConfigScanner scanner = new ConfigScanner(hardwareMap);
//        InternalFile active = new InternalFile("ACRobots/RobotSpecific/RobotVariables.txt");
//active.ensureDirectoryExists();
//FileContent writ = new FileContent(active);
//writ.writeToFile("%Pose%RobotPose: 1,1,1;");
//
//        // --- 2. IDENTIFY ACTIVE CONFIG ---
//        // This runs the logic: Get Files -> Read XML -> Check HardwareMap -> Find Match
//        String activeConfigName = scanner.identifyActiveConfig();
//
//        telemetry.addData("Active Config Found", activeConfigName);
//        telemetry.addLine("-----------------------------");
//
//        // --- 3. LOAD DATA BASED ON CONFIG ---
//        // Now that we know WHICH file is active, we can read specific settings from it
//        // Or create a new backup file for it.
//
//        InternalFile configFile = new InternalFile(activeConfigName);
//        FileContent fileReader = new FileContent(configFile);
//
//        // Example: Saving a timestamp to a log file specific to this config
//        InternalFile logFile = new InternalFile("Logs", activeConfigName.replace(".xml", "_log.txt"));
//        FileContent logWriter = new FileContent(logFile);
//        logWriter.writeToFile("Log started for config: " + activeConfigName + "\nStatus: Ready");
//
//        telemetry.addData("Log File", "Created: " + logFile.getName());
//        telemetry.update();
//
//        waitForStart();
//
//        // --- 4. RUNNING PHASE ---
//        while (opModeIsActive()) {
//            // Displaying the dynamic result
//            telemetry.addData("Mode", "Running");
//            telemetry.addData("Current Robot Config", activeConfigName);
//            telemetry.addData("Log Path", logFile.getFile().getAbsolutePath());
//            telemetry.update();
//        }
//    }
//}