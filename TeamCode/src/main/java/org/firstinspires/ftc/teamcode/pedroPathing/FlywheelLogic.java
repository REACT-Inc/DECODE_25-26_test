package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.System.LoggingSystem;
import org.firstinspires.ftc.teamcode.System.PanelsSystem;
import org.firstinspires.ftc.teamcode.System.PropertiesSystem;

import java.util.ArrayList;

/**
 * FlywheelLogic
 * - FSM-based shooter control
 * - Velocity PIDF
 * - Runtime debug enable / disable
 */
public class FlywheelLogic {
    VelocityInterpolator aimTable = new VelocityInterpolator();

    // ---------------------------------------------------------
    // HARDWARE
    // ---------------------------------------------------------
    private DcMotorEx motor1;
    private PropertiesSystem propertiesSystem;
    private static DcMotorEx motor2;
    private static DcMotorEx motor3;
    private static DcMotorEx motor4;
    private DcMotor FLM, FRM, BLM, BRM; // Back Right Motor
    private Servo LWS, RWS; // Right Wrist Servo (Normal Rotation)
    private static CRServo LRBS, RRBS, MIS;
//    private DigitalChannelssss
    // ---------------------------------------------------------
    // PIDF (Velocity Control)
    // ---------------------------------------------------------
    public static double P = 21.0;
    private static double I = 0.0;
    private static double D = 0.0;
    public static double F =  15.0;
    private boolean activated = true;

    // ---------------------------------------------------------
    // DEBUG MODE
    // ---------------------------------------------------------
    private boolean debugEnabled = false;

    public void setPropertiesFile(PropertiesSystem propertiesSystem) {
        this.propertiesSystem = propertiesSystem;
    }

    // ---------------------------------------------------------
    // STATE MACHINE
    // ---------------------------------------------------------
    public enum FlywheelState {
        IDLE,
        SPIN_UP,
        INTAKE, LAUNCH
    }

    private FlywheelState flywheelState = FlywheelState.IDLE;
    public FlywheelState getState(){
        return flywheelState;
    }
    private final ElapsedTime stateTimer = new ElapsedTime();

    // ---------------------------------------------------------
    // CONSTANTS
    // ---------------------------------------------------------
    private static final double TARGET_VELOCITY = 1500; // ticks/sec
    private static final double MAX_SPINUP_TIME = 2.0;

    // ---------------------------------------------------------
    // VARIABLES
    // ---------------------------------------------------------
    private int shotsRemaining = 0;
    public LoggingSystem logging;

    // ---------------------------------------------------------
    // INIT
    // ---------------------------------------------------------
    public void init(HardwareMap hardwareMap, PanelsSystem telemetry, LoggingSystem logging, PropertiesSystem propertiesSystem) {
        this.logging = logging;
        this.propertiesSystem = propertiesSystem;
        motor1 = hardwareMap.get(DcMotorEx.class, "1");
        motor2 = hardwareMap.get(DcMotorEx.class, "2");
        motor3 = hardwareMap.get(DcMotorEx.class, "3");
        motor4 = hardwareMap.get(DcMotorEx.class, "4");

        LWS = hardwareMap.get(Servo.class, "LeftWristServo");
        RWS = hardwareMap.get(Servo.class, "RightWristServo");
        MIS = hardwareMap.get(CRServo.class, "MIS");
        LRBS = hardwareMap.tryGet(CRServo.class, "LeftRubberBandServo");
        RRBS = hardwareMap.tryGet(CRServo.class, "RightRubberBandServo");
        RRBS.setDirection(CRServo.Direction.REVERSE); // Counter-rotational to work in sync
        LRBS.setDirection(CRServo.Direction.FORWARD);

//        cr = hardwareMap.get(ColorRangeSensor.class, "cr");
//        cr.getDistance(())

        DcMotorEx[] motors = { /*motor1,*/ motor2, motor3, motor4};

        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            applyPIDF(m);
            m.setVelocity(0);
        }
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        applyPIDF(motor2);
        motor2.setVelocity(0);
        flywheelState = FlywheelState.IDLE;
        telemetry.addData("Flywheels set to", flywheelState.name());


        // --- CALIBRATION ---
        // Add as many points as you want.
        // Format: add(Distance, Velocity);

        //target back motor 872.70
        //target velocity 1597.80

        // Close range
        //add the stuff to the table for knpwn points
//        aimTable.add(75.5, 2100.0); // At 10 inches, 1000 RPM works
        //at 70in it goes 850velc and it lands if the back wheel is uflly up to speed

        aimTable.add(110.0, 1597.0, 872.0);//back motor at 1900VELCOITY
        aimTable.add(90.0, 1124.0, 1812.0);
        aimTable.add(100.0, 1300, 1840.0);
        aimTable.add(45.0, 860, 1845.0);

        // Mid range
        aimTable.add(53.0, 980.0, 1828.0);
        aimTable.add(65.0, 970.0, 1828.0); // Non-linear jump!

        // Long range
        aimTable.add(75.0, 1000.0, 1828.0);//1623.17

    }

    // ---------------------------------------------------------
    // UPDATE LOOP (CALL EVERY OPMODE LOOP)
    // ---------------------------------------------------------
    public void update(PanelsSystem telemetry) {

        switch (flywheelState) {

            case IDLE:
                if (shotsRemaining > 0) {
                    setAllVelocity(TARGET_VELOCITY);
                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (stateTimer.seconds() >= MAX_SPINUP_TIME) {
                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;

            case LAUNCH:
                shotsRemaining--;
                stateTimer.reset();

                if (shotsRemaining <= 0) {
                    stopMotors();
                    flywheelState = FlywheelState.IDLE;
                } else {
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;
        }
        telemetry.addData("Flywheels set to", flywheelState.name());

    }

    // ---------------------------------------------------------
    // CONTROL
    // ---------------------------------------------------------
    public void fireShots(int shots, PanelsSystem telemetryS, DistanceSensor follower, PropertiesSystem propertiesSystem, Timer pathTimer) {
        /// TODO FORAUTO SHOT FIRING
        flywheelState = FlywheelState.SPIN_UP;
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double launcherCalculatedTarget = aimTable.getVelocityForLauncher((Double) 65.0);
        double backCalculatedTarget = aimTable.getVelocityForBack((Double) 65.0);
        telemetryS.addData("--------------------------", launcherCalculatedTarget);
        telemetryS.addData("--------------------------", backCalculatedTarget);
        setAllVelocity(launcherCalculatedTarget);
        setBackVelocity(backCalculatedTarget);
        telemetryS.addLine("LAUNCHER IF 1 NOT ACTIVE!!!!");
        if(motor2.getVelocity() + 100 > launcherCalculatedTarget){
            //OK launcher resady!
            telemetryS.addLine("LAUNCHER IF 1 ACTIVE");
            MIS.setPower(-100);

            LRBS.setPower(1);
            RRBS.setPower(1);
        }
        if(follower.getDistance(DistanceUnit.INCH) < propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.AUTO_BALL_DISTANCE_MAX) && pathTimer.getElapsedTimeSeconds() < propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.AUTO_BALL_TIMEOUT)){
            flywheelState = FlywheelState.SPIN_UP;


        }else if(pathTimer.getElapsedTimeSeconds() < propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.AUTO_BALL_REQUIRED_TIME)) {
            flywheelState = FlywheelState.SPIN_UP;
        }else{
            turnOffIntake();
            setAllVelocity(0);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flywheelState = FlywheelState.IDLE;
        }
        telemetryS.addData("Launcher Target Velocity", launcherCalculatedTarget);
        telemetryS.addData("Launcher Current Velocity", motor2.getVelocity());
        telemetryS.addData("Back Target Velocity" ,backCalculatedTarget);
        telemetryS.addData("Back Current Velocity", motor4.getVelocity());
        telemetryS.addLine("---------------------");
        telemetryS.addData("Tuning P (DPad U/D)", P);
        telemetryS.addData("Tuning F (DPad L/R)", F);
        telemetryS.addData("Step Size (B)", stepSizes[stepIndex]);
        telemetryS.addData("Launcher", launcherEnabled);
        telemetryS.addData("Distance Sensor", follower.getDistance(DistanceUnit.INCH));
        telemetryS.addData("Activated", activated);

    }

    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }

    // ---------------------------------------------------------
    // DEBUG CONTROL
    // ---------------------------------------------------------
    public void enableDebug() {
        debugEnabled = true;
    }

    public void disableDebug() {
        debugEnabled = false;
    }

    public boolean isDebugEnabled() {
        return debugEnabled;
    }
    public void turnOnIntake(){
        MIS.setPower(-100);

        LRBS.setPower(1);
        RRBS.setPower(1);
        motor4.setPower(-0.5);
    }
    public void reverseIntake(){
        MIS.setPower(100);

        LRBS.setPower(-1);
        RRBS.setPower(-1);
        motor4.setPower(-0.5);
    }
    public void turnOffIntake(){

        MIS.setPower(0);

        LRBS.setPower(0);
        RRBS.setPower(0);
        motor4.setPower(0);
    }

    public boolean launcherEnabled = false;

    /**
     * Live PIDF tuning (only applies if debug is enabled)
     */
    public static void setPIDF(double p, double i, double d, double f) {
//        if (!debugEnabled) return;

        P = p;
        I = i;
        D = d;
        F = f;

//        applyPIDF(motor1);
        applyPIDF(motor2);
        applyPIDF(motor3);
        applyPIDF(motor4);
    }

    // ---------------------------------------------------------
    // UTILITIES
    // ---------------------------------------------------------
    public static void applyPIDF(DcMotorEx motor) {
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void setAllVelocity(double velocity) {
//        motor1.setVelocity(velocity);

        if(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LAUNCHER_DISABLED) == -1f){
            stopMotors();
            //no launcher enabled
        }else{
            motor2.setVelocity(velocity);
            motor3.setVelocity(velocity);

            applyPIDF(motor3);
            applyPIDF(motor2);
        }
//        motor4.setVelocity(velocity);
    }
    public  void setBackVelocity(double velocity) {
        if(propertiesSystem.PROPERTIES.get(PropertiesSystem.Prop.LAUNCHER_DISABLED) == -1f){
            stopMotors();
            //no launcher enabled
        }else{
        motor4.setVelocity(velocity);
        applyPIDF(motor4);
        }
    }
    private void stopMotors() {
        motor1.setVelocity(0);
        motor2.setVelocity(0);
        motor3.setVelocity(0);
        motor4.setVelocity(0);
    }

    // ---------------------------------------------------------
    // OPTIONAL TELEMETRY HELPERS
    // ---------------------------------------------------------
    public double getVelocity() {
        return motor1.getVelocity();
    }

    public double getTargetVelocity() {
        return TARGET_VELOCITY;
    }

    public double backVelocity = 1400;
    public double midVelocity = 1700;
    public double topVelocity = 1842;
    public double lowVelocity = 900;

    double curTargetVelocity = lowVelocity;
double backTarget = lowVelocity;

    double[] stepSizes = {100.0, 100.0, 10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;
    PanelsSystem telemetryS;

    public void loop(Gamepad gamepad1, Gamepad gamepad2, PanelsSystem telemetryS, Object distance, TeleOp26.Pose6D robotPoseSystem, LoggingSystem logging) {
        telemetryS.addLine("------------- Flywheel Telemetry --------------");
//telemetryS.newSectionBreak(FlywheelLogic.class);
this.telemetryS = telemetryS;
        if (gamepad2.share) {
            //override the auto system
            activated = !activated;
while(gamepad2.share){

}
        }


        if (activated) {
            telemetryS.addData("Enabled Automated Systems");
            TeleOp26.Pose6D.Positional robotpos = robotPoseSystem.whereIsMyRobot();
            // 2. Calculate the needed velocity
            double launcherCalculatedTarget = aimTable.getVelocityForLauncher((Double) distance);
            double backCalculatedTarget = aimTable.getVelocityForBack((Double) distance);
            telemetryS.addGraphData("Launcher Target Velocity", launcherCalculatedTarget);
            telemetryS.addGraphData("BackMotor Target Velocity", backCalculatedTarget);

            setPIDF(P, 0, 0, F);
            ArrayList<TeleOp26.RobotTask> tasks = robotPoseSystem.position.getTasks();

            switch (robotpos.getCurrentPositon()) {
                case LAUNCH_ZONE:
                    switch (robotpos.getCurrentState()) {
                        case FULLY:
                            setAllVelocity(launcherCalculatedTarget);
                            setBackVelocity(backCalculatedTarget);
                            if(motor2.getVelocity() + 100 > launcherCalculatedTarget){
                                flywheelState = FlywheelState.LAUNCH;
                            }else{
                                flywheelState = FlywheelState.SPIN_UP;
                            }
                            for (TeleOp26.RobotTask task : tasks) {
                                task.runTaskWithAutoOverride(gamepad2.triangle || gamepad2.left_bumper || gamepad2.right_bumper, logging);
                            }
                            break;
                        case PARTIAL:
                            if(motor2.getVelocity() + 100 > launcherCalculatedTarget){
                                flywheelState = FlywheelState.LAUNCH;
                            }else{
                                flywheelState = FlywheelState.SPIN_UP;
                            }
                            setAllVelocity(launcherCalculatedTarget+150);
                            setBackVelocity(backCalculatedTarget+150);

                            break;
                        case APPROACHING:

                            //Doesnt apply here for now
                            break;
                    }
                    break;
                case LOADING_ZONE:
                    switch (robotpos.getCurrentState()) {
                        case FULLY:

                            setAllVelocity(-700);
                            setBackVelocity(-700);
                            flywheelState = FlywheelState.INTAKE;
                            for (TeleOp26.RobotTask task : tasks) {
                                task.runTaskWithAutoOverride(gamepad2.triangle || gamepad2.left_bumper || gamepad2.right_bumper, logging);
                            }
                            //We should spin everything backwars o intake from human  unless we are inmotion then we should intake because we could be trying to pickup
                            break;
                        case PARTIAL:
                            for (TeleOp26.RobotTask task : tasks) {
                                task.runTaskWithAutoOverride(gamepad2.triangle || gamepad2.left_bumper || gamepad2.right_bumper, logging);
                            }
                            //we should start ensuring we can launch and provide teh aim assist
                            break;
                        case APPROACHING:
                            //intake on
                            break;
                    }
                    break;
                case UNKNOWN:
                    setBackVelocity(0);
                    setAllVelocity(0);
flywheelState = FlywheelState.IDLE;
                    break;

                /**
                 * TODOOOOOOOOO
                 */
            }


            telemetryS.addData("Launcher Target Velocity", launcherCalculatedTarget);
            telemetryS.addData("Launcher Current Velocity", motor2.getVelocity());
            telemetryS.addData("Back Target Velocity" ,backCalculatedTarget);
            telemetryS.addData("Back Current Velocity", motor4.getVelocity());
            telemetryS.addLine("---------------------");
            telemetryS.addData("Tuning P (DPad U/D)", P);
            telemetryS.addData("Tuning F (DPad L/R)", F);
            telemetryS.addData("Step Size (B)", stepSizes[stepIndex]);
            telemetryS.addData("Launcher", launcherEnabled);
            telemetryS.addData("Distance", distance);
            telemetryS.addData("Activated", activated);
        }else{
//            telemetryS.speak("ROBOT NO LONGER AUTO");
//
            double launcherCalculatedTarget = aimTable.getVelocityForLauncher((Double) distance);
            double backCalculatedTarget = aimTable.getVelocityForBack((Double) distance);
            if(gamepad2.dpad_down) {
                 launcherCalculatedTarget = aimTable.getVelocityForLauncher((Double) distance);
                 backCalculatedTarget = aimTable.getVelocityForBack((Double) distance);
                setAllVelocity(launcherCalculatedTarget);
                setBackVelocity(backCalculatedTarget);
            }else if(gamepad2.dpad_up){
                launcherCalculatedTarget = aimTable.getVelocityForLauncher((Double) distance + 5);
                backCalculatedTarget = aimTable.getVelocityForBack((Double) distance+ 5);
                setAllVelocity(launcherCalculatedTarget);
                setBackVelocity(backCalculatedTarget);

            }else if(gamepad2.dpad_right){

                launcherCalculatedTarget = aimTable.getVelocityForLauncher((Double) distance -5);
                backCalculatedTarget = aimTable.getVelocityForBack((Double) distance -5);
                setAllVelocity(launcherCalculatedTarget);
                setBackVelocity(backCalculatedTarget);

            }else if(gamepad2.dpad_left){

                launcherCalculatedTarget = aimTable.getVelocityForLauncher((Double) distance + 100);
                backCalculatedTarget = aimTable.getVelocityForBack((Double) distance +100);
                setAllVelocity(launcherCalculatedTarget);
                setBackVelocity(backCalculatedTarget);

            }else{
                setAllVelocity(0);
                setBackVelocity(0);
                stopMotors();
            }
            telemetryS.addData("Launcher Target Velocity", launcherCalculatedTarget);
            telemetryS.addData("Launcher Current Velocity", motor2.getVelocity());
            telemetryS.addData("Back Target Velocity" ,backCalculatedTarget);
            telemetryS.addData("Back Current Velocity", motor4.getVelocity());
            telemetryS.addLine("---------------------");
            telemetryS.addData("Tuning P (DPad U/D)", P);
            telemetryS.addData("Tuning F (DPad L/R)", F);
            telemetryS.addData("Step Size (B)", stepSizes[stepIndex]);
            telemetryS.addData("Launcher", launcherEnabled);
            telemetryS.addData("Distance", distance);
            telemetryS.addData("Activated", activated);
        }
        telemetryS.addGraphData("Back Current Velocity", motor4.getVelocity());
        telemetryS.addGraphData("Launcher Current Velocity", Math.max(motor2.getVelocity(), motor3.getVelocity()));
        telemetryS.addGraphData("Launcher Power", Math.round(motor2.getCurrent(CurrentUnit.AMPS) + motor3.getCurrent(CurrentUnit.AMPS)));
        telemetryS.addGraphData("Back Power", Math.round(motor4.getCurrent(CurrentUnit.AMPS)));



    }
    /*

//        curTargetVelocity = (double) CalculatedTargetVelocity;

        /* Toggle target velocity *

if(activated){
    curTargetVelocity = CalculatedTargetVelocity;
}else{
    curTargetVelocity = lowVelocity;
}
if(activated){
    backTarget = calcbackTarget;
}else{
    backTarget= backVelocity;
}
        /* Change step size *
        if (gamepad1.b && !activated) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
            while(gamepad1.b){

            }
        }

        /* Tune F *
        if(debugEnabled){
            if (gamepad1.dpad_left) {
                F -= stepSizes[stepIndex];
            }
            if (gamepad1.dpad_right) {
                F += stepSizes[stepIndex];
            }

            /* Tune P *
            if (gamepad1.dpad_up) {
                P += stepSizes[stepIndex];
            }
            if (gamepad1.dpad_down) {
                P -= stepSizes[stepIndex];
            }
        }
        /* Tuning Buttons *
        if (gamepad2.dpad_left && !activated) {
            lowVelocity -= stepSizes[stepIndex];

            curTargetVelocity = lowVelocity
            ;
        }
        if (gamepad2.dpad_right && !activated) {
            lowVelocity += stepSizes[stepIndex];
            curTargetVelocity = lowVelocity;
        }
        if(gamepad2.dpad_up && !activated){
            backVelocity += stepSizes[stepIndex];
            backTarget = (backVelocity);
        }

        if(gamepad2.dpad_down && !activated){
            backVelocity -= stepSizes[stepIndex];
            backTarget = (backVelocity);

        }
        if (gamepad2.b && !activated) {

            setBackVelocity(backTarget);
        } else {
            setBackVelocity(0);
        }

        /* Apply new PIDF *
//        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
//        flywheelMotor.setPIDFCoefficients(
//                DcMotor.RunMode.RUN_USING_ENCODER,
//                pidf
//        );
        if (launcherEnabled) {
            setPIDF(P, 0, 0, F);
            /* Set velocity *
            setAllVelocity(curTargetVelocity);
        } else {
            setAllVelocity(0);
        }
        /* Telemetry *
        double curVelocity = motor2.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetryS.addData("Target Velocity", curTargetVelocity);
        telemetryS.addData("Current Velocity", "%.2f", curVelocity);
        telemetryS.addData("Current Back Motor", "%.2f", motor4.getVelocity());
        telemetryS.addData("Target Back Motor", "%.2f", backTarget);
        telemetryS.addData("Error", "%.2f", error);
        telemetryS.addLine("---------------------");
        telemetryS.addData("Tuning P (DPad U/D)", "%.5f", P);
        telemetryS.addData("Tuning F (DPad L/R)", "%.5f", F);
        telemetryS.addData("Step Size (B)", "%.5f", stepSizes[stepIndex]);
        telemetryS.addData("Launcher", launcherEnabled);
        telemetryS.addData("Distance", distance);
        telemetryS.addData("acrtuive", activated);
        // Get the digital sensor from the hardware map

            // Read the sensor state (true = HIGH, false = LOW)
            boolean stateHigh = Laser.getState();

            // Active-HIGH: HIGH means an object is detected
            boolean detected = stateHigh;

            // Display detection state
            if (detected) {
                telemetryS.addLine("Object detected!");
            } else {
                telemetryS.addLine("No object detected");
            }

            // Display the raw HIGH/LOW signal for reference
            telemetryS.addData("Raw (HIGH/LOW)", stateHigh);


    }
*/
    public void toggle() {
        launcherEnabled = !launcherEnabled;


    }
}