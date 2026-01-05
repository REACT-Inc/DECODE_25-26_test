package org.firstinspires.ftc.teamcode.OLD;

import android.os.Environment;
    import com.qualcomm.hardware.limelightvision.LLResult;
    import com.qualcomm.hardware.limelightvision.Limelight3A;
    import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.*;
    import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
    import com.qualcomm.robotcore.util.ElapsedTime;
    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
    import org.opencv.core.*;
    import org.opencv.imgproc.Imgproc;
    import org.openftc.easyopencv.OpenCvCameraRotation;
    import org.openftc.easyopencv.OpenCvWebcam;
    import org.openftc.easyopencv.OpenCvPipeline;
    import org.openftc.easyopencv.OpenCvCamera;
    import org.openftc.easyopencv.OpenCvCameraFactory;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    
    import java.io.*;
    import java.text.SimpleDateFormat;
    import java.util.*;
    import java.util.concurrent.Executors;
    import java.util.concurrent.ScheduledExecutorService;
    import java.util.concurrent.TimeUnit;
    
    @TeleOp(name = "Final TeleOp24")
    @Disabled
    public class TeleOp24 extends LinearOpMode {
    
        private final String VERSION = "MAR2 A";
        private final String[] resColors = {"Blue", "Yellow", "Red"};
    
        private DcMotor FL;
        private DcMotor FR;
        private DcMotor BL;
        private DcMotor BR;
        private CRServo HORIZONTAL_SLIDE;
        private Servo LOWER_WRIST;
        private Servo LOWER_GRIPPER;
        private Servo UPPER_GRIPPER;
        //        private Servo[] AS = new Servo[2];
        private DcMotor TILT1;
        private DcMotor TILT2;
        private DcMotorEx VERTICAL_SLIDE_1;
        private DcMotorEx VERTICAL_SLIDE_2;
        private Limelight3A LIMELIGHT;
        private RevBlinkinLedDriver BLINKIN;
        private Servo GRIP_INDICATOR;
        private AnalogInput AE;
        private TouchSensor SLIDE_TOUCH;
        private Servo UPPER_WRIST;
        private DcMotor CAMLIGHT;
        private OpenCvWebcam webcam;
    
        private ElapsedTime timer;
    
        private final ArrayList<String> ACTION_LOG = new ArrayList<>();
    
        private final String DATA_DIR = Environment.getExternalStorageDirectory().getPath() + "/ATEAM";
        private final String DATA_SAVE = DATA_DIR + "/PROPERTIES";
        private final String LOG_DIR = DATA_DIR + "/LOGS";
    
        private final ScheduledExecutorService sched = Executors.newScheduledThreadPool(1);
    
        //       _____   ______   _____   _____   _____   ______   ______   ___   _____   ______
        //      /  . /  /  .  /  /  . /  /  . /  /  __/  /  .  /  /_   _/  /  /  /  __/  /   __/
        //     /  __/  /    \   /  / /  /  __/  /  _/   /    \     / /    /  /  /  _/   /__   /
        //    /__/    /__/__/  /____/  /__/    /____/  /__/__/    /_/    /__/  /____/  /_____/
        // ---- PROPERTIES ---- //
        public enum Prop {
            /**
             * The speed the horizontal slides move at.
             ***/
            HS_SPEED,
            /**
             * The amount of samples used in averaging Limelight results.
             ***/
            COLOR_MEAN_RANGE,
            /**
             * The multiplier applied to the speed boost.
             **/
            SPEED_BOOST_MULTIPLIER,
            /**
             * The multiplier applied to the speed dampening.
             **/
            SPEED_DAMPEN_MULTIPLIER,
            /**
             * The lower position of the left alignment servo.
             **/
            ALIGN_LEFT_LOWER,
            /**
             * The upper position of the left alignment servo.
             **/
            ALIGN_LEFT_UPPER,
            /**
             * The lower position of the right alignment servo.
             **/
            ALIGN_RIGHT_LOWER,
            /**
             * The upper position of the right alignment servo.
             **/
            ALIGN_RIGHT_UPPER,
            /**
             * The speed he crane lifts and drops at.
             **/
            CRANE_LIFT_SPEED,
            /**
             * The position of the gripper when it's open.
             **/
            GRIPPER_CLOSE_POS,
            /**
             * The position of the gripper when it's closed.
             **/
            GRIPPER_OPEN_POS,
            /**
             * The color of the PWM light when the gripper is open.
             **/
            ICR_GRIPPER_OPEN_COLOR,
            /**
             * The color of the PWM light when the gripper is closed.
             **/
            ICR_GRIPPER_CLOSE_COLOR,
            /**
             * The max position of the slide.
             **/
            HS_MAX_POS,
            /**
             * The wrist's max pos
             */
            WR_MAX_POS,
            /**
             * The wrist's min pos
             */
            WR_MIN_POS,
            /**
             * How much the wrist's speed is dampened by.
             */
            WRIST_SPEED_DAMPEN,
            /**
             * A multiplier for the vertical slide's speed when L1 is pressed on gamepad2.
             */
            VERTICAL_SLOW_DAMPEN,
            /**
             * The amount to increment the servo by to rotate 45 degrees.
             */
            WRIST_45_AMOUNT,
            /**
             * The up position for the upper wrist.
             */
            UPPER_WRIST_UP_POS,
            /**
             * The down position for the upper wrist.
             */
            UPPER_WRIST_DOWN_POS,
            /**
             * The middle pos of the upper wrist.
             */
            UPPER_WRIST_MID_POS,
            /**
             * The delay before the upper wrist rotates after the gripper is toggled.
             */
            UPPER_WRIST_DELAY_SECONDS,
            /**
             * Preset positions for the VS
             */
            PRESET_CROSS_VS,
            PRESET_TRIANGLE_VS,
            PRESET_CIRCLE_VS,
            PRESET_SQUARE_VS,
            /**
             * The threshold when the VS will start slowing down when achieving a target position.
             */
            VS_AUTO_SLOWDOWN_THRESHOLD,
            /**
             * Gripper's narrow-open position.
             */
            GRIPPER_NARROW_OPEN_POSITION,
            /**
             * Upper gripper open position.
             */
            UPPER_GRIPPER_OPEN,
            /**
             * Upper gripper close position.
             */
            UPPER_GRIPPER_CLOSE,
            /**
             * Gripper mid pos light color.
             */
            ICR_GRIPPER_MID_COLOR,
            /**
             * Default lower wrist position.
             */
            LOWER_WRIST_STARTING_POS,
            /**
             * The minimum position allowed for the HS
             */
            HS_MIN_POS,
            /**
             * The position considered '0 degrees' by the wrist.
             */
            WRIST_0_POS,
            /**
             * The position considered '180 degrees' by the wrist.
             */
            WRIST_180_POS,
            /**
             * The position the HS will go to on startup.
             */
            HS_STARTING_POS,
            /**
             * How bright the camlight gets.
             */
            CAMLIGHT_BRIGHTNESS,
            /**
             * Vs
             */
            VERTICAL_HOLD,
        }
    
        private final HashMap<Prop, Float> PROPERTIES = new HashMap<Prop, Float>() {
            {
                put(Prop.HS_SPEED, 0.5f);
                put(Prop.COLOR_MEAN_RANGE, 60f);
                put(Prop.SPEED_BOOST_MULTIPLIER, 1f);
                put(Prop.SPEED_DAMPEN_MULTIPLIER, 3f);
                put(Prop.ALIGN_LEFT_LOWER, 0.4f);
                put(Prop.ALIGN_LEFT_UPPER, 0.9f);
                put(Prop.ALIGN_RIGHT_LOWER, 0.5f);
                put(Prop.ALIGN_RIGHT_UPPER, 0.9f);
                put(Prop.CRANE_LIFT_SPEED, 1f);
                put(Prop.GRIPPER_CLOSE_POS, 0.0f);
                put(Prop.GRIPPER_OPEN_POS, 0.3f);
                put(Prop.ICR_GRIPPER_CLOSE_COLOR, 0.28f);
                put(Prop.ICR_GRIPPER_OPEN_COLOR, 0.6f);
                put(Prop.HS_MAX_POS, 0.08f);
                put(Prop.WR_MAX_POS, 0.8218f);
                put(Prop.WR_MIN_POS, 0.6789f);
                put(Prop.WRIST_SPEED_DAMPEN, 220.0f);
                put(Prop.VERTICAL_SLOW_DAMPEN, 0.06f);
                put(Prop.WRIST_45_AMOUNT, 0.025f);
                put(Prop.UPPER_WRIST_UP_POS, 1.0f);
                put(Prop.UPPER_WRIST_DOWN_POS, 0.5f);
                put(Prop.UPPER_WRIST_DELAY_SECONDS, 1f);
                put(Prop.PRESET_CROSS_VS, 0f);
                put(Prop.PRESET_TRIANGLE_VS, 0f);
                put(Prop.PRESET_CIRCLE_VS, 0f);
                put(Prop.PRESET_SQUARE_VS, 0f);
                put(Prop.VS_AUTO_SLOWDOWN_THRESHOLD, 1000f);
                put(Prop.UPPER_WRIST_MID_POS, 0.75f);
                put(Prop.GRIPPER_NARROW_OPEN_POSITION, 1.5f);
                put(Prop.UPPER_GRIPPER_OPEN, 0.3f);
                put(Prop.UPPER_GRIPPER_CLOSE, 0.0f);
                put(Prop.ICR_GRIPPER_MID_COLOR, 0.44f);
                put(Prop.LOWER_WRIST_STARTING_POS, 0.5f);
                put(Prop.HS_MIN_POS, 0.0f);
                put(Prop.HS_STARTING_POS, 0.26f);
                put(Prop.CAMLIGHT_BRIGHTNESS, 0.5f);
                put(Prop.VERTICAL_HOLD, 0.1f);
            }
        };
        // END Properties
    
        private boolean adjustmentMode = false;
        private boolean logviewMode = false;
    
        private int lastLLMode = 0;
        private String llMessageManifest = null;
        private float llMessageTime = 0;
        private long lastTimeNs = 0;
        private HashMap<String, ArrayList<Point3>> llNumbers = new HashMap<String, ArrayList<Point3>>() {
            {
                put("Blue", new ArrayList<>());
                put("Yellow", new ArrayList<>());
                put("Red", new ArrayList<>());
            }
        };
    
        private boolean driving = false;
        private boolean gripped_l = false;
        private boolean gripped_u = false;
        private boolean openTight = false;
        private double drive_speed = 0.0;
        private boolean hold_hang = false;
        private boolean hsInMotion = false;
        private float HSmoveTime = 0f;
    
        //Button Presses
        private final float TRIGGER_DEADZONE = 0.02f;
        private final float JOYSTICK_THRESHOLD = 0.1f;
        private boolean rb_2_depr = false;
        private boolean dpd_1_depr = false;
        private boolean rtg_2_depr = false;
        private boolean ltg_2_depr = false;
        private boolean lb_2_depr = false;
        private boolean crs_2_depr = false;
        private boolean ccl_2_depr = false;
        private boolean dpl_2_depr = false;
        private boolean dpr_2_depr = false;
        private boolean dpd_2_depr = false;
        private boolean dpu_2_depr = false;
        private boolean dpl_1_depr = false;
    
        private boolean ljs_1_frwd = false;
        private boolean ljs_1_bkwd = false;
        private boolean ljs_1_lft = false;
        private boolean ljs_1_rgt = false;
        private boolean rjs_1_lft = false;
        private boolean rjs_1_rgt = false;
    
    
        private boolean lightsDisabled = false;
        private boolean camLightDisabled = true;
        private int autoGrabbingStage = 0;
    
        /// Setup and init hardware vars.
        private void setup() {
            timer = new ElapsedTime();
    
            FL = hardwareMap.get(DcMotor.class, "FL");
            FR = hardwareMap.get(DcMotor.class, "FR");
            BL = hardwareMap.get(DcMotor.class, "BL");
            BR = hardwareMap.get(DcMotor.class, "BR");
            HORIZONTAL_SLIDE = hardwareMap.get(CRServo.class, "HS"); //Horizontal Slide
            LOWER_WRIST = hardwareMap.get(Servo.class, "WR"); //Wrist
            LOWER_GRIPPER = hardwareMap.get(Servo.class, "GR2"); //Gripper
            UPPER_GRIPPER = hardwareMap.get(Servo.class, "GR"); //Gripper
            //            AS[0] = hardwareMap.get(Servo.class, "AS1"); //Whatever this does
            //             AS[1] = hardwareMap.get(Servo.class, "AS2"); //Whatever this does
            TILT1 = hardwareMap.get(DcMotor.class, "H1"); //Hanging
    //        TILT2 = hardwareMap.get(DcMotor.class, "H2");
            VERTICAL_SLIDE_1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "VS1"); //Vertical Slides
            VERTICAL_SLIDE_2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "VS2");
    //                LIMELIGHT = hardwareMap.get(Limelight3A.class, "LL"); //Limelight
            BLINKIN = hardwareMap.get(RevBlinkinLedDriver.class, "BLD"); //Blinkin
            GRIP_INDICATOR = hardwareMap.get(Servo.class, "ICR"); //Indicator Light
            AE = hardwareMap.get(AnalogInput.class, "AE"); //Analog Input
            SLIDE_TOUCH = hardwareMap.get(TouchSensor.class, "TS");
            UPPER_WRIST = hardwareMap.get(Servo.class, "UW");
            CAMLIGHT = hardwareMap.get(DcMotor.class, "CL");
    
            FL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    
            VERTICAL_SLIDE_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            VERTICAL_SLIDE_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //            VS1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //            VS2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            VERTICAL_SLIDE_1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            VERTICAL_SLIDE_2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    
            telemetry.setMsTransmissionInterval(10);
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    
            //initEOCV();
    
            wrPos = PROPERTIES.get(Prop.LOWER_WRIST_STARTING_POS);
            hsPos = PROPERTIES.get(Prop.HS_STARTING_POS);
    //                LIMELIGHT.pipelineSwitch(0);
    //                LIMELIGHT.start();
        }
    
        private final Runnable DRIVE_FUNCTIONS = () -> {
            while (opModeIsActive()) {
                drive();
            }
        };
    
        private final Thread DRIVE_THREAD = new Thread(DRIVE_FUNCTIONS);
    
        @Override
        public void runOpMode() throws InterruptedException {
            setup();
            try {
                loadProperties();
            } catch (IOException e) {
                telemetry.addLine("Warning: Properties failed the load. Will revert to defaults.");
                telemetry.addLine("__________________________________");
                telemetry.addLine();
            }
    
            //induvidualTelemetry(VERSION);
            telemetry.addLine("Ready to go!");
            telemetry.addLine();
            telemetry.addLine("v" + VERSION);
            telemetry.addLine();
            telemetry.addLine("*PLEASE PRESENT LUCKY FISH*");
            telemetry.update();
    
            waitForStart();
    
            if (gamepad1.triangle) {
                log("Initializing into a wheel test.");
                perWheelTest();
            }
            if (gamepad1.square) {
                log("Initializing into a slide test.");
                slideTest();//lucas pls keep its simplier and easier to test
            }
            if (gamepad1.cross) {
                log("Initializing into a servo test.");
                perServoTest();//lucas pls keep its simplier and easier to test
            }
    
            timer.reset();
            DRIVE_THREAD.start();
            while (opModeIsActive()) {
                deltaLoop(Math.abs(timer.nanoseconds() - lastTimeNs) / 1000000000d);
                lastTimeNs = timer.nanoseconds();
    
    
                if(pipelineSelection) {
                    selectPipeline();
                    continue;
                }
    
                if (gamepad1.share) {
                    adjustmentMode = !adjustmentMode;
                    try {
                        saveProperties();
                    } catch (IOException e) {
                        throw new RuntimeException(e);
                    }
                    while (gamepad1.share) ;
                }
    
                if (gamepad2.share) {
                    logviewMode = !logviewMode;
                    while (gamepad2.share) ;
                }
    
                if (adjustmentMode) {
                    adjus();
                    continue;
                }
    
                //drive();
                lgrip();
                ugrip();
                crane();
                recog();
                light();
                tilte();
                align();
    
                rtg_2_depr = gamepad2.right_trigger > TRIGGER_DEADZONE;
                ltg_2_depr = gamepad2.left_trigger > TRIGGER_DEADZONE;
                ljs_1_lft = gamepad1.left_stick_x < -JOYSTICK_THRESHOLD;
                ljs_1_rgt = gamepad1.left_stick_x > JOYSTICK_THRESHOLD;
                ljs_1_bkwd = gamepad1.left_stick_y < -JOYSTICK_THRESHOLD;
                ljs_1_frwd = gamepad1.left_stick_y > JOYSTICK_THRESHOLD;
                lb_2_depr = gamepad2.left_bumper;
                crs_2_depr = gamepad2.cross;
                ccl_2_depr = gamepad2.circle;
                dpl_2_depr = gamepad2.dpad_left;
                dpr_2_depr = gamepad2.dpad_right;
                rb_2_depr = gamepad2.right_bumper;
                dpu_2_depr = gamepad2.dpad_up;
                dpd_2_depr = gamepad2.dpad_down;
                dpl_1_depr = gamepad1.dpad_left;
                //            runLaterHandling();
    
                if (logviewMode) {
                    telemetry.clear();
                    solog();
                } else {
                    telemetry.addData("Analog Input Voltage", AE.getVoltage());
                    telemetry.addData("Touchpad", gamepad1.touchpad_finger_1_x + ", " + gamepad1.touchpad_finger_1_y);
                    telemetry.update();
                }
            }
    
            induvidualTelemetry("Program ended. Log saved.");
            try {
                saveLog();
            } catch (IOException e) {
                induvidualTelemetry("Program ended. Failed to save log.");
            }
            sched.shutdown();
            //            HS.setPosition(0);
        }
    
        /// Like a normal loop, but also provides a delta for timing control
        private void deltaLoop(double delta) {
    
            if (llMessageTime > 0) {
                llMessageTime -= (float) delta;
            }
    
            if (qv_time > 0) {
                qv_time -= delta;
                if (approxEquals(qv_time, 0.1)) {
                    qv_time = 0;
                    qv_locked = !qv_locked;
                }
            }
    
            if (gamepad2.circle) {
                HSmoveTime += delta;
            }
    //
    //        if (uwCooldown > 0) {
    //            uwCooldown -= delta;
    //            if (uwCooldown <= 0) {
    //                log("Updating the upper wrist.");
    //                UPPER_WRIST.setPosition(gripped ? PROPERTIES.get(Prop.UPPER_WRIST_UP_POS) : PROPERTIES.get(Prop.UPPER_WRIST_DOWN_POS));
    //            }
    //        }
    
        }
    
        /// Handles the driving/moving the robot.
        private void drive() {
    
            //        float walk = -(gamepad1.touchpad_finger_1 ? gamepad1.touchpad_finger_1_y : 0);
            //        float strafe = (gamepad1.touchpad_finger_1 ? gamepad1.touchpad_finger_1_x : 0);
            //        float pivot = (gamepad1.touchpad_finger_2 ? gamepad1.touchpad_finger_2_x : 0);
    
    //        if (gamepad1.left_stick_y > JOYSTICK_THRESHOLD && !ljs_1_frwd) {
    //            log("Driving forward.");
    //        }
    //        if (gamepad1.left_stick_y < -JOYSTICK_THRESHOLD && !ljs_1_bkwd) {
    //            log("Driving backward.");
    //        }
    //        if (gamepad1.left_stick_y < -JOYSTICK_THRESHOLD && !ljs_1_lft) {
    //            log("Strafing left.");
    //        }
    //        if (gamepad1.left_stick_y > JOYSTICK_THRESHOLD && !ljs_1_rgt) {
    //            log("Strafing right.");
    //        }
    //        if (gamepad1.right_stick_x > JOYSTICK_THRESHOLD && !rjs_1_rgt) {
    //            log("Rotating right.");
    //        }
    //        if (gamepad1.right_stick_x < -JOYSTICK_THRESHOLD && !rjs_1_lft) {
    //            log("Rotating left.");
    //        }
    
            float walk = -gamepad1.left_stick_y;
            float strafe = gamepad1.left_stick_x;
            float pivot = gamepad1.right_stick_x;
            float dampen = gamepad1.right_trigger;
            float boost = gamepad1.left_trigger + 1;
    
            float fl = walk + strafe;
            float fr = walk - strafe;
            float bl = walk - strafe;
            float br = walk + strafe;
    
            float bst_mult = (float) PROPERTIES.get(Prop.SPEED_BOOST_MULTIPLIER);
            float dmpn_mult = (float) PROPERTIES.get(Prop.SPEED_DAMPEN_MULTIPLIER);
    
            fl += pivot;
            fr -= pivot;
            bl += pivot;
            br -= pivot;
    
            fl /= (dampen * dmpn_mult) + 1;
            fr /= (dampen * dmpn_mult) + 1;
            bl /= (dampen * dmpn_mult) + 1;
            br /= (dampen * dmpn_mult) + 1;
    
            fl *= boost * bst_mult;
            fr *= boost * bst_mult;
            bl *= boost * bst_mult;
            br *= boost * bst_mult;
    
            FL.setPower(fl);
            FR.setPower(fr);
            BL.setPower(bl);
            BR.setPower(br);
    
            driving = fl != 0 && fr != 0 && bl != 0 && br != 0;
            drive_speed = Math.sqrt(Math.pow(strafe, 2) + Math.pow(walk, 2));
        }
    
        float hsPos = 0;
        float ugrPos = 0;
        float lgrPos = 0;
        float wrPos = 0.64f;
        float uwPos = 0.0f;
    
        private void align() {
            final float ALL = PROPERTIES.get(Prop.ALIGN_LEFT_LOWER);
            final float ALU = PROPERTIES.get(Prop.ALIGN_LEFT_UPPER);
            final float ARL = PROPERTIES.get(Prop.ALIGN_RIGHT_LOWER);
            final float ARU = PROPERTIES.get(Prop.ALIGN_RIGHT_UPPER);
    
            //            AS[1].setPosition(!gamepad1.square ? ALL : ALU); //Left
            //            AS[0].setPosition(!gamepad1.square ? ARL : ARU); //Right
    
            //            telemetry.addData("Alignment Servos", AS[0].getPosition() + ", " + AS[1].getPosition());
            telemetry.addLine();
        }
    
        /// Handles crane operations: Lifting, ...
        private void crane() {
            telemetry.addLine();
            final float liftSpeed = PROPERTIES.get(Prop.CRANE_LIFT_SPEED);
            //Lifting the crane:
    
            if (gamepad2.cross && !crs_2_depr && !gamepad2.right_bumper) {
                log("Vertical slide moving downwards.");
            }
            if (gamepad2.circle && !ccl_2_depr) {
                log("Vertical slide moving upwards.");
            }
    //        if (gamepad2.left_bumper && !lb_2_depr) {
    //            log("Vertical slide slowmode enabled.");
    //        }
    //        if (!gamepad2.left_bumper && lb_2_depr) {
    //            log("Vertical slide slowmode disabled.");
    //
    //        }
            if(SLIDE_TOUCH.isPressed() && VERTICAL_SLIDE_1.getPower() == 0){
                VERTICAL_SLIDE_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addLine("Touch activated.");
            }else{
                VERTICAL_SLIDE_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
    
            if (!vsPosRunning) {
                float vsSpeedMultiplier = (gamepad2.left_stick_button ? PROPERTIES.get(Prop.VERTICAL_SLOW_DAMPEN) : 1f);
                float liftPower = -gamepad2.right_stick_y;// * liftSpeed * vsSpeedMultiplier;
                if ((SLIDE_TOUCH.isPressed() && (gamepad2.right_stick_y < 0 || vsPosRunning)) && !gamepad1.dpad_down) {
                    liftPower = 0;
                    abortVsSlides();
                }
                liftPower = -gamepad2.right_stick_y;
                VERTICAL_SLIDE_1.setPower(liftPower);
                VERTICAL_SLIDE_2.setPower(liftPower);
                telemetry.addData("VS Pos", VERTICAL_SLIDE_1.getCurrentPosition());
                telemetry.addData("VS Speed", VERTICAL_SLIDE_1.getPower() + " (" + liftPower + ")");
            }
    
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                abortVsSlides();
                autoGrabbingStage = 0;
            }
            runVsSlidesPositionally();
    
            if (gamepad2.cross && !gamepad2.right_bumper) {
                setVsTarget(PROPERTIES.get(Prop.PRESET_CROSS_VS));
            } else if (gamepad2.circle) {
                setVsTarget(PROPERTIES.get(Prop.PRESET_CIRCLE_VS));
            } else if (gamepad2.triangle) {
                setVsTarget(PROPERTIES.get(Prop.PRESET_TRIANGLE_VS));
            } else if (gamepad2.square) {
                setVsTarget(PROPERTIES.get(Prop.PRESET_SQUARE_VS));
            }
    
            if (gamepad2.dpad_left && !dpl_2_depr) {
                log("Horizontal slide moving in.");
            }
            if (gamepad2.dpad_right && !dpr_2_depr) {
                log("Horizontal slide moving out.");
            }
    
            float hsPower = PROPERTIES.get(Prop.HS_SPEED);
            HORIZONTAL_SLIDE.setPower(gamepad2.dpad_left ? hsPower : (gamepad2.dpad_right ? -hsPower : 0));
            hsInMotion = HORIZONTAL_SLIDE.getPower() != 0;
    
            if (gamepad2.dpad_left) {
                hsPos -= hsPower;
                hsInMotion = true;
            } else if (gamepad2.dpad_right) {
                hsPos += hsPower;
                hsInMotion = true;
            } else {
                hsInMotion = false;
            }
    
            hsPos = (float) clamp(hsPos, PROPERTIES.get(Prop.HS_MIN_POS), PROPERTIES.get(Prop.HS_MAX_POS));
    //        HORIZONTAL_SLIDE.setPosition(hsPos);
    //        if (gamepad2.right_bumper){// && !rb_2_depr) {
    //            while(gamepad2.right_bumper);
    //            gripped = !gripped;
    ////            uwCooldown = PROPERTIES.get(Prop.UPPER_WRIST_DELAY_SECONDS);
    //            log("Toggled gripper.");
    //        }
    
    //        telemetry.addData("Horizontal Slide", hsPos + "(" + HS.getPosition() + ")");
            telemetry.addData("HS Power", HORIZONTAL_SLIDE.getPower());
    //        telemetry.addData("HS Pos", hsPos + "(" + HORIZONTAL_SLIDE.getPosition() + ")");
            telemetry.addData("Upper Gripped", gripped_u);
            telemetry.addData("Lower Gripped", gripped_l);
            telemetry.addData("Upper Gripper", ugrPos + "(" + UPPER_GRIPPER.getPosition() + ")");
            telemetry.addData("Lower Gripper", lgrPos + "(" + LOWER_GRIPPER.getPosition() + ")");
            telemetry.addData("Lower Wrist", wrPos + "(" + LOWER_WRIST.getPosition() + ")");
            telemetry.addData("Upper Wrist", uwPos + "(" + UPPER_WRIST.getPosition() + ")");
        }
    
        boolean wristReversed = false;
    
        /// Lower gripper mechanism operation.
        private void lgrip() {
            float wrMax = PROPERTIES.get(Prop.WR_MAX_POS);
            float wrMin = PROPERTIES.get(Prop.WR_MIN_POS);
            float inc = PROPERTIES.get(Prop.WRIST_45_AMOUNT);
            if (gamepad2.right_trigger > TRIGGER_DEADZONE && !gamepad2.right_bumper) {
                while (gamepad2.right_trigger > TRIGGER_DEADZONE) ;
                if (wristReversed) {
                    log("Wrist bumped down.");
                    wrPos -= inc;
                    if (wrPos <= wrMin) {
                        log("Wrist limit reached, reversing.");
                        wristReversed = false;
                    }
                } else {
                    log("Wrist bumped up.");
                    wrPos += inc;
                    if (wrPos >= wrMax) {
                        log("Wrist limit reached, reversing.");
                        wristReversed = true;
                    }
                }
            }
    //        if (gamepad2.left_trigger > TRIGGER_DEADZONE) {
    //            while(gamepad2.left_trigger > TRIGGER_DEADZONE);
    //            if (wrPos <= wrMin) {
    //                gamepad2.rumble(1, 0.1, 200);
    //                log("Failed to bump wrist left; limit reached.");
    //            } else {
    //                wrPos -= inc;
    //                log("Bump wrist left.");
    //            }
    //        } else if (gamepad2.right_trigger > TRIGGER_DEADZONE) {
    //            while(gamepad2.right_trigger > TRIGGER_DEADZONE);
    //            if (wrPos >= wrMax) {
    //                gamepad2.rumble(1, 0.1, 200);
    //                log("Failed to bump wrist right; limit reached.");
    //            } else {
    //                wrPos += inc;
    //                log("Bump wrist right.");
    //            }
    //        }
    
            if (gamepad2.left_trigger > TRIGGER_DEADZONE) {
                log("Gripper Toggled");
                if (gripped_l) {
                    gripped_l = false;
                    openTight = true;
                    while (gamepad2.left_trigger > TRIGGER_DEADZONE) {
                        if (gamepad2.left_trigger > 0.7) {
                            openTight = false;
                        }
                    }
                } else {
                    gripped_l = true;
                    while (gamepad2.left_trigger > TRIGGER_DEADZONE) ;
                }
            }
    
            wrPos = (float) clamp(wrPos, wrMin, wrMax);
    
            final float grOpen = PROPERTIES.get(Prop.GRIPPER_OPEN_POS);
            final float grOpenNarrow = PROPERTIES.get(Prop.GRIPPER_NARROW_OPEN_POSITION);
            final float grClose = PROPERTIES.get(Prop.GRIPPER_CLOSE_POS);
            lgrPos = gripped_l ? grClose : grOpen;
    
            LOWER_GRIPPER.setPosition(lgrPos);
            LOWER_WRIST.setPosition(wrPos);
        }
    
        int ugrip_stage = 0;
    
        /// Upper gripper mechanism operation.
        private void ugrip() {
            telemetry.addData("Upper Wrist Stage", ugrip_stage);
    
            if (gamepad2.left_bumper) {
                while (gamepad2.left_bumper) ;
                log("Upper wrist button pressed.");
                ugrip_stage = cycle(ugrip_stage, 1);
    
                final float DELAY = PROPERTIES.get(Prop.UPPER_WRIST_DELAY_SECONDS);
                if (ugrip_stage == 1 && gripped_u) {
                    log("Jumping back to state 0 to allow for the gripper to open.");
                    ugrip_stage = 0;
                }
                switch (ugrip_stage) {
                    case 0:
                        log("Transitioning the upper wrist to the DOWN & OPEN state. (0)");
                        gripped_u = false;
                        uwPos = (PROPERTIES.get(Prop.UPPER_WRIST_DOWN_POS));
                        break;
                    case 1:
                        log("Transitioning the upper wrist to the UP & CLOSED state. (1)");
                        gripped_u = true;
                        queueLater(DELAY, () -> {
                            uwPos = (PROPERTIES.get(Prop.UPPER_WRIST_UP_POS));
                            log("Delay passed, moving upper wrist.");
                        });
                        break;
                    case 2: //This stage is currently deprecated and will be skipped.
                        log("Transitioning the upper wrist to the MIDDLE & OPEN state. (2)");
                        uwPos = PROPERTIES.get(Prop.UPPER_WRIST_MID_POS);
                        queueLater(DELAY, () -> {
                            gripped_u = false;
                            log("Delay passed, moving upper gripper.");
                        });
                        break;
                }
            }
    
            final float ugrOpen = PROPERTIES.get(Prop.UPPER_GRIPPER_OPEN);
            final float ugrClose = PROPERTIES.get(Prop.UPPER_GRIPPER_CLOSE);
            ugrPos = gripped_u ? ugrClose : ugrOpen;
    
            UPPER_WRIST.setPosition(uwPos);
            UPPER_GRIPPER.setPosition(ugrPos);
        }
    
        private void tilte() {
            telemetry.addLine();
    
            if (gamepad1.triangle) {
                if (gamepad2.dpad_up && !dpu_2_depr) {
                    log("Tilting");
                }
                if (gamepad2.dpad_down && !dpd_2_depr) {
                    log("Untilting");
                }
    //            float VS_SPEED = (gamepad2.left_bumper ? PROPERTIES.get(Prop.VERTICAL_SLOW_DAMPEN) : 1f);
                float hsPower = (gamepad2.dpad_up ? 1 : (gamepad2.dpad_down ? -1 : 0));// * VS_SPEED;
                TILT1.setPower(hsPower);
    //            TILT2.setPower(hsPower);
            }

            if(!gamepad2.dpad_down && !gamepad2.dpad_up){
                TILT1.setPower(0);
            }
    
    //                if (gamepad2.left_bumper) {
    //                    hold_hang = true;
    //                }
        }
    
        /// Camera recognizing of colored blocks.
        @Deprecated
        private void DEPEDrecog() {
            //LL.pipelineSwitch(lastLLMode);
            telemetry.addLine();
    
            LLResult res = LIMELIGHT.getLatestResult();
            if (res != null) {
                if (res.isValid()) {
                    llMessageManifest = resColors[lastLLMode];
                    llMessageTime = 2;
                    telemetry.addData("Target X", res.getTx());
                    telemetry.addData("Target Y", res.getTy());
                    telemetry.addData("Target A", res.getTa());
    
                    List<Point3> lst = llNumbers.get(llMessageManifest);
                    lst.add(0, new Point3(res.getTx(), res.getTy(), res.getTa()));
                    if (lst.size() > (float) PROPERTIES.get(Prop.COLOR_MEAN_RANGE)) {
                        lst.remove((float) PROPERTIES.get(Prop.COLOR_MEAN_RANGE));
                    }
    
                    double xmean = 0;
                    double ymean = 0;
                    double amean = 0;
                    for (Point3 pt : lst) {
                        xmean += pt.x;
                        ymean += pt.y;
                        amean += pt.z;
                    }
                    xmean /= lst.size();
                    ymean /= lst.size();
                    amean /= lst.size();
    
                    telemetry.addData("TX Mean", xmean);
                    telemetry.addData("TY Mean", ymean);
                    telemetry.addData("TA Mean", amean);
                }
            }
    
    
            telemetry.addData("Current LL Test", resColors[lastLLMode]);
    
            if (llMessageTime > 0) {
                telemetry.addData("Limelight Alert!", llMessageManifest);
            }
            //lastLLMode = cycle(lastLLMode, 2);
            if (gamepad2.options) {
                lastLLMode = cycle(lastLLMode, 2);
                sleep(40);
            }
        }
    
        private void light() {
            telemetry.addLine();
            telemetry.addData("Light Power", CAMLIGHT.getPower());
            
            if (gamepad1.dpad_down && !dpd_1_depr) {
                lightsDisabled = !lightsDisabled;
            }
//            if (gamepad1.dpad_left && !dpl_1_depr) {
//                camLightDisabled = !camLightDisabled;
//            }
            dpd_1_depr = gamepad1.dpad_down;
    
//            CAMLIGHT.setPower(camLightDisabled || !gamepad2.right_bumper ? 0 : PROPERTIES.get(Prop.CAMLIGHT_BRIGHTNESS));
            if (lightsDisabled) {
                GRIP_INDICATOR.setPosition(0);
            } else {
                GRIP_INDICATOR.setPosition(gripped_l ? PROPERTIES.get(Prop.ICR_GRIPPER_CLOSE_COLOR) : PROPERTIES.get(Prop.ICR_GRIPPER_OPEN_COLOR)); //`PWM` light
            }
    
            if (lightsDisabled) {
                telemetry.addData("Blinkin State: ", "Disabled");
                BLINKIN.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            } else if (TILT1.getPower() != 0) {
                telemetry.addData("Blinkin State: ", "H1/H2 Powers");
                BLINKIN.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            } else if (hsInMotion) { //HS Prompting
                telemetry.addData("Blinkin State: ", "HS in Motion");
                BLINKIN.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (VERTICAL_SLIDE_1.getPower() != 0 || VERTICAL_SLIDE_2.getPower() != 0) { //VS Prompting
                telemetry.addData("Blinkin State: ", "VS Power");
                BLINKIN.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else if (driving && drive_speed > 0.7) { //Driving Fast
                telemetry.addData("Blinkin State: ", "High Speed Movement");
                BLINKIN.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST);
            } else { // Default
                telemetry.addData("Blinkin State: ", "Defualt");
                BLINKIN.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
            }
        }
    
        private int adjustmentIndex = 0;
        private Prop adjust_selected = null;
        private final int UI_BTN_PRESS_DELAY = 40;
        private final char[] unselected_numbers = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '.', '-'};
        private final String[] selected_numbers = {"1️⃣", "2️⃣", "3️⃣", "4️⃣", "5️⃣", "6️⃣", "7️⃣", "8️⃣", "9️⃣", "0️⃣", "⊡", "➖"};
        private int adjust_keypadIndex = 0;
        private String adjust_input = "";
    
        private void adjus() {
            telemetry.addLine("Adjustment Mode");
            telemetry.addLine("_______________");
            telemetry.addLine();
            if (adjust_selected == null) {
    //            List<Map.Entry<Prop, Float>> sortedEntries = new ArrayList<>(PROPERTIES.entrySet());
    //            sortedEntries.sort(Comparator.comparing(e -> e.getKey().name())); // DOESN'T WORK
    
                Prop current = null;
                int i = 0;
                for (Prop k : getPropertiesSorted()) {
                    Float v = PROPERTIES.get(k);
                    telemetry.addLine((adjustmentIndex == i ? ">" : " ") + k.toString() + ": " + v.toString());
                    if (i == adjustmentIndex) {
                        current = k;
                    }
                    i++;
                }
                telemetry.addLine("_______________");
                telemetry.addLine();
                telemetry.addLine("↕ Navigate    ✖ Select    ≡ Exit");
    
                //sleep(UI_BTN_PRESS_DELAY);
                if (gamepad1.dpad_up) {
                    adjustmentIndex--;
                    while (gamepad1.dpad_up) ;
                } else if (gamepad1.dpad_down) {
                    adjustmentIndex++;
                    while (gamepad1.dpad_down) ;
                } else if (gamepad1.cross) {
                    adjust_selected = current;
                    adjust_input = "";
                    adjust_keypadIndex = 0;
                    while (gamepad1.cross) ;
                }
            } else {
                boolean cloningAllowed = false;
                if (adjust_selected == Prop.PRESET_CIRCLE_VS ||
                        adjust_selected == Prop.PRESET_CROSS_VS ||
                        adjust_selected == Prop.PRESET_SQUARE_VS ||
                        adjust_selected == Prop.PRESET_TRIANGLE_VS) {
                    cloningAllowed = true;
                }
                telemetry.addLine("Editing: " + adjust_selected);
                telemetry.addLine();
                telemetry.addLine("Enter Value:");
                telemetry.addLine(adjust_input + "_");
                telemetry.addLine();
                String keypad = "";
                for (int i = 0; i < 12; i++) {
                    keypad += " ";
                    if (i == adjust_keypadIndex) {
                        keypad += selected_numbers[i];
                    } else {
                        keypad += unselected_numbers[i];
                    }
                }
                telemetry.addLine(keypad);
                telemetry.addLine("_______________");
                telemetry.addLine();
                telemetry.addLine("↔ Navigate    ❌ Select    🔺 Backspace    🔴 Return" + (cloningAllowed ? "    🟥 Clone from Device" : ""));
    
                if (gamepad1.dpad_left) {
                    adjust_keypadIndex--;
                    while (gamepad1.dpad_left) ;
                } else if (gamepad1.dpad_right) {
                    adjust_keypadIndex++;
                    while (gamepad1.dpad_right) ;
                } else if (gamepad1.circle) {
                    PROPERTIES.put(adjust_selected, Float.parseFloat(adjust_input));
                    adjust_selected = null;
                    adjust_input = "";
                    adjust_keypadIndex = 0;
                    while (gamepad1.circle) ;
                } else if (gamepad1.cross) {
                    adjust_input += unselected_numbers[adjust_keypadIndex];
                    while (gamepad1.cross) ;
                } else if (gamepad1.triangle) {
                    adjust_input = adjust_input.substring(0, adjust_input.length() - 1);
                    while (gamepad1.triangle) ;
                } else if (gamepad1.square) {
                    if (cloningAllowed) {
                        if (adjust_selected == Prop.PRESET_CIRCLE_VS ||
                                adjust_selected == Prop.PRESET_CROSS_VS ||
                                adjust_selected == Prop.PRESET_SQUARE_VS ||
                                adjust_selected == Prop.PRESET_TRIANGLE_VS) {
                            adjust_input = String.valueOf(VERTICAL_SLIDE_1.getCurrentPosition());
                        }
                    }
                }
            }
            telemetry.update();
    
            adjustmentIndex = (int) clamp(adjustmentIndex, 0, PROPERTIES.size());
        }
    
        ///Handling of run later tasks. Ensures it's they're time to run.
        //    private void runLaterHandling(){
        //        double currentTime = timer.milliseconds();
        //
        //        // Create an iterator for the entry set of the hashmap
        //        Iterator<Map.Entry<Runnable, Double>> iterator = runLaters.entrySet().iterator();
        //
        //        while (iterator.hasNext()) {
        //            Map.Entry<Runnable, Double> entry = iterator.next();
        //            Runnable task = entry.getKey();
        //            Double scheduledTime = entry.getValue();
        //
        //            // Check if the current time is greater than or equal to the scheduled time
        //            if (currentTime >= scheduledTime) {
        //                // Run the task
        //                task.run();
        //                // Remove the entry from the hashmap
        //                iterator.remove();
        //            }
        //        }
        //    }
    
        /// A test that will run each will uniquely on the robot.
        private void perWheelTest() {
            induvidualTelemetry("Welcome to the individual wheel test!");
            telemetry.speak("Welcome to the individual wheel test!");
            sleep(1000);
            induvidualTelemetry("Testing: FL");
            telemetry.speak("Front Left");
            FL.setPower(1);
            sleep(3000);
            FL.setPower(0);
            sleep(1000);
            induvidualTelemetry("Testing: FR");
            telemetry.speak("Front Right");
            FR.setPower(1);
            sleep(3000);
            FR.setPower(0);
            sleep(1000);
            induvidualTelemetry("Testing: BL");
            telemetry.speak("Back Left");
            BL.setPower(1);
            sleep(3000);
            BL.setPower(0);
            sleep(1000);
            induvidualTelemetry("Testing: BR");
            telemetry.speak("Back Right");
            BR.setPower(1);
            sleep(3000);
            BR.setPower(0);
            sleep(1000);
            induvidualTelemetry("Test Complete");
            telemetry.speak("Test Complete");
            sleep(3000);
        }
    
        private void perServoTest() {
            induvidualTelemetry("Welcome to the individual servo test!");
            telemetry.speak("Welcome to the individual servo test!");
            sleep(1000);
            induvidualTelemetry("Testing: HS");
            telemetry.speak("Horizontal slide");
    //        HORIZONTAL_SLIDE.setPosition(1);
            HORIZONTAL_SLIDE.setPower(1);
            sleep(3000);
    //        HORIZONTAL_SLIDE.setPosition(0);
            HORIZONTAL_SLIDE.setPower(0);
            sleep(1000);
            induvidualTelemetry("Testing: WR");
            telemetry.speak("Wrist");
            LOWER_WRIST.setPosition(1);
            sleep(3000);
            LOWER_WRIST.setPosition(0);
            sleep(1000);
            induvidualTelemetry("Testing: GR");
            telemetry.speak("Gripper");
            LOWER_GRIPPER.setPosition(1);
            sleep(3000);
            LOWER_GRIPPER.setPosition(0);
            sleep(1000);
            induvidualTelemetry("Test Complete");
            telemetry.speak("Test Complete");
            sleep(3000);
        }
    
        /// A test that will run each will uniquely on the robot.
        private void slideTest() {
            induvidualTelemetry("slide testing");
            telemetry.speak("Welcome to the slide test!");
            sleep(1000);
            induvidualTelemetry("Testing: up");
            telemetry.speak("going up");
            VERTICAL_SLIDE_1.setPower(1);
            VERTICAL_SLIDE_2.setPower(1);
            sleep(50);
            VERTICAL_SLIDE_1.setPower(0);
            VERTICAL_SLIDE_2.setPower(0);
            sleep(1000);
            induvidualTelemetry("Testing: Down");
            telemetry.speak("going down");
            VERTICAL_SLIDE_1.setPower(-1);
            VERTICAL_SLIDE_2.setPower(-1);
            sleep(50);
            VERTICAL_SLIDE_1.setPower(0);
            VERTICAL_SLIDE_2.setPower(0);
            sleep(1000);
            induvidualTelemetry("Test Complete");
            telemetry.speak("Test Complete");
            sleep(3000);
        }
    
        //Loop function for showing program log
        private void solog() {
            for (int i = ACTION_LOG.size() - 1; i >= 0; i--) {
                telemetry.addLine(ACTION_LOG.get(i));
            }
            telemetry.update();
        }
    
        ///////////////////////////////////////////////// UTILITIES ////////////////////////////////////////////////
    
        /// Shows just one line of telemetry.
        private void induvidualTelemetry(String line) {
            telemetry.addLine(line);
            telemetry.update();
        }
    
        /// Cycles an int back to zero if it exceeds the max.
        private int cycle(int input, int max) {
            input++;
            if (input > max) {
                input = 0;
            }
            return input;
        }
    
        /// Cycles a float back to zero if it exceeds the max.
        private float cycle(float input, float max) {
            input++;
            if (input > max) {
                input = 0;
            }
            return input;
        }
    
        /// Clamp a value within a range.
        private double clamp(double val, double min, double max) {
            return Math.max(min, Math.min(val, max));
        }
    
        /// Move a servo's position up or down based on its button presses.
        private float servoInc(Servo S, float value, boolean upBtn, boolean dnBtn) {
            final float INC = 0.03f;
            if (upBtn) {
                value += INC;
                sleep(10);
            } else if (dnBtn) {
                value -= INC;
                sleep(10);
            }
            value = (float) clamp(value, 0f, 1f);
            S.setPosition(value);
            return value;
        }
    
        boolean qv_locked = false;
    
        double qv_time = -1;
    
        private void quickVibrate(double lp, double hp, Gamepad pad) {
            if (qv_locked) {
                qv_time = 0.3f;
            } else {
                pad.rumble(lp, hp, 0);
                qv_time = 0.1;
            }
        }
    
        private boolean approxEquals(double a, double b) {
            final double DEVIATION = 0.08;
            return Math.abs(a - b) <= DEVIATION;
        }
    
        private double mean(double... set) {
            double sum = 0;
            for (double num : set) {
                sum += num;
            }
            return sum / set.length;
        }
    
        /// Queues a runnable to run later.
        private void queueLater(double seconds, Runnable run) {
            sched.schedule(run, (long) (seconds * 1000), TimeUnit.MILLISECONDS);
        }
    
        private Prop[] getPropertiesSorted(){
            Prop[] props = PROPERTIES.keySet().toArray(new Prop[0]);
    
            for(int x = 0; x < props.length - 1; x++){
                for(int y = 0; y < props.length - 1; y++){
                    if(props[x].name().compareTo(props[y].name()) > 0){
                        Prop temp = props[x];
                        props[x] = props[y];
                        props[y] = temp;
                    }
                }
            }
    
            return props;
        }
    
        ///////////////////////////////////// OTHER RANDOM STUFF //////////////////////////////////////////
    
        /// Saves the log.
        private void saveLog() throws IOException {
            File d = new File(LOG_DIR);
            if (!d.exists()) {
                d.mkdirs();
            }
            File f = new File(LOG_DIR + "/" + new SimpleDateFormat("dd-MM-yyyy_HHmmss").format(Calendar.getInstance().getTime()).toString() + ".txt");
            f.createNewFile();
            BufferedWriter wr = new BufferedWriter(new FileWriter(f));
            for (String l : ACTION_LOG) {
                wr.write(l + "\n");
            }
            wr.close();
        }
    
        private void saveProperties() throws IOException {
            File f = new File(DATA_DIR);
            if (!f.exists()) {
                f.mkdirs();
            }
            BufferedWriter wr = new BufferedWriter(new FileWriter(DATA_SAVE));
            PROPERTIES.forEach((k, v) -> {
                try {
                    wr.write(k.toString() + ":" + v + "\n");
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            });
            wr.close();
        }
    
        private void loadProperties() throws IOException {
    //                ArrayList<String> load_log = new ArrayList<>();
            BufferedReader rd = new BufferedReader(new FileReader(DATA_SAVE));
            while (true) {
                String line = rd.readLine();
    //                    load_log.add("Read Line: " + line);
                if (line == null) break;
                String[] split = line.split(":");
                String key = split[0];
                float value = Float.parseFloat(split[1]);
                try {
                    try {
                        if (!PROPERTIES.containsKey(Prop.valueOf(key))) {
                            continue;
                        }
                        PROPERTIES.put(Prop.valueOf(key), value);
                    } catch (IllegalArgumentException e) {
                        continue;
                    }
                } catch (IndexOutOfBoundsException e) {
                    continue;
                }
    //                    load_log.add("Key: " + key);
    //                    load_log.add("Value: " + value);
    //                    load_log.add("Enum: " + Prop.values()[key]);
            }
    //                telemetry.addLine("Finished loading; here's the log. Press 🔴 to continue.");
    //                for(String s : load_log){
    //                    telemetry.addLine(s);
    //                }
    //                telemetry.update();
    //                while(!gamepad1.circle || !gamepad2.circle);
        }
    
        //Appends something to the log
        private void log(String info) {
            info = "[" + Calendar.getInstance().get(Calendar.HOUR) + ":" + Calendar.getInstance().get(Calendar.MINUTE) + ":" + Calendar.getInstance().get(Calendar.SECOND) + " (" + timer.seconds() + " seconds in)] " + info;
            ACTION_LOG.add(info);
        }
    
        boolean vsPosRunning = false;
        int vsTargetPos = 0;
        int vsStartingPos = 0;
    
        private void setVsTarget(float pos) {
            VERTICAL_SLIDE_1.setPower(0);
            VERTICAL_SLIDE_2.setPower(0);
            vsStartingPos = VERTICAL_SLIDE_1.getCurrentPosition();
            vsTargetPos = (int) pos;
            vsPosRunning = true;
        }
    
        private void runVsSlidesPositionally() {
            if (vsPosRunning) {
                telemetry.addLine("Running to position.");
                int threshold = Math.round(PROPERTIES.get(Prop.VS_AUTO_SLOWDOWN_THRESHOLD));
                float speed = PROPERTIES.get(Prop.CRANE_LIFT_SPEED);
                int dist = Math.abs(vsTargetPos - VERTICAL_SLIDE_1.getCurrentPosition());
                if (vsTargetPos > vsStartingPos) {
                    if (dist < threshold) {
                        speed *= ((float) dist / (float) threshold);
                    }
                    VERTICAL_SLIDE_1.setPower(speed);
                    VERTICAL_SLIDE_2.setPower(speed);
                    if (vsTargetPos <= vsStartingPos) {
                        abortVsSlides();
                    }
                } else if (vsTargetPos < vsStartingPos) {
                    if (dist < threshold) {
                        speed *= ((float) dist / (float) threshold);
                    }
                    VERTICAL_SLIDE_1.setPower(-speed);
                    VERTICAL_SLIDE_2.setPower(-speed);
                    if (vsTargetPos >= vsStartingPos) {
                        abortVsSlides();
                    }
                }
                if (SLIDE_TOUCH.isPressed()) {
                    abortVsSlides();
                    telemetry.addLine("Touch activated -- aborting VS autodrive.");
                }
            }
        }
    
        private void abortVsSlides() {
            if (vsPosRunning) {
                VERTICAL_SLIDE_1.setPower(0);
                VERTICAL_SLIDE_2.setPower(0);
                vsPosRunning = false;
                autoGrabbingStage = 0;
            }
        }
    
        /////// OPEN CV THINGS //////// ------------------------------------------------------------------------
        //       _____   ____    ___    _   _
        //      /  __/  / _ /   /  _ / | | / /
        //     / _/_   / _ /   / /_    | |/ /
        //    /___/   /___/   /___/    |___/
    
    
        BlueSampleHSV bluePipeline = new BlueSampleHSV();
        RedSampleHSV redPipeline = new RedSampleHSV();
        TeleOpPipeline activePipe;
    
        boolean pipelineSelection = true;
        private void selectPipeline(){
            telemetry.addLine("Blue or Red?");
            telemetry.addLine("← For 🔵 Blue");
            telemetry.addLine("→ For 🔴 Red");
            telemetry.update();
    
            if(gamepad2.dpad_left){
                activePipe = bluePipeline;
                pipelineSelection = false;
                initEOCV();
            }
            if(gamepad2.dpad_right){
                activePipe = redPipeline;
                pipelineSelection = false;
                initEOCV();
            }
        }
    
        private void initEOCV(){
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    
            webcam.setPipeline((OpenCvPipeline) activePipe);
            webcam.setMillisecondsPermissionTimeout(5000);
    
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
    
                    webcam.getFocusControl().setMode(FocusControl.Mode.Fixed);
                    webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
                    webcam.getExposureControl().setExposure(33, TimeUnit.MILLISECONDS);
                    webcam.getGainControl().setGain(10);
                    webcam.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.MANUAL);
                    webcam.getWhiteBalanceControl().setWhiteBalanceTemperature(5000);
    
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
    
    
                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Camera Error", errorCode);
                    telemetry.update();
                }
            });
    
    
            //webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }
    
        boolean recogOnCooldown = false;
    
        private void recog(){
    
            telemetry.addLine();
            try{
                telemetry.addData("Current Pipeline", activePipe.getName());
                Point position = activePipe.getPosition();
                telemetry.addData("Center", (position != null) ? position.toString() : "No target detected");
                double angle = activePipe.getAngle();
                telemetry.addData("Angle", (angle != -2000) ? angle : "No target detected");
    
                if (gamepad2.right_bumper && autoGrabbingStage == 0){
                    if(!recogOnCooldown){
                        rotateWristToBlock(angle);
                        recogOnCooldown = true;
                        CAMLIGHT.setPower(PROPERTIES.get(Prop.CAMLIGHT_BRIGHTNESS));
                        queueLater(1.0, () -> {
                            recogOnCooldown = false;
                            CAMLIGHT.setPower(0);
                        });
                    }
                }
                if(gamepad2.right_stick_button && autoGrabbingStage == 0){
                    autoGrabbingStage = 1;
                    float vsPower = PROPERTIES.get(Prop.VERTICAL_HOLD);
                    VERTICAL_SLIDE_1.setPower(-vsPower);
                    VERTICAL_SLIDE_2.setPower(-vsPower);
                    camLightDisabled = false;
                    CAMLIGHT.setPower(PROPERTIES.get(Prop.CAMLIGHT_BRIGHTNESS));
                    rotateWristToBlock(angle);
                    queueLater(0.3, () -> {
                        autoGrabbingStage = 2;
                        queueLater(1, () -> {
                            if(autoGrabbingStage == 2){
                                autoGrabbingStage = 3;
                                setVsTarget(0);
                                log("Been in stage 2 for too long. Advancing to auto-grabbing stage 3.");
                                CAMLIGHT.setPower(0);
                            }
                        });
                        log("Attempting auto-grab: Stage 2.");
                    });
                    log("Attempting auto-grab: Stage 1.");
                    gripped_l = false;
                }
                if(autoGrabbingStage == 2){
                    final int BUFFER = 10;
                    final float SPEED = PROPERTIES.get(Prop.HS_SPEED);
                    if(position.y > 120 - BUFFER){
                        HORIZONTAL_SLIDE.setPower(SPEED);
                    }else if(position.y < 120 + BUFFER){
                        HORIZONTAL_SLIDE.setPower(-SPEED);
                    }else{
                        autoGrabbingStage = 3;
                        setVsTarget(0);
                        camLightDisabled = true;
                        log("Auto-grab: Stage 3");
                    }
                }
            }catch(IndexOutOfBoundsException e){
                telemetry.addLine("Failed to report CV data: Exception caught");
            }catch(NullPointerException e){
                telemetry.addLine("Oven CV unavailable: A pipeline is not selected or a value in NULL.");
                if(gamepad2.right_bumper){
                    gamepad2.rumble(0.5, 0.5, 50);
                    queueLater(0.1, () -> {
                        gamepad2.rumble(0.5, 0.5, 50);
                    });
                    while(gamepad2.right_bumper);
                }
            }
            switch (autoGrabbingStage){
                case 1:
                case 2:
                    break; //Refer to about 10 lines about here.
                case 3:
                    if(VERTICAL_SLIDE_1.getCurrentPosition() <= 4 || SLIDE_TOUCH.isPressed()){
                        abortVsSlides();
                        autoGrabbingStage = 4;
                        log("Auto-grab: Stage 4");
                    }
                    break;
                case 4:
                    gripped_l = true;
                    autoGrabbingStage = 0;
                    log("Auto-grab attempt complete.");
                    break;
            }
    
        }
    
        private void rotateWristToBlock(double angle){
            double camAngle = 1 - (wrPos * 180);
            double inAngle = camAngle + angle - 90;
            inAngle = (((inAngle % 180) + 180) % 180);
    
            telemetry.addData("Inner Angle", inAngle);
            telemetry.update();
            wrPos = (float) (1 - (inAngle / 180));
        }
    
        private TeleOpPipeline findBestPipeline(){
            if(bluePipeline.getAngle() <= -2000 || bluePipeline.getPosition() == null){
                if(redPipeline.getAngle() <= -2000 || redPipeline.getPosition() == null){
                    return null;
                }
                return redPipeline;
            }
            return bluePipeline;
        }
    
        interface TeleOpPipeline{
            public Point getPosition();
            public double getAngle();
            public String getName();
        }
    
        class BlueSampleHSV extends OpenCvPipeline implements TeleOpPipeline{
            public Scalar lowerHSV = new Scalar(51, 60, 148, 0.0);
            public Scalar upperHSV = new Scalar(130.0, 255.0, 254.6, 255.0);
            public int kernelSize = 5;
            public int dilate = 1;
            public int erode = 2;
            private Mat hsvBinaryMat = new Mat();
            private Mat inputMask = new Mat();
            private Mat output = new Mat();
            private ArrayList<MatOfPoint> contours = new ArrayList<>();
            private Mat hierarchy = new Mat();
            public MatOfPoint biggestContour = null;
            private MatOfPoint2f points2f = new MatOfPoint2f();
            private ArrayList<RotatedRect> rotRects = new ArrayList<>();
    
    
            @Override
            public Mat processFrame(Mat input) {
                if (input.empty())
                    return input;
    
                Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
    
                inputMask.release();
                output.release();
                hsvBinaryMat.release();
    
                Core.inRange(input, lowerHSV, upperHSV, hsvBinaryMat);
    
                Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(kernelSize, kernelSize));
    
                for (int i = 0; i < erode; i++) {
                    Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
                    Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
                    Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
                }
                for (int i = 0; i < dilate; i++) {
                    Imgproc.dilate(hsvBinaryMat, hsvBinaryMat, kernel);
                    Imgproc.dilate(hsvBinaryMat, hsvBinaryMat, kernel);
                    Imgproc.dilate(hsvBinaryMat, hsvBinaryMat, kernel);
                }
    
                Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
    
                Core.bitwise_and(input, input, output, hsvBinaryMat);
    
                contours.clear();
                hierarchy.release();
    
                Imgproc.findContours(hsvBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
    
                // Clear any previous "biggestContour"
                biggestContour = null;
    
                // Desired ratio ~ 2.33 (3.5 : 1.5)
                double targetRatio = 3.5 / 1.5;
    
                // Variables to track best contour
                double bestRatioDiff = Double.MAX_VALUE;
                double bestArea = 0;
                MatOfPoint bestContour = null;
    
                for (MatOfPoint contour : contours) {
                    if (contour == null)
                        continue;
    
                    double area = Imgproc.contourArea(contour);
                    // Skip small contours
                    if (area < 100)
                        continue;
    
                    // Convert to points2f to compute minAreaRect
                    MatOfPoint2f tempPoints2f = new MatOfPoint2f();
                    contour.convertTo(tempPoints2f, CvType.CV_32F);
    
                    // Create the rotated rect
                    RotatedRect rect = Imgproc.minAreaRect(tempPoints2f);
    
                    // Compute ratio = max(width, height) / min(width, height)
                    double w = rect.size.width;
                    double h = rect.size.height;
                    if (w < 1e-5 || h < 1e-5)
                        continue;
                    double ratio = Math.max(w, h) / Math.min(w, h);
    
                    // Compare how close ratio is to targetRatio
                    double ratioDiff = Math.abs(ratio - targetRatio);
    
                    // If ratio is closer OR same ratio but bigger area => update best
                    if (ratioDiff < bestRatioDiff) {
                        bestRatioDiff = ratioDiff;
                        bestArea = area;
                        bestContour = contour;
                    } else if (Math.abs(ratioDiff - bestRatioDiff) < 1e-5) {
                        // ratio is effectively the same; check area
                        if (area > bestArea) {
                            bestArea = area;
                            bestContour = contour;
                        }
                    }
                }
    
                // Update biggestContour to whichever is best
                biggestContour = bestContour;
    
                rotRects.clear();
                if (biggestContour != null && biggestContour.toArray().length >= 3) {
                    biggestContour.convertTo(points2f, CvType.CV_32F);
                    rotRects.add(Imgproc.minAreaRect(points2f));
                }
    
                Mat output = new Mat();
                Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 2);
    
                input.copyTo(output);
    
                for (RotatedRect rect : rotRects) {
                    if (rect != null) {
                        Point[] rectPoints = new Point[4];
                        rect.points(rectPoints);
                        Imgproc.polylines(output, Collections.singletonList(new MatOfPoint(rectPoints)), true,
                                new Scalar(0, 255, 0), 2);
                    }
                }
    
                if (!rotRects.isEmpty()){
                    Imgproc.circle(output, rotRects.get(0).center, 6, new Scalar(0, 255, 0), -1);
                }
                Imgproc.cvtColor(output, output, Imgproc.COLOR_HSV2RGB);
    
                return output;
            }
    
            public Point getPosition() {
                return (!rotRects.isEmpty()) ? rotRects.get(0).center : null;
            }
            public double getAngle() {
                if (rotRects.isEmpty()){
                    return -2000;
                }
    
    
                double subAngle = (!rotRects.isEmpty()) ? rotRects.get(0).angle : -2000;
                Size size = (!rotRects.isEmpty()) ? rotRects.get(0).size : null;
                if (subAngle != -2000 && size != null && size.height !=0 && size.width != 0){
                    if (size.width > size.height){
                        return subAngle;
                    } else {
                        return subAngle + 90;
                    }
                } else {
                    return -2000;
                }
            }
    
            @Override
            public String getName() {
                return "Blue";
            }
        }
    
        class RedSampleHSV extends OpenCvPipeline implements TeleOpPipeline {
            // Define two HSV ranges for red.
            // Lower red range (hue: 0-10)
            public Scalar lowerRed1 = new Scalar(0, 155, 236);
            public Scalar upperRed1 = new Scalar(10, 255, 255);
            // Upper red range (hue: 170-180)
            public Scalar lowerRed2 = new Scalar(170, 70, 50);
            public Scalar upperRed2 = new Scalar(180, 255, 255);
    
    
            public int kernelSize = 2;
            public int dilate = 2;
            public int erode = 0;
    
    
            private Mat hsvBinaryMat = new Mat();
            private Mat inputMask = new Mat();
            private Mat output = new Mat();
            private ArrayList<MatOfPoint> contours = new ArrayList<>();
            private Mat hierarchy = new Mat();
            public MatOfPoint biggestContour = null;
            private MatOfPoint2f points2f = new MatOfPoint2f();
            private ArrayList<RotatedRect> rotRects = new ArrayList<>();
    
    
            @Override
            public Mat processFrame(Mat input) {
                if (input.empty())
                    return input;
    
    
                // Convert input image from RGB to HSV color space.
                Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
    
    
                // Release previous mats (if any)
                inputMask.release();
                output.release();
                hsvBinaryMat.release();
    
    
                // Create two masks for the red ranges
                Mat mask1 = new Mat();
                Mat mask2 = new Mat();
                Core.inRange(input, lowerRed1, upperRed1, mask1);
                Core.inRange(input, lowerRed2, upperRed2, mask2);
                // Combine the two masks into one.
                Core.bitwise_or(mask1, mask2, hsvBinaryMat);
                mask1.release();
                mask2.release();
    
    
                hsvBinaryMat.copyTo(inputMask);
    
    
                // Create a structuring element for morphological operations.
                Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(kernelSize, kernelSize));
    
    
                // Apply the mask to the original input.
                Core.bitwise_and(input, input, output, hsvBinaryMat);
    
    
                Imgproc.cvtColor(output, output, Imgproc.COLOR_HSV2RGB);
                Imgproc.cvtColor(output, output, Imgproc.COLOR_RGB2GRAY);
    
    
                Imgproc.Canny(output, hsvBinaryMat, 50, 150);
    
    
                Imgproc.morphologyEx(hsvBinaryMat, hsvBinaryMat, Imgproc.MORPH_CLOSE, kernel);
    
    
                for (int i = 0; i < erode; i++) {
                    Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
                    Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
                    Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
                }
                for (int i = 0; i < dilate; i++) {
                    Imgproc.dilate(hsvBinaryMat, hsvBinaryMat, kernel);
                    Imgproc.dilate(hsvBinaryMat, hsvBinaryMat, kernel);
                    Imgproc.dilate(hsvBinaryMat, hsvBinaryMat, kernel);
                }
                Core.bitwise_not(hsvBinaryMat, hsvBinaryMat);
    
    
                Core.bitwise_and(hsvBinaryMat, inputMask, hsvBinaryMat);
    
    
                Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
                Imgproc.dilate(hsvBinaryMat, hsvBinaryMat, kernel);
                Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
    
    
                // Prepare for contour detection.
                contours.clear();
                hierarchy.release();
                Imgproc.findContours(hsvBinaryMat, contours, hierarchy,
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
    
    
                // Clear any previous "biggestContour"
                biggestContour = null;
    
    
                // Desired ratio ~ 2.33 (3.5 : 1.5)
                double targetRatio = 3.5 / 1.5;
                double bestRatioDiff = Double.MAX_VALUE;
                double bestArea = 0;
                MatOfPoint bestContour = null;
    
    
                // Iterate through found contours to identify the best match.
                for (MatOfPoint contour : contours) {
                    if (contour == null)
                        continue;
                    double area = Imgproc.contourArea(contour);
                    // Skip very small contours.
                    if (area < 100)
                        continue;
    
    
                    // Convert contour to a floating-point representation.
                    MatOfPoint2f tempPoints2f = new MatOfPoint2f();
                    contour.convertTo(tempPoints2f, CvType.CV_32F);
    
    
                    // Compute the minimum area rectangle for the contour.
                    RotatedRect rect = Imgproc.minAreaRect(tempPoints2f);
    
    
                    // Compute ratio = max(width, height) / min(width, height)
                    double w = rect.size.width;
                    double h = rect.size.height;
                    if (w < 1e-5 || h < 1e-5)
                        continue;
                    double ratio = Math.max(w, h) / Math.min(w, h);
    
    
                    double ratioDiff = Math.abs(ratio - targetRatio);
    
    
                    // Update best contour based on ratio closeness and area.
                    if (ratioDiff < bestRatioDiff) {
                        bestRatioDiff = ratioDiff;
                        bestArea = area;
                        bestContour = contour;
                    } else if (Math.abs(ratioDiff - bestRatioDiff) < 1e-5) {
                        if (area > bestArea) {
                            bestArea = area;
                            bestContour = contour;
                        }
                    }
                }
    
    
                // Save the best contour.
                biggestContour = bestContour;
    
    
                // Compute rotated rectangle for the best contour.
                rotRects.clear();
                if (biggestContour != null && biggestContour.toArray().length >= 3) {
                    biggestContour.convertTo(points2f, CvType.CV_32F);
                    rotRects.add(Imgproc.minAreaRect(points2f));
                }
    
    
                // For visualization, draw all contours and the best rotated rectangle.
                Mat output = new Mat();
                Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 2);
                input.copyTo(output);
                for (RotatedRect rect : rotRects) {
                    if (rect != null) {
                        Point[] rectPoints = new Point[4];
                        rect.points(rectPoints);
                        Imgproc.polylines(output, Collections.singletonList(new MatOfPoint(rectPoints)), true,
                                new Scalar(0, 255, 0), 2);
                    }
                }
                if (!rotRects.isEmpty()) {
                    Imgproc.circle(output, rotRects.get(0).center, 6, new Scalar(0, 255, 0), -1);
                }
                // Convert back to RGB for display.
                Imgproc.cvtColor(output, output, Imgproc.COLOR_HSV2RGB);
    
    
                return output;
            }
    
    
            public Point getPosition() {
                return (!rotRects.isEmpty()) ? rotRects.get(0).center : null;
            }
    
    
            public double getAngle() {
                if (rotRects.isEmpty()) {
                    return -2000;
                }
                double subAngle = (!rotRects.isEmpty()) ? rotRects.get(0).angle : -2000;
                Size size = (!rotRects.isEmpty()) ? rotRects.get(0).size : null;
                if (subAngle != -2000 && size != null && size.height != 0 && size.width != 0) {
                    if (size.width > size.height) {
                        return subAngle;
                    } else {
                        return subAngle + 90;
                    }
                } else {
                    return -2000;
                }
            }
    
            @Override
            public String getName() {
                return "Red";
            }
        }
    
    }
