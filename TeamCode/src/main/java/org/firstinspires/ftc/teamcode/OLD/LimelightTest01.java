package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.hardware.limelightvision.LLResult;
    import com.qualcomm.hardware.limelightvision.Limelight3A;
    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import java.util.ArrayList;

    @TeleOp(name = "Limelight Test #3971009")
    @Disabled

    public class LimelightTest01 extends LinearOpMode {

        private Limelight3A LL;
        private ArrayList<String> detections = new ArrayList<String>();
        private int successes = 0;

        @Override
        public void runOpMode() throws InterruptedException {
            LL = hardwareMap.get(Limelight3A.class, "LL");
            LL.pipelineSwitch(0);
            LL.setPollRateHz(100);

            telemetry.addLine(LL.getDeviceName() + " - : - " + LL.toString() + " - + - " + LL.getStatus() + " - + - " + LL.getConnectionInfo() + " - + - " + LL.isConnected());
            telemetry.update();

            waitForStart();

            telemetry.speak("From the screen to the ring to the pen to the king.");
            LL.start();
            LL.pipelineSwitch(0);
            LL.reloadPipeline();

            while(opModeIsActive()) {
                LL.captureSnapshot("x");
                LLResult res = LL.getLatestResult();
                detections.add(": " + res);
                if (res != null && res.isValid()){
                    successes++;
                    detections.add("X: " + res.getTx() + " Y: " + res.getTy() + " % " + res.getTa());
                }

                telemetry.addData("Successes", successes);
                telemetry.addData("Last Update", LL.getTimeSinceLastUpdate());
                telemetry.addData("Running", LL.isRunning());
                telemetry.addData("Calibration DF", LL.getCalDefault());
                for(int i = 0; i < detections.size(); i++){
                    String d = detections.get(i);
                    telemetry.addLine(i + ") " + d);
                }

                if(detections.size() >= 128){
                    detections.remove(0);
                }

                telemetry.update();

                if(gamepad1.start){
                    detections.clear();
                }
            }
        }
    }
