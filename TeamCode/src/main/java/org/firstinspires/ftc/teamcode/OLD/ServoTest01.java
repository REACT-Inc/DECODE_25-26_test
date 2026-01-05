package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Servo Test #479851865")
@Disabled
public class ServoTest01 extends LinearOpMode {

    private Servo HS;
    private Servo WR;
    private Servo GR;

    private ElapsedTime timer;
    private long lastTimeNs;

    private double globalPosition = 0.0;
    private boolean up = true;

    @Override
    public void runOpMode() throws InterruptedException {
        HS = hardwareMap.get(Servo.class, "HS");
        WR = hardwareMap.get(Servo.class, "WR");
        GR = hardwareMap.get(Servo.class, "GR");
        timer = new ElapsedTime();

        telemetry.addLine("Servo Test #479851865 - In & Out: Ready");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            double delta = Math.abs(timer.nanoseconds()-lastTimeNs)/1000000000d;

            if(up){
                globalPosition += delta;
                if(globalPosition > 1){
                    up = false;
                }
            }else {
                globalPosition -= delta;
                if(globalPosition < 0){
                    up = true;
                }
            }

            HS.setPosition(globalPosition);
            WR.setPosition(globalPosition);
            GR.setPosition(globalPosition);

            telemetry.addData("Controller", globalPosition);
            telemetry.addData("HS", HS.getPosition());
            telemetry.addData("WR", WR.getPosition());
            telemetry.addData("GR", GR.getPosition());
            telemetry.update();

            lastTimeNs = timer.nanoseconds();
        }
    }
}
