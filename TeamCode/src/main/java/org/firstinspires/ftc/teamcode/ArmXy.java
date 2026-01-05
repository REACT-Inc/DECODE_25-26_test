package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.*;

@TeleOp(name = "ArmXy")   //  use Arm config file

public class ArmXy extends LinearOpMode{

    private Servo SH;
    private Servo EL;
    private Servo WR;
    private Servo GR;
    private int posSh =50;
    private int posEl =500;
    private int posWr =50;
    private int posGr =50;
  
    private void setup(){
        SH = hardwareMap.get(Servo.class, "SH");
        EL = hardwareMap.get(Servo.class, "EL");
        WR = hardwareMap.get(Servo.class, "WR");
        GR = hardwareMap.get(Servo.class, "GR");
    }

    @Override
    public void runOpMode()  throws NumberFormatException {
        setup();
       
        waitForStart();

        while(opModeIsActive()){
            float lY = gamepad1.left_stick_y;
            float lX = gamepad1.left_stick_x;
            float rY = gamepad1.right_stick_y;
            float rX = gamepad1.right_stick_x;
            
            String s = String.format (" (%6.2f, %6.2f) (%6.2f, %6.2f)", lX, lY, rX, rY);

            if (true) {
                telemetry.addData("joysticks",s);
                String t = String.format (" %6d %6d %6d %6d",posSh,posEl,posWr,posGr);
                telemetry.addData("pos ",t);
            }
           
            
            double Min = 2;
            double L0  = 4.0;    // inches
            double x   = -2*L0*lY;
            if(x<Min)
                x = Min;
            double ang = Math.acos((x/2)/L0)*180/Math.PI;
            
            double posSh = ang/180;
            double posEl = 0.5+(ang/180/5)*2;
            
            SH.setPosition(posSh);
            EL.setPosition(posEl);
            String u = String.format (" %6.2f %6.2f %6.2f, %6.2f",posSh,posEl,x,ang);
            telemetry.addData("ang ",u);
            telemetry.update();
        }
        

    }
    private int setServo (Servo servo, int pos,double d, int sc){
        if (0.1 < d) {
            if (sc > pos)
                pos ++;
        }
        else if (-0.1 > d){
            if (0 < pos)
            pos --; 
        }
        servo.setPosition (pos*1.0/sc);
         return pos; 
    }
}
