package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class RomansProgram extends LinearOpMode{

DcMotor fl;
    @Override
    public void runOpMode(){
        fl = hardwareMap.get(DcMotor.class,"fl");
        
        
        while(opModeIsActive()){
            if(gamepad1.cross){
                fl.setPower(1);
                
            } else
            {
                fl.setPower(0);
            }
        }
    }
}