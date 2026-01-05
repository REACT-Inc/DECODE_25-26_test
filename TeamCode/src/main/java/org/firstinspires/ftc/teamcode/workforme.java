package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.DigitalIoDeviceConfigurationType;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;
import java.util.Random;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.*;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;



@TeleOp

public class workforme extends LinearOpMode {
    private TouchSensor touch;
    private DcMotor motor;
    private DistanceSensor distance;
    private ColorSensor color;
    private TouchSensor mag;
    private CRServo cont;
    private Servo servo;
    private Servo rbg;
    private GoBildaPrismDriver lights;

    private void rbg(){
            Random random = new Random();
            double value = random.nextDouble();
            rbg.setPosition(value);
            sleep(250);
            
    }   



    @Override
    public void runOpMode() {
        touch = hardwareMap.get(TouchSensor.class, "touch");
        motor = hardwareMap.get(DcMotor.class, "motor");
        distance = hardwareMap.get(DistanceSensor.class, "distance1");
        color = hardwareMap.get(ColorSensor.class, "color");
        mag = hardwareMap.get(TouchSensor.class, "mag");
        cont = hardwareMap.get(CRServo.class, "cont");
        servo = hardwareMap.get(Servo.class, "servo");
        rbg = hardwareMap.get(Servo.class, "rbg");
        lights = hardwareMap.get(GoBildaPrismDriver.class, "lights");


        
      
        

        // Wait for the game to start (driver presses PLAY)
                // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
            

            
            //LLResult result = limelight.getLatestResult();
            //telemetry.addData("?", result.getPipelineType());


                // Get all AprilTags/Fiducials visible to the camera
   
    
     


            

        
            if (touch.isPressed()){
                //Touch Sensor is pressed.
                telemetry.addData("Touch Sensor", "Is Pressed");
                if (distance.getDistance(DistanceUnit.CM) < 20){
                    motor.setPower(distance.getDistance(DistanceUnit.CM)/20);
                } else {
                    motor.setPower(0);
                }   
            } else {
                //Touch Sensor is not pressed 
                telemetry.addData("Touch Sensor", "Is Not Pressed");
                motor.setPower(0);
            }
            if (!mag.isPressed()){
                telemetry.addData("Magnetic", "True");
                cont.setPower(0.5);
            } else{
                telemetry.addData("Magnetic", "False");
                cont.setPower(0);
            }
            if (color.red() == 1){
                servo.setPosition(1);
            } else {
                servo.setPosition(0);
            }
            telemetry.addData("position", rbg.getPosition());
            rbg();
            lights.loadAnimationsFromArtboard(Artboard.ARTBOARD_3);
            telemetry.update();    
            
        }
         

        // run until the end of the match (driver presses STOP)
       
    }
}