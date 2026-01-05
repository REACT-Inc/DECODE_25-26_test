package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.System.PropertiesSystem.PropertyCleaner;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.System.LoggingSystem;
import org.firstinspires.ftc.teamcode.System.PropertiesSystem;
import org.firstinspires.ftc.teamcode.System.TelemetrySystem;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

@TeleOp(name="TestAllProgram", group="Tests")
public class SimpleTestProgram extends LinearOpMode{



    public TelemetrySystem telemetryS;
    @Override
    public void runOpMode() throws InterruptedException{
        telemetryS = new TelemetrySystem(telemetry);
//
        List<DcMotor> dcMotors = hardwareMap.getAll(DcMotor.class);
        List<Servo> servos = hardwareMap.getAll(Servo.class);
        List<CRServo> crServos = hardwareMap.getAll(CRServo.class);
        telemetryS.addLine("Elvate the robot make sure no wheels or spining objects touch the ground!");
        waitForStart();

        while(opModeIsActive()){

            for (DcMotor dcMotor : dcMotors) {
                telemetryS.addLine("Testing DC Motor: " + dcMotor.getDeviceName() + ", PORT:" + dcMotor.getPortNumber() );
                telemetryS.addLine("Connection: " + dcMotor.getConnectionInfo() + ", TYPE:" + dcMotor.getMotorType() );
                telemetryS.addLine("Manufacturer: " + dcMotor.getManufacturer() + ", Controller:" + dcMotor.getController() );
                telemetryS.update();
                sleep(2000);
                telemetryS.addLine("Beginning Test Now!");
                telemetryS.addLine("Testing Servo: " + dcMotor.getDeviceName() + ", PORT:" + dcMotor.getPortNumber() );
                telemetryS.addLine("Connection: " + dcMotor.getConnectionInfo() + "TYPE: " + dcMotor.getMotorType());
                telemetryS.addLine("Manufacturer: " + dcMotor.getManufacturer() + ", Controller:" + dcMotor.getController() );
                telemetryS.update();
                sleep(50);
                dcMotor.setPower(0.1);
                sleep(500);
                dcMotor.setPower(0.5);
                sleep(500);
                dcMotor.setPower(0.7);
                sleep(500);
                dcMotor.setPower(1);
                telemetryS.addLine("Motor Test For Ending in 5 Seconds");
                telemetryS.addLine("Testing Servo: " + dcMotor.getDeviceName() + ", PORT:" + dcMotor.getPortNumber() );
                telemetryS.addLine("Connection: " + dcMotor.getConnectionInfo() + "type:" + dcMotor.getMotorType());
                telemetryS.addLine("Manufacturer: " + dcMotor.getManufacturer() + ", Controller:" + dcMotor.getController() );
                telemetryS.update();
                sleep(5000);
                dcMotor.setPower(0);
                dcMotor.close();
            }
            telemetryS.clearAll();
            telemetryS.addLine("Starting servo test Soon");
            telemetryS.update();
            sleep(7500);
            for (Servo servo : servos) {
                telemetryS.addLine("Testing Servo: " + servo.getDeviceName() + ", PORT:" + servo.getPortNumber() );
                telemetryS.addLine("Connection: " + servo.getConnectionInfo());
                telemetryS.addLine("Manufacturer: " + servo.getManufacturer() + ", Controller:" + servo.getController() );
                telemetryS.update();
                sleep(2000);
                telemetryS.addLine("Beginning Test Now!");
                telemetryS.addLine("Testing Servo: " + servo.getDeviceName() + ", PORT:" + servo.getPortNumber() );
                telemetryS.addLine("Connection: " + servo.getConnectionInfo());
                telemetryS.addLine("Manufacturer: " + servo.getManufacturer() + ", Controller:" + servo.getController() );
                telemetryS.update();
                sleep(50);
                servo.setPosition(0.1);
                sleep(500);
                servo.setPosition(1);
                sleep(500);
                servo.setPosition(-0.7);
                sleep(500);
                servo.setPosition(-1);
                telemetryS.addLine("SERVO Test For Ending in 5 Seconds");

                telemetryS.addLine("Testing Servo: " + servo.getDeviceName() + ", PORT:" + servo.getPortNumber() );
                telemetryS.addLine("Connection: " + servo.getConnectionInfo());
                telemetryS.addLine("Manufacturer: " + servo.getManufacturer() + ", Controller:" + servo.getController() );
                telemetryS.update();
                sleep(5000);
                servo.setPosition(0);
                servo.close();
            }
            telemetryS.clearAll();
            telemetryS.addLine("Starting CRservo test Soon");
            telemetryS.update();
            sleep(7500);
            for (CRServo servo : crServos) {
                telemetryS.addLine("Testing CRServo: " + servo.getDeviceName() + ", PORT:" + servo.getPortNumber() );
                telemetryS.addLine("Connection: " + servo.getConnectionInfo());
                telemetryS.addLine("Manufacturer: " + servo.getManufacturer() + ", Controller:" + servo.getController() );
                telemetryS.update();
                sleep(2000);
                telemetryS.addLine("Beginning Test Now!");
                telemetryS.addLine("Testing Servo: " + servo.getDeviceName() + ", PORT:" + servo.getPortNumber() );
                telemetryS.addLine("Connection: " + servo.getConnectionInfo());
                telemetryS.addLine("Manufacturer: " + servo.getManufacturer() + ", Controller:" + servo.getController() );
                telemetryS.update();
                sleep(50);
                servo.setPower(0.1);
                sleep(500);
                servo.setPower(1);
                sleep(500);
                servo.setPower(-0.7);
                sleep(500);
                servo.setPower(-1);
                telemetryS.addLine("CRSERVO Test For Ending in 5 Seconds");
                telemetryS.addLine("Testing Servo: " + servo.getDeviceName() + ", PORT:" + servo.getPortNumber() );
                telemetryS.addLine("Connection: " + servo.getConnectionInfo());
                telemetryS.addLine("Manufacturer: " + servo.getManufacturer() + ", Controller:" + servo.getController() );
                telemetryS.update();
                sleep(5000);
                servo.setPower(0);
                servo.close();
            }
        }
    }

}
