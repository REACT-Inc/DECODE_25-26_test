import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.*;

@TeleOp(name = "TeleOp24")
public class TeleOp24 extends LinearOpMode{

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private Servo HS;
    private Servo WR;
    private Servo GR;
    private DcMotor H1;
    private DcMotor H2;
    private DcMotor VS1;
    private DcMotor VS2;

    private void setup(){
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        HS = hardwareMap.get(Servo.class, "HS");
        WR = hardwareMap.get(Servo.class, "WR");
        GR = hardwareMap.get(Servo.class, "GR");
        //H1 = hardwareMap.get(DcMotor.class, "H1");
        //H2 = hardwareMap.get(DcMotor.class, "H2");
        //VS1 = hardwareMap.get(DcMotor.class, "VS1");
        //VS2 = hardwareMap.get(DcMotor.class, "VS2");

        FL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void runOpMode() throws InterruptedException{
        setup();

        waitForStart();

        while(opModeIsActive()){
            drive();

            telemetry.addData("Touchpad", gamepad1.touchpad_finger_1_x + ", " + gamepad1.touchpad_finger_1_y);
            telemetry.update();
        }
    }

    private void drive(){

        float walk = gamepad1.left_stick_y;
        float strafe = gamepad1.left_stick_x;
        float pivot = gamepad1.right_stick_x;
        float dampen = gamepad1.right_trigger;

        float fl = walk + strafe;
        float fr = walk - strafe;
        float bl = walk + strafe;
        float br = walk - strafe;

        fl += pivot;
        fr -= pivot;
        bl += pivot;
        br -= pivot;

        fl -= dampen/1.5f;
        fr -= dampen/1.5f;
        bl -= dampen/1.5f;
        br -= dampen/1.5f;

        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);

    }

}
