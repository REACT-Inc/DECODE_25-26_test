import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.*;

@TeleOp(name = "Arm")   //  use Arm config file

public class Arm extends LinearOpMode{

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

            if (false) {
                telemetry.addData("Touchpad", lX + ", " + lY + "," + rX + ", " + rY);
                telemetry.addData("joysticks",s);
                String t = String.format (" %6d %6d %6d %6d",posSh,posEl,posWr,posGr);
                telemetry.addData("pos ",t);
            }
            telemetry.update();
            
            posSh = setServo (SH, posSh, lY, 100);
            posEl = setServo (EL, posEl, -rY, 1000);
            posWr = setServo (WR, posWr, rX, 100);
            posGr = setServo (GR, posGr, lX, 100);
            sleep (20);
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
