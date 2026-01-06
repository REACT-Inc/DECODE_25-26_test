package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelLogic {

    // ---------------- HARDWARE ----------------
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    // ---------------- TIMING ----------------
    private ElapsedTime stateTimer = new ElapsedTime();

    // ---------------- STATE MACHINE ----------------
    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET_GATE
    }

    private FlywheelState flywheelState = FlywheelState.IDLE;

    // ---------------- CONSTANTS ----------------
    private static final double GATE_CLOSE_TIME = 0.4;
    private static final double GATE_OPEN_TIME = 0.25;

    private static final double GATE_CLOSE_ANGLE = 0.0;
    private static final double GATE_OPEN_ANGLE = 0.5;

    private static final double TARGET_FLYWHEEL_POWER = 1.0;
    private static final double FLYWHEEL_MAX_SPINUP_TIME = 2.0;

    // ---------------- VARIABLES ----------------
    private int shotsRemaining = 0;

    // ---------------- INIT ----------------
    public void init(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotor.class, "1");
        motor2 = hardwareMap.get(DcMotor.class, "2");
        motor3 = hardwareMap.get(DcMotor.class, "3");
        motor4 = hardwareMap.get(DcMotor.class, "4");

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        flywheelState = FlywheelState.IDLE;
    }

    // ---------------- UPDATE LOOP ----------------
    public void update() {
        switch (flywheelState) {

            case IDLE:
                if (shotsRemaining > 0) {
                    motor1.setPower(TARGET_FLYWHEEL_POWER);
                    motor3.setPower(TARGET_FLYWHEEL_POWER);
                    motor2.setPower(TARGET_FLYWHEEL_POWER);
                    motor4.setPower(TARGET_FLYWHEEL_POWER);
                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {
                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;

            case LAUNCH:
                if (stateTimer.seconds() > GATE_OPEN_TIME) {
                    shotsRemaining--;
                    stateTimer.reset();
                    flywheelState = FlywheelState.RESET_GATE;
                }
                break;

            case RESET_GATE:
                if (stateTimer.seconds() > GATE_CLOSE_TIME) {
                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    } else {
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        motor4.setPower(0);
                        flywheelState = FlywheelState.IDLE;
                    }
                }
                break;
        }
    }

    // ---------------- CONTROL METHODS ----------------
    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
        }
    }

    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }
}
