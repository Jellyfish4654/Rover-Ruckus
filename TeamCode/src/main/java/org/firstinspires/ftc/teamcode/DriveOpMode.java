package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "JelleTele")
public class DriveOpMode extends OpMode {

    DcMotor left, right, rack;
    DcMotor slurp;
    DcMotor extend;
    //    DcMotor latch;
    State state = State.DRIVE;

//    Servo marker;

    enum State {
        DRIVE,
        REVERSE,
        TANK
    }

    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        rack = hardwareMap.dcMotor.get("rack");

        slurp = hardwareMap.dcMotor.get("slurp");
        extend = hardwareMap.dcMotor.get("extend");
//        latch = hardwareMap.dcMotor.get("latch");
//        marker = hardwareMap.servo.get("marker");

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        rack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        // DRIVE CONTROLS

        double driveMult = gamepad1.left_bumper ? 0.5 : (gamepad1.right_bumper ? 0.2 : 1.0);

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        switch (state) {
            case DRIVE:
                left.setPower(driveMult * (y - x));
                right.setPower(driveMult * (y + x));
                break;
            case REVERSE:
                left.setPower(driveMult * -(y + x));
                right.setPower(driveMult * -(y - x));
                break;
            case TANK:
                left.setPower(driveMult * gamepad1.left_stick_y);
                right.setPower(driveMult * gamepad1.right_stick_y);
                break;
        }

        if (gamepad1.dpad_up) {
            state = State.DRIVE;
        } else if (gamepad1.dpad_down) {
            state = State.REVERSE;
        } else if (gamepad1.dpad_left) {
            state = State.TANK;
        }

        // AUX CONTROLS

        double auxMult = gamepad2.left_bumper ? 0.5 : (gamepad2.right_bumper ? 0.2 : 1.0);

        rack.setPower(auxMult * ((gamepad2.dpad_up ? 1 : 0) + (gamepad2.dpad_down ? -1 : 0)));

        // extend.setPower(auxMult * gamepad2.left_stick_y);
        slurp.setPower(auxMult * gamepad2.right_stick_y);

//        latch.setPower(mult * gamepad2.right_stick_x);

        if (gamepad2.a) {
            rack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // TELEMETRY

        telemetry.addData("Drive Mode: ", state);
        telemetry.addData("Rack Encoder: ", rack.getCurrentPosition());
    }
}
