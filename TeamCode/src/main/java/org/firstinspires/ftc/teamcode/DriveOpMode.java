package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "JelleTele")
public class DriveOpMode extends IterativeBaseOpMode {

    DcMotor left, right;
    //DcMotor slurp;
    DcMotor extend;
    DcMotor latch;
    State state = State.DRIVE;

    Servo marker;

    public void init() {

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
//        slurp = hardwareMap.dcMotor.get("slurp");
//        extend = hardwareMap.dcMotor.get("extend");

//        latch = hardwareMap.dcMotor.get("latch");

//        marker = hardwareMap.servo.get("marker");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void loop() {
        double mult = gamepad1.left_bumper ? 0.5 : (gamepad1.right_bumper ? 0.2 : 1.0);

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        switch (state) {

            case TANK:
                left.setPower(mult * gamepad1.left_stick_y);
                right.setPower(mult * gamepad1.right_stick_y);
                break;

            case DRIVE:
                left.setPower(mult * (y - x));
                right.setPower(mult * (y + x));
                break;

        }

        if (gamepad1.dpad_up) {
            state = State.DRIVE;
        } else if (gamepad1.dpad_down) {
            state = State.TANK;
        }

//        extend.setPower(gamepad2.left_stick_y);
//        slurp.setPower(gamepad2.right_stick_y);

//        latch.setPower(mult * gamepad2.right_stick_x);
    }
}


