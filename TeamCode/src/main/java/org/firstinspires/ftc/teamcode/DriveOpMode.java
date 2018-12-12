package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "jelletele")
public class DriveOpMode extends IterativeBaseOpMode {

    DcMotor left, right;
    DcMotor slurp, extend;

    State state = State.TANK;

    public void init() {

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        slurp = hardwareMap.dcMotor.get("slurp");
        extend = hardwareMap.dcMotor.get("extend");

    }

    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        switch (state) {

            case TANK:
                left.setPower(gamepad1.left_stick_y);
                right.setPower(gamepad1.right_stick_y);
                break;

            case DRIVE:
                left.setPower(y - x);
                right.setPower(y + x);
                break;


        }

        state = gamepad1.dpad_up ? State.TANK : gamepad1.dpad_down ? State.DRIVE : state;

        extend.setPower(gamepad2.left_stick_y);
        slurp.setPower(gamepad2.right_stick_y);
    }
}


