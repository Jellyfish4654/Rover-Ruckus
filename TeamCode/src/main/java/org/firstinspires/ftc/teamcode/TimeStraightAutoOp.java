//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//@Autonomous(name = "timeStraight")
//public class TimeStraightAutoOp extends IterativeBaseOpMode {
//
//    double startTime;
//    double currentTime = System.currentTimeMillis();
//    double deltaT;
//
//    public void init() {
//    }
//
//    public void start() {
//        startTime = System.currentTimeMillis();
//    }
//
//
//    public void loop() {
//        currentTime = System.currentTimeMillis();
//        deltaT = currentTime-startTime;
//        if (deltaT < 1000) {
//            leftDrive.setPower(1);
//            rightDrive.setPower(1);
//        } else if (deltaT > 2500 && deltaT < 4000) {
//            marker.setPosition(.5);
//            leftDrive.setPower(-.5);
//            rightDrive.setPower(-.5);
//        }
//
//    }
//
//}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "timeStraight")
public class TimeStraightAutoOp extends IterativeBaseOpMode {

    double startTime;
    double currentTime = System.currentTimeMillis();
    double deltaT;

    private long movementTime = 1000;
    private boolean leftForward = true;
    private boolean rightForward = true;

    private final long TIME_INCREMENT = 250;

    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    private boolean leftBumperPressed = false;
    private boolean rightBumperPressed = false;

    public void init() {
        super.init();
    }

    public void init_loop() {
        super.init_loop();

        // If the dpad down button is pressed, decrement the time by TIME_INCREMENT
        if (gamepad1.dpad_down && !dpadDownPressed) {
            dpadDownPressed = true;
            movementTime -= TIME_INCREMENT;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // If the dpad up button is pressed, increment the time by TIME_INCREMENT
        if (gamepad1.dpad_up && !dpadUpPressed) {
            dpadUpPressed = true;
            movementTime += TIME_INCREMENT;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        // If the left bumper is pressed, invert the direction of the left motor
        if (gamepad1.left_bumper && !leftBumperPressed) {
            leftBumperPressed = true;
            leftForward = !leftForward;
        } else if (!gamepad1.left_bumper) {
            leftBumperPressed = false;
        }

        // If the right bumper is pressed, invert the direction of the right motor
        if (gamepad1.right_bumper && !rightBumperPressed) {
            rightBumperPressed = true;
            rightForward = !leftForward;
        } else if (!gamepad1.right_bumper) {
            rightBumperPressed = false;
        }

        telemetry.addData("Time to run", movementTime);
        telemetry.addData("Left forwards", leftForward);
        telemetry.addData("Right forwards", rightForward);
    }

    public void start() {
        super.start();

        startTime = System.currentTimeMillis();
    }


    public void loop() {
        super.loop();

        currentTime = System.currentTimeMillis();
        deltaT = currentTime - startTime;
        if (deltaT < movementTime) {
            leftDrive.setPower(leftForward ? 1 : -1);
            rightDrive.setPower(rightForward ? 1 : -1);
        } else if (deltaT > movementTime + 1500 && deltaT < movementTime + 3000) {
            marker.setPosition(0.5);
            leftDrive.setPower(leftForward ? -0.5 : 0.5);
            rightDrive.setPower(rightForward ? -0.5 : 0.5);
        }

    }

}
