package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "turnAuto")


public class TurnParkingAutoOp extends IterativeBaseOpMode {

    float distanceToGoOne = 25 * super.GEAR_RATIO;
    float distanceToGoTwo = 60 * super.GEAR_RATIO;
    float theta = (float) Math.PI / 4;
    PIDController pid = new PIDController(theta, 1, 1, 1);
    //25,45,60 in,deg,in respectively

    @Override
    public void init() {
        super.init();
      //  detach(latch);

    }


    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
//        if (distanceToGoOne > 1) {
//            distanceToGoOne = super.moveDistance(distanceToGoOne);
//        } else if (theta > .1) {
//            double strength = super.turnTo(theta, pid);
//            leftDrive.setPower(-strength);
//            rightDrive.setPower(strength);
//            theta -= super.imu.getAngularOrientation().firstAngle;
//        } else if (distanceToGoTwo > 1) {
//            distanceToGoTwo -= super.moveDistance(distanceToGoTwo);
//        } else {
//            dropMarker();
//        }

    }
}
