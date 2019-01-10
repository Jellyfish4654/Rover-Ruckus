package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "Ma3 Auto Op - OnBotJava")
public class MaThreeAutoOp extends LinearOpMode {

    DcMotor leftDrive, rightDrive;
    DcMotor latch;

    BNO055IMU imu;

    public void runOpMode() {

        // INITIALIZATION

        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");
//        latch = hardwareMap.dcMotor.get("latch");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        double degrees = 90;
        double inches = 12;

        // Set up variables during init
        while(!isStarted() && !isStopRequested()) {
            degrees += 0.5 * ((gamepad2.dpad_right ? 1 : 0) + (gamepad2.dpad_left ? -1 : 0));
            inches += 0.1 * ((gamepad2.dpad_up ? 1 : 0) + (gamepad2.dpad_down ? -1 : 0));
            initTelemetry(degrees, inches);
        }


        // START

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        drive(30);
        sleep(100);

        drive(-13.5);
        sleep(100);

        turn(-77);
        sleep(100);

        drive(41);
        sleep(100);

        turn(122);
        sleep(100);

        drive(39);
        sleep(100);

        drive(-66);
        sleep(100);
    }

    private void initTelemetry(double degrees, double inches) {
        telemetry.addData("Degrees: ", degrees);
        telemetry.addData("Inches: ", inches);

        telemetry.update();
    }

    private void setPower(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    private void drive(double inches) {
        // 2240 ticks per motor rotation, 40 tooth motor sprocket, 20 tooth wheel sprockets, 90mm wheels

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Voodoo magic 100.54 / 2 gives good value
        int targetPos = (int) (50.27 * inches);
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + targetPos);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + targetPos);

        leftDrive.setPower(0.125);
        rightDrive.setPower(0.125);

        while(!isStopRequested() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            driveTelemetry();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveTelemetry() {
        telemetry.addData("Left Power: ", leftDrive.getPower());
        telemetry.addData("Left Position: ", leftDrive.getCurrentPosition());
        telemetry.addData("Left Target: ", leftDrive.getTargetPosition());
        telemetry.addLine();
        telemetry.addData("Right Power: ", rightDrive.getPower());
        telemetry.addData("Right Position: ", rightDrive.getCurrentPosition());
        telemetry.addData("Right Target: ", rightDrive.getTargetPosition());

        telemetry.update();
    }


    final double turnSpeed = 0.1, startSlow = 50, endSlow = 5, minSlow = 0.15;
    private void turn(double degrees) {
        double initialAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double currentAngle = initialAngle;
        double targetAngle = initialAngle - degrees;
        if (targetAngle > 180)
            targetAngle -= 360;
        else if (targetAngle < -180)
            targetAngle += 360;

        double delta = getDelta(targetAngle, currentAngle);
        double lastDelta = delta;
        double dir = Math.abs(delta) / delta;

        turnTelemetry(initialAngle, currentAngle, targetAngle, delta, lastDelta, dir);

        while (!isStopRequested() && !(delta * dir <= 0 && lastDelta * dir >= 0)) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            lastDelta = delta;
            delta = getDelta(targetAngle, currentAngle);
            double speed = turnSpeed * (minSlow + (Math.max(endSlow, Math.min(startSlow, Math.abs(delta))) - endSlow) / (startSlow - endSlow) * (1 - minSlow));
            setPower(-dir * speed, dir * speed);

            //telemetry.log().add("< Last Delta   - ", lastDelta);
            //telemetry.log().add("  Curr Delta > - ", delta);

            turnTelemetry(initialAngle, currentAngle, targetAngle, delta, lastDelta, dir);
        }
        setPower(0, 0);
    }

    private double getDelta(double target, double current) {
        double delta = target - current;
        if (delta > 180)
            delta -= 360;
        else if (delta < -180)
            delta += 360;
        return delta;
    }

    private void turnTelemetry(double initialAngle, double currentAngle, double targetAngle, double delta, double lastDelta, double dir) {
        Orientation orient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.addData("First Angle: ", orient.firstAngle);
        telemetry.addData("Second Angle: ", orient.secondAngle);
        telemetry.addData("Third Angle: ", orient.thirdAngle);
        telemetry.addData("Initial: ", initialAngle);
        telemetry.addData("Current: ", currentAngle);
        telemetry.addData("Target: ", targetAngle);
        telemetry.addData("Delta: ", delta);
        telemetry.addData("Last Delta: ", lastDelta);
        telemetry.addData("Direction: ", dir);
        telemetry.addData("Stop Requested: ", isStopRequested());
//        telemetry.addData("Position X: ", pos.x);
//        telemetry.addData("Position Y: ", pos.y);
//        telemetry.addData("Position Z: ", pos.z);

        telemetry.update();
    }
}
