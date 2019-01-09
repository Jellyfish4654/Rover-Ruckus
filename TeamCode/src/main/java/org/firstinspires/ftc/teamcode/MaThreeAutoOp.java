package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "Ma3 Auto Op")
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

        waitForStart();


        // START

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        turn(90);
        sleep(10000);
    }

    private void setPower(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    private void drive(double distance) {
        // 2240 ticks per motor rotation, 40 tooth motor sprocket, 20 tooth wheel sprockets, 90mm wheels
        while(!isStopRequested()) {
            driveTelemetry();
        }
    }

    private void driveTelemetry() {
        telemetry.addData("Right Motor Position: ", rightDrive.getCurrentPosition());
        telemetry.update();
    }

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
        double dir = -Math.abs(delta) / delta;

        turnTelemetry(initialAngle, currentAngle, targetAngle, delta, lastDelta, dir);
        sleep(1000);

        while (!isStopRequested() && !(delta * dir <= 0 && lastDelta * dir >= 0)) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            lastDelta = delta;
            delta = getDelta(targetAngle, currentAngle);
            double speed = 0.2 * (0.2 + (Math.max(10, Math.min(80, Math.abs(delta))) - 10.0) / 70.0 * 0.8);
            setPower(dir * speed, -dir * speed);
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
