package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

@Autonomous(name = "Ma3 Auto Op")
public class MaThreeAutoOp extends LinearOpMode {

    DcMotor leftDrive, rightDrive;
    DcMotor rack;

    BNO055IMU imu;

    double baseAngle, trueAngle;

    public void runOpMode() {

        // INITIALIZATION

        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");
        rack = hardwareMap.dcMotor.get("rack");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rack.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int gold = 0;

        // Set up variables during init
        while(!isStarted() && !isStopRequested()) {
            gold = gamepad1.dpad_left ? -1 : (gamepad1.dpad_up ? 0 : (gamepad1.dpad_right ? 1 : gold));
            telemetry.addData("Gold Position: ", gold);
            telemetry.update();
        }


        // START

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        baseAngle = trueAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        land();

//        int gold = sample();


        drive(16.5);
        sleep(100);

        if (gold == 0) {
            drive(13.5);
            sleep(100);
            drive(-13.5);
            sleep(100);
        } else {
            turn(45 * gold == 1 ? 1 : -1);
            sleep(2000);
            drive(20);
            sleep(2000);
            drive(-20);
            sleep(2000);
            turn(-45 * gold == 1 ? 1 : -1);
            sleep(2000);
        }

//        drive(30);
//        sleep(100);
//
//        drive(-13.5);
//        sleep(100);

        turn(-77);
        sleep(100);

        drive(40);
        sleep(100);

        turn(122);
        sleep(100);

        drive(39);
        sleep(1000);

        // TODO: Drop marker

        drive(-66);
        sleep(100);
    }

    private void setPower(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    private void land() {
        rack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rack.setTargetPosition(20000); // TODO: Get correct value

        rack.setPower(1);
        ElapsedTime timer = new ElapsedTime();
        while (!isStopRequested() && rack.isBusy() && timer.seconds() < 10) {
            telemetry.addData("Rack Position: ", rack.getCurrentPosition());
            telemetry.addData("Rack Target: ", rack.getTargetPosition());
            telemetry.update();
        }
        rack.setPower(0);
        rack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turn(-10, 0.6);
        timer.reset();
        rack.setPower(-1);
        while (timer.seconds() < 2);
        rack.setPower(0);
        turn(10);
    }

    private int sample() {
        GoldAlignDetector detector;

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        ElapsedTime timer = new ElapsedTime();
        while(!isStopRequested() && !detector.isFound() && timer.seconds() < 3);

        double x = detector.getXPosition();
        telemetry.addData("X Position: ", x);
        telemetry.update();

        detector.disable();
        return x > 400 ? 1 : (x < 400 ? -1 : 0);
    }

    private void drive(double inches) {
        drive(inches, 0.15);
    }

    private void drive(double inches, double power) {
        // 2240 ticks per motor rotation, 40 tooth motor sprocket, 20 tooth wheel sprockets, 90mm wheels

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Voodoo magic 100.54 / 2 gives good value
        int targetPos = (int) (50.27 * inches);
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + targetPos);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + targetPos);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

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


    private void turn(double degrees) {
        turn(degrees, 0.1);
    }

    final double startSlow = 50, endSlow = 5, minSlow = 0.2;
    private void turn(double degrees, double power) {
        trueAngle -= degrees;

        double initialAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double currentAngle = initialAngle;
        double targetAngle = trueAngle;
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
            double speed = Math.min(1, power * (minSlow + (Math.max(endSlow, Math.min(startSlow, Math.abs(delta))) - endSlow) / (startSlow - endSlow) * (1 - minSlow)));
            setPower(-dir * speed, dir * speed);

            //telemetry.log().add("< Last Delta   - ", lastDelta);
            //telemetry.log().add("  Curr Delta > - ", delta);

            turnTelemetry(initialAngle, currentAngle, targetAngle, delta, lastDelta, dir);
        }
        setPower(0, 0);
    }
    private void vturn(double degrees, double ratio) {
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
            if (ratio > 0){
                setPower(-dir * speed, dir * speed * ratio);
            }
            else if (ratio < 0) {
                setPower(-dir * speed * Math.abs(ratio), dir * speed);
            }

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
