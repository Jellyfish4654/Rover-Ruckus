package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

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
    Servo marker;

    BNO055IMU imu;

    double baseAngle, trueAngle;

    public void runOpMode() {

        // INITIALIZATION

        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");
        rack = hardwareMap.dcMotor.get("rack");
        marker = hardwareMap.servo.get("marker");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rack.setDirection(DcMotorSimple.Direction.REVERSE);

        marker.setPosition(1);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        long interWait = 50;
        int gold = 0;
        int drop = 12750, clear = 8850;
        boolean dropPress = false, clearPress = false;
        boolean land = true;
        boolean depotSide = true;


        // Set up variables during init
        while(!isStarted() && !isStopRequested()) {
            gold = gamepad1.dpad_left ? -1 : (gamepad1.dpad_up ? 0 : (gamepad1.dpad_right ? 1 : gold));
            if (!dropPress)
                drop += gamepad2.dpad_up ? 10 : (gamepad2.dpad_down ? -10 : 0);
            if (!clearPress)
                clear += gamepad2.dpad_up ? 10 : (gamepad2.dpad_down ? -10 : 0);
            depotSide = gamepad1.a ? false : (gamepad1.b ? true : depotSide);
            land = gamepad2.a ? false : (gamepad2.b ? true : land);

            dropPress = gamepad2.dpad_up;
            clearPress = gamepad2.dpad_down;

            telemetry.addData("Gold Position: ", gold);
            telemetry.addData("Drop: ", drop);
            telemetry.addData("Clear: ", clear);
            telemetry.addData("depotSide: ", depotSide);
            telemetry.addData("Land: ", land);
            telemetry.addData("Angle: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();
        }


        // START

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        baseAngle = trueAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle + 3;

        if (land) {
            land(drop, clear);
        }

        drive("Getting space to sample", 3);

        gold = sample(0.1);

        drive("Driving to sample turning point", 4);
        sleep(interWait);

        final double endAngle = depotSide ? -55 : -90, sideAngle = 35;
        switch (gold) {
            case 0:
                turn("Checking angle towards center", 0, 0.3, false);
                drive("Pushing center mineral", 18.5);
                sleep(interWait);
                drive("Backing from central mineral", -18.5);
                sleep(interWait);
                break;
            case -1:
            case 1:
                double dir = gold == -1 ? -1 : 1;
                turn("Turning to side mineral", sideAngle * dir, 0.3, false);
                sleep(interWait);
                drive("Pushing side mineral", 22);
                sleep(interWait);
                drive("Backing from side mineral", -22);
                sleep(interWait);
                turn("Realigning turn", -sideAngle * dir);
                sleep(interWait);
                break;
        }

        turn("Turning towards side", endAngle);
        sleep(interWait);

//        if (gold == 0) {
//
//        } else {
//            telemetry.addData("True Angle: ", trueAngle);
//            telemetry.addData("Angle: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
//
//            telemetry.update();
//
//
//        }

//        drive(30);
//        sleep(interWait);
//
//        drive(-13.5);
//        sleep(interWait);


        drive("Driving towards side", 37.5, 0.4);
        sleep(interWait);

        if (depotSide) {
            turn("Turning right towards depot", 45 - endAngle + 15);
            sleep(interWait);
        } else {
            turn("Turning left towards depot", -135 - endAngle);
            sleep(interWait);
        }

        drive("Driving towards depot", depotSide ? 35 : 45, 0.4);
        sleep(interWait);

        turn("Turning to deposit", -27);

        dropMarker(0.6, 750);
        turn("Turning towards crater", 28.5);
        marker.setPosition(1);

        drive("Parking on crater", -70, 0.5);
        sleep(interWait);
    }

    private void autoTelemetry(String phase) {
        telemetry.addLine("Ma3 AutoOp 1.0");
        telemetry.addData("Phase: ", phase);
        telemetry.addLine();
    }

    private void setPower(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    private void land(int drop, int clear) {
        rack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rack.setTargetPosition(drop);

        rack.setPower(1);
        ElapsedTime timer = new ElapsedTime();
        while (!isStopRequested() && rack.isBusy() && timer.seconds() < 10) {
            autoTelemetry("Landing (dropping)");
            telemetry.addData("Rack Position: ", rack.getCurrentPosition());
            telemetry.addData("Rack Target: ", rack.getTargetPosition());
            telemetry.update();
        }
        rack.setPower(0);
        rack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        absoluteTurn("Landing (unhooking)", (baseAngle + 10));
//        turn(-10, 0.25, true);
        timer.reset();
        rack.setPower(-1);
        while (!isStopRequested() && rack.getCurrentPosition() > clear && timer.seconds() < 3);
        rack.setPower(0);
        absoluteTurn("Landing (realigning)", baseAngle);
//        turn(10, 0.2, true);

        //drive(-3);
        //trueAngle = 0;

//        sleep(3);
//        rack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rack.setTargetPosition(0);
//        rack.setPower(1);
//        while (!isStopRequested() && rack.isBusy()) {
//            telemetry.addData("Rack Position: ", rack.getCurrentPosition());
//            telemetry.addData("Rack Target: ", rack.getTargetPosition());
//            telemetry.update();
//        }
    }

    private int sample(double power) {
        turn("Sampling (pre-turn)", 35);

        GoldAlignDetector detector;

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        autoTelemetry("Sampling (inter)");
        telemetry.addData("X Position: ", detector.getXPosition());
        telemetry.addData("True Angle: ", trueAngle);
        telemetry.addData("Angle: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.update();
        sleep(100);


        setPower(power, -power);
        double angle;
        double delta = 0;
//        while(!isStopRequested()) {
//            setPower(power * gamepad1.left_stick_x, -power * gamepad1.left_stick_x);
//
//            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//            delta = Math.abs(angle - trueAngle);
//            telemetry.addData("Delta: ", delta);
//            telemetry.addData("X Position: ", detector.getXPosition());
//            telemetry.addData("True Angle: ", trueAngle);
//            telemetry.addData("Angle: ", angle);
//
//            telemetry.update();
//        }

        while(!isStopRequested() && !detector.isFound() && delta < 50) {
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            delta = Math.abs(angle - trueAngle);
            autoTelemetry("Sampling (searching)");
            telemetry.addData("Delta: ", delta);
            telemetry.update();
        }
        setPower(0, 0);

        detector.disable();

        int gold = delta < 15 ? -1 : (delta > 30 ? 1 : 0);

        double x = detector.getXPosition();
        autoTelemetry("Sampling (found)");
        telemetry.addData("Delta: ", delta);
        telemetry.addData("X Position: ", x);
        telemetry.addData("Gold Position: ", gold);
        telemetry.update();

        sleep(100);

        turn("Sampling (post-turn)", -35);
        return gold;
    }

    private void drive(String phase, double inches) {
        drive(phase, inches, 0.25);
    }

    private void drive(String phase, double inches, double power) {
        // 2240 ticks per motor rotation, 40 tooth motor sprocket, 20 tooth wheel sprockets, 90mm wheels

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Voodoo magic 100.54 / 2 gives good value
        int targetPos = (int) (50.27 * inches);
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + targetPos);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + targetPos);

        setPower(power, power);

        while(!isStopRequested() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            autoTelemetry(phase);
            driveTelemetry();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
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


    private void absoluteTurn(String phase, double degrees) {
        trueAngle = degrees;
        turn(phase, 0);
    }

    private void turn(String phase, double degrees) {
        turn(phase, degrees, 0.4, false);
    }

    final double startSlow = 50, endSlow = 5, minSlow = 0.2;
    private void turn(String phase, double degrees, double power, boolean ignoreCurve) {
        trueAngle -= degrees;

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        while (!isStopRequested() && !(delta * dir <= 0 && lastDelta * dir >= 0)) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            lastDelta = delta;
            delta = getDelta(targetAngle, currentAngle);
            double speed;
            if (!ignoreCurve) {
                speed = Math.min(1, power * (minSlow + (Math.max(endSlow, Math.min(startSlow, Math.abs(delta))) - endSlow) / (startSlow - endSlow) * (1 - minSlow)));
            } else {
                speed = power;
            }
            setPower(-dir * speed, dir * speed);

            autoTelemetry(phase);
            turnTelemetry(speed, initialAngle, currentAngle, targetAngle, delta, lastDelta, dir);
        }
        setPower(0, 0);
    }
//    private void vturn(double degrees, double ratio) {
//        double initialAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//        double currentAngle = initialAngle;
//        double targetAngle = initialAngle - degrees;
//        if (targetAngle > 180)
//            targetAngle -= 360;
//        else if (targetAngle < -180)
//            targetAngle += 360;
//
//        double delta = getDelta(targetAngle, currentAngle);
//        double lastDelta = delta;
//        double dir = Math.abs(delta) / delta;
//
//        turnTelemetry(initialAngle, currentAngle, targetAngle, delta, lastDelta, dir);
//
//        while (!isStopRequested() && !(delta * dir <= 0 && lastDelta * dir >= 0)) {
//            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//            lastDelta = delta;
//            delta = getDelta(targetAngle, currentAngle);
//            double speed = 0; // turnSpeed * (minSlow + (Math.max(endSlow, Math.min(startSlow, Math.abs(delta))) - endSlow) / (startSlow - endSlow) * (1 - minSlow));
//            if (ratio > 0){
//                setPower(-dir * speed, dir * speed * ratio);
//            }
//            else if (ratio < 0) {
//                setPower(-dir * speed * Math.abs(ratio), dir * speed);
//            }
//
//            //telemetry.log().add("< Last Delta   - ", lastDelta);
//            //telemetry.log().add("  Curr Delta > - ", delta);
//
//            turnTelemetry(initialAngle, currentAngle, targetAngle, delta, lastDelta, dir);
//        }
//        setPower(0, 0);
//    }

    private double getDelta(double target, double current) {
        double delta = target - current;
        if (delta > 180)
            delta -= 360;
        else if (delta < -180)
            delta += 360;
        return delta;
    }

    private void turnTelemetry(double speed, double initialAngle, double currentAngle, double targetAngle, double delta, double lastDelta, double dir) {
        Orientation orient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.addData("Speed: ", speed);
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

    private void dropMarker(double dropPos, long waitTime) {
        marker.setPosition(dropPos);
        sleep(waitTime);
    }
}
