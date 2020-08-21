

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@Autonomous(name="Auto Challenge")


public class SensorChallengeExample extends LinearOpMode {


    private DcMotor motorLeft;
    private DcMotor motorRight;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    private TouchSensor digitalTouch;

    private ColorSensor sensorColor;



    @Override

    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        digitalTouch = hardwareMap.get(TouchSensor.class, "touch1");

        sensorColor = hardwareMap.get(ColorSensor.class, "color1");

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();


        double DRIVE_POWER = 0.7;


        waitForStart();


        telemetry.addData("Mode", "running");
        telemetry.update();




        DriveForwardByTime(DRIVE_POWER,1000);

        rotate(90, 0.3);

        DriveForwardByTime(DRIVE_POWER,1500);

        rotate(-90, 0.3);

        DriveTillTouch(1);

        DriveForwardByTime(-DRIVE_POWER,2000);

        DriveTillTouch(1);

        DriveTillTape(-0.5);

    }


    public void DriveForwardByTime(double power, long time) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
        sleep(time);

        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(250);
    }


    public void TurnRightByTime(double power, long time) {
        motorLeft.setPower(power);
        motorRight.setPower(-power);
        sleep(time);

        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(250);
    }


    public void TurnLeftByTime(double power, long time) {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
        sleep(time);

        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(250);
    }


    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    public void rotate(int degrees, double power)
    {
        double leftPower, rightPower;

        resetAngle();

        if (degrees < 0) {
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {
            leftPower = -power;
            rightPower = power;
        } else return;


        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);

        if (degrees < 0) {
            while (opModeIsActive() && getAngle() == 0) { }

            while (opModeIsActive() && getAngle() > degrees) { }
        } else
            while (opModeIsActive() && getAngle() < degrees) { }


        motorLeft.setPower(0);
        motorRight.setPower(0);

        sleep(1000);

        resetAngle();
    }


    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }


    public void DriveTillTouch(double power)
    {

        motorLeft.setPower(power);
        motorRight.setPower(power);

        while (opModeIsActive() && !digitalTouch.isPressed())
        {
            telemetry.addData("Digital Touch", "Is Not Pressed");
            telemetry.update();
        }

        telemetry.addData("Digital Touch", "Pressed");
        telemetry.update();

        motorRight.setPower(0);
        motorLeft.setPower(0);

        sleep(250);

    }



    public void DriveTillTape(double power)
    {
        motorLeft.setPower(power);
        motorRight.setPower(power);
        float hsvValues[] = {120F, 300F, 300F};
        final double SCALE_FACTOR = 255;
        float BLUE_HUE = 150;
        float RED_HUE = 80;

        while (opModeIsActive()&&hsvValues[0]>=RED_HUE&&hsvValues[0]<=BLUE_HUE)
        {
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
        }

        motorRight.setPower(0);
        motorLeft.setPower(0);

        sleep(250);

    }


}
