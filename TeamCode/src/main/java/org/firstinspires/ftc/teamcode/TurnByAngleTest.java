/*
To add this code to your folder, right click on org.firstinspires.ftc.teamcode on the left Project navigation.
Select New -> Java Class
Make sure you name the class TeleOpTutorial
Copy and paste everything from this doc into the Class replacing what is there with this code.
This should work.

This code was created by following the tutorial created by Swerve Robotics found on YouTube:
https://www.youtube.com/watch?v=OT_PGYIFBGE&list=PLJIJCo7cYsE-ma0iYtbCf27s7zgLq-73i&index=8

Note: Since the Jersey Bots do not have servos, that part of the code was omitted.

IT IS STRONGLY RECOMMENDED YOU TAKE THE TIME TO DO THE TUTORIAL YOURSELF.
IT DOESN'T TAKE LONG AND WILL COVER A FEW IMPORTANT BASIC CONCEPTS.
*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="TurnByAngle")


public class TurnByAngleTest extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;
    BNO055IMU imu;
    Orientation angles;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    @Override


    public void runOpMode()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double DRIVE_POWER = 0.5;


        waitForStart();



        DriveForwardByTime(DRIVE_POWER,1000);
        rotate(90, 0.3);
        DriveForwardByTime(DRIVE_POWER,1000);
        rotate(-90, 0.3);




    }


    public void DriveForwardByTime(double power, long time)
    {
    motorLeft.setPower(power);
    motorRight.setPower(power);
    sleep(time);

    motorLeft.setPower(0);
    motorRight.setPower(0);
    sleep(250);
    }


    public void TurnRightByTime(double power, long time)
    {
    motorLeft.setPower(power);
    motorRight.setPower(-power);
    sleep(time);

    motorLeft.setPower(0);
    motorRight.setPower(0);
    sleep(250);
    }


    public void TurnLeftByTime(double power, long time)
    {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
        sleep(time);

        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(250);
    }


    public double getAngle()
    {
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
        double  leftPower, rightPower;

        resetAngle();

        if (degrees < 0)
        {
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {
            leftPower = -power;
            rightPower = power;
        }
        else return;

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);

        if (degrees < 0)
        {
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else
            while (opModeIsActive() && getAngle() < degrees) {}

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




}
