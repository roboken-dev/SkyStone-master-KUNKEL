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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="ChrisK1234")




public class ChrisK1234 extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;
    @Override


    public void runOpMode()
    {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double DRIVE_POWER = 0.5;


        waitForStart();



        DriveForwardByTime(DRIVE_POWER,1000);

        TurnRightByTime(DRIVE_POWER,1000);

        DriveForwardByTime(DRIVE_POWER,750);

        TurnLeftByTime(DRIVE_POWER,1000);

        DriveForwardByTime(DRIVE_POWER,1000);


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



}
