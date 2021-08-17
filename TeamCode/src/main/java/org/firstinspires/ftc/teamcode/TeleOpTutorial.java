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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp Tutorial")


public class TeleOpTutorial extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;
    @Override


    public void runOpMode()
    {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive())
        {

            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            idle();

        }
    }
}
