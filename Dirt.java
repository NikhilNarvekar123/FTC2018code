package org.firstinspires.ftc.teamcode.Tank.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tank.vals;

//@TeleOp(name = "Dirt Knowsli")

public class Dirt extends LinearOpMode implements vals {

    /* Hardware Vars */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor collector = null;
    private DcMotor extender = null;
    private DcMotor lifter = null;
    private DcMotor flipper = null;

    /* Drive Vars */
    int mode = 1;
    double drive = 0.0;

    /* Controls */



    public void initialize()
    {
        lifter = hardwareMap.get(DcMotor.class, "pivotMotor");
        //leftDrive = hardwareMap.get(DcMotor.class,"MotorLeft");
        //rightDrive = hardwareMap.get(DcMotor.class,"MotorRight");
        extender = hardwareMap.get(DcMotor.class,"extender");
        flipper = hardwareMap.get(DcMotor.class,"flipper");
        //collector = hardwareMap.get(DcMotor.class,"Collector");

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {


            /* Uses triggers to move rack/pinion */
            if(gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0)
            {

                if(gamepad2.right_trigger > gamepad2.left_trigger)
                {
                    lifter.setPower(gamepad2.right_trigger);

                } else if(gamepad2.right_trigger < gamepad2.left_trigger)
                {
                    lifter.setPower(-gamepad2.left_trigger);
                }
            } else
            {
                lifter.setPower(0);
            }




            /* Two mechs to deposit */
            extender.setPower(gamepad2.left_stick_y);
            flipper.setPower(gamepad2.right_stick_y * SLOW_SPEED);



        }


    }




}






