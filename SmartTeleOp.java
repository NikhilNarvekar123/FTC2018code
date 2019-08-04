package org.firstinspires.ftc.teamcode.Tank.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Tank.vals;

//@TeleOp(name = "Utopia")

public class SmartTeleOp extends LinearOpMode implements vals {

    /* Hardware Vars */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private CRServo collector = null;
    private CRServo spinner = null;
    private DcMotor extender = null;
    private DcMotor lifter = null;
    private DcMotor flipper = null;

    /* Drive Vars */
    int mode = 1;
    double drive = 0.0;
    double drive1 = 0.0;
    int mode2 = 1;

    public void initialize() {

        lifter = hardwareMap.get(DcMotor.class, "pivotMotor");
        leftDrive = hardwareMap.get(DcMotor.class, "MotorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorRight");
        extender = hardwareMap.get(DcMotor.class, "extender");
        flipper = hardwareMap.get(DcMotor.class, "flipper");
        collector = hardwareMap.get(CRServo.class, "collector");
        spinner = hardwareMap.get(CRServo.class, "spinner");

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void restartMotors() {
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void runOpMode() {

        initialize();
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        restartMotors();

        waitForStart();


        while (opModeIsActive()) {


            if (gamepad2.left_bumper) {
                restartMotors();
                lifter.setTargetPosition(MAX_POS_LIFT);
                lifter.setPower(1);
            }

            if (gamepad2.right_bumper) {
                restartMotors();
                lifter.setTargetPosition(0);
                lifter.setPower(1);
            }

            if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {

                lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (gamepad2.right_trigger > gamepad2.left_trigger) {
                    lifter.setPower(gamepad2.right_trigger);
                    telemetry.addData("PosLift ", lifter.getCurrentPosition());
                    telemetry.update();
                } else if (gamepad2.right_trigger < gamepad2.left_trigger) {
                    lifter.setPower(-gamepad2.left_trigger);
                    telemetry.addData("PosLift ", lifter.getCurrentPosition());
                    telemetry.update();
                } else {
                    lifter.setPower(0);
                }

            }


            /* Collects Minerals */
            if (gamepad2.dpad_down) {
                spinner.setPower(SPIN_SPEED);
            } else if (gamepad2.dpad_up) {
                spinner.setPower(-SPIN_SPEED);
            } else {
                spinner.setPower(0);
            }


            /* Flips Collecting Bucket */
            if (gamepad2.y) {
                collector.setPower(SPIN_SPEED);
            } else if (gamepad2.a) {
                collector.setPower(-SPIN_SPEED);
            } else {
                collector.setPower(0);
            }


            if(gamepad2.x || Math.abs(gamepad2.left_stick_y) > 0) {
                mode2++;
            }

            if(mode2 % 2 != 0){
                extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extender.setPower(-gamepad2.left_stick_y * EXTEND_SPEED);
            }else if(mode2 % 2 == 0) {
                extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extender.setTargetPosition(0);
                extender.setPower(1);
            }


            flipper.setPower(gamepad2.right_stick_y * SLOW_SPEED);


            /* Movement */
            if (gamepad1.y) {
                mode++;
            }

            if (mode % 2 == 0) {
                drive = gamepad1.left_stick_y * FACT;
                drive1 = gamepad1.right_stick_y * FACT;
            } else {
                drive = gamepad1.left_stick_y * FAST_SPEED;
                drive1 = gamepad1.right_stick_y * FAST_SPEED;
            }

            leftDrive.setPower((drive1));
            rightDrive.setPower((drive));
        }
    }

}


