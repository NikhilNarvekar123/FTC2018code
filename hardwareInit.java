package org.firstinspires.ftc.teamcode.Tank;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/** UNLISTED OPMODE, CONTAINS METHODS/SETUP
 *  @author NIKHIL NARVEKAR               */

public class hardwareInit extends LinearOpMode  {

    public void runOpMode(){}

    /* Hardware Vars */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public CRServo collector = null;
    public CRServo spinner = null;
    public DcMotor extender = null;
    public DcMotor lifter = null;
    public DcMotor flipper = null;
    public static BNO055IMU imu = null;

    /* Drive Vars */
    int mode = 1;

    /* IMU Vars */
    private static Orientation angles;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    /* Encoder Vars */
    private static final double INCHES_PER_ROT = (288) / (5.5 * Math.PI);

    /* Sample Multi-Thread - CURRENTLY DISABLED */
    //private Secondary sc = new Secondary();


    /** TeleOp methods.
     *  TBD
     */

    /* Init Methods */
    public void initializeTele(HardwareMap hw) {

        lifter = hw.get(DcMotor.class, "pivotMotor");
        leftDrive = hw.get(DcMotor.class, "MotorLeft");
        rightDrive = hw.get(DcMotor.class, "MotorRight");
        extender = hw.get(DcMotor.class, "extender");
        flipper = hw.get(DcMotor.class, "flipper");
        collector = hw.get(CRServo.class, "collector");
        spinner = hw.get(CRServo.class, "spinner");

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);


    }



    /** Autonomous period methods. Include movement by motor encoders,
     *  turning by IMU sensor, and init methods.
     */

    /* Movement */
    public void encoderMoveForward(double inches,double speed){

        motorControl(0);
        motorControl(2);

        int tarPosR = (int)(rightDrive.getCurrentPosition() + (inches * INCHES_PER_ROT));
        int tarPosL = (int)(leftDrive.getCurrentPosition() + (inches * INCHES_PER_ROT));

        rightDrive.setTargetPosition(tarPosR);
        leftDrive.setTargetPosition(tarPosL);

        rightDrive.setPower(speed);
        leftDrive.setPower(speed);

        while(leftDrive.isBusy() && rightDrive.isBusy() && opModeIsActive()) {}

        motorControl(0);
    }
    public void encoderMoveBack(double inches,double speed){

        motorControl(0);
        motorControl(2);

        int tarPosR = (int)(rightDrive.getCurrentPosition() - inches * INCHES_PER_ROT);
        int tarPosL = (int)(leftDrive.getCurrentPosition() - inches * INCHES_PER_ROT);

        rightDrive.setTargetPosition(tarPosR);
        leftDrive.setTargetPosition(tarPosL);

        rightDrive.setPower(speed);
        leftDrive.setPower(speed);

        while(leftDrive.isBusy() && rightDrive.isBusy() && opModeIsActive()) {}

        motorControl(0);

    }
    private void motorControl(int i){
        if(i == 0) {
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(i == 1){
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else if(i == 2){
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else if(i == 3){
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


    /* Init Methods */
    public void initGyro(){

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    public void initializeAuto(HardwareMap hw) {

        lifter = hw.get(DcMotor.class, "pivotMotor");
        leftDrive = hw.get(DcMotor.class, "MotorLeft");
        rightDrive = hw.get(DcMotor.class, "MotorRight");
        extender = hw.get(DcMotor.class, "extender");
        flipper = hw.get(DcMotor.class, "flipper");
        collector = hw.get(CRServo.class, "collector");
        spinner = hw.get(CRServo.class, "spinner");
        imu = hw.get(BNO055IMU.class, "imu");

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }


    /* Turning Methods */
    public void imuTurnCounter (double speed, double angle, double coeff) {

        motorControl(0);
        double heading = 0.0;

        while (opModeIsActive()) {

            heading = returnAngles();
            leftDrive.setPower(-speed * coeff);
            rightDrive.setPower(speed * coeff);

            if (heading >= angle) {
                break;
            }
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);


    }
    public void imuTurnClock (double speed, double angle, double coeff) {

        motorControl(0);
        double heading = 0.0;

        while (opModeIsActive()) {

            heading = returnAngles();
            leftDrive.setPower(speed * coeff);
            rightDrive.setPower(-speed * coeff);

            if(heading <= angle){
                break;
            }
        }


        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    private static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    private static String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public double returnAngles(){
        //starting IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //return necessary angle
        return(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
    }


    /* General */
    public void delay(double sec) throws InterruptedException{
        long time = (long)(sec * 1000);
        Thread.sleep(time);
    }
    public void tele(String s,Object o,int flag){
        if(flag == 0) {
            telemetry.addData(s, o);
            telemetry.update();
        }else if(flag == 1){
            telemetry.addLine(s);
            telemetry.update();
        }
    }


}
