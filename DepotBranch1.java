package org.firstinspires.ftc.teamcode.Tank.Auto.Depot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Tank.OpenCV.AutoCVAlgorithm;

import java.util.Locale;

/**
 * Auto of the left path, given cube is in left position.
 * Status:
 */


//@Autonomous(name = "DepotLeft")

public class DepotBranch1 extends LinearOpMode {

    //All class components (soon to extend lowering motor/marker servo)
    DcMotor motorL;
    DcMotor motorR;
    static BNO055IMU imu;

    //Encoder values, might need to account for gear train reduction
    public static double inchesPerRot = (228) / (5.5 * Math.PI);
    public static double target;
    public static int tarPosL;
    public static int tarPosR;

    //IMU setup and values
    static Orientation angles;
    static double heading = 0.0;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    //CV components (unused in this file)
    public static AutoCVAlgorithm c1= new AutoCVAlgorithm();
    public static int xCor;
    public static int yCor;

    //Values for combination auto
    public static int branchPath;


    public void runOpMode() throws InterruptedException{

        //Run hardware maps
        motorL = hardwareMap.dcMotor.get("MotorLeft");
        motorR = hardwareMap.dcMotor.get("MotorRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Set reverse to allow both motor commands to be same polarity
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set up the IMU
        imuSetUp();
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        waitForStart();

        while(opModeIsActive()) {

            //Turn clockwise (negative) to left
            imuTurnClock(0.3, -21);
            target = 50;

            motorL.setPower(0);
            motorR.setPower(0);

            returnAngles();
            telemetry.addData("Angle: ", heading);
            telemetry.update();

            Thread.sleep(1000);

            //Start moving to hit cube given set target distance to cover
            encoderMove(target, 0.4, false);
            Thread.sleep(1000);

            //Turn to be perpendicular to depot
            imuTurnCounter(0.25, 45);
            target = 18;

            motorL.setPower(0);
            motorR.setPower(0);

            returnAngles();
            telemetry.addData("Angle", heading);
            telemetry.update();

            Thread.sleep(1000);

            //Move towards depot to deposit marker
            encoderMove(target,0.4, false);
            Thread.sleep(1000);


            //servo code
            Thread.sleep(3000);


            //Move towards crater
            encoderMove(target, 0.4, true);
            Thread.sleep(1000);

            Thread.sleep(1000);


        }

    }


    //Methods
    private void encoderMove(double inches,double speed,boolean movingBackwards){

        tarPosR = motorR.getCurrentPosition() + (int)(inches * inchesPerRot);
        tarPosL = motorL.getCurrentPosition() + (int)(inches * inchesPerRot);

        if(!movingBackwards) {
            motorR.setTargetPosition(tarPosR);
            motorL.setTargetPosition(tarPosL);
        } else {
            motorR.setTargetPosition(-tarPosR);
            motorL.setTargetPosition(-tarPosL);
        }
        motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorR.setPower(speed);
        motorL.setPower(speed);

        while(motorR.getCurrentPosition() != motorR.getTargetPosition() && motorL.getCurrentPosition() != motorL.getTargetPosition()){
            telemetry.addLine("Moving to hit Cube...");
            idle();
        }

        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorR.setPower(0);
        motorL.setPower(0);
    }

    private void imuTurnCounter (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle)) {
            // Update telemetry & Allow time for other processes to run.

            telemetry.addData("Heading",heading);
            telemetry.update();
        }

    }

    private void imuTurnClock (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeadingNegative(speed, angle)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("Heading:",heading);
            telemetry.update();
        }

    }

    private boolean onHeading(double speed, double angle) {

        boolean onTarget = false ;
        returnAngles();

        motorL.setPower(-speed);
        motorR.setPower(speed);

        if(heading > angle){
            onTarget = true;
            motorL.setPower(0);
            motorR.setPower(0);
        }

        return onTarget;
    }

    private boolean onHeadingNegative(double speed, double angle) {

        boolean onTarget = false ;
        returnAngles();

        motorL.setPower(speed);
        motorR.setPower(-speed);

        if(heading < angle){
            onTarget = true;
            motorR.setPower(0);
            motorL.setPower(0);
        }

        return onTarget;
    }

    private static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private static String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private static void returnAngles(){
        //starting IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //return necessary angle
        heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    public void imuSetUp() {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }

    private void sleep1(double seconds){
        try{
            Thread.sleep((int)(seconds * 1000));
        }catch(Exception e){}
    }


}


