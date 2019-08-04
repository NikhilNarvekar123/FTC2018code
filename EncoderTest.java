/**
 *  Working branch paths (probably) + value tuning
 */

package org.firstinspires.ftc.teamcode.Tank.Auto;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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


//@Autonomous(name = "TestEncoders")

public class EncoderTest extends LinearOpMode {

    //All class components (soon to extend lowering motor/marker servo)
    public DcMotor motorL;
    public DcMotor motorR;
    public static BNO055IMU imu;
    public CRServo markerServo;
    public DcMotor latchM;

    //Encoder values, might need to account for gear train reduction
    private static double inchesPerRot = (228) / (5.5 * Math.PI);
    private static double target;
    private static int tarPosL;
    private static int tarPosR;

    //IMU setup and values
    static Orientation angles;
    static double heading = 0.0;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    //CV components
    private AutoCVAlgorithm c1 = new AutoCVAlgorithm();
    private static int xCor;
    private static int yCor;





    public void runOpMode() throws InterruptedException {


        //Hardware Maps
        motorL = hardwareMap.dcMotor.get("MotorLeft");
        motorR = hardwareMap.dcMotor.get("MotorRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        latchM = hardwareMap.get(DcMotor.class, "pivotMotor");
        markerServo = hardwareMap.get(CRServo.class,"collector");

        //Set reverse on left motor to get motors to have same polarity
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Get OpenCV initialized
        imuSetUp();
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        waitForStart();



        while (opModeIsActive()) {


            markerServo.setPower(-1);
            Thread.sleep(1350);

            markerServo.setPower(0);
            Thread.sleep(100);

            markerServo.setPower(1);
            Thread.sleep(1350);
            markerServo.setPower(0);


/**
            encoderMove(10,0.4,false);
            imuTurnClock(0.5,90);
            imuTurnCounter(0.5,-90);
**/

            Thread.sleep(30000);
        }




    }





    //Movement
    private void encoderMove(double inches,double speed,boolean movingBackwards){

        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tarPosR = motorR.getCurrentPosition() + (int)(inches * inchesPerRot);
        tarPosL = motorL.getCurrentPosition() + (int)(inches * inchesPerRot);

        if(!movingBackwards) {
            motorR.setTargetPosition(-tarPosR);
            motorL.setTargetPosition(-tarPosL);
        } else {
            motorR.setTargetPosition(tarPosR);
            motorL.setTargetPosition(tarPosL);
        }

        motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorR.setPower(speed);
        motorL.setPower(speed);

        while(motorR.getCurrentPosition() != motorR.getTargetPosition() && motorL.getCurrentPosition() != motorL.getTargetPosition()){
            idle();
        }

        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorR.setPower(0);
        motorL.setPower(0);
    }



    //Turning Methods
    private void imuTurnCounter (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, 0.1)) {
            // Update telemetry & Allow time for other processes to run.

            telemetry.addData("DD",heading);
            telemetry.update();
        }

    }

    private void imuTurnClock (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeadingNegative(speed, angle, 0.1)) {
            // Update telemetry & Allow time for other processes to run.

            telemetry.addData("DD",heading);
            telemetry.update();
        }

    }

    private boolean onHeading(double speed, double angle, double PCoeff) {

        boolean onTarget = false ;

        returnAngles();

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorL.setPower(-speed);
        motorR.setPower(speed);

        if(heading > angle){
            onTarget = true;
            motorL.setPower(0);
            motorR.setPower(0);
        }

        return onTarget;
    }

    private boolean onHeadingNegative(double speed, double angle, double PCoeff) {


        boolean onTarget = false ;

        returnAngles();

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    private void imuSetUp() {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }







}

