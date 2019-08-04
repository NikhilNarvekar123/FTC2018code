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

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Tank.OpenCV.AutoCVAlgorithm;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;


@Autonomous(name = "Depot Auto (W Latching)")
public class ModdedDepotAuto extends LinearOpMode {

    //All class components (soon to extend lowering motor/marker servo)
    public DcMotor motorL;
    public DcMotor motorR;
    public static BNO055IMU imu;
    public CRServo markerServo;
    public DcMotor latchM;
    public DcMotor flipper;

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
        markerServo = hardwareMap.get(CRServo.class,"collector");
        latchM = hardwareMap.get(DcMotor.class,"pivotMotor");
        flipper = hardwareMap.get(DcMotor.class, "flipper");

        //Set reverse on left motor to get motors to have same polarity
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Get OpenCV initialized
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        waitForStart();



        while (opModeIsActive()) {



            latchM.setPower(0.8);
            Thread.sleep(3150);
            latchM.setPower(0);

            imuTurnCounter(0.4,15);
            encoderMove(3,0.3,false);
            imuTurnClock(0.4,0);



            //OpenCV detection of mineral
            c1.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            c1.setShowCountours(true);
            c1.enable();

            Thread.sleep(2500);

            List<MatOfPoint> contours = c1.getContours();
            for (int i = 0; i < contours.size(); i++) {
                Rect boundingRect = Imgproc.boundingRect(contours.get(i));
                xCor = ((boundingRect.x + boundingRect.width) / 2);
                yCor = ((boundingRect.y + boundingRect.height) / 2);
            }

            telemetry.addLine("xCor: " + xCor);
            telemetry.update();

            c1.disable();

            Thread.sleep(2000);


            if(xCor < 150 && xCor != 0){

                //Turn counter-clockwise (positive) to hit ball and move past
                imuTurnCounter(0.4,30);
                target = 52;

                Thread.sleep(500);

                //Move to hit ball
                encoderMove(target,0.4,false);
                Thread.sleep(500);



            } else if(xCor >= 150){

                //Move straight ahead to hit cube and get to depot
                target = 50;
                encoderMove(target,0.45,false);
                Thread.sleep(500);

            } else {

                //Turn clockwise (negative) to left
                imuTurnClock(0.4, -9);
                target = 52;
                Thread.sleep(500);

                //Start moving to hit cube given set target distance to cover
                encoderMove(target, 0.4, false);
                Thread.sleep(500);

            }


            Thread.sleep(30000);

        }




    }


    //Movement
    public void encoderMove(double inches,double speed,boolean movingBackwards){
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        if(!movingBackwards){
            tarPosR = motorR.getCurrentPosition() + (int)(inches * inchesPerRot);
            tarPosL = motorL.getCurrentPosition() + (int)(inches * inchesPerRot);
        } else {
            tarPosR = motorR.getCurrentPosition() - (int)(inches * inchesPerRot);
            tarPosL = motorL.getCurrentPosition() - (int)(inches * inchesPerRot);
        }


        motorR.setTargetPosition(-tarPosR);
        motorL.setTargetPosition(-tarPosL);


        motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if(!movingBackwards) {
            do {
                motorR.setPower(speed);
                motorL.setPower(speed);
            }
            while (motorR.isBusy() && motorL.isBusy());
        }else {
            do {
                motorR.setPower(speed);
                motorL.setPower(speed);
            }
            while (motorR.isBusy() && motorL.isBusy());
        }


        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorR.setPower(0);
        motorL.setPower(0);
    }


    //Turning Methods
    public void imuTurnCounter (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, 0.1)) {
            // Update telemetry & Allow time for other processes to run.
            idle();
        }

    }

    public void imuTurnClock (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeadingNegative(speed, angle, 0.1)) {
            // Update telemetry & Allow time for other processes to run.
            idle();
        }

    }

    public boolean onHeading(double speed, double angle, double PCoeff) {

        boolean onTarget = false ;

        returnAngles();

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorL.setPower(-speed);
        motorR.setPower(speed);

        if(heading >= angle){
            onTarget = true;
            motorL.setPower(0);
            motorR.setPower(0);
        }

        return onTarget;
    }

    public boolean onHeadingNegative(double speed, double angle, double PCoeff) {


        boolean onTarget = false ;


        returnAngles();

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorL.setPower(speed);
        motorR.setPower(-speed);

        if(heading <= angle){
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













}

