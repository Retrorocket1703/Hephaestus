package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous
public class TestReturnToHeading extends LinearOpMode {

    DcMotor ForwardRight, ForwardLeft, BackLeft, BackRight, EncoderY, EncoderX, LeftLift, RightLift, CableSpool, Intake;
    Servo Flap;
    BNO055IMU imu;
    boolean Straight = false;
    private String Position = "NULL";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private float zAxis, xAxis;

    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Test RTH Initialized");
        telemetry.update();
        telemetry.setAutoClear(true);

        //Init the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "IMU Initialized");
        telemetry.update();

        //Init hardware
        ForwardLeft = hardwareMap.get(DcMotor.class, "ForwardLeft");
        ForwardLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ForwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ForwardRight = hardwareMap.get(DcMotor.class, "ForwardRight");
        ForwardRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ForwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        EncoderY = hardwareMap.get(DcMotor.class, "EncoderY");
        EncoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        EncoderX = hardwareMap.get(DcMotor.class, "EncoderX");
        EncoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift = hardwareMap.get(DcMotor.class, "EncoderY");
        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftLift.setDirection(DcMotor.Direction.REVERSE);
        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightLift = hardwareMap.get(DcMotor.class, "RightLift");
        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CableSpool = hardwareMap.get(DcMotor.class, "CableSpool");
        CableSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CableSpool.setDirection(DcMotor.Direction.REVERSE);
        CableSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake = hardwareMap.get(DcMotor.class, "EncoderX");
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Flap = hardwareMap.get(Servo.class, "Flap");

        ForwardLeft.setDirection(DcMotor.Direction.FORWARD);
        ForwardRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();

        waitForStart();
        changeMode(3);
        changeMode(4);
        //while (!Straight && !isStopRequested() && opModeIsActive()) {
            zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            long startTime = System.currentTimeMillis();
            long currentTime = startTime;

            while (currentTime - startTime < 5000 && opModeIsActive() && isStopRequested() != true) {
                zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

                telemetry.addData("time", currentTime - startTime);
                telemetry.addData("Status", "Returning to heading 0");
                telemetry.update();

                while (zAxis < 0) {
                    zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                    ForwardLeft.setPower(.2);
                    ForwardRight.setPower(-.2);
                    BackLeft.setPower(.2);
                    BackRight.setPower(-.2);
                }

                while (zAxis > 0) {
                    zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                    ForwardLeft.setPower(-.2);
                    ForwardRight.setPower(.2);
                    BackLeft.setPower(-.2);
                    BackRight.setPower(.2);
                }

                currentTime = System.currentTimeMillis();
            }
            telemetry.addData("Rotation Z", zAxis);
            telemetry.update();
            //turnToFace(zAxis, 270);
        //}
        ForwardRight.setPower(0);
        ForwardLeft.setPower(0);
        BackRight.setPower(0);
        BackLeft.setPower(0);
    }

    public void turnToFace(double currentAngle, double desiredAngle) {
        changeMode(3);
        changeMode(4);
        //while(Math.abs(currentAngle - desiredAngle) < desiredAngle) {
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
        double angleBetween = desiredAngle - currentAngle;
        double tempTurnMult = 0;
        double turnMult = 1;
        if (Math.abs(desiredAngle - currentAngle) <= desiredAngle) {
            Straight = false;
        } else {
            Straight = true;
        }
        if (Math.sin(angleBetween) < 0) {
            ForwardRight.setPower(Math.sin(angleBetween) < 0 ? -1 : 1);
            ForwardLeft.setPower(Math.sin(angleBetween) < 0 ? -1 : 1);
            BackRight.setPower(Math.sin(angleBetween) < 0 ? -1 : 1);
            BackLeft.setPower(Math.sin(angleBetween) < 0 ? -1 : 1);
        } else {
            double sin = Math.sin(angleBetween);
            tempTurnMult = Math.abs(sin) + 1;
            double rightPow = (-tempTurnMult * turnMult * sin);
            double leftPow = (tempTurnMult * turnMult * sin);
            ForwardRight.setPower(rightPow);
            ForwardLeft.setPower(leftPow);
            BackRight.setPower(rightPow);
            BackLeft.setPower(leftPow);
        }
    }
    //}

    void changeMode(int Mode) {
        if (Mode == 1) {
            ForwardLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ForwardRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ForwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ForwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (Mode == 2) {
            ForwardLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ForwardRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (Mode == 3) {
            ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (Mode == 4) {
            ForwardLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ForwardRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
