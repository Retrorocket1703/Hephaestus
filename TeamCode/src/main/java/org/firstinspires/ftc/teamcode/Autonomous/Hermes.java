package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name = "Hermes - Facing Depot - Pos 2", group = "Coeus")
//Hermes is for Position 2 aka when you're facing the depot
public class Hermes extends LinearOpMode {


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AXSqUwf/////AAABmVi9ifY9YkX7hla6dk//kFRSGKk+yWO9wuL666vmsKegzx+5Io14hzrDHPPSIOUIuBd89kbyiEj6ckbCr0Ak3uWm37MVO6WuHj2iyDGBiFASfDpXymVBKsTiT12M0KxuGVLdxg7JBLeNMq2f7lV/vtdmBf+UOVaruaibuICmy0jdCALIN7Edd3WcSYlY8V6VCxMPqLw4MrRbkPSshdxQ2WVZsDSZJgPqBE43qSmCFHhPwggb4+ZFSe6nn6SureHq5pNgyxeUXCaCsaIDbwHqEL7NrhOT6sqQTrWCsay3dFs6uVPY4YqJEP6+YGq7vYG7aJOC6nro3Z9NnVBhfwZEM07o6imRE0nkNd380XSO46rZ";
    DcMotor ForwardRight, ForwardLeft, BackLeft, BackRight, EncoderY, EncoderX, LeftLift, RightLift, CableSpool, Intake;
    Servo Flap;
    BNO055IMU imu;
    private String Position = "NULL";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private float zAxis, xAxis;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Hermes Protocol Initialized");
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

        ForwardLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ForwardRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();

        //Init Vuforia
        initVuforia();

        telemetry.addData("Status", "Vuforia Initialized");

        //Init TFOD
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

        telemetry.addData("Status", "TFOD Initialized");
        telemetry.update();

        //Activate TFOD
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("Status", "TFOD Activated. Beginning Recognition");
        telemetry.update();

        //Begin Recognition steps
        while (isStopRequested() != true && !opModeIsActive()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                Position = "LEFT";
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                Position = "RIGHT";
                            } else {
                                Position = "CENTER";
                            }
                        }
                    }
                }
            }
            telemetry.addData("Position", Position);
            telemetry.addData("Status", "VARaTiFf DLNN is running");
            telemetry.update();
        }

        telemetry.addData("Status", "No longer running VARaTiFf DLNN");
        telemetry.update();

        // +=---------------------------------------------------------------------------------------------------------=+


        waitForStart();

        tfod.deactivate();

        try {
            Thread.sleep(500);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        telemetry.addData("Rotation zAxis", zAxis);

        telemetry.addData("Status", "All systems online, Running");
        telemetry.update();

        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        changeMode(5);

        try {
            Thread.sleep(6000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        RightLift.setPower(-.4);
        LeftLift.setPower(-.4);
        RightLift.setTargetPosition(-2200);

        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setPower(.2);
        BackRight.setPower(.2);

        try {
            Thread.sleep(2500);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        BackRight.setPower(0);
        BackLeft.setPower(0);
        RightLift.setPower(0);
        LeftLift.setPower(0);
        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //xAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;

        changeMode(1);

        try {
            Thread.sleep(900);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        changeMode(4);

        /*while (opModeIsActive() && !isStopRequested() && xAxis >= 10) {
            BackRight.setPower(-.6);
            BackLeft.setPower(.6);
            xAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Rotation x Axis", xAxis);
            telemetry.update();
        }*/


        ForwardLeft.setPower(.4);
        ForwardRight.setPower(.4);
        BackLeft.setPower(.4);
        BackRight.setPower(.4);

        BackRight.setTargetPosition(400);
        BackLeft.setTargetPosition(400);
        ForwardRight.setTargetPosition(400);
        ForwardLeft.setTargetPosition(400);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        changeMode(1);

        ForwardLeft.setTargetPosition(-240);
        ForwardRight.setTargetPosition(240);
        BackLeft.setTargetPosition(240);
        BackRight.setTargetPosition(-240);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        changeMode(1);

        BackRight.setTargetPosition(-200);
        BackLeft.setTargetPosition(-200);
        ForwardRight.setTargetPosition(-200);
        ForwardLeft.setTargetPosition(-200);
        RightLift.setPower(1);
        RightLift.setTargetPosition(-1500);

        try {
            Thread.sleep(400);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        changeMode(1);

        ForwardLeft.setTargetPosition(240);
        ForwardRight.setTargetPosition(-240);
        BackLeft.setTargetPosition(-240);
        BackRight.setTargetPosition(240);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        //changeMode(3);

        /*long startTime = System.currentTimeMillis();
        long currentTime = startTime;

        while (currentTime - startTime < 5000 && opModeIsActive() && isStopRequested() != true) {

            telemetry.addData("time", currentTime - startTime);
            telemetry.addData("Status", "Returning to heading 0");
            telemetry.update();

            while (zAxis < -5) {
                zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                ForwardLeft.setPower(.3);
                ForwardRight.setPower(-.3);
                BackLeft.setPower(.3);
                BackRight.setPower(-.3);
            }

            while (zAxis > 5) {
                zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                ForwardLeft.setPower(-.3);
                ForwardRight.setPower(.3);
                BackLeft.setPower(-.3);
                BackRight.setPower(.3);
            }

            currentTime = System.currentTimeMillis();
        }

        ForwardLeft.setPower(0);
        ForwardRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);*/


        //zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        if (Position == "CENTER" || Position == "NULL") {

            changeMode(1);

            ForwardLeft.setPower(.3);
            ForwardRight.setPower(.3);
            BackLeft.setPower(.3);
            BackRight.setPower(.3);

            ForwardLeft.setTargetPosition(250);
            ForwardRight.setTargetPosition(-250);
            BackLeft.setTargetPosition(250);
            BackRight.setTargetPosition(-250);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(-4000);
            ForwardRight.setTargetPosition(-4000);
            BackLeft.setTargetPosition(-4000);
            BackRight.setTargetPosition(-4000);

            try {
                Thread.sleep(3300);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            RightLift.setPower(1);
            LeftLift.setPower(.2);
            RightLift.setTargetPosition(-1000);
            Intake.setPower(1);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            LeftLift.setPower(0);
            RightLift.setTargetPosition(-1900);
            Intake.setPower(0);

            changeMode(1);

            ForwardLeft.setTargetPosition(-1250);
            ForwardRight.setTargetPosition(1250);
            BackLeft.setTargetPosition(-1250);
            BackRight.setTargetPosition(1250);

            try {
                Thread.sleep(2500);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(500);
            ForwardRight.setTargetPosition(-500);
            BackLeft.setTargetPosition(-500);
            BackRight.setTargetPosition(500);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(1100);
            ForwardRight.setTargetPosition(1100);
            BackLeft.setTargetPosition(1100);
            BackRight.setTargetPosition(1100);

            try {
                Thread.sleep(1200);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(400);
            ForwardRight.setTargetPosition(-400);
            BackLeft.setTargetPosition(400);
            BackRight.setTargetPosition(-400);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setPower(.6);
            ForwardRight.setPower(.6);
            BackLeft.setPower(.6);
            BackRight.setPower(.6);

            ForwardLeft.setTargetPosition(5000);
            ForwardRight.setTargetPosition(5000);
            BackLeft.setTargetPosition(5000);
            BackRight.setTargetPosition(5000);

            try {
                Thread.sleep(7000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }


        if (Position == "RIGHT") {

            changeMode(1);

            ForwardLeft.setPower(.3);
            ForwardRight.setPower(.3);
            BackLeft.setPower(.3);
            BackRight.setPower(.3);

            /*ForwardLeft.setTargetPosition(-300);
            ForwardRight.setTargetPosition(-300);
            BackLeft.setTargetPosition(-300);
            BackRight.setTargetPosition(-300);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }*/

            BackRight.setTargetPosition(-4000);
            ForwardLeft.setTargetPosition(-4000);

            try {
                Thread.sleep(3300);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(600);
            ForwardRight.setTargetPosition(-600);
            BackLeft.setTargetPosition(600);
            BackRight.setTargetPosition(-600);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(-2000);
            ForwardRight.setTargetPosition(-2000);
            BackLeft.setTargetPosition(-2000);
            BackRight.setTargetPosition(-2000);

            try {
                Thread.sleep(1500);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            RightLift.setPower(1);
            LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            LeftLift.setPower(.2);
            RightLift.setTargetPosition(-1000);

            try {
                Thread.sleep(500);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            Intake.setPower(1);

            try {
                Thread.sleep(2000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            LeftLift.setPower(0);
            RightLift.setTargetPosition(-1900);
            Intake.setPower(0);

            changeMode(1);

            ForwardLeft.setTargetPosition(-500);
            ForwardRight.setTargetPosition(500);
            BackLeft.setTargetPosition(-500);
            BackRight.setTargetPosition(500);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(1900);
            ForwardRight.setTargetPosition(-1900);
            BackLeft.setTargetPosition(-1900);
            BackRight.setTargetPosition(1900);

            try {
                Thread.sleep(4000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(500);
            ForwardRight.setTargetPosition(-500);
            BackLeft.setTargetPosition(500);
            BackRight.setTargetPosition(-500);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(10000);
            ForwardRight.setTargetPosition(-10000);
            BackLeft.setTargetPosition(-10000);
            BackRight.setTargetPosition(10000);

            try {
                Thread.sleep(7000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }


        if (Position == "LEFT") {
            changeMode(1);

            ForwardLeft.setPower(.3);
            ForwardRight.setPower(.3);
            BackLeft.setPower(.3);
            BackRight.setPower(.3);

            /*ForwardLeft.setTargetPosition(-300);
            ForwardRight.setTargetPosition(-300);
            BackLeft.setTargetPosition(-300);
            BackRight.setTargetPosition(-300);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }*/

            ForwardRight.setPower(.4);
            BackLeft.setPower(.4);
            ForwardRight.setTargetPosition(-4000);
            BackLeft.setTargetPosition(-4000);

            try {
                Thread.sleep(3300);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(-300);
            ForwardRight.setTargetPosition(300);
            BackLeft.setTargetPosition(-300);
            BackRight.setTargetPosition(300);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(-2000);
            ForwardRight.setTargetPosition(-2000);
            BackLeft.setTargetPosition(-2000);
            BackRight.setTargetPosition(-2000);

            try {
                Thread.sleep(3000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }


            RightLift.setPower(1);
            LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            LeftLift.setPower(.2);
            RightLift.setTargetPosition(-1000);

            try {
                Thread.sleep(500);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            Intake.setPower(1);

            try {
                Thread.sleep(2000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            LeftLift.setPower(0);
            RightLift.setTargetPosition(-1900);
            Intake.setPower(0);

            changeMode(1);

            ForwardLeft.setTargetPosition(-600);
            ForwardRight.setTargetPosition(600);
            BackLeft.setTargetPosition(-600);
            BackRight.setTargetPosition(600);

            try {
                Thread.sleep(1900);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            changeMode(1);

            ForwardLeft.setTargetPosition(21000);
            ForwardRight.setTargetPosition(21000);
            BackLeft.setTargetPosition(21000);
            BackRight.setTargetPosition(21000);

            try {
                Thread.sleep(4000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }

        telemetry.addData("Status", "Completed Detection Execution");
        telemetry.update();


    }

    public void turnToFace(double currentAngle, double desiredAngle) {
        changeMode(3);
        double angleBetween = desiredAngle - currentAngle;
        double tempTurnMult = 0;
        double turnMult = 1;
        if (Math.sin(angleBetween) < 0) {
            ForwardRight.setPower(Math.cos(angleBetween) < 0 ? -1 : 1);
            ForwardLeft.setPower(Math.cos(angleBetween) < 0 ? -1 : 1);
            BackRight.setPower(Math.cos(angleBetween) < 0 ? -1 : 1);
            BackLeft.setPower(Math.cos(angleBetween) < 0 ? -1 : 1);
        } else {
            double cos = Math.cos(angleBetween);
            tempTurnMult = Math.abs(cos) + 1;
            double rightPow = (-tempTurnMult * turnMult * cos);
            double leftPow = (tempTurnMult * turnMult * cos);
            ForwardRight.setPower(rightPow);
            ForwardLeft.setPower(leftPow);
            BackRight.setPower(rightPow);
            BackLeft.setPower(leftPow);
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

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
        if (Mode == 5) {
            ForwardLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            ForwardRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

}
