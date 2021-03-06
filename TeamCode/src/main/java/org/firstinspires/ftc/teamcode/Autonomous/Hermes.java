package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    private double previousHeading = 0;
    private double integratedHeading = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Hermes Protocol Initialized");
        telemetry.update();

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

        //todo: I don't really know if the drop code will work right anymore? Besides it was kind of crappy to begin with. Try to find a better solution, moron.

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
            Thread.sleep(200);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        RightLift.setPower(-.4);
        LeftLift.setPower(-.4);
        RightLift.setTargetPosition(-2800);

        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setPower(.2);
        BackRight.setPower(.2);

        try {
            Thread.sleep(3000);
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

        ForwardLeft.setPower(.4);
        ForwardRight.setPower(.4);
        BackLeft.setPower(.4);
        BackRight.setPower(.4);

        BackRight.setTargetPosition(400);
        BackLeft.setTargetPosition(400);
        ForwardRight.setTargetPosition(400);
        ForwardLeft.setTargetPosition(400);

        while(ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){

        }

        changeMode(1);

        ForwardLeft.setTargetPosition(-240);
        ForwardRight.setTargetPosition(240);
        BackLeft.setTargetPosition(240);
        BackRight.setTargetPosition(-240);

        while(ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){

        }

        /*try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }*/

        changeMode(1);

        BackRight.setTargetPosition(-200);
        BackLeft.setTargetPosition(-200);
        ForwardRight.setTargetPosition(-200);
        ForwardLeft.setTargetPosition(-200);
        RightLift.setPower(1);
        RightLift.setTargetPosition(-1500);

        while(ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){

        }

        /*try {
            Thread.sleep(400);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }*/

        changeMode(1);

        ForwardLeft.setTargetPosition(240);
        ForwardRight.setTargetPosition(-240);
        BackLeft.setTargetPosition(-240);
        BackRight.setTargetPosition(240);

        while(ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){

        }

        /*try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }*/

        //Have burned through 8 seconds so far

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

            while(ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){

            }

            changeMode(1);

            ForwardLeft.setTargetPosition(-4000);
            ForwardRight.setTargetPosition(-4000);
            BackLeft.setTargetPosition(-4000);
            BackRight.setTargetPosition(-4000);

            while(ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){

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

            while(ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){

            }

            changeMode(1);

            ForwardLeft.setTargetPosition(500);
            ForwardRight.setTargetPosition(-500);
            BackLeft.setTargetPosition(-500);
            BackRight.setTargetPosition(500);

            while(ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){

            }

            changeMode(1);

            ForwardLeft.setTargetPosition(1100);
            ForwardRight.setTargetPosition(1100);
            BackLeft.setTargetPosition(1100);
            BackRight.setTargetPosition(1100);

            while(ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){

            }

            changeMode(1);

            ForwardLeft.setTargetPosition(400);
            ForwardRight.setTargetPosition(-400);
            BackLeft.setTargetPosition(400);
            BackRight.setTargetPosition(-400);

            while(ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){

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

            while(ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){

            }

        }


        if (Position == "RIGHT") {
            changeMode(1);

            ForwardLeft.setPower(.3);
            ForwardRight.setPower(.3);
            BackLeft.setPower(.3);
            BackRight.setPower(.3);

            ForwardLeft.setTargetPosition(-300);
            ForwardRight.setTargetPosition(-300);
            BackLeft.setTargetPosition(-300);
            BackRight.setTargetPosition(-300);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

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

            ForwardLeft.setTargetPosition(-300);
            ForwardRight.setTargetPosition(-300);
            BackLeft.setTargetPosition(-300);
            BackRight.setTargetPosition(-300);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

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


            ForwardRight.setPower(.3);
            BackLeft.setPower(.3);

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

            ForwardLeft.setTargetPosition(10000);
            ForwardRight.setTargetPosition(10000);
            BackLeft.setTargetPosition(10000);
            BackRight.setTargetPosition(10000);

            try {
                Thread.sleep(6000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }


        telemetry.addData("Status", "Completed Detection Execution");
        telemetry.update();


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

    /**
     * @param speed    can be any number UNDER 1, the lower the better
     * @param maxThres must be equal to 1 - speed, no more or less
     * @param distance is how far you want to move, in ticks.
     * @param heading  is the wanted heading in degrees
     */

    /*public void driveByPID(double speed, double maxThres, int distance, double heading) {
        //This grabs the error, and passes it the heading.
        double error = getError(heading);
        //This creates the proportional multiplier.
        double multiplier = (error / 180) * maxThres;

        //I forget if it works exactly like this or not. Guess we'll find you.

        ForwardRight.setTargetPosition(distance);
        BackRight.setTargetPosition(distance);
        ForwardLeft.setTargetPosition(distance);
        BackLeft.setTargetPosition(distance);

        //The theory behind this is that it will turn like a PID would.
        while (ForwardRight.isBusy() && BackRight.isBusy() && ForwardLeft.isBusy() && BackLeft.isBusy()) {
            //This grabs the error, and passes it the heading.
            error = getError(heading);
            //This creates the proportional multiplier.
            multiplier = (error / 180) * maxThres;
            ForwardRight.setPower(speed - multiplier);
            BackRight.setPower(speed - multiplier);
            ForwardLeft.setPower(speed + multiplier);
            BackLeft.setPower(speed + multiplier);
        }

    }

    public double getError(double heading) {
        double error;
        error = heading - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        return error;
    }*/

    /*
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     *
    public void gyroDrive(double speed, double distance, double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        //todo yeah like all of this is probs broken soooo fix it

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = EncoderY.getCurrentPosition() + moveCounts;
            newRightTarget = EncoderX.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            ForwardLeft.setTargetPosition(newLeftTarget);
            ForwardRight.setTargetPosition(newRightTarget);
            BackLeft.setTargetPosition(newLeftTarget);
            BackRight.setTargetPosition(newRightTarget);

            ForwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ForwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            ForwardLeft.setPower(speed);
            ForwardRight.setPower(speed);
            BackLeft.setPower(speed);
            BackRight.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (ForwardLeft.isBusy() && ForwardRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                //todo: this part is messed up too probably

                ForwardLeft.setPower(rightSpeed);
                ForwardRight.setPower(leftSpeed);
                BackLeft.setPower(rightSpeed);
                BackRight.setPower(leftSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d" /*robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            ForwardLeft.setPower(0);
            ForwardRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);


        }
    }

    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }


    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        //todo: Fix this, cause it's most likely broken...
        ForwardLeft.setPower(rightSpeed);
        ForwardRight.setPower(leftSpeed);
        BackLeft.setPower(rightSpeed);
        BackRight.setPower(leftSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }*/

    private double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }

    /*public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getIntegratedHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }*/

}
