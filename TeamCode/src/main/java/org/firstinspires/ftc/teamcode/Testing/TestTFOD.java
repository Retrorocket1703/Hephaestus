package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.Dogeforia;

import java.util.List;
@Disabled
@TeleOp
public class TestTFOD extends LinearOpMode {


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AXSqUwf/////AAABmVi9ifY9YkX7hla6dk//kFRSGKk+yWO9wuL666vmsKegzx+5Io14hzrDHPPSIOUIuBd89kbyiEj6ckbCr0Ak3uWm37MVO6WuHj2iyDGBiFASfDpXymVBKsTiT12M0KxuGVLdxg7JBLeNMq2f7lV/vtdmBf+UOVaruaibuICmy0jdCALIN7Edd3WcSYlY8V6VCxMPqLw4MrRbkPSshdxQ2WVZsDSZJgPqBE43qSmCFHhPwggb4+ZFSe6nn6SureHq5pNgyxeUXCaCsaIDbwHqEL7NrhOT6sqQTrWCsay3dFs6uVPY4YqJEP6+YGq7vYG7aJOC6nro3Z9NnVBhfwZEM07o6imRE0nkNd380XSO46rZ";
    BNO055IMU imu;
    double positionY, positionX;
    Orientation angles;
    DcMotor ForwardRight, ForwardLeft, BackLeft, BackRight, EncoderY, EncoderX, LeftLift, RightLift, CableSpool, Intake;
    private String Position = "Null";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private GoldAlignDetector detector;
    WebcamName webcamName;
    Dogeforia Vuforia;

    @Override
    public void runOpMode() {


        initVuforia();
        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData(">-", "Press Play to start tracking");
        telemetry.update();
        Vuforia.setDogeCVDetector(detector);
        Vuforia.enableDogeCV();
        Vuforia.showDebug();
        Vuforia.start();
        //InitAll();

        waitForStart();

       /* if (tfod != null) {
            tfod.activate();
        }

        Recog(4000);

        telemetry.addData("Current Pos",Position);
        telemetry.update();

        try {
            Thread.sleep(8000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }


        if (tfod != null) {
            tfod.shutdown();
        }*/

       while(opModeIsActive()){
           telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
           telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.
       }
    }


    void ReturnToHeading(int MinHeading, int MaxHeading) {
        ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float zAxis = angles.firstAngle;

        //If this is a positive number, it means the inside distance from the right to the 0 heading
        while (zAxis < MinHeading) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            zAxis = angles.firstAngle;
            ForwardLeft.setPower(.3);
            ForwardRight.setPower(-.3);
            BackLeft.setPower(.3);
            BackRight.setPower(-.3);
        }

        //if this is a negative number, it means the inside distance from the left to the 0 heading
        while (zAxis > MaxHeading) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            zAxis = angles.firstAngle;
            ForwardLeft.setPower(-.3);
            ForwardRight.setPower(.3);
            BackLeft.setPower(-.3);
            BackRight.setPower(.3);
        }
    }

    void InitAll() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        //LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftLift.setDirection(DcMotor.Direction.REVERSE);
        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightLift = hardwareMap.get(DcMotor.class, "RightLift");
        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CableSpool = hardwareMap.get(DcMotor.class, "CableSpool");
        CableSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CableSpool.setDirection(DcMotor.Direction.REVERSE);
        CableSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake = hardwareMap.get(DcMotor.class, "EncoderX");
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ForwardLeft.setDirection(DcMotor.Direction.FORWARD);
        ForwardRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData(">-", "Press Play to start tracking");
        telemetry.update();
    }

    void Recog(int timeout) {
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        while (currentTime - startTime < timeout && opModeIsActive()) {
            telemetry.addData("time", currentTime - startTime);
            telemetry.update();
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.update();
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;

                        // This just records values, and is unchanged

                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft(); //changed this from getTop to getLeft
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }

                        // If there is no gold (-1) and there two silvers (not -1) the gold
                        // is not visible, and must be on the right

                        if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            Position = "RIGHT";
                            telemetry.update();
                        }

                        // If you can see one gold and one silver ...

                        else if (goldMineralX != -1 && silverMineral1X != -1) {
                            // ... if the gold is to the right of the silver, the gold is in the center ...


                            if (goldMineralX > silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Center");
                                Position = "CENTER";
                                telemetry.addData("Position", Position);
                                telemetry.update();
                            }

                            // ... otherwise it is on the left

                            else {
                                telemetry.addData("Gold Mineral Position", "Left");
                                Position = "LEFT";
                                telemetry.addData("Position", Position);
                                telemetry.update();
                            }
                        }
                    }
                }
            }
            telemetry.update();

            currentTime = System.currentTimeMillis();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        Dogeforia.Parameters parameters = new Dogeforia.Parameters();
        VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia = new Dogeforia(parameters);
        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
