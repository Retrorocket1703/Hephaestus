/* Copyright (c) 2018 The Light Brigade, Team 13507. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

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

//Aeolus Is to be used when the robot is facing the Depot in the starting position
@Disabled
@Autonomous(name = "Aeolus", group = "Coeus")
public class Aeolus extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AXSqUwf/////AAABmVi9ifY9YkX7hla6dk//kFRSGKk+yWO9wuL666vmsKegzx+5Io14hzrDHPPSIOUIuBd89kbyiEj6ckbCr0Ak3uWm37MVO6WuHj2iyDGBiFASfDpXymVBKsTiT12M0KxuGVLdxg7JBLeNMq2f7lV/vtdmBf+UOVaruaibuICmy0jdCALIN7Edd3WcSYlY8V6VCxMPqLw4MrRbkPSshdxQ2WVZsDSZJgPqBE43qSmCFHhPwggb4+ZFSe6nn6SureHq5pNgyxeUXCaCsaIDbwHqEL7NrhOT6sqQTrWCsay3dFs6uVPY4YqJEP6+YGq7vYG7aJOC6nro3Z9NnVBhfwZEM07o6imRE0nkNd380XSO46rZ";
    DcMotor ForwardRight, ForwardLeft, BackLeft, BackRight, EncoderY, EncoderX, LeftLift, RightLift, CableSpool, Intake;
    Servo Flap;
    BNO055IMU imu;
    Orientation angles;
    private String Position = "NULL";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    //ColorSensor color_sensor;
    public void runOpMode() {


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


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

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData(">-", "Press Play to start tracking");
        telemetry.update();

        waitForStart();

        if (tfod != null) {
            tfod.activate();
        }

        float zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        telemetry.addData("Rotation", zAxis);

        telemetry.addData("Status", "Running");
        telemetry.update();

        /*RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightLift.setPower(-.5);
        LeftLift.setPower(-.5);*/
        RightLift.setPower(.5);
        RightLift.setTargetPosition(-1900);

        //DriveRunToPos(400, 400, 400, 400, .6, .6, .6, .6, 1500);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        /*RightLift.setPower(0);
        LeftLift.setPower(0);*/

        StopAndReset(1);

        DriveRunToPos(350, 350, 350, 350, .6, .6, .6, .6, 1500);

        StopAndReset(1);

        DriveRunToPos(-200, 200, 200, -200, .5, .5, .5, .5, 1000);

       /* StopAndReset(1);

        DriveRunToPos(-200, -200, -200, -200, .5, .5, .5, .5, 900);

        StopAndReset(1);*/

        try {
            Thread.sleep(2000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        telemetry.addData("Rotation", zAxis);
        ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //If this is a positive number, it means the inside distance from the right to the 0 heading
        boolean Straight = false;
        /*while (NotStraight != false) {
            ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (zAxis < 0) {
                zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                telemetry.addData("Rotation", zAxis);
                telemetry.update();
                ForwardLeft.setPower(.3);
                ForwardRight.setPower(-.3);
                BackLeft.setPower(.3);
                BackRight.setPower(-.3);

            }

            while (zAxis > 0) {
                zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                telemetry.addData("Rotation", zAxis);
                telemetry.update();
                ForwardLeft.setPower(-.3);
                ForwardRight.setPower(.3);
                BackLeft.setPower(-.3);
                BackRight.setPower(.3);

            }
            if (zAxis <= 5 && zAxis >= -5) {
                NotStraight = false;
            } else {
                NotStraight = true;
            }
        }*/

        ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        InitialReturn(5000);
        /*ForwardLeft.setPower(-.3);
        ForwardRight.setPower(.3);
        BackLeft.setPower(-.3);
        BackRight.setPower(.3);
        telemetry.addData("Currently Sleeping?", "True");
        telemetry.update();
        try {
            Thread.sleep(450);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        ForwardLeft.setPower(0);
        ForwardRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);*/

        Recog(3000);

        StopAndReset(1);

        telemetry.addData("Sample Position", Position);
        telemetry.update();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zAxis = angles.firstAngle;
        if (Position == "CENTER" || Position == "NULL") {
            DriveRunToPos(-3000, -3000, -3000, -3000, .4, .4, .4, .4, 3300);

            /*ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            zAxis = angles.firstAngle;

            //If this is a positive number, it means the inside distance from the right to the 0 heading
            /*while (zAxis < -5) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                zAxis = angles.firstAngle;
                ForwardLeft.setPower(.3);
                ForwardRight.setPower(-.3);
                BackLeft.setPower(.3);
                BackRight.setPower(-.3);
            }

            //if this is a negative number, it means the inside distance from the left to the 0 heading
            while (zAxis > 5) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                zAxis = angles.firstAngle;
                ForwardLeft.setPower(-.3);
                ForwardRight.setPower(.3);
                BackLeft.setPower(-.3);
                BackRight.setPower(.3);
            }*/
        }

        if (Position == "RIGHT") {
            DriveRunToPos(100, 100, 100, 100, .3, .3, .3, .3, 1000);
            BackRight.setPower(.4);
            ForwardLeft.setPower(.4);
            BackRight.setTargetPosition(-3000);
            ForwardLeft.setTargetPosition(-3000);

            try {
                Thread.sleep(3300);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            zAxis = angles.firstAngle;

            //If this is a positive number, it means the inside distance from the right to the 0 heading
            while (zAxis < -40) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                zAxis = angles.firstAngle;
                ForwardLeft.setPower(.3);
                ForwardRight.setPower(-.3);
                BackLeft.setPower(.3);
                BackRight.setPower(-.3);
            }

            //if this is a negative number, it means the inside distance from the left to the 0 heading
            while (zAxis > -50) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                zAxis = angles.firstAngle;
                ForwardLeft.setPower(-.3);
                ForwardRight.setPower(.3);
                BackLeft.setPower(-.3);
                BackRight.setPower(.3);
            }
        }

        if (Position == "LEFT") {
            DriveRunToPos(100, 100, 100, 100, .3, .3, .3, .3, 1000);
            ForwardRight.setPower(.4);
            BackLeft.setPower(.4);
            ForwardRight.setTargetPosition(-3000);
            BackLeft.setTargetPosition(-3000);
            try {
                Thread.sleep(3300);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            zAxis = angles.firstAngle;

            //If this is a positive number, it means the inside distance from the right to the 0 heading
            while (zAxis < 40) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                zAxis = angles.firstAngle;
                ForwardLeft.setPower(.3);
                ForwardRight.setPower(-.3);
                BackLeft.setPower(.3);
                BackRight.setPower(-.3);
            }

            //if this is a negative number, it means the inside distance from the left to the 0 heading
            while (zAxis > 50) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                zAxis = angles.firstAngle;
                ForwardLeft.setPower(-.3);
                ForwardRight.setPower(.3);
                BackLeft.setPower(-.3);
                BackRight.setPower(.3);
            }
        }

        StopAndReset(1);

        DriveRunToPos(-800, -800, -800, -800, .5, .5, .5, .5, 1000);

        RightLift.setPower(-.2);
        LeftLift.setPower(-.2);
        // TODO: Adjust to following value to reflect where it actually needs to be
        Flap.setPosition(.5);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        RightLift.setPower(0);
        LeftLift.setPower(0);


    }

    void InitialReturn(long timeout) {
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        while (currentTime - startTime < timeout && opModeIsActive()) {
            float zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            if (zAxis > 6) {
                zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                telemetry.addData("Rotation", zAxis);
                telemetry.update();
                ForwardLeft.setPower(-.2);
                ForwardRight.setPower(.2);
                BackLeft.setPower(-.2);
                BackRight.setPower(.2);
            }
            if (zAxis < -6) {
                zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                telemetry.addData("Rotation", zAxis);
                telemetry.update();
                ForwardLeft.setPower(.2);
                ForwardRight.setPower(-.2);
                BackLeft.setPower(.2);
                BackRight.setPower(-.2);
            }
            currentTime = System.currentTimeMillis();
        }
    }

    void ReturnToHeadingNegative(int MinHeading, int MaxHeading) {
        ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float zAxis = angles.firstAngle;

        //If this is a positive number, it means the inside distance from the right to the 0 heading
        while (zAxis > MinHeading) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            zAxis = angles.firstAngle;
            ForwardLeft.setPower(.3);
            ForwardRight.setPower(-.3);
            BackLeft.setPower(.3);
            BackRight.setPower(-.3);
        }

        //if this is a negative number, it means the inside distance from the left to the 0 heading
        while (zAxis < MaxHeading) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            zAxis = angles.firstAngle;
            ForwardLeft.setPower(-.3);
            ForwardRight.setPower(.3);
            BackLeft.setPower(-.3);
            BackRight.setPower(.3);
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

    void StopAndReset(int Mode) {
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
    }

    void DriveRunToPos(int ForwardLeftPos, int ForwardRightPos, int BackLeftPos, int BackRightPos, double ForwardLeftPower, double ForwardRightPower, double BackLeftPower, double BackRightPower, long timeout) {
        ForwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ForwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ForwardLeft.setPower(ForwardLeftPower);
        ForwardRight.setPower(ForwardRightPower);
        BackLeft.setPower(BackLeftPower);
        BackRight.setPower(BackRightPower);
        ForwardLeft.setTargetPosition(ForwardLeftPos);
        ForwardRight.setTargetPosition(ForwardRightPos);
        BackLeft.setTargetPosition(BackLeftPos);
        BackRight.setTargetPosition(BackRightPos);
        try {
            Thread.sleep(timeout);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
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
                                goldMineralX = (int) recognition.getLeft();
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

                        // ... otherwise it is on the left

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
}

