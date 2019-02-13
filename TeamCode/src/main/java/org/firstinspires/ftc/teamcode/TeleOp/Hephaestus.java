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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name = "Hephaestus", group = "Chronos")
public class Hephaestus extends LinearOpMode {

    protected boolean DriverControlled = true;
    DcMotor ForwardRight, ForwardLeft, BackLeft, BackRight;
    DcMotor EncoderY, EncoderX;
    DcMotor LeftLift, RightLift;
    DcMotor CableSpool, Intake;
    Servo Flap;
    BNO055IMU imu;
    double positionY, positionX;
    int RightLeaderPos;
    Orientation angles;
    private double previousHeading = 0;
    private double integratedHeading = 0;

    //ExpansionHubEx expansionHub;


    //ColorSensor color_sensor;
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        double ForwardLeftPower;
        double ForwardRightPower;
        double BackLeftPower;
        double BackRightPower;
        double G1LY;
        double G1LX;
        double G1RX;
        double G1RY;
        double G2LY;
        double G2RY;
        double SpeedMod = .6;
        double LastHeading = 0;
        double CurrentCompassHeading = 0;

        int ArmPos = 1, CableSpoolPos = 1, ArmPosStop = 2, CableSpoolPosStop = 2;
        //String LiftMovingDown = "false";

        Flap = hardwareMap.get(Servo.class, "Flap");

        ForwardLeft = hardwareMap.get(DcMotor.class, "ForwardLeft");
        /*ForwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        ForwardLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ForwardRight = hardwareMap.get(DcMotor.class, "ForwardRight");
        /*ForwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        ForwardRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        /*BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        /*BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*ArmLifter = hardwareMap.get(DcMotor.class, "ArmExtend");
        ArmLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        EncoderY = hardwareMap.get(DcMotor.class, "EncoderY");
        EncoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        EncoderX = hardwareMap.get(DcMotor.class, "EncoderX");
        EncoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift = hardwareMap.get(DcMotor.class, "EncoderY");
        //BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftLift.setDirection(DcMotor.Direction.REVERSE);
        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightLift = hardwareMap.get(DcMotor.class, "RightLift");
        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CableSpool = hardwareMap.get(DcMotor.class, "CableSpool");
        //BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CableSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CableSpool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        boolean isG2APressed = false, isG2YPressed = false, HasPassed180 = false;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            float zAxis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            telemetry.addData("RightLiftActualPos", RightLift.getCurrentPosition());
            telemetry.addData("CableSpoolActualPos", CableSpool.getCurrentPosition());
            /**
             * Main Init
             */
            G1LY = gamepad1.left_stick_y;
            G1LX = gamepad1.left_stick_x;
            G1RX = gamepad1.right_stick_x;
            G1RY = gamepad1.right_stick_y;
            G2LY = gamepad2.left_stick_y;
            G2RY = -gamepad2.right_stick_y;
            telemetry.addData("Status", "Running");
            telemetry.update();

            /**
             * Driver Controls
             */
            //if(DriverControlled = true) {

            ForwardLeftPower = G1LY + -G1RX;
            ForwardRightPower = G1LY + G1RX;
            BackRightPower = G1LY + -G1RX;
            BackLeftPower = G1LY + G1RX;

            if (gamepad1.a) {
                SpeedMod = .4;
            }
            if (gamepad1.b) {
                SpeedMod = .6;
            }
            if (gamepad1.y) {
                SpeedMod = .8;
            }
            if (gamepad1.x) {
                SpeedMod = 1;
            }

            while (gamepad1.dpad_left) {
                ForwardLeft.setPower(1 * SpeedMod);
                ForwardRight.setPower(-1 * SpeedMod);
                BackLeft.setPower(1 * SpeedMod);
                BackRight.setPower(-1 * SpeedMod);
                ForwardLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                ForwardRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            while (gamepad1.dpad_right) {
                ForwardLeft.setPower(-1 * SpeedMod);
                ForwardRight.setPower(1 * SpeedMod);
                BackLeft.setPower(-1 * SpeedMod);
                BackRight.setPower(1 * SpeedMod);
                ForwardLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                ForwardRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if (gamepad1.atRest()) {
                ForwardLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                ForwardRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                ForwardLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                ForwardRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            ForwardLeft.setPower(ForwardLeftPower * SpeedMod);
            ForwardRight.setPower(ForwardRightPower * SpeedMod);
            BackLeft.setPower(BackLeftPower * SpeedMod);
            BackRight.setPower(BackRightPower * SpeedMod);

            /**
             * Operator controls
             * */

            if (gamepad2.x) {
                Flap.setPosition(.37);
            } else {
                Flap.setPosition(.1);
            }

            if (gamepad2.dpad_right) {
                Intake.setPower(.3);
            }
            if (gamepad2.dpad_left) {
                Intake.setPower(-1);
            }
            if (gamepad2.dpad_down) {
                Intake.setPower(0);
            }

            if (RightLift.getCurrentPosition() >= -30) {
                ArmPos = 1;
            } else if (RightLift.getCurrentPosition() <= -6000) {
                ArmPos = 3;
            } else {
                ArmPos = 2;
            }

            if (ArmPos != 3 && G2RY <= 0) {
                RightLift.setPower(G2RY);
                LeftLift.setPower(G2RY);
            } else if (ArmPos != 1 && G2RY >= 0) {
                RightLift.setPower(G2RY);
                LeftLift.setPower(G2RY);
            } else if (CableSpoolPos != 3 && G2RY >= 0) {
                RightLift.setPower(0);
                LeftLift.setPower(0);
            } else if (CableSpoolPos != 1 && G2LY <= 0) {
                RightLift.setPower(0);
                LeftLift.setPower(0);
            } else {
                RightLift.setPower(G2RY);
                LeftLift.setPower(G2RY);
            }


            if (CableSpool.getCurrentPosition() >= -5 && CableSpoolPos != 13) {
                CableSpoolPos = 1;
            } else if (CableSpool.getCurrentPosition() <= -3300 && CableSpoolPos != 13) {
                CableSpoolPos = 3;
            } else {
                CableSpoolPos = 2;
            }

            if (CableSpoolPos != 3 && G2LY <= 0) {
                CableSpool.setPower(G2LY);
            } else if (CableSpoolPos != 1 && G2LY >= 0) {
                CableSpool.setPower(G2LY);
            } else if (CableSpoolPos != 3 && G2LY >= 0) {
                CableSpool.setPower(0);
            } else if (CableSpoolPos != 1 && G2LY <= 0) {
                CableSpool.setPower(0);
            } else {
                CableSpool.setPower(G2LY);
            }

            while (gamepad2.right_bumper) {
                RightLift.setPower(1);
                LeftLift.setPower(1);
                RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

            if (gamepad2.a) {
                EncoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                EncoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                EncoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                EncoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            //TODO: Fix this crap
            while (gamepad2.y) {

                DriverControlled = false;
                getIntegratedHeading();
                int m_MODPOSY = EncoderY.getCurrentPosition() - (int) getIntegratedHeading();
                int m_MODPOSX = EncoderX.getCurrentPosition() - (int) getIntegratedHeading();
                long zAxisExtrap = Math.abs((int) zAxis);

                telemetry.addData("Mod Pos Y", m_MODPOSY);
                telemetry.addData("Mod Pos X", m_MODPOSX);
                telemetry.update();

                while (zAxis < -5) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    zAxis = angles.firstAngle;
                    ForwardLeft.setPower(.3);
                    ForwardRight.setPower(-.3);
                    BackLeft.setPower(.3);
                    BackRight.setPower(-.3);
                }

                while (zAxis > 5) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    zAxis = angles.firstAngle;
                    ForwardLeft.setPower(-.3);
                    ForwardRight.setPower(.3);
                    BackLeft.setPower(-.3);
                    BackRight.setPower(.3);
                }

                try {
                    Thread.sleep(zAxisExtrap);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }

                long waitfor = Math.abs(m_MODPOSX + m_MODPOSY + (int) zAxis);

                while (m_MODPOSX <= -40) {
                    ForwardLeft.setPower(-.4);
                    ForwardRight.setPower(.4);
                    BackLeft.setPower(.4);
                    BackRight.setPower(-.4);
                    m_MODPOSY = EncoderY.getCurrentPosition();
                    m_MODPOSX = EncoderX.getCurrentPosition();
                }

                while (m_MODPOSX >= 40) {
                    ForwardLeft.setPower(.4);
                    ForwardRight.setPower(-.4);
                    BackLeft.setPower(-.4);
                    BackRight.setPower(.4);
                    m_MODPOSY = EncoderY.getCurrentPosition();
                    m_MODPOSX = EncoderX.getCurrentPosition();
                }

                if (m_MODPOSX <= 40 || m_MODPOSX >= -40) {
                    ForwardLeft.setPower(0);
                    ForwardRight.setPower(0);
                    BackLeft.setPower(0);
                    BackRight.setPower(0);
                    m_MODPOSY = EncoderY.getCurrentPosition();
                    m_MODPOSX = EncoderX.getCurrentPosition();
                }

                try {
                    Thread.sleep(waitfor / 2);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }

                while (m_MODPOSY <= -40) {
                    ForwardLeft.setPower(.4);
                    ForwardRight.setPower(.4);
                    BackLeft.setPower(.4);
                    BackRight.setPower(.4);
                    m_MODPOSY = EncoderY.getCurrentPosition();
                    m_MODPOSX = EncoderX.getCurrentPosition();
                }

                while (m_MODPOSY >= 40) {
                    ForwardLeft.setPower(-.4);
                    ForwardRight.setPower(-.4);
                    BackLeft.setPower(-.4);
                    BackRight.setPower(-.4);
                    m_MODPOSY = EncoderY.getCurrentPosition();
                    m_MODPOSX = EncoderX.getCurrentPosition();
                }

                if (m_MODPOSY <= 40 || m_MODPOSY >= -40) {
                    ForwardLeft.setPower(0);
                    ForwardRight.setPower(0);
                    BackLeft.setPower(0);
                    BackRight.setPower(0);
                    m_MODPOSY = EncoderY.getCurrentPosition();
                    m_MODPOSX = EncoderX.getCurrentPosition();
                }

                try {
                    Thread.sleep(waitfor / 2);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }

                ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                DriverControlled = true;
            }
        }
    }

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
}