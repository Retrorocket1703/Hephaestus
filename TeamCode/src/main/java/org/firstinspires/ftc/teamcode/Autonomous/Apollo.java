/* Copyright (c) 2018 The Light Brigade Team 13507. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Object;
import java.util.concurrent.TimeUnit;
@Disabled
@Autonomous(name = "ApolloAutomotous", group = "Coeus")
public class Apollo extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private String Position = "Null";
    private DcMotor ForwardRight = null;
    private DcMotor ForwardLeft = null;
    private DcMotor BackRight = null;
    private DcMotor BackLeft = null;
    //private DcMotor EncoderY = null;
    //private DcMotor EncoderX = null;
    private DcMotor Lift = null;
    private static final String VUFORIA_KEY = "AXSqUwf/////AAABmVi9ifY9YkX7hla6dk//kFRSGKk+yWO9wuL666vmsKegzx+5Io14hzrDHPPSIOUIuBd89kbyiEj6ckbCr0Ak3uWm37MVO6WuHj2iyDGBiFASfDpXymVBKsTiT12M0KxuGVLdxg7JBLeNMq2f7lV/vtdmBf+UOVaruaibuICmy0jdCALIN7Edd3WcSYlY8V6VCxMPqLw4MrRbkPSshdxQ2WVZsDSZJgPqBE43qSmCFHhPwggb4+ZFSe6nn6SureHq5pNgyxeUXCaCsaIDbwHqEL7NrhOT6sqQTrWCsay3dFs6uVPY4YqJEP6+YGq7vYG7aJOC6nro3Z9NnVBhfwZEM07o6imRE0nkNd380XSO46rZ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addData(">-", "Press Play to start tracking");
        telemetry.update();

        double ForwardLeftPower;
        double ForwardRightPower;
        double BackLeftPower;
        double BackRightPower;
        int positionY;
        int positionX;
        int positionForwardLeft;
        int positionForwardRight;
        int positionBackLeft;
        int positionBackRight;
        boolean FinishedMove = false;
        ForwardLeft = hardwareMap.get(DcMotor.class, "ForwardLeft");
        ForwardLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ForwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ForwardRight = hardwareMap.get(DcMotor.class, "ForwardRight");
        ForwardRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ForwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /*EncoderY = hardwareMap.get(DcMotor.class, "EncoderY");
        EncoderX = hardwareMap.get(DcMotor.class, "EncoderX");*/
        ForwardLeft.setDirection(DcMotor.Direction.FORWARD);
        ForwardRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Lift.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*EncoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EncoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        //[{______~~~~~~~-------/|\-------~~~~~~~______}]

        waitForStart();

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">-", "Running");

        telemetry.update();

        /*Lift.setPower(1);

        Lift.setTargetPosition(2000);

        try
        {
            Thread.sleep(3000);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }

        DriveLeft(900, .4, 2000);

        DriveForward(800, .4, 2000);*/

        Recog(3000);

        telemetry.addData("Status", "VARaTiFf Finished Compilation");
        telemetry.addData("Position Mineral", Position);
        telemetry.update();

        if (Position == "CENTER" || Position == "Null") {
            BackRight.setPower(.4);
            ForwardLeft.setPower(.4);
            ForwardRight.setPower(.4);
            BackLeft.setPower(.4);
            BackRight.setTargetPosition(-1700);
            ForwardLeft.setTargetPosition(-1700);
            ForwardRight.setTargetPosition(-1700);
            BackLeft.setTargetPosition(-1700);
            try {
                Thread.sleep(3300);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            FinishedMove = true;
        }

        if (Position == "RIGHT") {
            BackRight.setPower(.4);
            ForwardLeft.setPower(.4);
            BackRight.setTargetPosition(-3000);
            ForwardLeft.setTargetPosition(-3000);
            try {
                Thread.sleep(3300);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            FinishedMove = true;
        }

        if (Position == "LEFT") {
            ForwardRight.setPower(.4);
            BackLeft.setPower(.4);
            ForwardRight.setTargetPosition(-3300);
            BackLeft.setTargetPosition(-3300);
            try {
                Thread.sleep(3300);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            FinishedMove = true;
        }


        /*if(Position == "CENTER" || Position == "Null")
        {
            if(FinishedMove == true) {
                BackRight.setPower(.4);
                ForwardLeft.setPower(.4);
                ForwardRight.setPower(.4);
                BackLeft.setPower(.4);
                BackRight.setTargetPosition(1700);
                ForwardLeft.setTargetPosition(1700);
                ForwardRight.setTargetPosition(1700);
                BackLeft.setTargetPosition(1700);
                try {
                    Thread.sleep(3300);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
                ForwardRight.setPower(.4);
                BackLeft.setPower(.4);
                ForwardRight.setTargetPosition(-2800);
                BackLeft.setTargetPosition(-2800);
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
            }
        }

        if(Position == "RIGHT")
        {
            if(FinishedMove == true) {
                BackRight.setPower(.4);
                ForwardLeft.setPower(.4);
                BackRight.setTargetPosition(3000);
                ForwardLeft.setTargetPosition(3000);
                try {
                    Thread.sleep(3500);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
                ForwardRight.setPower(.4);
                BackLeft.setPower(.4);
                ForwardRight.setTargetPosition(-2800);
                BackLeft.setTargetPosition(-2800);
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
            }
        }

        if(Position == "LEFT")
        {
            if(FinishedMove == true) {
                ForwardRight.setPower(.4);
                BackLeft.setPower(.4);
                ForwardRight.setTargetPosition(500);
                BackLeft.setTargetPosition(500);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
            }
        }*/

        telemetry.addData("Move back finished", "true");
        telemetry.update();

        /*positionY = EncoderY.getCurrentPosition() * 1120/1440;
        telemetry.addData("EncoderY", positionY);
        positionX = EncoderX.getCurrentPosition() * 1120/1440;
        telemetry.addData("EncoderX", positionX);
        positionForwardLeft = ForwardLeft.getCurrentPosition();
        positionForwardRight = ForwardRight.getCurrentPosition();
        telemetry.addData("EncoderForwardLeft", positionForwardLeft);
        telemetry.addData("EncoderForwardRight", positionForwardRight);
        positionBackLeft = BackLeft.getCurrentPosition();
        positionBackRight = BackRight.getCurrentPosition();
        telemetry.addData("EncoderForwardLeft", positionBackLeft);
        telemetry.addData("EncoderForwardRight", positionBackRight);*/


        telemetry.update();

    }

    public void DriveLeft(int movement, double power, int sleep) {
        BackRight.setPower(power);
        ForwardLeft.setPower(power);
        ForwardRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setTargetPosition(movement);
        ForwardLeft.setTargetPosition(movement);
        ForwardRight.setTargetPosition(-movement);
        BackLeft.setTargetPosition(-movement);
        try {
            Thread.sleep(sleep);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    public void DriveForward(int movement, double power, int sleep) {
        BackRight.setPower(power);
        ForwardLeft.setPower(power);
        ForwardRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setTargetPosition(-movement);
        ForwardLeft.setTargetPosition(-movement);
        ForwardRight.setTargetPosition(-movement);
        BackLeft.setTargetPosition(-movement);
        try {
            Thread.sleep(sleep);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    public void Recog(int timeout) {
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
                                goldMineralX = (int) recognition.getTop();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getTop();
                            } else {
                                silverMineral2X = (int) recognition.getTop();
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

//\~~~~****Boneyard****~~~~/

//private TFObjectDetector tfod;


// color_sensor = hardwareMap.colorSensor.get("color");
//color_sensor.enableLed(true);


            /*if (color_sensor.alpha() > 200 ) {
                telemetry.addData("Color", "White");
            }
            else {
                telemetry.addData("Color", "Not White");
            }*/

            /*telemetry.addData("ColorCode", color_sensor.alpha());
            telemetry.addData("Red", color_sensor.red());
            telemetry.addData("Green", color_sensor.green());
            telemetry.addData("Blue", color_sensor.blue());*/


            /*positionY = EncoderY.getCurrentPosition() * 1120/1440;
        positionX = EncoderX.getCurrentPosition() * 1120/1440;
        telemetry.addData("EncoderY", positionY);
        telemetry.addData("EncoderX", positionX);
        telemetry.update();
        /*positionForwardLeft = ForwardLeft.getCurrentPosition();
        positionForwardRight = ForwardRight.getCurrentPosition();
        telemetry.addData("EncoderForwardLeft", positionForwardLeft);
        telemetry.addData("EncoderForwardRight", positionForwardRight);
        positionBackLeft = BackLeft.getCurrentPosition();
        positionBackRight = BackRight.getCurrentPosition();
        telemetry.addData("EncoderForwardLeft", positionBackLeft);
        telemetry.addData("EncoderForwardRight", positionBackRight);*/

        /*positionForwardLeft = 1440/1120 * ForwardLeft.getCurrentPosition();
        telemetry.addData("EncoderForwardLeft", positionForwardLeft);
        telemetry.update();*/

       /* if(Position == "CENTER"){

        }

        if(Position == "RIGHT"){

        }

        if(Position == "LEFT"){

        }
        waitForTimeoutNTW(10000);*/
        /*ForwardRight.setPower(.4);
        BackLeft.setPower(.4);  //
        ForwardRight.setTargetPosition(-500);
        BackLeft.setTargetPosition(-500);
        waitForTimeoutNTW(3000);
        waitForTimeout(5000);
        telemetry.addData("Status", "VARaTiFf Finished Compilation");
        telemetry.addData("Position Mineral", Position);
        telemetry.update();
        waitForTimeoutNTW(10000);
        //telemetry.addData("Status", "VARaTiFf Finished Compilation");
        telemetry.update();

        if(Position == "CENTER")
        {

        }

        if(Position == "RIGHT")
        {

        }

        if(Position == "LEFT")
        {
            ForwardRight.setPower(1);
            BackLeft.setPower(1);
            ForwardRight.setTargetPosition(-2500);
            BackLeft.setTargetPosition(-2500);
        }*/


    /*public void waitForTimeoutNTW(int timeout) {
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        while (currentTime - startTime < timeout && opModeIsActive()) {
            telemetry.addData("Current Wait Time", currentTime - startTime);
            telemetry.update();

        }
    }*/

