package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.sun.tools.javac.util.Position;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.sql.Driver;

@Disabled
@TeleOp(name = "Ares", group = "Chronos")
public class Ares extends LinearOpMode {

    DcMotor ForwardRight = null;
    DcMotor ForwardLeft = null;
    DcMotor BackRight = null;
    DcMotor BackLeft = null;
    DcMotor EncoderY = null;
    DcMotor EncoderX = null;
    //DcMotor Lift = null;
    DcMotor Arm = null;
    DcMotor ArmLifter = null;
    CRServo Intake = null;
    boolean DriverControlled = true;
    double positionY;
    double positionX;
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
        double SpeedMod = .4;
        //String LiftMovingDown = "false";

        ForwardLeft = hardwareMap.get(DcMotor.class, "ForwardLeft");
        /*ForwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        ForwardLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ForwardRight = hardwareMap.get(DcMotor.class, "ForwardRight");
        /*ForwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        ForwardRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        /*BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        /*BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*ArmLifter = hardwareMap.get(DcMotor.class, "ArmExtend");
        ArmLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        EncoderY = hardwareMap.get(DcMotor.class, "EncoderY");
        EncoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        EncoderX = hardwareMap.get(DcMotor.class, "EncoderX");
        EncoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ForwardLeft.setDirection(DcMotor.Direction.FORWARD);
        ForwardRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        /*Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Main Init
            //DriverControlled = true;
            G1LY = gamepad1.left_stick_y;
            G1LX = gamepad1.left_stick_x;
            G1RX = gamepad1.right_stick_x;
            G1RY = gamepad1.right_stick_y;
            G2LY = gamepad2.left_stick_y;
            G2RY = gamepad2.right_stick_y;
            telemetry.addData("Status", "Running");
            telemetry.update();
            positionX = EncoderX.getCurrentPosition();
            positionY = EncoderY.getCurrentPosition();
            telemetry.addData("Encoder X Position", EncoderX.getCurrentPosition());
            telemetry.addData("Encoder Y Position", EncoderY.getCurrentPosition());
            //Driver controls
            if(DriverControlled = true) {

            ForwardLeftPower = G1LY + -G1RX;
            ForwardRightPower = G1LY + G1RX;
            BackRightPower = G1LY + -G1RX;
            BackLeftPower = G1LY + G1RX;

            /*if (gamepad2.a) {
                Lift.setPower(1);
            } else {
                Lift.setPower(0);
            }*/

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
            }


            //Operator controls
            if(gamepad2.a){
                /*DriverControlled = false;
                Lift.setPower(1);
                //Lift.setTargetPosition(-400);
                //LiftMovingDown = "true";
                try
                {
                    Thread.sleep(5000);
                }
                catch(InterruptedException ex)
                {
                    Thread.currentThread().interrupt();
                }
            DriverControlled = true;*/
            }
            if(gamepad2.y){
                int PositivePositionY;
                int PositivePositionX;
                //ReturnToZero();
                DriverControlled = false;
                BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ForwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ForwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                /*if(EncoderX.getCurrentPosition() <= 0){
                    PositivePositionX = EncoderX.getCurrentPosition() * -1;
                }
                else{
                    PositivePositionX = EncoderX.getCurrentPosition();
                }
                if(EncoderY.getCurrentPosition() <= 0){
                    PositivePositionY = EncoderY.getCurrentPosition() * -1;
                }
                else{
                    PositivePositionY = EncoderY.getCurrentPosition();
                }*/
                telemetry.update();
                BackLeft.setPower(1);
                BackRight.setPower(1);
                ForwardLeft.setPower(1);
                ForwardRight.setPower(1);
                BackLeft.setTargetPosition(EncoderX.getCurrentPosition());
                BackRight.setTargetPosition(EncoderX.getCurrentPosition());
                ForwardLeft.setTargetPosition(EncoderY.getCurrentPosition());
                ForwardRight.setTargetPosition(EncoderY.getCurrentPosition());

                try
                {
                    Thread.sleep(5000);
                }
                catch(InterruptedException ex)
                {
                    Thread.currentThread().interrupt();
                }
                DriverControlled = true;
                BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ForwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ForwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            //if(LiftMovingDown == "true"){
                //if(Lift.getCurrentPosition() <= 500){
                    //Lift.setPower(1);
                    //Lift.setTargetPosition(-500);
                //}
            //}

            //Arm.setPower(G2LY);
            //ArmLifter.setPower(G2RY);

            /*if(gamepad2.left_bumper){
                Intake.setDirection(CRServo.Direction.FORWARD);
            }
            if(gamepad2.right_bumper){
                Intake.setDirection(CRServo.Direction.REVERSE);
            }*/

        }
    }

    public void ReturnToZero(){
        DriverControlled = false;
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ForwardLeft = hardwareMap.get(DcMotor.class, "ForwardLeft");
        ForwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ForwardRight = hardwareMap.get(DcMotor.class, "ForwardRight");
        ForwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /*positionX = EncoderX.getCurrentPosition();
        positionY = EncoderY.getCurrentPosition();*/

        ForwardLeft.setTargetPosition(EncoderX.getCurrentPosition());
        ForwardRight.setTargetPosition(EncoderX.getCurrentPosition());
        BackLeft.setTargetPosition(EncoderY.getCurrentPosition());
        BackRight.setTargetPosition(EncoderY.getCurrentPosition());
        try
        {
            Thread.sleep(EncoderX.getCurrentPosition()+EncoderY.getCurrentPosition());
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
        DriverControlled = true;
    }
}