package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name = "Machai", group = "Chronos")
public class Machai extends LinearOpMode{

    DcMotor ForwardRight = null;
    DcMotor ForwardLeft = null;
    DcMotor BackRight = null;
    DcMotor BackLeft = null;
    DcMotor EncoderY = null;
    DcMotor EncoderX = null;
    DcMotor Lift = null;
    DcMotor Arm = null;
    DcMotor ArmExtend = null;

    public void runOpMode() {

        telemetry.addData(">-", "Press Play to start tracking");
        telemetry.update();

        double ForwardLeftPower;
        double ForwardRightPower;
        double BackLeftPower;
        double BackRightPower;
        int positionY;
        int positionX;
        float LeftStickY;
        float RightStickY;

        ForwardLeft  = hardwareMap.get(DcMotor.class, "ForwardLeft");
        ForwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ForwardLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ForwardRight = hardwareMap.get(DcMotor.class, "ForwardRight");
        ForwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ForwardRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //EncoderY = hardwareMap.get(DcMotor.class, "EncoderY");
        //EncoderX = hardwareMap.get(DcMotor.class, "EncoderX");
        ForwardLeft.setDirection(DcMotor.Direction.FORWARD);
        ForwardRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtend = hardwareMap.get(DcMotor.class, "ArmExtend");
        ArmExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*EncoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EncoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        waitForStart();

        while (opModeIsActive()) {

            LeftStickY = gamepad1.left_stick_y;
            RightStickY = gamepad1.right_stick_y;

            /*positionY = EncoderY.getCurrentPosition();
            positionX = EncoderX.getCurrentPosition();*/
            telemetry.addData("EncoderY", EncoderY);
            telemetry.addData("EncoderX", EncoderX);
            telemetry.addData("ForwardLeft", ForwardLeft.getCurrentPosition());
            telemetry.addData("ForwardRight", ForwardRight.getCurrentPosition());
            telemetry.addData("BackLeft", BackLeft.getCurrentPosition());
            telemetry.addData("BackRight", BackRight.getCurrentPosition());
            telemetry.update();

            /*Arm.setPower(LeftStickY);
            ArmExtend.setPower(RightStickY * .4);*/

            if(gamepad1.a){
                Lift.setPower(1);
            }
            if(gamepad1.b){
                Lift.setPower(-1);
            }
            if(gamepad1.y){
                Lift.setPower(0);
            }

        }
    }
}
