package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Pasithea", group = "Coeus")

public class Pasithea extends LinearOpMode {

    DcMotor Main;
    double G2LY;

    public void runOpMode() {
        Main = hardwareMap.get(DcMotor.class, "Main");

        waitForStart();

        while (opModeIsActive()) {
            G2LY = gamepad1.left_stick_y;
            Main.setPower(G2LY);
        }
    }
}