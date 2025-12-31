package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor front_left_drive = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        DcMotor front_right_drive = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        DcMotor Intake = hardwareMap.dcMotor.get("Intake");
        DcMotor Outtake = hardwareMap.dcMotor.get("Outtake");

        // Reverse the right side motors and back left motor
        front_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_drive.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Left joystick for movement (forward/backward/strafe)
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing

            // Right joystick for rotation (reversed)
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            front_left_drive.setPower(frontLeftPower);
            back_left_drive.setPower(backLeftPower);
            front_right_drive.setPower(frontRightPower);
            back_right_drive.setPower(backRightPower);

            // Control Intake and Outtake motors with triggers
            if (gamepad1.right_trigger > 0.1) {
                Intake.setPower(1.0);
            } else {
                Intake.setPower(0);
            }

            if (gamepad1.left_trigger > 0.1) {
                Outtake.setPower(0.4);
            } else {
                Outtake.setPower(0);
            }
        }
    }
}