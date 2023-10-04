package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="RawBaconTeleop")

public class RawBaconTeleop extends OpMode {

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotorEx ArmMotor;
    Servo Grabber;

    Double Speed;
    Double Velocity;

    @Override
    public void init() {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        Grabber = hardwareMap.get(Servo.class, "Grabber");

        Speed = 0.6;

        ArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        Velocity = 0.0;
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {

        if (gamepad1.left_trigger == 1) {
            Speed = 0.2;
        } else if (gamepad1.left_trigger == 0) {
            Speed = 0.6;
        }

        if (gamepad2.left_bumper) {
            ArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        double Pad2LeftStickY = gamepad2.left_stick_y;
        double LeftStickY = gamepad1.left_stick_y;
        double LeftStickX = -gamepad1.left_stick_x;
        double RightStickX = -gamepad1.right_stick_x;

        frontright.setPower((-RightStickX / 1.5 + (LeftStickY - LeftStickX)) * Speed);
        backright.setPower((-RightStickX / 1.5 + (LeftStickY + LeftStickX)) * Speed);
        frontleft.setPower((RightStickX / 1.5 + (LeftStickY + LeftStickX)) * Speed);
        backleft.setPower((RightStickX / 1.5 + (LeftStickY - LeftStickX)) * Speed);

//D-PAD STRAFING CODE (NOT WORKING RIGHT)
      /*  if (gamepad1.dpad_left) {
            frontright.setPower(-0.6);
            frontleft.setPower(0.6);
            backleft.setPower(-0.6);
            backright.setPower(0.6);
        } else if (gamepad1.dpad_right) {
            frontright.setPower(0.6);
            frontleft.setPower(-0.6);
            backleft.setPower(0.6);
            backright.setPower(-0.6);
        } else if (gamepad1.dpad_up) {
            frontright.setPower(1);
            frontleft.setPower(1);
            backleft.setPower(1);
            backright.setPower(1);
        } else if (gamepad2.dpad_down) {
            frontright.setPower(-1);
            frontleft.setPower(-1);
            backleft.setPower(-1);
            backright.setPower(-1);
        }*/

        ArmMotor.setPower((Pad2LeftStickY / 1.5));

        if (gamepad2.right_bumper) {
            Grabber.setPosition(0.4);

        } else if (gamepad2.right_trigger == 1) {
            Grabber.setPosition(0.15);
        }


        if (gamepad2.a) {
            ArmMotor.setTargetPosition(-1450);
            ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            ArmMotor.setVelocity(3000);

        } else if (gamepad2.b) {
            ArmMotor.setTargetPosition(-2300);
            ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            ArmMotor.setVelocity(3000);

        } else if (gamepad2.x) {
            ArmMotor.setTargetPosition(-425);
            ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            ArmMotor.setVelocity(2500);

        } else if (gamepad2.y) {
            ArmMotor.setTargetPosition(-3175);
            ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            ArmMotor.setVelocity(3000);

        } else if (!gamepad2.y && !gamepad2.b && !gamepad2.a && !gamepad2.x) {
            ArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
}