package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Adafruit_Sensor_Test")
public class Adafruit_Sensor_Test extends LinearOpMode {

    ColorSensor colorSensor;

    public void runOpMode() {

        // initGyro();

        // frontleft = hardwareMap.dcMotor.get("frontleft");
        //  frontright = hardwareMap.dcMotor.get("frontright");
        // backleft = hardwareMap.dcMotor.get("backleft");
        // backright = hardwareMap.dcMotor.get("backright");

        // leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");
        // rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        //  touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");


        colorSensor.enableLed(true);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Red: ", colorSensor.red());
            telemetry.addData("Green: ", colorSensor.green());
            telemetry.addData("Blue: ", colorSensor.blue());

            // telemetry.addData("Master argb Value: ", colorSensor.argb());

            //    telemetry.addData("Arm Value: ", ArmMotor.getCurrentPosition());

            //  telemetry.addData("Left Distance Sensor: ", leftSensor.getDistance(DistanceUnit.MM));
            //   telemetry.addData("Right Distance Sensor: ", rightSensor.getDistance(DistanceUnit.MM));

           /* if (colorSensor.argb() < 100 && colorSensor.argb() > 0) {
                specialCone = 1;
            } else if (colorSensor.argb() > 100 && colorSensor.argb() < 200) {
                specialCone = 2;
            } else if (colorSensor.argb() > 200 && colorSensor.argb() < 300) {
                specialCone = 3;
            } */
            //  telemetry.addData("Position: ", Grabber.getPosition());

            //telemetry.addData("Touch Sensor: ", touchSensor.getValue());

            telemetry.update();

            /* Copyright (c) 2017 FIRST. All rights reserved.
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
             * Neither the name of FIRST nor the names of its contributors may be used to endorse or
             * promote products derived from this software without specific prior written permission.
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

//package org.firstinspires.ftc.robotcontroller.external.samples;

//import android.app.Activity;
//import android.graphics.Color;
//import android.view.View;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
//import com.qualcomm.robotcore.hardware.DigitalChannel;


        }
    }
}