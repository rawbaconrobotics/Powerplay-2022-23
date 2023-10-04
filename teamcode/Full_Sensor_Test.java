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

@Autonomous(name="Full_Sensor_Test")
public class Full_Sensor_Test extends LinearOpMode {

   // DistanceSensor leftSensor;
   // DistanceSensor rightSensor;

   // TouchSensor touchSensor;

    ColorSensor colorSensor;

   // Servo poleMeasure;

    int specialCone;

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotorEx ArmMotor;
    Servo Grabber;

    //Double width = 17.32; //inches
    Double cpr = 28.0; //counts per rotation
    Double gearratio = 13.7;
    Double diameter = 3.5;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8; //default 0.8
    Double meccyBias = 1.0; //change to adjust only strafing movement

    Double conversion = cpi * bias;
    Boolean exit = false;

    //BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    public void runOpMode() {

       // initGyro();

        frontleft = hardwareMap.dcMotor.get("frontleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");

        // leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");
        // rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        //  touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        Grabber = hardwareMap.get(Servo.class, "Grabber");
        //poleMeasure = hardwareMap.get(Servo.class, "poleMeasure");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);

        ArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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
    }

//TEST DISTANCE SENSOR CODE (VALUES ARE CURRENTLY RANDOM)
        /*
        while(rightSensor.getDistance(DistanceUnit.MM) >= 200) {

             strafeToPosition(1, 0.1);
             sleep(10);
         } */

//TEST COLOR SENSOR READ CODE (VALUES ARE CURRENTLY RANDOM)

      //  telemetry.update();

       // sleep(5000);

//STANDARD AUTO (CAPS ONE (1) CONE)
       /* ArmMotor.setTargetPosition(-500);
        ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmMotor.setVelocity(2000);

        sleep(1000);

        moveToPosition(3.5, 0.3);

        sleep(1000);

        ArmMotor.setTargetPosition(-250);
        ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmMotor.setVelocity(1600);

        Grabber.setPosition(0.35);

        sleep(1000);

        ArmMotor.setTargetPosition(-600);
        ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmMotor.setVelocity(2000);

        sleep(1000);

        moveToPosition(-2, 0.3);

        sleep(250);

        strafeToPosition(-24, 0.2);

        sleep(500);

        moveToPosition(24, 0.2);

        sleep(500);

        strafeToPosition(-13.5, 0.2);

        sleep(500);

        moveToPosition(3, 0.3);

        sleep(250);

        ArmMotor.setTargetPosition(-3175);
        ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmMotor.setVelocity(2000);

        sleep(3000);

        moveToPosition(7, 0.1);

        sleep(1000);

        Grabber.setPosition(0.15);

        sleep(1000);

        moveToPosition(-6, 0.1);

        sleep(1000);

        ArmMotor.setTargetPosition(-500);
        ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmMotor.setVelocity(1600);

        sleep(350);

// PARKING CODE (MOVEMENT VALUES ARE CURRENTLY RANDOM
        if (specialCone == 1) {
            strafeToPosition(-30, 0.3);

            sleep(150);

            moveToPosition(-10, 0.3);

        } else if (specialCone == 2) {
            moveToPosition(-3, 0.6);

            sleep(150);

            strafeToPosition(36, 0.6);

        } else if (specialCone == 3) {
            moveToPosition(-3, 0.6);

            sleep(150);

            strafeToPosition(12, 0.6);
        }

        sleep(50); */
    }


    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() - move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }

   /* public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devert(yaw);
                second = degrees + devert(yaw);
            }else{
                first = devert(yaw);
                second = degrees + devert(yaw);
            }
            //</editor-fold>
        }else {
            //<editor-fold desc="turn left">
            if (degrees > 10) {
                first = devert(-(degrees - 10) + devert(yaw));
                second = devert(-degrees + devert(yaw));
            } else {
                first = devert(yaw);
                second = devert(-degrees + devert(yaw));
            }

        }
        Double firsta = convert(first - 5);//175
        Double firstb = convert(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convert(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convert(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convert(second - 5);//175
        Double secondb = convert(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convert(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convert(second));
                telemetry.update();
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        //</editor-fold>
        //
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    */

    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() - move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){}
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }
    //
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */

    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devert(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convert(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
  /*  public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    } */
    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }
    //
}