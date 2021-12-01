package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="autotestcheck", group="")


public class autotestcheck extends LinearOpMode {


    //Color Sensor
    private ColorSensor sensorColor;
    private ColorSensor sensorColor2;
    private ColorSensor sensorColor3;
    private ColorSensor sensorColor4;
    //private DistanceSensor sensorDistance;
    public int distance;
    public org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit DistanceUnit;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private Blinker expansion_hub_2;
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private Servo tester = null;

    private double PowerFactor = 0.8;
    double positionTarget = 0.21;
    private View relativeLayout;
    private Servo grabby = null;
    public Servo turny = null;
    private DcMotor superShooter = null;
    private DcMotor rainbow = null;
    static final double EncoderTicks = 537.6;
    //BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double GEAR_RATIO = 2.0;  // MKING - added on 11/27 to account to wheel gear drive instead of direct.  Bevel gear set is 2:1
    // static final double COUNTS_PER_INCH = (EncoderTicks / 3.145 * WHEEL_DIAMETER_INCHES);  // MKING - this formula is wrong!  Need to move parenthesis!
    static final double COUNTS_PER_INCH = EncoderTicks / (3.1416 * WHEEL_DIAMETER_INCHES);    // MKING - corrected formula on 11/27/20
    static final double MAX_SPEED = 0.8;
    static final double MIN_SPEED = 0.3;
    static final int ACCEL = 100;  // Scaling factor used in accel / decel code
    static final double SCALE_ADJUST = 3.0;  // also use 4.0, 1.8?  Scaling factor used in encoderDiff calculation
    char square;
    int NmbrRings;
    int time = 5;
    float gain = 2;

    // hsvValues is an array that will hold the hue, saturation, and value information
    float hsvValues[] = {0F, 0F, 0F};
    float hsvValues2[] = {0F, 0F, 0F};
    // values is a reference to the hsvValues array
    final float values[] = hsvValues;

    // somtimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attenuate the measured values.
    final double SCALE_FACTOR = 255;  //  sometimes it helps to multiply the raw RGB values with a scale factor

    static final int RING_LIMIT = 75;  //  Hue threshold for the ring color.  WILL NEED TO ADJUST!!
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        */

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        // imu = hardwareMap.get(BNO055IMU.class, "imu");
        // imu.initialize(parameters);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("LB Current Position (before move): ", LB.getCurrentPosition());
        telemetry.addData("RB Current Position (before move): ", RB.getCurrentPosition());
        telemetry.update();  // MKING - look at Left and Right Back wheel encoder position before move
        // move forward to ring stack
        // Use to consistently move straight and accelerate/decelerate
        // Parameters:  totalDistIn, maxPower, minPower, accel

        sleep(7000);

        goStraight(20,MAX_SPEED,MIN_SPEED,ACCEL);

        strafeBuddy(2400,.55,-.55,-.55,.55);

        goStraight(50, MAX_SPEED, MIN_SPEED,ACCEL);

    }


    //  MKing - Method to move straight, from 1st year Nerdettes FTC team
    //  MKIng - Adapted to mecanum 4-wheel drive instead of regular 4-wheel drive
//  MKing - Includes accelerate/decelerate and error correction for encoder difference
    public void goStraight(double totalDistIn, double maxPower, double minPower, int accel) {
        int distance;
        int rampUpDist;
        int rampDownDist;
        double currentPower;
        int currentDistLB = 0;
        int currentDistRB = 0;
        int encoderDiff;  // difference in LB and RB wheel encoder count
        double powerL;  // modified Left-side motor power to equalize motors
        double powerR;  // modified Right-side motor power to equalize motors
        boolean forward = true;

        // Use this to determine to go backward or forward
        // + totalDistIn means go forward 'totalDistIn' inches
        // - totalDistIn means go backward 'totalDistIn' inches


        if (totalDistIn > 0) {
            forward = true;
        } else {
            forward = false;
        }


        // Convert inches to encoder ticks
        distance = (int) (Math.abs(totalDistIn) * COUNTS_PER_INCH);  // distance is encoder ticks, not inches
        rampUpDist = (int) ((maxPower - minPower) * 100 * accel);  // calculates number of encoder ticks (distance) to get to full speed
        rampDownDist = distance - rampUpDist;  // calculates when (in encoder ticks) to start slowing down

        // Need our ramp-up distance to be less than half or else would not have time to decelerate
        if (rampUpDist > distance / 2) {
            rampUpDist = distance / 2;
            rampDownDist = distance / 2;
        }

        // Prepare motor encoders, turns off since not running to set position
        // Calculating power instead
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Setting power to motors
        currentPower = minPower;
        if (forward) {
            LF.setPower(currentPower);
            RF.setPower(currentPower);
            LB.setPower(currentPower);
            RB.setPower(currentPower);
        } else {
            LF.setPower(-currentPower);
            RF.setPower(-currentPower);
            LB.setPower(-currentPower);
            RB.setPower(-currentPower);
        }
        // MKing - go forward or backward AND use encoder comparison code for error correction!
        while (currentDistLB < distance) {  // While distance not met
            if (currentDistLB < rampUpDist) {  // Accelerating
                currentPower = minPower + ((double) currentDistLB / (double) accel) / 100.0;
                maxPower = currentPower;
            } else if (currentDistRB >= rampDownDist) {  // Decelerating
                currentPower = maxPower - (((double) currentDistLB - (double) rampDownDist) / (double) accel) / 100.0;
            }

            currentDistLB = Math.abs(LB.getCurrentPosition());
            currentDistRB = Math.abs(RB.getCurrentPosition());

            // MKing - code for encoder comparison error correcting to run straight!
            if (currentDistLB < currentDistRB) {  // Left side is lagging right side
                encoderDiff = currentDistRB - currentDistLB;
                powerL = currentPower;
                powerR = currentPower * ((100.0 - (encoderDiff * SCALE_ADJUST)) / 100.0);
            } else {  // Right side is lagging left side
                encoderDiff = currentDistLB - currentDistRB;
                powerR = currentPower;
                powerL = currentPower * ((100.0 - (encoderDiff * SCALE_ADJUST)) / 100.0);
            }

            if (forward) {
                LF.setPower(powerL);
                RF.setPower(powerR);
                LB.setPower(powerL);
                RB.setPower(powerR);
            } else {
                LF.setPower(-powerL);
                RF.setPower(-powerR);
                LB.setPower(-powerL);
                RB.setPower(-powerR);
            }
        }
        encoderBrake(200);
    }

    public void encoderBrake(int pause) {  // Stop motors and sleep for 'pause' milliseconds
        // int newLeftTarget;  // MKing - not sure what this is used for so commented out
        // int newRightTarget; // MKing - not sure what this is used for so commented out

        if (opModeIsActive()) {
            LF.setPower(0);
            RF.setPower(0);
            LB.setPower(0);
            RB.setPower(0);

            sleep(pause);
        }
    }

    public void strafeBuddy(int distanceMove, double LFPOWER, double RFPOWER, double LBPOWER, double RBPOWER) {
        sleep(500);

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setPower(LFPOWER);
        RF.setPower(RFPOWER);
        RB.setPower(RBPOWER);
        LB.setPower(LBPOWER);

        while (LB.getCurrentPosition() < (distanceMove) && RB.getCurrentPosition() < (distanceMove) && opModeIsActive()) {
            idle();
        }

        LF.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

        telemetry.addData("LB Current Position (before move): ", LB.getCurrentPosition());
        telemetry.addData("RB Current Position (before move): ", RB.getCurrentPosition());
        telemetry.update();  // MKING - look at Left and Right Back wheel encoder position before move
        sleep(500); // MKING - sleep for 3 secs to allow time to see telemetry

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void turnLeft2(double distanceMove, double speed) {
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setPower(-speed);
        RF.setPower(speed);
        RB.setPower(speed);
        LB.setPower(-speed);

        while (LB.getCurrentPosition() < (distanceMove) && RB.getCurrentPosition() < (distanceMove) && opModeIsActive()) {
            idle();
        }

        LF.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

        telemetry.addData("LB Current Position (before move): ", LB.getCurrentPosition());
        telemetry.addData("RB Current Position (before move): ", RB.getCurrentPosition());
        telemetry.update();  // MKING - look at Left and Right Back wheel encoder position before move
        sleep(500); // MKING - sleep for 3 secs to allow time to see telemetry


        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sleep(500);

    }

    private void turnRight2(double distanceMove, double speed) {
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setPower(speed);
        RF.setPower(-speed);
        RB.setPower(-speed);
        LB.setPower(speed);

        while (LB.getCurrentPosition() < (distanceMove) && RB.getCurrentPosition() < (distanceMove) && opModeIsActive()) {
            idle();
        }

        LF.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

        telemetry.addData("LB Current Position (before move): ", LB.getCurrentPosition());
        telemetry.addData("RB Current Position (before move): ", RB.getCurrentPosition());
        telemetry.update();  // MKING - look at Left and Right Back wheel encoder position before move
        sleep(500); // MKING - sleep for 3 secs to allow time to see telemetry


        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sleep(500);

    }
}





