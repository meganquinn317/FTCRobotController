package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name="CollinsPathing", group="")

public class CollinsPathing extends LinearOpMode {


    public int distance;
    public double desiredHeading;  // IMU measurement of current Heading

    public org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit DistanceUnit;
    private ElapsedTime runtime = new ElapsedTime();

    //Motors and servos declaration
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private CRServo spinspinducky = null;

    static final double EncoderTicks = 537.6;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final float ENCODER_TICKS_MOD = 34F/25F;
    static final double COUNTS_PER_INCH = EncoderTicks / (3.1416 * WHEEL_DIAMETER_INCHES * ENCODER_TICKS_MOD);    // MKING - corrected formula on 11/27/20
    static final double MAX_SPEED = 0.8;
    static final double MIN_SPEED = 0.3;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
    static final double SCALE_ADJUST = 3.0;  // also use 4.0, 1.8?  Scaling factor used in encoderDiff calculation

    static final float STRAFE_MOD = 18f; // Changes desired distance to encoder ticks.
    static final double TURN_SPEED = 0.4d;  // need to check this in robotTurn() method!! 1 is max.
    static final double TURN_SPEED_LOW = 0.2d;
    private Servo dumper = null;
    private DcMotor armboom = null;
    private static final float BUCKETCLEAR = .8f;
    private static final float BUCKETDUMP = 0f;
    private static final float BUCKETIN = 1f;

    double tsstart = 0d;
    double tsend = 0d;

    double headingStraight;  // Hailey added 8/29/21 for Heading error correction
    double errorHeading;    // Hailey added 8/29/21 for Heading error correction

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    private float allowableHeadingDeviation = 2.0f;

    //Code to run ONCE when the driver hits INIT

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize each of the motors and servos
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        spinspinducky = hardwareMap.get(CRServo.class, "spinspinducky");
        dumper  = hardwareMap.get(Servo.class, "dumper");
        armboom = hardwareMap.get(DcMotor.class, "armboom");

        // It drives me crazy that some of these are Reversed, then we apply negative power.
        // Should be simplified.

        armboom.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);  // motor direction set for mecanum wheels with mitre gears
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        // Should reset all encoders to zero
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armboom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn on the encoders that have been wired.
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armboom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armboom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();  // need to add this method at end of code

        dumper.setPosition(BUCKETIN);
        telemetry.addData("Status", "Initialized");

        // wait for the start button to be pressed.
        waitForStart();

        // -------------------------
        // Define the path here!!!!

        // go straight away from wall
        goStraight(24, MAX_SPEED, MIN_SPEED, ACCEL);
        turnCW(90);
        goStraight(24, MAX_SPEED, MIN_SPEED, ACCEL);
        turnCW(90);
        goStraight(24, MAX_SPEED, MIN_SPEED, ACCEL);
        turnCW(90);
        goStraight(24, MAX_SPEED, MIN_SPEED, ACCEL);
        turnCW(90);

        goStraight(24, MAX_SPEED, MIN_SPEED, ACCEL);
        turnACW(90);
        goStraight(24, MAX_SPEED, MIN_SPEED, ACCEL);
        turnACW(90);
        goStraight(24, MAX_SPEED, MIN_SPEED, ACCEL);
        turnACW(90);
        goStraight(24, MAX_SPEED, MIN_SPEED, ACCEL);
        turnACW(90);

        // robotTurn(45, "CW");

        //turn 45 degrees
        //robotTurn(75,"CW");
        //sleep(1000);

        //goStraight(17.8,MAX_SPEED,MIN_SPEED,ACCEL);
        //sleep(500);

        // strafeBuddy(20);

        //spinspinducky.setPower(-1);

        //sleep(10000);
        //spinspinducky.setPower(0);
        //goStraight(-6,MAX_SPEED,MIN_SPEED,ACCEL);
        //sleep(500);
        //robotTurn(15,"CW");
        //sleep(500);
        //goStraight(-110,1, .8, ACCEL);

        // End of the actual path
        // -------------------------




    }

    //TO strafe rightt make rf and lb power -
    public void strafeBuddy(float distanceMoveInches) {

        distanceMoveInches*=STRAFE_MOD;

        if (distanceMoveInches > 0) {
            while (LB.getCurrentPosition() < (distanceMoveInches) && RB.getCurrentPosition() < (distanceMoveInches) && opModeIsActive()) {
                LF.setPower(MAX_SPEED);
                RF.setPower(-MAX_SPEED);
                RB.setPower(MAX_SPEED);
                LB.setPower(-MAX_SPEED);
            }
        } else {
            distanceMoveInches = 0-distanceMoveInches;
            while (LB.getCurrentPosition() < (distanceMoveInches) && RB.getCurrentPosition() < (distanceMoveInches) && opModeIsActive()) {
                LF.setPower(-MAX_SPEED);
                RF.setPower(MAX_SPEED);
                RB.setPower(-MAX_SPEED);
                LB.setPower(MAX_SPEED);
            }
        }

        // Once the strafe is complete, reset the state of the motors.

        LF.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        headingStraight = getHeading(AngleUnit.DEGREES);
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

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration (used by IMU code)
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

    }

    //----------------------------------------------------------------------------------------------
    // Formatting (used by composeTelemetry() method)
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    //----------------------------------------------------------------------------------------------
    // Simplified Heading retrieval code from "Learn Java for FTC" book
    //----------------------------------------------------------------------------------------------
    public double getHeading(AngleUnit angleUnit) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                angleUnit);
        return angles.firstAngle;
    }

    public void robotTurn(double turnAngle, String turnDirection) {
        tsstart = getHeading(AngleUnit.DEGREES);

        if (turnAngle <= 180) {
            if (turnDirection.equals("CW")) {  // robot turn Clockwise (right)

//                errorHeading = turnAngle - headingStraight;

                //  ***  NEED SOME HEADING ERROR CORRECTION CODE HERE!!  ***

                double newHeading = getHeading(AngleUnit.DEGREES) - turnAngle;
                if (newHeading < -180)
                    newHeading = 360 + newHeading;  // use '+' since newHeading is a negative number

                //      while (getHeading(AngleUnit.DEGREES) >= newHeading) {
                while (Math.abs(getHeading(AngleUnit.DEGREES) - newHeading) >= 5d){
                    if (Math.abs(getHeading(AngleUnit.DEGREES) - newHeading) >= 15d) {
                        LF.setPower(TURN_SPEED);
                        LB.setPower(TURN_SPEED);
                        RF.setPower(-TURN_SPEED);
                        RB.setPower(-TURN_SPEED);
                    } else {
                        LF.setPower(TURN_SPEED_LOW);
                        LB.setPower(TURN_SPEED_LOW);
                        RF.setPower(-TURN_SPEED_LOW);
                        RB.setPower(-TURN_SPEED_LOW);
                    }
                }
                // Done with turn.  Set motors back to zero.
                LF.setPower(0);
                LB.setPower(0);
                RF.setPower(0);
                RB.setPower(0);

                tsend = getHeading(AngleUnit.DEGREES);


                // ADD IF STATEMENT FOR CCW (LEFT) TURN HERE!!

            }
        }
    }

    private void turnCW(float turnDegrees) {
        desiredHeading -= turnDegrees;
        if (desiredHeading < -180) {
            desiredHeading += 360;
        }
        turnToHeading();
    }

    private void turnACW(float turnDegrees) {
        desiredHeading += turnDegrees;
        if (desiredHeading > 180) {
            desiredHeading -= 360;
        }
        turnToHeading();
    }

    private void turnToHeading() {
        float actualHeading = (float) getHeading(AngleUnit.DEGREES);
        float turnSpeed = 0.2f;
        boolean isACW = false;

        // If we have called the function but are already on that heading
        // (within acceptableDeviation), just return.
        if (Math.abs(desiredHeading - actualHeading)%180 < allowableHeadingDeviation) {
            return;
        }

        // Inverting speed in the case of an AntiClockwise movement.
        if ((desiredHeading - actualHeading) > 0 || (desiredHeading - actualHeading) < -180) {
            turnSpeed *= -1f;
            isACW = true;
        }

        if (isACW) {
            while ((desiredHeading - actualHeading)%180 > allowableHeadingDeviation) {
                LF.setPower(turnSpeed);
                LB.setPower(turnSpeed);
                RF.setPower(-turnSpeed);
                RB.setPower(-turnSpeed);
                actualHeading = (float) getHeading(AngleUnit.DEGREES);
                if ((desiredHeading - actualHeading)%180 < -allowableHeadingDeviation) {
                    telemetry.addData("Turn: ", "We overshot.");
                }
            }
        } else {
            while ((desiredHeading - actualHeading)%180 < -allowableHeadingDeviation) {
                LF.setPower(turnSpeed);
                LB.setPower(turnSpeed);
                RF.setPower(-turnSpeed);
                RB.setPower(-turnSpeed);
                actualHeading = (float) getHeading(AngleUnit.DEGREES);
                if ((desiredHeading - actualHeading)%180 > allowableHeadingDeviation) {
                    telemetry.addData("Turn: ", "We overshot.");
                }
            }
        }

        telemetry.addData("DH: ", desiredHeading);
        telemetry.addData("AH: ", actualHeading);
        telemetry.update();

        resetEncoders();
    }

    private void resetEncoders() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Just a little time to make sure encoders have reset
        sleep(200);

        // Only using the LB Encoder
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Not technically encoder but...
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}


