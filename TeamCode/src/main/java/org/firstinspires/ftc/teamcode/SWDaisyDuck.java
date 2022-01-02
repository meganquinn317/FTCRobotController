package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.*;

@Autonomous(name="Template", group="")

public class movementTemp extends LinearOpMode {
    // Declarations
    private float desiredHeading;

    // ---------------------
    // Turn variables - this took a lot of experimentation.  Don't recommend changing.
    private static final float TURN_SPEED_HIGH = 1f;
    private static final float TURN_SPEED_LOW = 0.15f;
    private static final float TURN_HIGH_ANGLE = 45.0f;
    private static final float TURN_LOW_ANGLE = 5.0f;
    // End turn variables
    // ---------------------

    // ---------------------
    // Variables for straight method
    static final double EncoderTicks = 537.6;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final float ENCODER_TICKS_MOD = 34F/25F;
    static final double COUNTS_PER_INCH = EncoderTicks / (3.1416 * WHEEL_DIAMETER_INCHES * ENCODER_TICKS_MOD);    // MKING - corrected formula on 11/27/20
    static final double MAX_SPEED = 0.8;
    static final double MIN_SPEED = 0.3;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
    static final double SCALE_ADJUST = 3.0;  // also use 4.0, 1.8?  Scaling factor used in encoderDiff calculation
    double headingStraight;
    // End straight variables
    // ---------------------

    // ---------------------
    // Imu variables
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    private float allowableHeadingDeviation = 3.0f;
    // End imu variables
    // ---------------------

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;

    private CRServo spinspinducky = null;
    private Servo dumper = null;
    private DcMotor armboom = null;

    // ---------------------
    // Bucket variables
    private static final float BUCKETCLEAR = .8f;
    private static final float BUCKETDUMP = 0f;
    private static final float BUCKETIN = 1f;
    // End bucket variables
    // ---------------------

    static final float STRAFE_MOD = 18f; // Changes desired distance to encoder ticks.

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializations

        // IMU initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Initialize motors
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        spinspinducky = hardwareMap.get(CRServo.class, "spinspinducky");
        dumper  = hardwareMap.get(Servo.class, "dumper");
        armboom = hardwareMap.get(DcMotor.class, "armboom");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        armboom.setDirection(DcMotorSimple.Direction.FORWARD);

        resetEncoders();

        waitForStart();

        // -------------------------
        // Path belongs here.
        // This should be the only part that is modified once it is correct.

        goStraight(-6,MAX_SPEED,MIN_SPEED,ACCEL);
        turnCW(45);
        goStraight(-10,MAX_SPEED,MIN_SPEED,ACCEL);
        //movethatarm(78 whatever numbers y'all need);
        turnACW(135);
        goStraight(64,MAX_SPEED,MIN_SPEED,ACCEL);
        turnCW(90);
        goStraight(10,MAX_SPEED,MIN_SPEED,ACCEL);
        //spinspinducky
        turnCW(90);
        strafeBuddy(4);
        goStraight(70,MAX_SPEED,MIN_SPEED,ACCEL);

        // End Modifications of path
        // -------------------------
    }


    private float getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                DEGREES);
        return angles.firstAngle;
    }

    public void resetEncoders() {
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

    private void turnCW(float turnDegrees) {
        desiredHeading -= turnDegrees;
        if (desiredHeading <= -180) {
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
        boolean isCW = deltaHeading() > 0;

        if (isCW) {
            telemetry.addData("Turning ", "CW");
            // 1st stage - high power rough heading.
            if (deltaHeading() > TURN_HIGH_ANGLE) {
                setAllMotorsPower(TURN_SPEED_HIGH);
                while (deltaHeading() > TURN_HIGH_ANGLE) {
                }
            }
            // 2nd stage - low power fine heading.
            if (deltaHeading() > TURN_LOW_ANGLE) {
                setAllMotorsPower(TURN_SPEED_LOW);
                while (deltaHeading() > TURN_LOW_ANGLE) {
                }
            }
        } else { // Going ACW
            telemetry.addData("Turning ", "ACW");
            // 1st stage - high power rough heading.
            if (deltaHeading() < -TURN_HIGH_ANGLE) {
                setAllMotorsPower(-TURN_SPEED_HIGH);
                while (deltaHeading() < -TURN_HIGH_ANGLE) {
                }
            }
            // 2nd stage - low power fine heading.
            if (deltaHeading() < -TURN_LOW_ANGLE) {
                setAllMotorsPower(-TURN_SPEED_LOW);
                while (deltaHeading() < -TURN_LOW_ANGLE) {
                }
            }
        }
        resetEncoders();
        telemetry.addData("Final Heading: ", getHeading());
        telemetry.addData("Position ", imu.getPosition());
        telemetry.update();
        sleep(3000);
    }

    private void setAllMotorsPower(float turnPower) {
        LF.setPower(turnPower);
        LB.setPower(turnPower);
        RF.setPower(-turnPower);
        RB.setPower(-turnPower);
    }

    private float deltaHeading() {
        float dH = getHeading() - desiredHeading;
        if (dH < -180) { dH += 360; }
        if (dH > 180) { dH -= 360; }
        return dH;
    }
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

    }

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
    private void movethatarm(int getthatdistance)
    {

        while (armboom.getCurrentPosition() <  getthatdistance)
        {
            armboom.setPower(1);
        }

    }


}
