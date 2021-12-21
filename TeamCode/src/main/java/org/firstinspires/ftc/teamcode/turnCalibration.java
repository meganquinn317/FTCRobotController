package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Turn Calibration", group="")

public class turnCalibration extends LinearOpMode {
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

    BNO055IMU imu;
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;

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

        desiredHeading = getHeading();

        // Initialize motors
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        waitForStart();

        // -------------------------
        // Path belongs here.
        // This should be the only part that is modified once it is correct.

        turnCW(90);
        turnCW(89);
        turnCW(91);
        turnCW(90);
        turnACW(90);
        turnACW(89);
        turnACW(91);
        turnACW(90);

        // End Modifications of path
        // -------------------------
    }

    private float getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES);
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
}
