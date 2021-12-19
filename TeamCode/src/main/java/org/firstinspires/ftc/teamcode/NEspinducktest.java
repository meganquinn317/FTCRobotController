package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
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

    @Autonomous(name="NEspinducktest", group="")

    public class NEspinducktest extends LinearOpMode {


        public int distance;
        public double currentHeading;  // IMU measurement of current Heading
        //public double turnAngle;  // used for the desired turn angle, in degrees
        //public String turnDirection;  // used for the desired direction to turn, CW or CCW

        public org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit DistanceUnit;
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        //private Blinker expansion_hub_2;
        private DcMotor LF = null;
        private DcMotor RF = null;
        private DcMotor LB = null;
        private DcMotor RB = null;

        private CRServo spinspinducky = null;
        private double PowerFactor = 0.8;
        static final double EncoderTicks = 537.6;
        static final double WHEEL_DIAMETER_INCHES = 4.0;
        static final double GEAR_RATIO = 1.0;  // MKING - updated 8/28/21 to account to wheel Bevel Gear set is 1:1
        // NOTE:  WHY IS GEAR_RATIO NOT USED IN COUNTS_PER_INCH CALCULATION?
        // static final double COUNTS_PER_INCH = (EncoderTicks / 3.145 * WHEEL_DIAMETER_INCHES);  // MKING - this formula is wrong!  Need to move parenthesis!
        static final double COUNTS_PER_INCH = EncoderTicks / (3.1416 * WHEEL_DIAMETER_INCHES * 34f / 25f);    // MKING - corrected formula on 11/27/20
        //the 25/34 is nonsense that i was told to put here
        static final double MAX_SPEED = 0.8;
        static final double MIN_SPEED = 0.3;
        static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
        static final double SCALE_ADJUST = 3.0;  // also use 4.0, 1.8?  Scaling factor used in encoderDiff calculation

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

        // ---------------------------------------------------------------------------------------------
        // Adding IMU setup stuff here (8/26/2021)
        // The IMU sensor object
        BNO055IMU imu;

        // State used for updating telemetry
        Orientation angles;
        Acceleration gravity;

        //--------------END OF IMU ADD------------------------------------------------------------------

        //Code to run ONCE when the driver hits INIT

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
            spinspinducky = hardwareMap.get(CRServo.class, "spinspinducky");
            dumper  = hardwareMap.get(Servo.class, "dumper");
            armboom = hardwareMap.get(DcMotor.class, "armboom");
            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery

            armboom.setDirection(DcMotorSimple.Direction.FORWARD);
            LF.setDirection(DcMotor.Direction.REVERSE);  // motor direction set for mecanum wheels with mitre gears
            RF.setDirection(DcMotor.Direction.FORWARD);
            LB.setDirection(DcMotor.Direction.REVERSE);
            RB.setDirection(DcMotor.Direction.FORWARD);

            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armboom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

            // -----------------------------------------------------------------------------------------
            // Adding IMU setup stuff here (8/26/2021)
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            // Set up our telemetry dashboard
            composeTelemetry();  // need to add this method at end of code


            //-----------END OF IMU ADD-----------------------------------------------------------------

            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
            //sense for our element here
            dumper.setPosition(BUCKETIN);
            // wait for the start button to be pressed.
            waitForStart();

            telemetry.addData("LB Current Position (before move): ", LB.getCurrentPosition());
            telemetry.addData("RB Current Position (before move): ", RB.getCurrentPosition());
            telemetry.update();  // MKING - look at Left and Right Back wheel encoder position before move
            // move forward to ring stack
            // Use to consistently move straight and accelerate/decelerate
            // Parameters:  totalDistIn, maxPower, minPower, accel


            //go straight away from wall
            goStraight(-6, MAX_SPEED, MIN_SPEED, ACCEL);

            sleep(500);

            // robotTurn(45, "CW");

            //turn 45 degrees
            robotTurn(-75,"CW");
            sleep(1000);

            goStraight(17.8,MAX_SPEED,MIN_SPEED,ACCEL);
            sleep(500);

            // strafeBuddy(20);

            spinspinducky.setPower(-1);

            sleep(10000);
            spinspinducky.setPower(0);
            goStraight(-6,MAX_SPEED,MIN_SPEED,ACCEL);
            sleep(500);
            robotTurn(15,"CW");
            sleep(500);
            goStraight(-110,1, .8, ACCEL);




       /* telemetry.addData("Heading: ",getHeading(AngleUnit.DEGREES));
        telemetry.addData("Started: ",tsstart);
        telemetry.addData("Ended: ",tsend);
        telemetry.addData("Delta ", tsend-tsstart );

        telemetry.update();


        sleep(5000);

       // robotTurn(45, "CW");
        //turnTo(-180);
        telemetry.addData("Heading: ",getHeading(AngleUnit.DEGREES));
        telemetry.addData("Started: ",tsstart);
        telemetry.addData("Ended: ",tsend);
        telemetry.addData("Delta ", tsend-tsstart );

        telemetry.update();


        //sleep(5000);

        //robotTurn(45, "CW");
        telemetry.addData("Heading: ",getHeading(AngleUnit.DEGREES));
        telemetry.addData("Started: ",tsstart);
        telemetry.addData("Ended: ",tsend);
        telemetry.addData("Delta ", tsend-tsstart );

        telemetry.update();


        //sleep(5000);

        //robotTurn(45, "CW");
        telemetry.addData("Heading: ",getHeading(AngleUnit.DEGREES));
        telemetry.addData("Started: ",tsstart);
        telemetry.addData("Ended: ",tsend);
        telemetry.addData("Delta ", tsend-tsstart );

        telemetry.update();


        sleep(5000);
*/
        /*
        //go straight
        goStraight(24, MAX_SPEED, MIN_SPEED, ACCEL);

        sleep(1000);

        //turn back to 0
        robotTurn(-45, "CW");

        sleep(1000);

        //go backwards to hit wall
        goStraight(-20, MAX_SPEED, MIN_SPEED, ACCEL);

        sleep(1000);

        //strafe to duck turn
        strafeBuddy(100);

        sleep(1000);

        //spin so wheel faces duck spinner
        robotTurn(135, "CW");

        sleep(1000);

        //spin duck code add here
        spinspinducky.setPower(1);

        sleep(1000);

        spinspinducky.setPower(0);

        //go straight
        goStraight(-6,MAX_SPEED,MIN_SPEED,ACCEL);

        sleep(1000);
*/
            //turn 90 degrees so facing front towards bars
            // robotTurn(90, "CW");

            //go straight into parking space
            //goStraight(90, MAX_SPEED,MIN_SPEED, ACCEL);

        }

        private void movethatarm(int getthatdistance) {

            while (armboom.getCurrentPosition() <  getthatdistance) {
                armboom.setPower(1);
            }

        }

        //TO strafe rightt make rf and lb power -
        public void strafeBuddy(float distanceMoveInches) {

            distanceMoveInches*=18;
            float desiredposL = LB.getCurrentPosition()-distanceMoveInches;

            float desiredposR = RB.getCurrentPosition()-distanceMoveInches;

            telemetry.addData("Lb pos", LB.getCurrentPosition());
            telemetry.update();
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
        public void strafeTesty(float distanceMoveInches) {

            distanceMoveInches*=18;
            float desiredposL = LB.getCurrentPosition()-distanceMoveInches;

            float desiredposR = RB.getCurrentPosition()-distanceMoveInches;

            telemetry.addData("Lb pos", LB.getCurrentPosition());
            telemetry.update();
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
/*  COMMENT OUT "GRVTY" AND "MAG" IN TELEMETRY STREAM TO DS
       telemetry.addLine()
           .addData("grvty", new Func<String>() {
               @Override public String value() {
                   return gravity.toString();
   }
               })
           .addData("mag", new Func<String>() {
               @Override public String value() {
                   return String.format(Locale.getDefault(), "%.3f",
                           Math.sqrt(gravity.xAccel*gravity.xAccel
                                   + gravity.yAccel*gravity.yAccel
                                   + gravity.zAccel*gravity.zAccel));
                   }
               });
 END OF COMMENT OUT "GRVTY" AND "MAG" IN TELEMETRY STREAM TO DS
*/
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
                if (turnDirection == "CW") {  // robot turn Clockwise (right)

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

        private void turnTo(float desiredAngle) {
            float allowedError = 5.0f;
            while (getHeading(AngleUnit.DEGREES) - desiredAngle > allowedError) {
                LF.setPower(TURN_SPEED_LOW);
                LB.setPower(TURN_SPEED_LOW);
                RF.setPower(-TURN_SPEED_LOW);
                RB.setPower(-TURN_SPEED_LOW);
            }
        /*while (getHeading(AngleUnit.DEGREES) - desiredAngle < allowedError) {
            LF.setPower(-TURN_SPEED);
            LB.setPower(-TURN_SPEED);
            RF.setPower(TURN_SPEED);
            RB.setPower(TURN_SPEED);
        }*/
        }
    }


