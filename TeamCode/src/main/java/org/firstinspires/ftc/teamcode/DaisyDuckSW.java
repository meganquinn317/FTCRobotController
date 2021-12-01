package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="DaisyDuckSW", group="")

public  class DaisyDuckSW extends LinearOpMode {
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    static final double TURN_SPEED = 0.25;  // need to check this in robotTurn() method!! 1 is max.
    Pipeline modifyPipeline = new Pipeline();

    // double headingStraight;  // Hailey added 8/29/21 for Heading error correction
    // double errorHeading;    // Hailey added 8/29/21 for Heading error correction
    double absoluteHeading = 0;  // MKing 9/11/21 - cumulative Heading that the robot has been commanded

    // ---------------------------------------------------------------------------------------------
    // Adding IMU setup stuff here (8/26/2021)
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    private static int BOTTOMLEFT = 1;
    private static int MIDDLE = 2;
    private static int RIGHTTOP = 3;

    private double PowerFactor = 0.8;
    static final double EncoderTicks = 537.6;
    //BNO055IMU imu;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double GEAR_RATIO = 2.0;  // MKING - added on 11/27 to account to wheel gear drive instead of direct.  Bevel gear set is 2:1
    // static final double COUNTS_PER_INCH = (EncoderTicks / 3.145 * WHEEL_DIAMETER_INCHES);  // MKING - this formula is wrong!  Need to move parenthesis!
    static final double COUNTS_PER_INCH = EncoderTicks / (3.1416 * WHEEL_DIAMETER_INCHES);    // MKING - corrected formula on 11/27/20
    static final double MAX_SPEED = 0.8;
    static final double MIN_SPEED = 0.3;
    static final int ACCEL = 100;  // Scaling factor used in accel / decel code
    static final double SCALE_ADJUST = 3.0;  // also use 4.0, 1.8?  Scaling factor used in encoderDiff calculation
    final double MM_PER_INCH = 25.40;   //  Metric conversion

    public static final String VUFORIA_KEY = "AVXWcGz/////AAABmZfYj2wlVElmo2nUkerrNGhEBBg+g8Gq1KY3/lN0SEBYx7HyMslyrHttOZoGtwRt7db9nfvCiG0TBEp7V/+hojHXCorf1CEvmJWWka9nFfAbOuyl1tU/IwdgHIvSuW6rbJY2UmMWXfjryO3t9nNtRqX004LcE8O2zkKdBTw0xdqq4dr9zeA9gX0uayps7t0TRmiToWRjGUs9tQB3BDmSinXxEnElq+z3SMJGcn5Aj44iEB7uy/wuB8cGCR6GfOpDRYqn/R8wwD757NucR5LXA48rulTdthGIuHoEjud1QzyQOv4BpaODj9Oi0TMuBmBzhFJMwWzyZ4lKVyOCbf3uCRia7Q+HO+LbFbghNIGIIzZC";

    VuforiaLocalizer vuforia = null;
    OpenGLMatrix targetPose = null;
    String targetName = "";
    private OpenCvWebcam webCam;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId2);
        webCam.setPipeline(modifyPipeline);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                webCam.startStreaming(320, 240);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", "Something went wrong :(");
                telemetry.update();
            }
        });
        while (opModeIsActive())
        {
            if (gamepad2.right_bumper)
            {
                modifyPipeline.lowX +=10;
            }
            if (gamepad2.left_bumper)
            {
                modifyPipeline.lowX-=10;
            }
            if (gamepad2.a)
            {
                modifyPipeline.highX+=10;
            }
            if (gamepad2.b)
            {
                modifyPipeline.highX-=10;
            }
            if (gamepad2.x)
            {
                modifyPipeline.lowZ+=10;
            }
            if (gamepad2.y)
            {
                modifyPipeline.lowZ-=10;
            }
            if (gamepad2.dpad_right)
            {
                modifyPipeline.highZ+=10;
            }
            if (gamepad2.dpad_left)
            {
                modifyPipeline.highZ-=10;
            }
            telemetry.addData("Low: ", modifyPipeline.lowX + ", " + modifyPipeline.lowY+", "+modifyPipeline.lowZ);
            telemetry.addData("High: ", modifyPipeline.highX + ", " + modifyPipeline.highY+", "+modifyPipeline.highZ);
            telemetry.update();
            sleep(500);
        }
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);


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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        //      parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
        //VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
        //VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromFile("C:\\Users\\Debbie\\Downloads\\FtcRobotController-7.0\\FtcRobotController-7.0\\TeamCode\\src\\main\\res\\assets");

        //targetsFreightFrenzy.get(0).setName("Duck");//renamed from Blue Storage -Megan 10/31/21
        //targetsFreightFrenzy.get(1).setName("Blue Alliance Wall");
        //targetsFreightFrenzy.get(2).setName("Red Storage");
        //targetsFreightFrenzy.get(3).setName("Red Alliance Wall");


        // Start tracking targets in the background
        //targetsFreightFrenzy.activate();
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized the duck model.");


        // wait for the start button to be pressed.
        waitForStart();
        //int towerlevel = scanForTarget(targetsFreightFrenzy);
        //telemetry.addData("The tower level is", towerlevel);
        //attempt to turn lol w/ robot turn method
     //   robotTurn(45, "CW");
        //goStraight(48, MAX_SPEED, MIN_SPEED, ACCEL);

        //to strafe right:
        // LF:BAck     RF: Forward   LB: Forward   RB: BAck

       // strafeBuddy(24,PowerFactor,-PowerFactor,-PowerFactor,PowerFactor);

        sleep(500);
        //call for scanForTarget();
        // double towerLevel = scanForTarget();


        //goStraight(movemove, MAX_SPEED,MIN_SPEED,ACCEL);
        //

    }

   /* public int scanForTarget(VuforiaTrackables target) {

        boolean targetFound = false;    // Set to true when a target is detected by Vuforia
        double targetRange = 0;        // Distance from camera to target in Inches
        double targetBearing = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
        // Start tracking targets in the background
        target.activate();
        while (opModeIsActive()) {
            // Look for first visible target, and save its pose.
            targetFound = false;
            for (VuforiaTrackable trackable : target) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    targetPose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getVuforiaCameraFromTarget();

                    // if we have a target, process the "pose" to determine the position of the target relative to the robot.
                    if (targetPose != null) {
                        targetFound = true;
                        targetName = trackable.getName();
                        VectorF trans = targetPose.getTranslation();

                        // Extract the X & Y components of the offset of the target relative to the robot
                        double targetX = trans.get(0) / MM_PER_INCH; // Image X axis
                        double targetY = trans.get(2) / MM_PER_INCH; // Image Z axis

                        // target range is based on distance from robot position to origin (right triangle).
                        targetRange = Math.hypot(targetX, targetY);

                        // target bearing is based on angle formed between the X axis and the target range line
                        targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));

                        break;  // jump out of target tracking loop if we find a target.
                    }
                }
            }
**/
  /*          // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Target", " %s", targetName);
                telemetry.addData("Range", "%5.1f inches", targetRange);
                telemetry.addData("Bearing", "%3.0f degrees", targetBearing);

                if (targetBearing > 8) {
                    //turn right to 0 degree
                    // turnToHeading(targetBearing, 0.5);
                    return RIGHTTOP;
                }
                if (targetBearing < -8) {
                    //turn left to 0 degree
                    //turnToHeading(targetBearing, 0.5);
                    return BOTTOMLEFT;
                }
                return MIDDLE;

            } else {
                telemetry.addData("No", "Target not found :((");
            }
        }
        return 4;

    }
**/

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
        //to strafe left:
        //LF: Forward   RF: Back   LB: BACK   RB : foRWARD

        //to strafe right:
        // LF:BAck     RF: Forward   LB: Forward   RB: BAck


        while (LB.getCurrentPosition() < (distanceMove) && RB.getCurrentPosition() < (distanceMove) && opModeIsActive())
        {

            LF.setDirection(DcMotor.Direction.REVERSE);
            RF.setDirection(DcMotor.Direction.FORWARD);
            LB.setDirection(DcMotor.Direction.REVERSE);
            RB.setDirection(DcMotor.Direction.FORWARD);

            LF.setPower(LFPOWER);
            RF.setPower(RFPOWER);
            RB.setPower(RBPOWER);
            LB.setPower(LBPOWER);
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

    public double getHeading(AngleUnit angleUnit)
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                angleUnit);
        return angles.firstAngle;
    }

    public void robotTurn(int turnAngle, String turnDirection)
    {
        turnAngle-=4;
        telemetry.addData("Absolute Heading", absoluteHeading);
        telemetry.addData("Imu heading", getHeading(AngleUnit.DEGREES));
        telemetry.update();
        sleep(2000);
        //for (int i=0; i<6000; i++)
        while (getHeading(AngleUnit.DEGREES)>(0-turnAngle))
        {
            LF.setDirection(DcMotor.Direction.REVERSE);
            RF.setDirection(DcMotor.Direction.REVERSE);
            LB.setDirection(DcMotor.Direction.REVERSE);
            RB.setDirection(DcMotor.Direction.REVERSE);

            LF.setPower(.5);
            RF.setPower(.5);
            RB.setPower(.5);
            LB.setPower(.5);
        }
        LF.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
        telemetry.addData("Absolute Heading", absoluteHeading);
        telemetry.addData("Imu heading", getHeading(AngleUnit.DEGREES));
        telemetry.update();
        sleep(20000);
    }
    public void turnToHeading(double tB, double speed) {
        if (tB > 0) { //right

            LF.setDirection(DcMotor.Direction.REVERSE);
            RF.setDirection(DcMotor.Direction.REVERSE);
            LB.setDirection(DcMotor.Direction.REVERSE);
            RB.setDirection(DcMotor.Direction.REVERSE);

            LF.setPower(speed);
            RF.setPower(speed);
            RB.setPower(speed);
            LB.setPower(speed);
        }
        if (tB < 0){ //left
            LF.setDirection(DcMotor.Direction.FORWARD);
            RF.setDirection(DcMotor.Direction.FORWARD);
            LB.setDirection(DcMotor.Direction.FORWARD);
            RB.setDirection(DcMotor.Direction.FORWARD);

            LF.setPower(speed);
            RF.setPower(speed);
            LB.setPower(speed);
            RB.setPower(speed);
        }
    }
}
