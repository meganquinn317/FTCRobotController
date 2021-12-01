package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="Drive To Target", group = "Concept")

public class webcamtest1 extends LinearOpMode {
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)
    //  The GAIN constants set the relationship between the measured position error,
    //  and how much power is applied to the drive motors.  Drive = Error * Gain
    //  Make these values smaller for smoother control.
    final double SPEED_GAIN = 0.02;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MM_PER_INCH = 25.40;   //  Metric conversion



    private static final String VUFORIA_KEY = "AVXWcGz/////AAABmZfYj2wlVElmo2nUkerrNGhEBBg+g8Gq1KY3/lN0SEBYx7HyMslyrHttOZoGtwRt7db9nfvCiG0TBEp7V/+hojHXCorf1CEvmJWWka9nFfAbOuyl1tU/IwdgHIvSuW6rbJY2UmMWXfjryO3t9nNtRqX004LcE8O2zkKdBTw0xdqq4dr9zeA9gX0uayps7t0TRmiToWRjGUs9tQB3BDmSinXxEnElq+z3SMJGcn5Aj44iEB7uy/wuB8cGCR6GfOpDRYqn/R8wwD757NucR5LXA48rulTdthGIuHoEjud1QzyQOv4BpaODj9Oi0TMuBmBzhFJMwWzyZ4lKVyOCbf3uCRia7Q+HO+LbFbghNIGIIzZC";

    VuforiaLocalizer vuforia = null;
    OpenGLMatrix targetPose = null;
    String targetName = "";

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override
    public void runOpMode() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * To get an on-phone camera preview, use the code below.
         * If no camera preview is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
*/
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
      //  parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);


        // Load the trackable objects from the Assets file, and give them meaningful names
       // VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
        VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("duckduckno");
        //VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromFile("C:\\Users\\Debbie\\Downloads\\FtcRobotController-7.0\\FtcRobotController-7.0\\FtcRobotController\\src\\main\\assets\\RubberDuck.xml");

     /*   targetsFreightFrenzy.get(0).setName("Blue Storage");
        targetsFreightFrenzy.get(1).setName("Blue Alliance Wall");
        targetsFreightFrenzy.get(2).setName("Red Storage");
        targetsFreightFrenzy.get(3).setName("Red Alliance Wall");
     I*/targetsFreightFrenzy.get(0).setName("Rubber_Duck");

   //     VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromFile("RubberDuck.xml");
 //       targetsFreightFrenzy.get(0).setName("Rubber_Duck");


        // Start tracking targets in the background
        targetsFreightFrenzy.activate();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        boolean targetFound = false;    // Set to true when a target is detected by Vuforia
        double targetRange = 0;        // Distance from camera to target in Inches
        double targetBearing = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
        double drive = 0;        // Desired forward power (-1 to +1)
        double turn = 0;        // Desired turning power (-1 to +1)
        //telemetry.addData("Bearing", "%3.0f degrees", targetBearing);

        while (opModeIsActive()) {
            // Look for first visible target, and save its pose.
            targetFound = false;
            for (VuforiaTrackable trackable : targetsFreightFrenzy) {
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

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", " %s", targetName);
                telemetry.addData("Range", "%5.1f inches", targetRange);
                telemetry.addData("Bearing", "%3.0f degrees", targetBearing);

               if (targetBearing > 5) {
                    //turn right to 0 degree
                   // turnToHeading(targetBearing, 0.5);
                }
                if (targetBearing < -5) {
                    //turn left to 0 degree
                    //turnToHeading(targetBearing, 0.5);
                }

            } else {
                telemetry.addData(">", "Drive using joystick to find target\n");

            }

            // Drive to target Automatically if Left Bumper is being pressed, AND we have found a target.
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading and range error so we can use them to control the robot automatically.
                double rangeError = (targetRange - DESIRED_DISTANCE);
                double headingError = targetBearing;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = rangeError * SPEED_GAIN;
                turn = headingError * TURN_GAIN;

                telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
            } else {

                // drive using manual POV Joystick mode.
                drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                turn = gamepad1.right_stick_x / 4.0;  // Reduce turn rate to 25%.
                telemetry.addData("Manual", "Drive %5.2f, Turn %5.2f", drive, turn);
            }
            telemetry.update();

            // Calculate left and right wheel powers and send to them to the motors.
            double leftPower = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);
            LF.setPower(leftPower);
            LB.setPower(leftPower);
            RF.setPower(rightPower);
            RB.setPower(rightPower);


            sleep(10);
        }
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