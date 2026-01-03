package org.firstinspires.ftc.teamcode.teleOp;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "main code", group = "working")
@Config
public class mainCodeCurrent extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private IMU imu;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor intake;
    private DcMotor sequencer;
    private Servo flapServo;
    private ColorSensor colorDetector;

    private boolean intakeToggle = false;
    private boolean sequencerToggle = false;
    private boolean lastA = false;

    public static double flapNorm = -0.01;
    public static double flapUp = 0.35;

    private DcMotor lowerFlywheel;
    private DcMotor upperFlywheel;


    public static double lowerPower = 0.3;
    public static double upperPower = -0.7;


    public static double lowerAdjustRange = 0.15;
    public static double upperAdjustRange = 0.25;

    private boolean emoteActive = false;


    private boolean shooterPriming = false;
    private long shooterPrimeStartTime = 0;

    public static long shooterPrimeDurationMs = 1000;

    private void yatharthEmote(){
        if (emoteActive) {return;}


    }

    private void gideonEmote(){
        if (emoteActive) {return;}


    }


    private void hardwareMapping() {

        imu = hardwareMap.get(IMU.class, "imu");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");

        intake = hardwareMap.get(DcMotor.class, "intake");
        sequencer = hardwareMap.get(DcMotor.class, "sequencer");
        flapServo = hardwareMap.get(Servo.class, "flap");

        colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");

        lowerFlywheel = hardwareMap.get(DcMotor.class, "perp");
        upperFlywheel = hardwareMap.get(DcMotor.class, "par");
    }

    private void setupServos() {
        //set servos to init pos here
        flapServo.setPosition(flapNorm);
    }

    private void setupChassis() {
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        imu.resetYaw();

        // STANDARD MECANUM DIRECTIONS
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sequencer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sequencer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lowerFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        upperFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    private void initializeAndSetUp() {
        hardwareMapping();
        setupChassis();
        setupServos();
    }

    private void chassisMovement(double forward, double strafe, double turn) {
        // FTC stick convention: up on stick is negative, so invert forward
        forward = -forward;

        // Standard robot-centric mecanum drive calculations
        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        // Normalize wheel powers so no value exceeds 1.0
        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br))
        );

        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }


    private String colorDetection() {
        int red = colorDetector.red();
        int green = colorDetector.green();
        int blue = colorDetector.blue();

        // Background rejection
        if (red < 90 && green < 90 && blue < 90) {
            return "unknown";
        }

        int total = red + green + blue;
        double redRatio = (double) red / total;
        double greenRatio = (double) green / total;
        double blueRatio = (double) blue / total;

        // GREEN
        if (
                green > 120 &&
                        greenRatio > 0.45 &&
                        greenRatio > redRatio &&
                        greenRatio > blueRatio
        ) {
            return "green";
        }

        // PURPLE: red + blue
        if (
                red > 100 &&
                        blue > 100 &&
                        (redRatio + blueRatio) > 0.55
        ) {
            return "purple";
        }

        return "unknown";
    }


    private void printThings() {
        telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Sequencer: ", sequencer.getCurrentPosition());

        telemetry.addData("R", colorDetector.red());
        telemetry.addData("G", colorDetector.green());
        telemetry.addData("B", colorDetector.blue());

        telemetry.addData(
                "Shooter Mode",
                shooterPriming &&
                        (System.currentTimeMillis() - shooterPrimeStartTime < shooterPrimeDurationMs)
                        ? "PRIMING"
                        : "RUNNING"
        );


        //telemetry.addData("Shooter Mix", stickY);
        telemetry.addData("Lower Flywheel", lowerPower);
        telemetry.addData("Upper Flywheel", upperPower);

        telemetry.addData("Color: ", colorDetection());
        telemetry.addData("flap pos", flapServo.getPosition());

        telemetryAprilTag();
        telemetry.update();
    }


    private void controls(){
        //a = toggle intake (done)
        //right trigger = shoot all balls
        //dpad up = servo (done)
        //dpad right = move drum right (done)
        //dpad left = move drum left (done)


        if (gamepad2.a && !lastA) {
            intakeToggle = !intakeToggle;
        }
        lastA = gamepad2.a;

        if (gamepad2.dpad_up) {
            flapServo.setPosition(flapUp);
        } else {
            flapServo.setPosition(flapNorm);
        }

        if (gamepad2.dpad_right) {
            sequencer.setPower(0.2);
        } else if (gamepad2.dpad_left) {
            sequencer.setPower(-0.2);
        } else{
            sequencer.setPower(0);
        }

        if (gamepad2.right_trigger > 0.6){
            //drum macro
        }

        if (intakeToggle){
            intake.setPower(-1);
        } else{
            intake.setPower(0);
        }

        boolean shooterActive = gamepad2.left_trigger > 0.2;

        double stickY = -gamepad2.left_stick_y;

        double lowerAdjusted =
                lowerPower - stickY * lowerAdjustRange;

        double upperAdjusted =
                upperPower - stickY * upperAdjustRange;

        lowerAdjusted = Math.max(0.0, Math.min(1.0, lowerAdjusted));
        upperAdjusted = Math.max(-1.0, Math.min(0.0, upperAdjusted));
        /*
        if (shooterActive) {

            if (!shooterPriming) {
                shooterPriming = true;
                shooterPrimeStartTime = System.currentTimeMillis();
            }

            long elapsed = System.currentTimeMillis() - shooterPrimeStartTime;

            if (elapsed < shooterPrimeDurationMs) {
                // UNSTICK PHASE
                lowerFlywheel.setPower(0);
                upperFlywheel.setPower(1.0);
            } else {
                // NORMAL SHOOTING
                lowerFlywheel.setPower(lowerAdjusted);
                upperFlywheel.setPower(upperAdjusted);
            }

        } else {
            shooterPriming = false;
            lowerFlywheel.setPower(0);
            upperFlywheel.setPower(0);
        }

         */

        if (shooterActive){
            //lowerFlywheel.setPower(lowerAdjusted);
            upperFlywheel.setPower(upperAdjusted);
        } else{
            //lowerFlywheel.setPower(0);
            upperFlywheel.setPower(0);
        }



        if (gamepad1.right_trigger > 0.8){
            //jig

        }

        if (gamepad2.right_trigger > 0.8){

        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAndSetUp();
        initAprilTag();
        waitForStart();
        while (opModeIsActive()) {


            chassisMovement(
                    gamepad1.left_stick_y,   // forward / backward
                    gamepad1.left_stick_x,   // strafe
                    gamepad1.right_stick_x   // rotate
            );

            printThings();
            controls();

            // sleep(20);
        }
        visionPortal.close();
    }

    private void initAprilTag(){
        aprilTag = new AprilTagProcessor.Builder()
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setLensIntrinsics(117, 117, 320, 240)
            .build();

        aprilTag.setDecimation(2);
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags", detections.size());

        if (detections.isEmpty()) {
            telemetry.addLine("No tags visible");
            return;
        }

        AprilTagDetection tag = detections.get(0);

        telemetry.addLine("---- AprilTag ----");
        telemetry.addData("ID", tag.id);

        // ONLY print pose data if available
        if (tag.ftcPose != null) {

            telemetry.addData("X (right, in)", "%.1f", tag.ftcPose.x);
            telemetry.addData("Y (forward, in)", "%.1f", tag.ftcPose.y);
            telemetry.addData("Z (up, in)", "%.1f", tag.ftcPose.z);

            telemetry.addData("Range (in)", "%.1f", tag.ftcPose.range);
            telemetry.addData("Bearing (deg)", "%.1f", tag.ftcPose.bearing);
            telemetry.addData("Yaw (deg)", "%.1f", tag.ftcPose.yaw);

        } else {
            telemetry.addLine("Pose data NOT available");
            telemetry.addData("Center X (px)", "%.0f", tag.center.x);
            telemetry.addData("Center Y (px)", "%.0f", tag.center.y);
        }
    }



}
