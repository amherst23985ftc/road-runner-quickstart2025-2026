package org.firstinspires.ftc.teamcode.teleOp;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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


@TeleOp(name = "main code")
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

    private double flapNorm = 0;
    private double flapUp = 0.5;

    private void hardwareMapping() {

        imu = hardwareMap.get(IMU.class, "imu");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        intake = hardwareMap.get(DcMotor.class, "intake");
        sequencer = hardwareMap.get(DcMotor.class, "sequencer");
        flapServo = hardwareMap.get(Servo.class, "flap");

        colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");

    }

    private void setupServos() {
        //set servos to init pos here
        flapServo.setPosition(flapNorm);
    }

    private void setupChassis() {
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        sequencer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sequencer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeAndSetUp() {
        hardwareMapping();
        setupChassis();
        setupServos();
    }

    private void chassisMovement(double y, double x, double rx) {
        y = -y; // FTC forward stick convention

        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rx), 1
        );

        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
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

        telemetry.addData("Color: ", colorDetection());

        telemetryAprilTag();
        telemetry.update();
    }


    private void controls(){
        //a = toggle intake (done)
        //right trigger = shoot all balls
        //dpad up = servo (done)
        //dpad right = move drum right (done)
        //dpad left = move drum left (done)


        if (gamepad1.a){
            intakeToggle = !intakeToggle;
        }

        if (gamepad1.dpad_up) {
            flapServo.setPosition(flapUp);
        } else {
            flapServo.setPosition(flapNorm);
        }

        if (gamepad1.dpad_right) {
            sequencer.setPower(0.5);
        } else if (gamepad1.dpad_left) {
            sequencer.setPower(-0.5);
        } else{
            sequencer.setPower(0);
        }

        if (gamepad1.right_trigger > 1){
            //drum macro
        }

        if (intakeToggle){
            intake.setPower(1);
        } else{
            intake.setPower(0);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAndSetUp();
        initAprilTag();
        waitForStart();
        while (opModeIsActive()) {


            chassisMovement(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            printThings();
            controls();

            sleep(20);
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
