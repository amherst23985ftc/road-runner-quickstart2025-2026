package org.firstinspires.ftc.teamcode;

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
public class intakeTestBecauseExpansionhubBroke extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private IMU imu;
    private DcMotor intake;
    private DcMotor sequencer;

    private boolean intakeToggle = false;
    private boolean sequencerToggle = false;
    private boolean lastA = false;

    private void hardwareMapping() {

        imu = hardwareMap.get(IMU.class, "imu");
        intake = hardwareMap.get(DcMotor.class, "intake");
        sequencer = hardwareMap.get(DcMotor.class, "sequencer");

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
        sequencer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sequencer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    private void initializeAndSetUp() {
        hardwareMapping();
        setupChassis();
    }



    private void printThings() {
        telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Sequencer: ", sequencer.getCurrentPosition());
        telemetry.addData("Intake: ", intakeToggle);
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
