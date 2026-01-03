package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Internal;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;
import java.util.Vector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name = "RR_AutoTest", group = "Test")
@Config
public class autoTest extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public static double lowerFlywheelPower = 0.3;
    public static double upperFlywheelPower = -0.7;
    public static double flapNorm = -0.01;
    public static double flapUp = 0.35;
    public static int sequencerTicksPerBall = 165;
    public static double intakePower = -0.7;
    public static double sequencerPower = 0.2;
    public static double flywheelSeconds = 3;
    public static int sequencerStartTicks = -92;
    Pose2d newPose;

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

    private int getAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags", detections.size());
        if (detections.isEmpty()) {
            telemetry.addLine("No tags visible");
            return -1;
        }
        AprilTagDetection tag = detections.get(0);
        telemetry.addLine("---- AprilTag ----");
        telemetry.addData("ID", tag.id);
        return  tag.id;
    }



    String[] codes = {"GPP", "PGP", "PPG"};
    Double[][] powers = {{0.3, -0.7}, {0.3, -0.7}, {0.3, -0.7}}; //lower,  upper
    Integer[] ids = {21, 22, 23};
    Vector2d[] ballRowPositions = {new Vector2d(33, 30), new Vector2d(10, 30), new Vector2d(-15, 30)};

    public static Vector2d goalPosition = new Vector2d(-60, 60);

    public static Vector2d shootPosition = new Vector2d(12,12);

    public class flywheels{
        private DcMotor lowerFlywheel;
        private DcMotor upperFlywheel;

        public flywheels(HardwareMap hardwareMap) {
            lowerFlywheel = hardwareMap.get(DcMotor.class, "perp");
            upperFlywheel = hardwareMap.get(DcMotor.class, "par");
            lowerFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lowerFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            upperFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            upperFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class spinUpFlywheels implements Action {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    upperFlywheel.setPower(upperFlywheelPower);
                    lowerFlywheel.setPower(lowerFlywheelPower);
                    timer.reset();
                    started = true;
                }

                return timer.seconds() < flywheelSeconds;
            }
        }

        public class turnOffFlywheels implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                upperFlywheel.setPower(0);
                lowerFlywheel.setPower(0);

                return true;
            }

        }

        public Action spinUpFlywheels(){
            return new spinUpFlywheels();
        }

        public Action turnOffFlywheels(){
            return new turnOffFlywheels();
        }

    }

    public class flapServo{
        public Servo flapServo;
        public flapServo(HardwareMap hardwareMap){
            flapServo = hardwareMap.get(Servo.class, "flap");
        }

        public class fireBall implements Action {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    flapServo.setPosition(flapUp);
                    timer.reset();
                    started = true;
                }

                if (timer.seconds() > 0.3) {
                    flapServo.setPosition(flapNorm);
                    return false;
                }

                return true;
            }
        }

        public Action fireBall() {
            return new fireBall();
        }
    }

    public class motors{
        private DcMotor intake;
        private DcMotor sequencer;

        public motors(HardwareMap hardwareMap){
            intake = hardwareMap.get(DcMotor.class, "intake");
            sequencer = hardwareMap.get(DcMotor.class, "sequencer");

            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sequencer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sequencer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class intakeForTime implements Action{
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;
            private final double duration;
            private final double power;

            public intakeForTime(double seconds, double intakePower) {
                this.duration = seconds;
                this.power = intakePower;
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    intake.setPower(power);
                    timer.reset();
                    started = true;
                }

                if (timer.seconds() >= duration) {
                    intake.setPower(0);
                    return false;
                }

                return true;
            }
        }

        // Basic version with default power
        public Action intakeForTime(double seconds){
            return new intakeForTime(seconds, intakePower);
        }

        // Advanced version with custom power
        public Action intakeForTime(double seconds, double customPower){
            return new intakeForTime(seconds, customPower);
        }

        public class toggleIntake implements Action{
            private boolean on = false;


            public toggleIntake(boolean toggle){
                this.on = toggle;
            }

            @Override
            public boolean run(TelemetryPacket packet){
                if (on) {
                    intake.setPower(intakePower);
                } else{
                    intake.setPower(0);
                }
                return false;
            }
        }

        public Action toggleIntake(boolean toggle) {
            return  new toggleIntake(toggle);
        }

        public class advanceSequencer implements Action {
            private boolean started = false;
            private boolean finished = false;
            private final int ticks;
            private final double power;
            private int startPosition;
            private int targetPosition;

            public advanceSequencer(int relativeTicks, double motorPower) {
                this.ticks = relativeTicks;
                this.power = motorPower;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    // Capture current position for relative movement
                    startPosition = sequencer.getCurrentPosition();
                    targetPosition = startPosition - ticks;
                    sequencer.setTargetPosition(targetPosition);
                    sequencer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sequencer.setPower(Math.abs(power)); // Use absolute value for RUN_TO_POSITION
                    started = true;

                    // Debug output
                    packet.put("Sequencer Start", startPosition);
                    packet.put("Sequencer Target", targetPosition);
                    packet.put("Sequencer Ticks", ticks);
                }

                // Add current position to telemetry
                packet.put("Sequencer Current", sequencer.getCurrentPosition());
                packet.put("Sequencer Busy", sequencer.isBusy());

                if (!sequencer.isBusy() && !finished) {
                    sequencer.setPower(0);
                    // CRITICAL: Reset to RUN_USING_ENCODER for next action
                    sequencer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    finished = true;
                    return false; // Action complete
                }

                return true; // Continue running
            }
        }

        // Default version using config values
        public Action advanceSequencer() {
            return new advanceSequencer(sequencerTicksPerBall, sequencerPower);
        }

        // Custom version with specific ticks
        public Action advanceSequencer(int ticks) {
            return new advanceSequencer(ticks, sequencerPower);
        }

        // Fully custom version
        public Action advanceSequencer(int ticks, double power) {
            return new advanceSequencer(ticks, power);
        }

    }

    public class colorDetector{
        private ColorSensor colorDetector;

        public colorDetector(HardwareMap hardwareMap){
            colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");
        }
    }

    public class imu{
        private IMU imu;

        public imu(HardwareMap hardwareMap){
            imu = hardwareMap.get(IMU.class, "imu");
        }

        public class getPose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return true;
            }
        }
    }

    private double calculateHeadingBackwards(Pose2d currentPose) {
        double dx = goalPosition.x - currentPose.position.x;
        double dy = goalPosition.y - currentPose.position.y;
        double angleToGoal = Math.atan2(dy, dx);

        // Add 180° to face the opposite direction
        angleToGoal += Math.PI;

        // Wrap between -π and π
        if (angleToGoal > Math.PI) angleToGoal -= 2 * Math.PI;
        if (angleToGoal < -Math.PI) angleToGoal += 2 * Math.PI;

        return angleToGoal;
    }


    @Override
    public void runOpMode() {
        Vector2d ballRow1 = new Vector2d(34, 30);

        motors motors = new motors(hardwareMap);
        flywheels flywheels = new flywheels(hardwareMap);
        flapServo flap = new flapServo(hardwareMap);

        flap.flapServo.setPosition(flapNorm);

        motors.sequencer.setTargetPosition(sequencerStartTicks);
        motors.sequencer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.sequencer.setPower(Math.abs(sequencerPower));

        while (motors.sequencer.isBusy() && !isStopRequested()) {
            telemetry.addData("Sequencer Position", motors.sequencer.getCurrentPosition());
            telemetry.addData("Sequencer Target", sequencerStartTicks);
            telemetry.update();
        }

        motors.sequencer.setPower(0);
        motors.sequencer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Starting pose
        Pose2d startPose = new Pose2d(60, 12, Math.toRadians(180));

        // Create drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        drive.lazyImu.get().resetYaw();

        // Move forward more to get second ball
        Action driveToSecondBall = new Action() {
            private Action moveAction = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (moveAction == null) {
                    Pose2d currentPose = drive.localizer.getPose();
                    moveAction = drive.actionBuilder(currentPose)
                            .fresh()
                            .lineToY(45)  // Move forward to second ball position
                            .build();
                }
                return moveAction.run(packet);
            }
        };

        // Turn to shooting position
        Action moveToFire = new Action() {
            private Action turnAction = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (turnAction == null) {
                    Pose2d currentPose = drive.localizer.getPose();
                    double targetHeading = calculateHeadingBackwards(currentPose);

                    turnAction = drive.actionBuilder(currentPose)
                            .fresh()
                            .turnTo(targetHeading)
                            //  .splineTo(new Vector2d(12,12), calculateHeadingBackwards(new Pose2d(12, 12, Math.toRadians(90))))
                            .build();
                }
                return turnAction.run(packet);
            }
        };

        Action debugPose = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Pose2d p = drive.localizer.getPose();
                packet.put("X", p.position.x);
                packet.put("Y", p.position.y);
                packet.put("Heading (deg)", Math.toDegrees(p.heading.toDouble()));
                telemetry.addData("X", p.position.x);
                telemetry.addData("Y", p.position.y);
                telemetry.addData("Heading", Math.toDegrees(p.heading.toDouble()));
                telemetry.update();
                return false;
            }
        };

        telemetry.addData("heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry.addLine("Ready to run RR Auto Test");
        telemetry.update();

        final ElapsedTime apriltagTimer = new ElapsedTime();

        initAprilTag();

        waitForStart();
        if (isStopRequested()) return;

        int aprilTagCode = 21;
        int index = 0;

        while (apriltagTimer.seconds() <= 5){
            int got = getAprilTag();
            if (got != -1){
                aprilTagCode = got;

                for (int i = 0; i < 3; i++){
                    if (ids[i] == aprilTagCode) {
                        index = i;
                    }
                }

                break;
            }
        }


        // Initial approach to ball row - this sets up position and orientation
        Action driveToBalls = drive.actionBuilder(startPose)
                .splineTo(ballRowPositions[index], Math.toRadians(90))  // Approach facing 90° (north)
                .lineToY(38)  // Move forward slightly to get first ball
                .build();

        drive.lazyImu.get().resetYaw();

        // Run the action sequence
        Actions.runBlocking(
                new SequentialAction(
                        // Turn on intake
                        motors.toggleIntake(true),

                        // Drive to first ball position and collect it
                        driveToBalls,

                        //debugPose,

                        // Brief pause to ensure ball is collected
                        new SleepAction(0.3),

                        // Advance sequencer to index first ball
                        motors.advanceSequencer(sequencerTicksPerBall),

                        new SleepAction(0.3),

                        // Move forward to second ball (intake still running)
                        driveToSecondBall,

                        //debugPose,

                        // Brief pause to collect second ball
                        new SleepAction(0.3),

                        // Advance sequencer to index second ball
                        motors.advanceSequencer(sequencerTicksPerBall),

                        new SleepAction(0.3),

                        // Turn off intake - done collecting
                        motors.toggleIntake(false),

                        // move to firing pos
                        moveToFire,

                       // debugPose,

                        // Spin up flywheels
                        flywheels.spinUpFlywheels(),

                        // Small delay to ensure flywheels are up to speed
                        new SleepAction(0.5),

                        // Fire first ball
                        flap.fireBall(),

                        new SleepAction(0.5),

                        // Index to second ball
                        motors.advanceSequencer(sequencerTicksPerBall),

                        new SleepAction(0.3),

                        // Fire second ball
                        flap.fireBall(),

                        new SleepAction(1),

                        // Turn off flywheels
                        flywheels.turnOffFlywheels()
                )
        );
    }

}