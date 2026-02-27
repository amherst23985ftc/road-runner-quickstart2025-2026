package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.*;
import java.lang.Math;
import java.util.List;

@Autonomous(name = "autoRed", group = "Test")
@Config
public class autoRed extends LinearOpMode {

    /* ================= CONFIG ================= */

    public static double lowerFlywheelPower = 0.6;
    public static double upperFlywheelPower = 0.55;

    public static double flapUp = -0.3;
    public static double flapNorm = 0.35;

    public static int sequencerTicksPerBall = 184;
    public static double sequencerPower = 0.2;
    public static double flywheelSeconds = 3;

    public static Vector2d goalPosition = new Vector2d(-60, 60);

    /* ================= APRILTAG ================= */

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setLensIntrinsics(117,117,320,240)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    private int getAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return -1;
        return detections.get(0).id;
    }

    /* ================= UTIL ================= */

    private double calculateHeadingBackwards(Vector2d pos) {

        double dx = goalPosition.x - pos.x;
        double dy = goalPosition.y - pos.y;

        double angle = Math.atan2(dy, dx) + Math.PI;

        if (angle > Math.PI) angle -= 2*Math.PI;
        if (angle < -Math.PI) angle += 2*Math.PI;

        return angle;
    }

    /* ================= SUBSYSTEMS ================= */

    class Flywheels {

        DcMotor lower, upper;

        Flywheels(HardwareMap hw) {
            lower = hw.get(DcMotor.class,"lowerFlywheel");
            upper = hw.get(DcMotor.class,"perp");

            lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        Action spinUp() {
            return new Action() {
                ElapsedTime t = new ElapsedTime();
                boolean started=false;

                public boolean run(@NonNull TelemetryPacket p) {
                    if(!started){
                        lower.setPower(lowerFlywheelPower);
                        upper.setPower(upperFlywheelPower);
                        t.reset();
                        started=true;
                    }
                    return t.seconds()<flywheelSeconds;
                }
            };
        }

        Action stop() {
            return packet -> {
                lower.setPower(0);
                upper.setPower(0);
                return false; // FIXED
            };
        }
    }

    class Flap {

        Servo servo;

        Flap(HardwareMap hw){
            servo = hw.get(Servo.class,"flap");
        }

        Action fire(){
            return new Action() {

                ElapsedTime t=new ElapsedTime();
                boolean started=false;

                public boolean run(TelemetryPacket p){
                    if(!started){
                        servo.setPosition(flapUp);
                        t.reset();
                        started=true;
                    }

                    if(t.seconds()>0.3){
                        servo.setPosition(flapNorm);
                        return false;
                    }
                    return true;
                }
            };
        }
    }

    class Motors {

        DcMotor sequencer;

        Motors(HardwareMap hw){
            sequencer = hw.get(DcMotor.class,"sequencer");

            sequencer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sequencer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sequencer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        Action advance(int ticks){

            return new Action(){

                boolean started=false;
                int target;

                public boolean run(@NonNull TelemetryPacket p){

                    if(!started){
                        target = sequencer.getCurrentPosition()-ticks;
                        sequencer.setTargetPosition(target);
                        sequencer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sequencer.setPower(Math.abs(sequencerPower));
                        started=true;
                    }

                    if(!sequencer.isBusy()){
                        sequencer.setPower(0);
                        sequencer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        return false;
                    }

                    return true;
                }
            };
        }
    }

    /* ================= AUTO ================= */

    @Override
    public void runOpMode() {

        Flywheels flywheels = new Flywheels(hardwareMap);
        Flap flap = new Flap(hardwareMap);
        Motors motors = new Motors(hardwareMap);

        flap.servo.setPosition(flapNorm);

        Pose2d startPose = new Pose2d(60,12,Math.toRadians(180));
        Vector2d firePos = new Vector2d(-30,25);

        MecanumDrive drive = new MecanumDrive(hardwareMap,startPose);

        // SINGLE IMU RESET ONLY
        drive.lazyImu.get().resetYaw();

        initAprilTag();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if(isStopRequested()) return;

        /* ---------- AprilTag read ---------- */

        ElapsedTime tagTimer = new ElapsedTime();
        tagTimer.reset();

        int aprilTagCode = -1;

        while(opModeIsActive() && tagTimer.seconds()<3){
            int id=getAprilTag();
            if(id!=-1){
                aprilTagCode=id;
                break;
            }
        }

        /* ---------- Build shooting trajectory ---------- */

        Pose2d firePose = new Pose2d(
                firePos.x,
                firePos.y,
                calculateHeadingBackwards(firePos)
        );

        Action moveToFire =
                drive.actionBuilder(startPose)
                        .splineToSplineHeading(firePose, firePose.heading)
                        .build();

        /* ---------- RUN AUTO ---------- */

        Actions.runBlocking(
                new SequentialAction(

                        moveToFire,

                        flywheels.spinUp(),
                        new SleepAction(1.5),

                        flap.fire(),
                        new SleepAction(0.4),

                        motors.advance(sequencerTicksPerBall),
                        flap.fire(),
                        new SleepAction(0.4),

                        motors.advance(sequencerTicksPerBall),
                        flap.fire(),

                        flywheels.stop()
                )
        );
    }
}