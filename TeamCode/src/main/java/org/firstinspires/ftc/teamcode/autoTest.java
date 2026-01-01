package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "RR_AutoTest", group = "Test")
@Config
public class autoTest extends LinearOpMode {

    public static double lowerFlywheelPower = 0.3;
    public static double upperFlywheelPower = -0.7;
    public static double flapNorm = -0.01;
    public static double flapUp = 0.35;
    public static int sequencerTicksPerBall = 300;
    public static double intakePower = -0.7;
    public static double sequencerPower = 0.2;
    public static double flywheelSeconds = 3;


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

        public Action spinUpFlywheels(){
            return new spinUpFlywheels();
        }

    }

    public class flapServo{
        public Servo flapServo;
        public flapServo(HardwareMap hardwareMap){
            flapServo = hardwareMap.get(Servo.class, "flapServo");
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

            public intakeForTime(double seconds) {
                this.duration = seconds;
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    intake.setPower(intakePower);
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

        public Action intakeForTime(double seconds){
            return new intakeForTime(seconds);
        }

        public class advanceSequencer implements Action{
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    sequencer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sequencer.setTargetPosition(sequencerTicksPerBall);
                    sequencer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sequencer.setPower(sequencerPower);
                    started = true;
                }

                if (!sequencer.isBusy()) {
                    sequencer.setPower(0);
                    return false;
                }

                return true;
            }

        }

        public Action advanceSequencer() {
            return new advanceSequencer();
        }

    }

    public class colorDetector{
        private ColorSensor colorDetector;

        public colorDetector(HardwareMap hardwareMap){
            colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");
        }
    }

    @Override
    public void runOpMode() {

        motors motors = new motors(hardwareMap);
        flywheels flywheels = new flywheels(hardwareMap);
        flapServo flap = new flapServo(hardwareMap);

        flap.flapServo.setPosition(flapNorm);

        // Starting pose
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        // Create drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        drive.lazyImu.get().resetYaw();

        Action driveForward = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(24, 0), 0)
                .build();

        Action shootOneBall = new SequentialAction(
                motors.intakeForTime(5.0),
                motors.advanceSequencer(),
                flywheels.spinUpFlywheels(),
                flap.fireBall()
        );

        telemetry.addData("heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry.addLine("Ready to run RR Auto Test");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        drive.lazyImu.get().resetYaw();

        // Run the action sequence
        Actions.runBlocking(
                new SequentialAction(
                        driveForward,
                        shootOneBall
                )
        );

    }
}