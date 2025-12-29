package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "chassisTest")
public class chassisTest extends LinearOpMode {

    private IMU imu;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor intake;
    private DcMotor sequencer;
    private Servo flapServo;

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



    private void printThings() {
        telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Sequencer: ", sequencer.getCurrentPosition());
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
        waitForStart();
        while (opModeIsActive()) {

            //chassisMovement(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            chassisMovement(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            printThings();
            controls();
        }
    }

}
