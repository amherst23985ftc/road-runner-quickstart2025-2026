package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;


@Autonomous(name = "RR Turn Test 180", group = "Test")
public class TurnTest180 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Starting pose: (0, 0, 0)
        Pose2d startPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("Ready - press START");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Build a simple 180 degree turn
        Action turnAction = drive.actionBuilder(startPose)
                .turn(Math.toRadians(180))
                .build();

        // Run the action
        Actions.runBlocking(turnAction);


        // Allow pose to settle
        sleep(500);

        Pose2d finalPose = drive.localizer.getPose();

        telemetry.addLine("Turn complete");
        telemetry.addData("Final heading (deg)",
                Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.addData("X", finalPose.position.x);
        telemetry.addData("Y", finalPose.position.y);
        telemetry.update();

        // Keep telemetry visible
        while (opModeIsActive()) {
            idle();
        }
    }
}
