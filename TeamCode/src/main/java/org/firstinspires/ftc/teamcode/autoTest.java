package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "RR_AutoTest", group = "Test")
public class autoTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // 1️⃣ Starting pose (CHANGE THIS to match where your robot is placed)
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        // 2️⃣ Create drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        drive.lazyImu.get().resetYaw();

        // 3️⃣ Build a simple trajectory
        Action testTrajectory = drive.actionBuilder(startPose)
                .lineToX(2)
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .lineToY(2)
                .build();
        telemetry.addData("heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

        telemetry.addLine("Ready to run RR Auto Test");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        drive.lazyImu.get().resetYaw();
        // 4️⃣ Run the action (this is where motion happens)
        Actions.runBlocking(
                new SequentialAction(
                        testTrajectory
                )
        );
    }
}
