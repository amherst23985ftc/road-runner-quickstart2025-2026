package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;

public class MeepMeepTesting {

    public static Vector2d goalPosition = new Vector2d(-60, 60);

    private static double calculateHeadingBackwards(Pose2d currentPose) {
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

    public static void main(String[] args) {
        Vector2d ballRow1 = new Vector2d(33, 30);
        Vector2d ballRow2 = new Vector2d(10, 30);
        Vector2d ballRow3 = new Vector2d(-15, 30);

        Vector2d decision;

        decision = ballRow3;

        Pose2d startPose = new Pose2d(60, 12, Math.toRadians(180));

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // First trajectory - drive to balls and collect first one
        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .splineTo(decision, Math.toRadians(90))  //approach ball row facing balls
                .lineToY(35)  // Move forward to first ball

                // pause for ball collection
                .waitSeconds(0.6)

                // Move forward to second ball
                .lineToY(40)

                // pause for second ball collection
                .waitSeconds(0.6)

                .lineToY(35)

                // Turn to face goal
                //.turnTo(calculateHeadingBackwards(new Pose2d(34, 45, Math.toRadians(90))))
                                .splineTo(new Vector2d(12,12), calculateHeadingBackwards(new Pose2d(12, 12, Math.toRadians(90))))

                // Simulate shooting sequence
                .waitSeconds(3.9)  // Flywheel spin up + first shot + index + second shot

                //move to third ball
                .splineTo( new Vector2d(decision.x, 50 ), Math.toRadians(90))

                //simulate pickup
                .waitSeconds(0.6)

                //turn to face goal

                //.turnTo(calculateHeadingBackwards(new Pose2d( decision.x, 50 , Math.toRadians(90))))
                .splineTo(new Vector2d(12,12), calculateHeadingBackwards(new Pose2d(12, 12, Math.toRadians(90))))

                // Simulate shooting sequence
                .waitSeconds(3.9)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}