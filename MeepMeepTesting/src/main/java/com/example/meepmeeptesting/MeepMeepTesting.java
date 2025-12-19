package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        DriveShim drive = myBot.getDrive();

        Pose2d initialPose = new Pose2d(-50, -50, Math.toRadians(225));

        Action lineUp = drive.actionBuilder(initialPose)
                //.afterTime(0, launcher.setLauncherVelocity(1100))
                .strafeToLinearHeading(new Vector2d(-10, -10), Math.toRadians(225))
                .build();

        Action exitLaunchZone = drive.actionBuilder(new Pose2d(-10, -10, Math.toRadians(225)))
                //.afterTime(0,launcher.setLauncherVelocity(0))
                .strafeToLinearHeading(new Vector2d(20, -20), Math.toRadians(90+360))
                .build();


        Action fullAuto = new SequentialAction(
                //gate.setGatePosition(Gate.openPosition),
                lineUp,
                //gate.cycleGate(),
                //gate.cycleGate(),
                //gate.cycleGate(),
                /*gate.setGatePosition(Gate.openPosition),
                new SleepAction(2.0),
                gate.setGatePosition(Gate.closePosition),
                new SleepAction(2.0),
                gate.setGatePosition(Gate.openPosition),
                new SleepAction(2.0),*/
                exitLaunchZone
        );

        myBot.runAction(fullAuto);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}