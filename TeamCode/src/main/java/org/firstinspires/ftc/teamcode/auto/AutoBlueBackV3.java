package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Gate;
import org.firstinspires.ftc.teamcode.Mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "AutoBlueBackV3", group = "Autonomous")
public class AutoBlueBackV3 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(60, -20, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Gate gate = new Gate(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Action lineUp = drive.actionBuilder(initialPose)
                .afterTime(0, launcher.setLauncherVelocity(1315))
                .strafeToLinearHeading(new Vector2d(55, -20), Math.toRadians(210))
                .build();

        // would drive.localizer.getPose() work here instead of making our own pose?
        Action exitLaunchZone = drive.actionBuilder(new Pose2d(55, -20, Math.toRadians(201)))
                .afterTime(0,launcher.setLauncherVelocity(0))
                .strafeToLinearHeading(new Vector2d(30, -30), Math.toRadians(90+360))
                .build();


        Action fullAuto = new SequentialAction(
                gate.setGatePosition(Gate.openPosition),
                lineUp,
                gate.threeGate(),
                new SleepAction(2.0),
                exitLaunchZone,
                new SleepAction(2.0)
        );

        Actions.runBlocking(fullAuto);
        PoseStorage.currentPose = drive.localizer.getPose();
    }

}
