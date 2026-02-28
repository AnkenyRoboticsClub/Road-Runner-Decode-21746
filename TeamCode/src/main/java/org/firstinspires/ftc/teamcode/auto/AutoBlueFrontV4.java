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

@Autonomous(name = "AutoBlueFrontV4", group = "Autonomous")
public class AutoBlueFrontV4 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-50, -50, Math.toRadians(225));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Gate gate = new Gate(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Action lineUp = drive.actionBuilder(initialPose)
                .afterTime(0, launcher.setLauncherVelocity(1040))
                .strafeToLinearHeading(new Vector2d(-20, -20), Math.toRadians(225))
                .build();

        // would drive.localizer.getPose() work here instead of making our own pose?
        Action exitLaunchZone = drive.actionBuilder(new Pose2d(-20, -20, Math.toRadians(225)))
                .afterTime(0,launcher.setLauncherVelocity(0))
                .strafeToLinearHeading(new Vector2d(20, -40), Math.toRadians(90+360))
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
