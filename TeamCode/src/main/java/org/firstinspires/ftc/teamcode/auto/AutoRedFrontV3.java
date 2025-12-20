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
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "AutoRedFrontV3", group = "Autonomous")
public class AutoRedFrontV3 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-50, 50, Math.toRadians(135));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Gate gate = new Gate(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Action lineUp = drive.actionBuilder(initialPose)
                .afterTime(0, launcher.setLauncherVelocity(1050))
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))
                .build();

        // would drive.localizer.getPose() work here instead of making our own pose?
        Action exitLaunchZone = drive.actionBuilder(new Pose2d(-20, 20, Math.toRadians(135)))
                .afterTime(0,launcher.setLauncherVelocity(0))
                .strafeToLinearHeading(new Vector2d(20, 40), Math.toRadians(-90-360))
                .build();


        Action fullAuto = new SequentialAction(
                gate.setGatePosition(Gate.openPosition),
                lineUp,
                gate.cycleGate(),
                gate.cycleGate(),
                gate.cycleGate(),
                exitLaunchZone,
                new SleepAction(2.0)
        );

        Actions.runBlocking(fullAuto);

    }

}
