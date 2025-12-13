package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Gate;
import org.firstinspires.ftc.teamcode.Mechanisms.Launcher;

@Autonomous(name = "AutoLaunchingTest", group = "Autonomous")
public class AutoLaunchingTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-48, -48, Math.toRadians(52));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Gate gate = new Gate(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Action lineUp = drive.actionBuilder(initialPose)
                .lineToX(-30)
                .turn(Math.toRadians(180))
                .build();

        Action exitLaunchZone = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(0, -20))
                .build();

        Action fullAuto = new SequentialAction(
                launcher.setLauncherVelocity(1.0),
                gate.setGatePosition(Gate.openPosition),
                new SleepAction(1.0),
                gate.setGatePosition(Gate.closePosition),
                new SleepAction(1.0)
        );

        Actions.runBlocking(fullAuto);

    }

}
