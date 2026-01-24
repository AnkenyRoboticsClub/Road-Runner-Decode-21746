package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Gate;
import org.firstinspires.ftc.teamcode.Mechanisms.Launcher;
@Disabled
@Autonomous(name = "AutoLaunchingTest", group = "Autonomous")
public class AutoLaunchingTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-50, -50, Math.toRadians(235));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Gate gate = new Gate(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Action lineUp = drive.actionBuilder(initialPose)
                //.strafeTo(new Vector2d(-15, -15))
                .afterTime(0, launcher.setLauncherVelocity(1100))
                .strafeToLinearHeading(new Vector2d(-10, -10), Math.toRadians(235))
                .build();

        Action exitLaunchZone = drive.actionBuilder(new Pose2d(-10, -10, 235))
                //.strafeTo(new Vector2d(0, -20))
                .strafeToLinearHeading(new Vector2d(20, -20), Math.toRadians(90+360))
                .build();

        Action fullAuto = new SequentialAction(
                lineUp,
                //new SleepAction(3.0),
                //launcher.setLauncherVelocity(1000),
                gate.setGatePosition(Gate.openPosition),
                new SleepAction(2.0),
                gate.setGatePosition(Gate.closePosition),
                new SleepAction(2.0),
                gate.setGatePosition(Gate.openPosition),
                new SleepAction(2.0),
                exitLaunchZone
        );

        Actions.runBlocking(fullAuto);

    }

}
