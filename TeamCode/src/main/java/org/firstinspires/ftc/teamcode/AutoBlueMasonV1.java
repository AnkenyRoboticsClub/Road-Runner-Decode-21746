package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "AutoBlueMasonV1", group = "Autonomous")
public class AutoBlueMasonV1 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-48, -48, Math.toRadians(52));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Mechanisms.Launcher launcher = new Mechanisms.Launcher(hardwareMap);
        Mechanisms.Gate gate = new Mechanisms.Gate(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        //initialFieldPositioning(drive, initialPose);
        launchArtifact(launcher, gate);
        //finalFieldPosition();

    }

    private void launchArtifact(Mechanisms.Launcher launcher,Mechanisms.Gate gate)
    {
        Actions.runBlocking(launcher.setLauncherPower(0.55));
        //Wait for launcher to speed up to the desired speed
        sleep(1000);
        //Release ball - turn on the motor that opens the gate
        Actions.runBlocking(gate.openGate());
        sleep(2000);
        //Stop launcher
        Actions.runBlocking(launcher.stopLauncher());
        //Close - turn on the motor that opens the gate opposite speed
        Actions.runBlocking(gate.closeGate());

    }

    private void finalFieldPosition()
    {}


    private void initialFieldPositioning(MecanumDrive drive, Pose2d initialPose)
    {
        Action path = drive.actionBuilder(initialPose)
                .lineToX(-30)
                .turn(Math.toRadians(180))
                // launch
                //.strafeTo(new Vector2d(0, -20))
                .build();


        Actions.runBlocking(path);
    }


}
