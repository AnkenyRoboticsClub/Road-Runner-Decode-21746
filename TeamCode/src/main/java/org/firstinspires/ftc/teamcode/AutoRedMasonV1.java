package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "AutoRedMasonV1", group = "Autonomous")
public class AutoRedMasonV1 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-48, 48, Math.toRadians(308));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        if (isStopRequested()) return;

        Action path = drive.actionBuilder(initialPose)
                .lineToX(-30)
                .turn(Math.toRadians(180))
                // launch
                .strafeTo(new Vector2d(0, 30))
                .build();

        Actions.runBlocking(new SequentialAction(path));

    }

}
