package org.firstinspires.ftc.teamcode.opmodes.concept;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "RR_BASE", group = "Autonomous")
public class RRBase extends LinearOpMode {

    // instant function example
    Servo claw;
    public class SetClawPos implements InstantFunction {

        double targetPosition;

        // this is called when making a new instance of the class
        public SetClawPos(double position) {
            this.targetPosition = position;
        }

        // this function is called when the path is ran
        @Override
        public void run() {
                claw.setPosition(this.targetPosition);
        }
    }

    @Override
    public void runOpMode() {
        // claw variable used for the instant function example
        claw = hardwareMap.get(Servo.class, "claw");

        // set starting point
        Pose2d initialPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // put on initialize code here

        waitForStart();

        if (isStopRequested()) return;

        // creating the auto path
        Action path = drive.actionBuilder(initialPose)
                // put your trajectory builder functions here
                .strafeTo(new Vector2d(30, 30))

                // you can also add your own custom functions using an instant function
                .stopAndAdd(new SetClawPos(0.5))
                .build();

        // sequential action runs your path one at a time
        // use parallel action to do multiple paths at once
        Actions.runBlocking(new SequentialAction(path));
    }

}
