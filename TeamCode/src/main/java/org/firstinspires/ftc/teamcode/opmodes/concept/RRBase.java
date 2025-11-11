package org.firstinspires.ftc.teamcode.opmodes.concept;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "RR_BASE", group = "Autonomous")
public class RRBase extends LinearOpMode {

    DcMotor lift = hardwareMap.get(DcMotor.class, "liftMotor");

    Servo claw = hardwareMap.get(Servo.class, "clawServo");

    // instant function example
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

    // custom action example
    public class LiftTo implements Action {
        private double targetPos;
        private double direction;
        private double speed = 0.5;
        private double tolerance = 10;
        private boolean initialized = false;
        public LiftTo(double position) {
            this.targetPos = position;

            if (lift.getCurrentPosition() < this.targetPos) {
                this.direction = 1;
            } else {
                this.direction = -1;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // on first time through loop set the power to the lift
            if (!this.initialized) {
                lift.setPower(this.speed * this.direction);
                this.initialized = true;
            }

            // Continue running if outside tolerance range
            if (Math.abs(lift.getCurrentPosition() - targetPos) > tolerance) {
                return true;
            }

            // once lift reaches the target position stop power and exit loop
            lift.setPower(0);
            return false;
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
                // instant functions are similar to custom actions except for they can't
                // run in parallel with another action
                .stopAndAdd(new SetClawPos(0.5))
                .build();

        // sequential action runs your path one at a time
        // use parallel action to do multiple paths at once
        Actions.runBlocking(new SequentialAction(path));

        // parallel actions run multiple actions at once
        // will set the lift position to 100 while running the path
        Actions.runBlocking(new ParallelAction(
                path,
                new LiftTo(100)
        ));
    }

}
