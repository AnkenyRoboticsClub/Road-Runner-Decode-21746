package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOpV1", group = "TeleOp")
public class TeleOpV1 extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();
    public LazyImu lazyImu;
    boolean driverControlled = true;

    @Override
    public void runOpMode() throws InterruptedException {
        double xin = 0;
        double yin = 0;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        //Imu
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                MecanumDrive.PARAMS.logoFacingDirection,
                MecanumDrive.PARAMS.usbFacingDirection
        ));

        //Drive Motors
        DcMotorEx leftFront, leftBack, rightBack, rightFront;
        leftFront = drive.leftFront;
        leftBack = drive.leftBack;
        rightBack = drive.rightBack;
        rightFront = drive.rightFront;

        //Gamepads
        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad1);

        Mechanisms.Launcher launcher = new Mechanisms.Launcher(hardwareMap);
        Mechanisms.Gate gate = new Mechanisms.Gate(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.updatePoseEstimate();

            // Retrieve your pose
            Pose2d myPose = drive.localizer.getPose();

            telemetry.addData("Roadrunner X:", myPose.position.x);
            telemetry.addData("Roadrunner Y:", myPose.position.y);
            telemetry.addData("Roadrunner Heading:", myPose.heading);
            RobotLog.ii("DbgLog", "Roadrunner Pose" + "(x: " + (int) myPose.position.x + ", y: " + (int) myPose.position.y + ", heading: " + myPose.heading + ")");

            driver1.readButtons();
            driver2.readButtons();

            TelemetryPacket packet = new TelemetryPacket();

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            if (driverControlled) {
                double xMult = 0.5;
                double yMult = 0.5;
                double rMult = 0.5;
                if (/*driver1.getButton(GamepadKeys.Button.Y)*/false) {
                    xMult = 1;
                    yMult = 1;
                    rMult = 1;
                } else {
                    xMult += driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)/2;
                    yMult += driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)/2;
                    rMult += driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)/2;
                }

                double y = -gamepad1.left_stick_y * xMult; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * yMult;
                double rx = gamepad1.right_stick_x * rMult;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (driver1.getButton(GamepadKeys.Button.START)) {
                    lazyImu.get().resetYaw();
                }

                double botHeading = lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                leftFront.setPower(frontLeftPower);
                leftBack.setPower(backLeftPower);
                rightFront.setPower(frontRightPower);
                rightBack.setPower(backRightPower);
            }

            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                runningActions.add(new ParallelAction(
                        launcher.setLauncherPower(0.55)
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                runningActions.add(new ParallelAction(
                        launcher.setLauncherPower(0.0)
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
                runningActions.add(new ParallelAction(
                        launcher.setLauncherPower(0.625)
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.Y)) {
                runningActions.add(new ParallelAction(
                        launcher.setLauncherPower(0.5+(driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)/5))
                ));
            }

            if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                runningActions.add(new ParallelAction(
                        gate.setGatePosition(Mechanisms.Gate.openPosition)
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                runningActions.add(new ParallelAction(
                        gate.setGatePosition(Mechanisms.Gate.closePosition)
                ));
            }


            //More telemetry
            telemetry.addData("lazyImu: ", lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
