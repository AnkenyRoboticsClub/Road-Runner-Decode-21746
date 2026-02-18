package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOpV7", group = "TeleOp")
public class TeleOpV7 extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();
    public Limelight3A limelight;
    public IMU imu;
    public MecanumDrive drive;
    boolean driverControlled = true;
    boolean targetLock = false;
    boolean team = false; //false = red true = blue
    boolean launcherAutoVelocity = false;

    double[][] launchSpeedLookup2DArray = {
            //distance, velocity (tps)
            {0, 1000},
            {15, 1000},
            {67, 1060},
            {83, 1190},//usually 1200
            {90, 1275},
            {100, 1315},//1280
            {110, 1335},
            {100000000, 1335}
    };

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // change the pipeline to what you set on "limelight.local:5801"
        // you can change the pipeline while the camera is running to detect different types of tags (motif vs. localization)
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50); //100 // This sets how often we ask Limelight for data (# times per second)

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                MecanumDrive.PARAMS.logoFacingDirection,
                MecanumDrive.PARAMS.usbFacingDirection
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

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

        // wait to start because the camera takes a lot of power to run
        limelight.start();

        while (opModeIsActive() && !isStopRequested()) {
            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.updatePoseEstimate();

            //Read camera data
            LLResult result = limelight.getLatestResult();
            double tagX = 10000;
            double tagDistance = 0;
            if (result != null && result.isValid()) {
                tagX = result.getTx();
                Pose3D robotPose = result.getBotpose();
                double cameraXInches;
                double cameraYInches;
                double cameraHeadingRadians;
                double cameraDistance;
                cameraXInches = robotPose.getPosition().x * 39.3701; // convert from LL meters to RR inches
                cameraYInches = robotPose.getPosition().y * 39.3701; // convert from LL meters to RR inches
                cameraHeadingRadians = Math.toRadians(robotPose.getOrientation().getYaw()); // convert from LL degrees to RR radians
                drive.localizer.setPose(new Pose2d(cameraXInches, cameraYInches, cameraHeadingRadians)); // sets the RR pose to pose from LL
                cameraDistance = result.getBotposeAvgDist();

                // print out data from results
                telemetry.addData("target x", result.getTx());
                telemetry.addData("target y", result.getTy());
                telemetry.addData("distance", result.getBotposeAvgDist());
                telemetry.addData("camera yaw", robotPose.getOrientation().getYaw());
                telemetry.addData("camera x", robotPose.getPosition().x);
                telemetry.addData("camera y", robotPose.getPosition().y);
                telemetry.addData("camera z", robotPose.getPosition().z);
            }

            driver1.readButtons();
            driver2.readButtons();

            TelemetryPacket packet = new TelemetryPacket();

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                // runs all actions in runningActions list and re-adds them if they need to continue running
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            double goalX = -70;
            double goalY;
            if(team){
                goalY=-70;
            } else {
                goalY=70;
            }

            double loadingZoneX = 70;
            double loadingZoneY;
            if(team){
                loadingZoneY=70;
            } else {
                loadingZoneY=-70;
            }

            double distanceToTarget = Math.hypot(goalX-drive.localizer.getPose().position.x, goalY-drive.localizer.getPose().position.y);
            double distanceToLoadingZone = Math.hypot(loadingZoneX-drive.localizer.getPose().position.x, loadingZoneY-drive.localizer.getPose().position.y);

            if (driverControlled) {
                double xMult = 0.5;
                double yMult = 0.5;
                double rMult = 0.5;

                xMult += driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)/2;
                yMult += driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)/2;
                rMult += driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)/2;

                double y = -gamepad1.left_stick_y * xMult; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * yMult;
                double rx = 0;

                double goalAngle = Math.toDegrees(Math.atan2(goalY-drive.localizer.getPose().position.y, goalX-drive.localizer.getPose().position.x));
                if (team&&distanceToTarget>80) {
                    goalAngle-=3;
                } else {
                    goalAngle+=3;
                }
                double currentAngle = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
                goalAngle+=360;
                goalAngle%=360;
                currentAngle+=360;
                currentAngle%=360;
                double difference = ((goalAngle-currentAngle+540)%360)-180;

                if(targetLock) {
                    if (Math.abs(difference) < 20) {
                        limelight.setPollRateHz(25);
                    } else {
                        limelight.setPollRateHz(50);
                    }
                } else {
                    limelight.setPollRateHz(15);
                }

                if(targetLock&&distanceToLoadingZone>25) {
                    /*double goalAngle = Math.toDegrees(Math.atan2(goalY-drive.localizer.getPose().position.y, goalX-drive.localizer.getPose().position.x));
                    if (team&&distanceToTarget>80) {
                        goalAngle-=3;
                    } else {
                        goalAngle+=3;
                    }
                    double currentAngle = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
                    goalAngle+=360;
                    goalAngle%=360;
                    currentAngle+=360;
                    currentAngle%=360;
                    double difference = ((goalAngle-currentAngle+540)%360)-180;*/

                    if(!(tagX==10000)&&Math.abs(difference)<30&&distanceToTarget<80) {
                        /*if(distanceToTarget<80||team){
                            rx=(tagX-2)/-40;
                        } else {
                            rx=(tagX-5)/-40;
                        }*/
                        /*if(distanceToTarget>80){
                            if(team) {
                                rx = (tagX + 4) / -40.0;
                            } else {
                                rx = (tagX - 4) / -40.0;
                            }
                        } else {
                            rx=tagX/-40.0;
                        }*/
                        rx=tagX/-40.0;
                        //rx = (difference / 55) * -1;
                    } else if (Math.abs(difference)>30) {
                        rx = (difference / 55) * -1;
                    } else {
                        rx = (difference / 90) * -1;
                    }

                    if ((rx<0.05&&rx>-0.05)&&Math.abs(difference)<10){
                        rx=0.05*(rx/Math.abs(rx));
                    } else if (rx<0.1&&rx>-0.1){
                        rx=0.1*(rx/Math.abs(rx));
                    }

                    if (rx > 1) {
                        rx = 1;
                    }
                    if (rx < -1) {
                        rx = -1;
                    }
                } else {
                    rx = gamepad1.right_stick_x * rMult;
                }

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (driver1.getButton(GamepadKeys.Button.START)) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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

            double velocity = interpolate(launchSpeedLookup2DArray, distanceToTarget);//+95

            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                launcherAutoVelocity = false;
                runningActions.add(new ParallelAction(
                        launcher.setLauncherVelocity(1100)
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                launcherAutoVelocity = false;
                runningActions.add(new ParallelAction(
                        launcher.setLauncherVelocity(0)
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
                launcherAutoVelocity = false;
                runningActions.add(new ParallelAction(
                        launcher.setLauncherVelocity(1300)
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.Y)) {
                launcherAutoVelocity = true;
            }

            /*if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                launcherAutoVelocity = true;
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                launcherAutoVelocity = false;
            }*/

            if(launcherAutoVelocity){
                if(distanceToLoadingZone>25) {
                    launcher.launcher1.setVelocity(-velocity);
                    launcher.launcher2.setVelocity(velocity);
                } else {
                    launcher.launcher1.setVelocity(0);
                    launcher.launcher2.setVelocity(0);
                }
            } else {
                /*launcher.launcher1.setVelocity(0);
                launcher.launcher2.setVelocity(0);*/
            }

            telemetry.addData("distanceToTarget", distanceToTarget);
            telemetry.addData("distanceToLoadingZone", distanceToLoadingZone);
            telemetry.addData("autoLaunchSpeed", velocity);

            if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                if(launcher.launcher2.getVelocity()>500) {
                    runningActions.add(new ParallelAction(
                            //gate.cycleGate()
                            gate.threeGate()
                    ));
                }
            }
            /*if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                runningActions.add(new ParallelAction(
                        gate.setGatePosition(1)
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                runningActions.add(new ParallelAction(
                        gate.setGatePosition(0.7)
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                launcherAutoVelocity = false;
                if(launcher.launcher2.getVelocity()>500) {
                    runningActions.add(new ParallelAction(
                            gate.emptyGate(launcher)
                    ));
                }
            }*/
            /*if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                runningActions.add(new ParallelAction(
                        gate.setGatePosition(Mechanisms.Gate.openPosition)
                ));
            }*/

            if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                team = true;//blue
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                team = false;//red
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                targetLock = !targetLock;
            }

            /*double launcher1Velocity = launcher.launcher1.getVelocity();
            double launcher2Velocity = launcher.launcher2.getVelocity();*/

            telemetry.addData("launcher 1 velocity", launcher.launcher1.getVelocity());
            telemetry.addData("launcher 2 velocity", launcher.launcher2.getVelocity());

            telemetry.addData("imu:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("pose X", drive.localizer.getPose().position.x);
            telemetry.addData("pose Y", drive.localizer.getPose().position.y);
            telemetry.addData("pose heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.update();
        }
    }

    public static double interpolate(double[][] table, double x) {
        // Assume table is sorted by table[i][0] ascending
        for (int i = 0; i < table.length - 1; i++) {
            double x1 = table[i][0];
            double y1 = table[i][1];
            double x2 = table[i + 1][0];
            double y2 = table[i + 1][1];

            if (x >= x1 && x <= x2) {
                // Linear interpolation
                return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
            }
        }

        throw new IllegalArgumentException("x out of range");
    }
}