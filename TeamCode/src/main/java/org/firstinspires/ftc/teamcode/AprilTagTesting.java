package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "AprilTagTesting", group = "TeleOp")
public class AprilTagTesting extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // change the pipeline to what you set on "limelight.local:5801"
        // you can change the pipeline while the camera is running to detect different types of tags (motif vs. localization)
        limelight.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                MecanumDrive.PARAMS.logoFacingDirection,
                MecanumDrive.PARAMS.usbFacingDirection
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        waitForStart();

        // wait to start because the camera takes a lot of power to run
        limelight.start();

        while (opModeIsActive() && !isStopRequested()) {

            // pass the current orientation from the imu to the camera
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            // get the results from the camera
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // read the results
                Pose3D robotPose = result.getBotpose_MT2();

                double xInches = robotPose.getPosition().x * 39.3701; // convert from LL meters to RR inches
                double yInches = robotPose.getPosition().y * 39.3701; // convert from LL meters to RR inches
                double headingRadians = Math.toRadians(robotPose.getOrientation().getYaw()); // convert from LL degrees to RR radians
                drive.localizer.setPose(new Pose2d(xInches, yInches, headingRadians)); // sets the RR pose to pose from LL

                // print out data from results
                telemetry.addData("target x", result.getTx());
                telemetry.addData("target y", result.getTy());
                telemetry.addData("target area", result.getTa());

                telemetry.addData("robot heading", robotPose.getOrientation().getYaw());
                telemetry.addData("robot position", robotPose.getPosition());

            }

        }

    }

}
