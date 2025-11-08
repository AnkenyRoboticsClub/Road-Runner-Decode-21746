package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
//Need the autnomous tag in order for it show up on driver station as an autonomous program
// You can also set the name of the autonomous and the group
@Autonomous(name = "AutoV1", group = "Autonomous")
public class AutoV1 extends LinearOpMode {

    @Override
    public void runOpMode() {
        //set the starting position
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        //initialize our roadrunner drivetrain
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //init stuff from mechanism class
        Mechanisms.Launcher launcher = new Mechanisms.Launcher(hardwareMap);

        TrajectoryActionBuilder square = drive.actionBuilder(initialPose)
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                ;

        //wait for autonomous to start
        waitForStart();

        //check if stop is requested
        if (isStopRequested()) return;

        //run actions sequentially, so it will run each action in order
        Actions.runBlocking(
                new SequentialAction(
                        square.build()
                )
        );
    }
}