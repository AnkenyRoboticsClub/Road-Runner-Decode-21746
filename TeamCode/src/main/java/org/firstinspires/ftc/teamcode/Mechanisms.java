package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class Mechanisms {

    public static class Launcher {
        private final int rampUpTime = 1000;

        public DcMotorEx launcher1;
        public DcMotorEx launcher2;

        public Launcher(HardwareMap hardwareMap) {
            launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
            launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        }

        public class SetLauncherVelocity implements Action {
            private boolean initialized = false;
            private long startingTime = System.currentTimeMillis();
            private long timeElapsed = 0;
            private double velocity;

            public SetLauncherVelocity(double velocity) {
                this.velocity = velocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcher1.setPower(-velocity);
                    launcher2.setPower(velocity);
                    startingTime = System.currentTimeMillis();
                    initialized = true;
                }
                timeElapsed = System.currentTimeMillis() - startingTime;
                return timeElapsed < rampUpTime;
            }
        }

        public Action setLauncherVelocity(double velocity) {
            return new Launcher.SetLauncherVelocity(velocity);
        }

        // This is an old method which we kept in so we don't get errors from the old teleops
        // Do not use
        @Deprecated
        public Action setLauncherPower(double power) {
            RobotLog.w("Warning: 'setLauncherPower' is a depreciated method. Use 'setLauncherVelocity'.");
            return new SleepAction(0.0);
        }
    }

    public static class Gate {
        public Servo gate;

        public static double openPosition = 0.3;
        public static double closePosition = 0.0;

        public Gate(HardwareMap hardwareMap) {
            gate = hardwareMap.get(Servo.class, "gate");
        }

        public class SetGatePosition implements Action {
            private double position;

            public SetGatePosition(double position) {
                this.position = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gate.setPosition(position);
                return false;
            }
        }

        public Action setGatePosition(double position) {
            return new SetGatePosition(position);
        }
    }

}
