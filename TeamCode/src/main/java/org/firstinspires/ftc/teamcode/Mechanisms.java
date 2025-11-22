package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Mechanisms {

    public static class Launcher {
        private final int rampUpTime = 1000;

        public DcMotor launcher1;
        public DcMotor launcher2;

        public Launcher(HardwareMap hardwareMap) {
            launcher1 = hardwareMap.get(DcMotor.class, "launcher1");
            launcher2 = hardwareMap.get(DcMotor.class, "launcher2");
        }

        public class SetLauncherPower implements Action {
            private boolean initialized = false;
            private long startingTime = System.currentTimeMillis();
            private long timeElapsed = 0;
            private double power;

            public SetLauncherPower(double power) {
                this.power = power;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcher1.setPower(-power);
                    launcher2.setPower(power);
                    startingTime = System.currentTimeMillis();
                    initialized = true;
                }
                timeElapsed = System.currentTimeMillis() - startingTime;
                return timeElapsed < rampUpTime;
            }
        }

        public Action setLauncherPower(double launchPower) {
            return new Launcher.SetLauncherPower(launchPower);
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
