package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class Mechanisms {

    public static class Launcher {
        private final int rampUpTime = 0;//1000

        public DcMotorEx launcher1;
        public DcMotorEx launcher2;


        public Launcher(HardwareMap hardwareMap) {
            launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
            launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
            launcher1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            launcher2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        public class SetLauncherVelocity implements Action {
            private boolean initialized = false;
            private long startingTime = System.currentTimeMillis();
            private long timeElapsed = 0;
            private double velocity;

            public SetLauncherVelocity(double velocity) {
                this.velocity = velocity*1;
            } //1 can be changed to whatever the gear ratio is

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcher1.setVelocity(-velocity);
                    launcher2.setVelocity(velocity);

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

        // This is an old method which we kept in so we don't get errors from the old teleops
        // Do not use
        @Deprecated
        public Action setLauncherPower(double power) {
            RobotLog.w("Warning: 'setLauncherPower' is a depreciated method. Use 'setLauncherVelocity' instead.");
            return new Launcher.SetLauncherPower(power);
        }
    }

    public static class Gate {
        public Servo gate;

        public static double openPosition = 1;
        public static double closePosition = 0.3;

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

        public Action cycleGate() {
            return new SequentialAction(
                    /*setGatePosition(Gate.openPosition),
                    new SleepAction(2.0),*/
                    setGatePosition(Gate.closePosition),
                    new SleepAction(1),
                    setGatePosition(Gate.openPosition),
                    new SleepAction(1)
            );
        }

        public Action threeGate() {
            return new SequentialAction(
                    setGatePosition(Gate.closePosition),
                    new SleepAction(1.8),
                    setGatePosition(Gate.openPosition),
                    new SleepAction(0.4),
                    setGatePosition(Gate.closePosition),
                    new SleepAction(1.5),
                    setGatePosition(Gate.openPosition),
                    new SleepAction(0.4),
                    setGatePosition(Gate.closePosition),
                    new SleepAction(1.5),
                    setGatePosition(Gate.openPosition),
                    new SleepAction(0.3)
            );
        }

        public Action emptyGate(Launcher launcher) {
            return new SequentialAction(
                    setGatePosition(0.7),
                    new SleepAction(0.25),
                    new SleepAction(0.52),//0.6
                    launcher.setLauncherVelocity(launcher.launcher2.getVelocity()-20),//100
                    new SleepAction(0.40),//0.05
                    launcher.setLauncherVelocity(launcher.launcher2.getVelocity()-25),//100
                    new SleepAction(0.52),//0.5
                    new SleepAction(0.25),
                    launcher.setLauncherVelocity(0),
                    setGatePosition(1)

            );
        }
    }

}
