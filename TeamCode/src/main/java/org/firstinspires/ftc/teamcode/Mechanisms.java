package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Mechanisms {
    public static class Launcher {
        private long startingTime = System.currentTimeMillis();
        private long currentTime = 0;
        private final int rampUpTime =1000;

        public DcMotor launcher1;
        public DcMotor launcher2;
        public Launcher(HardwareMap hardwareMap) {
            launcher1 = hardwareMap.get(DcMotor.class, "launcher1");
            launcher2 = hardwareMap.get(DcMotor.class, "launcher2");
        }

        public class StartLaunch implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    if(launcher1.getPower()==1){
                        return false;
                    }
                    launcher1.setPower(-1);
                    launcher2.setPower(1);
                    startingTime = System.currentTimeMillis();
                    initialized = true;
                }
                currentTime = System.currentTimeMillis() - startingTime;
                return currentTime < rampUpTime;
            }
        }
        public Action startLaunch() {
            return new Launcher.StartLaunch();
        }
        public class StopLauncher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher1.setPower(0);
                launcher2.setPower(0);
                return false;
            }
        }
        public Action stopLauncher() {
            return new Launcher.StopLauncher();
        }

        public class SetLauncherPower implements Action {
            private boolean initialized = false;
            private double power;

            public SetLauncherPower(double power) {
                power = power;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcher1.setPower(-power);
                    launcher2.setPower(power);
                    startingTime = System.currentTimeMillis();
                    initialized = true;
                }
                currentTime = System.currentTimeMillis() - startingTime;
                return currentTime < rampUpTime;
            }
        }
        public Action setLauncherPower(double launchPower) {
            return new Launcher.SetLauncherPower(launchPower);
        }
    }




    /*public static class Gate {
        public Servo gate;
        public Gate(HardwareMap hardwareMap) {
            gate = hardwareMap.get(Servo.class, "gate");
        }
        public class OpenGate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gate.setPosition(1);
                return false;
            }
        }public Action openGate() {
            return new Gate.OpenGate();
        }
        public class CloseGate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gate.setPosition(-1);
                return false;
            }
        }
        public Action closeGate() {
            return new Gate.CloseGate();
        }
    }*/
}
