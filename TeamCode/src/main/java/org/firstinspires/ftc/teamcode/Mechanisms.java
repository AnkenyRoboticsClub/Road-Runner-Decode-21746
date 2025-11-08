package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Mechanisms {
    public static class Launcher {
        private long startingTime = System.currentTimeMillis();
        private long currentTime = 0;
        private final int rampUpTime =50000;

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
                    launcher1.setPower(1);
                    launcher2.setPower(-1);
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
    }
}
