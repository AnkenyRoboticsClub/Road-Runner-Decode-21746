package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Mechanisms {
    //class to create a wrist
    public static class Launcher {
        public DcMotor launcher1;
        public DcMotor launcher2;
        public Launcher(HardwareMap hardwareMap) {
            launcher1 = hardwareMap.get(DcMotor.class, "launcher1");
            launcher2 = hardwareMap.get(DcMotor.class, "launcher2");
        }
        public class StartLaunch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher1.setPower(1);
                launcher2.setPower(-1);
                return launcher1.getPower()<1;
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
                return launcher1.getPower()>0;
            }
        }
        public Action stopLauncher() {
            return new Launcher.StopLauncher();
        }
    }
}
