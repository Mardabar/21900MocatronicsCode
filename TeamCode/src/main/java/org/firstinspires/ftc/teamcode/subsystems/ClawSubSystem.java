package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ClawSubSystem {

        private Servo imaTouchU;
        public ClawSubSystem(HardwareMap hardwareMap) {
            imaTouchU = hardwareMap.get(Servo.class, "imaTouchU");

            double clawMin = 0;
            double clawMax = 1;
            imaTouchU.scaleRange(clawMin, clawMax);
        }
        public Action snap() {
            return new Action() {
                public boolean run(@NonNull TelemetryPacket packet) {
                    imaTouchU.setPosition(0);
                    return false;
                }
            };
        }
        public Action open() {
            return new Action() {
                public boolean run(@NonNull TelemetryPacket packet) {
                    imaTouchU.setPosition(1);
                    return false;
                }
            };
        }

}
