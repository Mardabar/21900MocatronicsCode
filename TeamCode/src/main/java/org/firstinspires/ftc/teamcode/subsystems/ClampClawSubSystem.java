package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClampClawSubSystem {

    /**
     * Enum to represent claw states and corresponding servo positions.
     */
    public enum ClawState {
        Open(0.5),
        Close(0.16);

        public final double clawPosition;

        ClawState(double clawPosition) {
            this.clawPosition = clawPosition;
        }
    }


    private final Servo ankel;

    public ClampClawSubSystem (HardwareMap hardwareMap) {
        this.ankel = hardwareMap.get(Servo.class, "ankel");
    }

    /**
     * Action class to change the claw state and hold it for a specified duration.
     */
    private class SetState implements Action {
        private final double duration;
        private final ClawState state;
        private double startTime = -1.0;

        public SetState(double duration, ClawState state) {
            this.duration = duration;
            this.state = state;
        }

        private double getCurrentTime() {
            return System.nanoTime() / 1e9; // Convert nanoseconds to seconds
        }

        @Override
        public boolean run(TelemetryPacket p) {
            if (startTime < 0) {
                startTime = getCurrentTime();
                ClawState clawState = state;
                ankel.setPosition(state.clawPosition);
            }
            double elapsedTime = getCurrentTime() - startTime;
            return elapsedTime < duration;
        }
    }

    /**
     * Methods to set claw actions with default durations.
     */
    public Action close(double duration) {
        return new SetState(duration, ClawState.Close);
    }

    public Action close() {
        return close(0.2);
    }

    public Action open(double duration) {
        return new SetState(duration, ClawState.Open);
    }

    public Action open() {
        return open(0.2);
    }
}
