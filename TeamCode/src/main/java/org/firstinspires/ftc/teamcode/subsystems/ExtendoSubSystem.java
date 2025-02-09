package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import  com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExtendoSubSystem extends SubsystemBase{
    private DcMotor motor;
    private Telemetry telemetry;

    public ExtendoSubSystem(DcMotor motor, Telemetry telemetry) {
        this.motor = motor;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("Extendo Running", motor.getPower() != 0);
        telemetry.update();
    }

    public void runMotor() {
        motor.setPower(.6);
    }

    public void stopMotor() {
        motor.setPower(0);
    }
}
