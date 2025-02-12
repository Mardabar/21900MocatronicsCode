package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import java.lang.Math;


@Autonomous(name = "RrRightAuto", group = "autonomous")
public class RrRightAuto extends LinearOpMode {

    private Servo imaTouchU;

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(5, -68, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);


        waitForStart();
        if (isStopRequested()) return;

        // Define a trajectory sequence


        Actions.runBlocking(

                drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(5,-25))
                        .waitSeconds(.5)
                        .strafeTo(new Vector2d(5, -30))
                        .turn(Math.toRadians(-180))
                        .strafeTo(new Vector2d(44,-30))
                        .strafeTo(new Vector2d(44, 1))

                        //.strafeTo(new Vector2d(42,))
                        /*  .turn(Math.toRadians(-90))
                          .strafeTo(new Vector2d(50,-58))
                          .strafeTo(new Vector2d(47,-12))
                          .strafeTo(new Vector2d(55,-12))
                          .strafeTo(new Vector2d(55,-60))
                          .waitSeconds(.5)
                          .turn(Math.toRadians(-180))
                          .strafeTo(new Vector2d(3, -30))
                          .waitSeconds(.5)
                          .strafeTo(new Vector2d(40,-56))
                          .turn(Math.toRadians(180))
                          .strafeTo(new Vector2d(50,-56))
                          .waitSeconds(.5)
                          .strafeTo(new Vector2d(3,-30)) */
                        .build());

    }
}
