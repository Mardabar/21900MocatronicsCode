package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


//RR-specific imports

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.jvm.Code;


import java.lang.Math;

@Config
@Autonomous(name = "DavidRightAuto", group = "autonomous", preselectTeleOp = "StraferOpV3")
public class DavidRightAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(5, -68, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder hangPreload = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(5,-25))
                .waitSeconds(.5);
        TrajectoryActionBuilder pushSecondSample = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(5, -30))
                .turn(Math.toRadians(-180))
                .strafeTo(new Vector2d(44,-30))
                .strafeTo(new Vector2d(44, 1))
                .strafeTo(new Vector2d(48,-62 ));
       TrajectoryActionBuilder pushThirdSample = drive.actionBuilder(initialPose)
               .strafeTo(new Vector2d(47,1))
               .strafeTo(new Vector2d(57,-4.5));




        /* stuff in here will happen on initialization but keep commented for rn
            Actions.runBlocking(
                    new SequentialAction(

                    )
            ); */

            waitForStart();

            //Code Runnin

            if (isStopRequested()) return;

        // parralell actions happen at the same time
        // sequential actions go one after another

            if (opModeIsActive()) {
                Actions.runBlocking(
                        new SequentialAction(
                                new SequentialAction(
                                        hangPreload.build()
                                ),
                                new SequentialAction(
                                        pushSecondSample.build()
                                ),
                                new SequentialAction(
                                        pushThirdSample.build()
                                )
                        )
                );
            };

        }
    }
