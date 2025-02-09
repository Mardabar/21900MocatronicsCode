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
import org.firstinspires.ftc.teamcode.subsystems.ClampClawSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ClampClawSubSystem.ClawState;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;


@Autonomous(name = "RrRightAuto", group = "autonomous")
public class RrRightAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        // Intitalize the specimin claw
        ClampClawSubSystem ankel = new ClampClawSubSystem(hardwareMap);
        
        // vision here that outputs position
        int visionOutputPosition = 1;


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(15,15))
                .waitSeconds(1.5)
                .turn(90);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .turn(90)
                .strafeTo(new Vector2d(10,10));
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(ankel.closeClaw());



        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Remember to add to this loop for every new action
        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else {
            trajectoryActionChosen = tab2.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                     //   LiftSlides.LiftDown(),
                     //   lift.liftUp(),
                     //   claw.openClaw(),
                     //   lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );



/* This is usin pinpoint from wat i know how to do keep comented for rn cause guy said ig
        Actions.runBlocking(

                drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(0, 46))
                        .strafeTo(new Vector2d(0, 25))
                        .turn(Math.toRadians(-90))
                        .strafeTo(new Vector2d(100,25))

                        .build()); */


    }
}
