package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive

@Autonomous
@Disabled
class TwoSpecimenTwoSample : OpMode() {
    var beginPose: Pose2d? = null
    lateinit var drive: PinpointDrive
    lateinit var scoringArm: ScoringArm
    lateinit var specimenClaw: SpecimenClaw

    override fun init() {
        specimenClaw = SpecimenClaw(hardwareMap)
        beginPose = Pose2d(-17.5, 66.0, -Math.PI / 2)
        drive = PinpointDrive(hardwareMap, beginPose)
        scoringArm = ScoringArm(hardwareMap)
    }

    override fun start() {
        runBlocking(scoringArm.collect())
        runBlocking(specimenClaw.open(0.5))
        runBlocking(specimenClaw.close(2.0))
        runBlocking(scoringArm.score())
        runBlocking(
            drive.actionBuilder(beginPose)
                .setTangent(0.0)
                .splineToConstantHeading(Vector2d(2.0, 40.0), -Math.PI / 2)
                .strafeTo(Vector2d(2.0, 30.0))
                .build()
        )
        runBlocking(specimenClaw.open())
        runBlocking(
            drive.actionBuilder(drive.getPose())
                .strafeTo(Vector2d(-8.0, 54.0))
                .build()
        )
        runBlocking(specimenClaw.close())
        runBlocking(scoringArm.collect())
        runBlocking(specimenClaw.approach())
        runBlocking(
            drive.actionBuilder(drive.getPose())
                .setTangent(Math.PI / 2)
                .splineToLinearHeading(Pose2d(Vector2d(-27.0, 59.0), Math.PI), Math.PI)
                .strafeTo(Vector2d(-35.0, 59.0))
                .build()
        )
        runBlocking(specimenClaw.open())
        runBlocking(specimenClaw.close())
        runBlocking(scoringArm.score())
        runBlocking(
            drive.actionBuilder(drive.getPose())
                .setTangent(0.0)
                .splineToLinearHeading(Pose2d(Vector2d(-4.0, 55.0), -Math.PI / 2), -Math.PI / 2)
                .strafeTo(Vector2d(-4.0, 28.0))
                .build()
        )
        runBlocking(specimenClaw.open())
        runBlocking(
            drive.actionBuilder(drive.getPose())
                //.waitSeconds(0.3)
                //.strafeTo(Vector2d(-6.0, 44.0))
                //.waitSeconds(0.3)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(Vector2d(-24.0, 54.0), Math.PI)
                .splineToConstantHeading(Vector2d(-24.0, 48.0), Math.PI)
                .splineToConstantHeading(Vector2d(-49.0, 14.0), Math.PI)
                .build()
        )

        runBlocking(drive.actionBuilder(drive.getPose())
            .strafeToConstantHeading(Vector2d(-46.0,60.0)) //go to 1st sample
            .strafeToConstantHeading(Vector2d(-46.0,7.0)) //go to 1st sample
            .strafeToConstantHeading(Vector2d(-60.0,7.0)) //strafe over
            .strafeToConstantHeading(Vector2d(-55.0,60.0))//go to 2nd sample
            //.strafeToConstantHeading(Vector2d(-55.0,7.0))//go to 2nd sample
            //.strafeToConstantHeading(Vector2d(-63.0,7.0)) //strafe over
            //.strafeToConstantHeading(Vector2d(-63.0,60.0)) //go to 3rd sample
            .build());
    }

    override fun loop() {
    }
}

