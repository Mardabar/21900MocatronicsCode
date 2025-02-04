/* package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive

@Autonomous
@Disabled
class LeftOneSpecimenPark : OpMode() {
    val beginPose = Pose2d(30.5, 66.0, -Math.PI / 2)
    lateinit var drive: PinpointDrive
    lateinit var scoringArm: scoringArm
    lateinit var specimenClaw: specimenClaw

    override fun init() {
        specimenClaw = specimenClaw(hardwareMap)
        drive = PinpointDrive(hardwareMap, beginPose)
        scoringArm = scoringArm(hardwareMap)
    }

    override fun start() {
        runBlocking(scoringArm.collect())
        runBlocking(specimenClaw.open(0.5))
        runBlocking(specimenClaw.close(2.0))
        runBlocking(scoringArm.score())
        runBlocking(
            drive.actionBuilder(beginPose)
                .setTangent(Math.PI)
                .splineToConstantHeading(Vector2d(2.0, 40.0), -Math.PI / 2)
                .strafeTo(Vector2d(2.0, 30.0))
                .build()
        )
        runBlocking(specimenClaw.open())
        runBlocking(
            drive.actionBuilder(drive.getPose())
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(Vector2d(-24.0, 54.0), Math.PI)
                .splineToConstantHeading(Vector2d(-24.0, 48.0), Math.PI)
                .splineToConstantHeading(Vector2d(-41.5, 64.0), Math.PI)
                .build()
        )

        //runBlocking(drive.actionBuilder(drive.getPose())
        //    .strafeToConstantHeading(Vector2d(-46.0,60.0)) //go to 1st sample
        //    .strafeToConstantHeading(Vector2d(-46.0,7.0)) //go to 1st sample
        //    .strafeToConstantHeading(Vector2d(-60.0,7.0)) //strafe over
        //    .strafeToConstantHeading(Vector2d(-55.0,60.0))//go to 2nd sample
        //    .strafeToConstantHeading(Vector2d(-55.0,7.0))//go to 2nd sample
        //    .strafeToConstantHeading(Vector2d(-63.0,7.0)) //strafe over
        //    .strafeToConstantHeading(Vector2d(-63.0,60.0)) //go to 3rd sample
        //    .build());
    }

    override fun loop() {
    }
}

*/