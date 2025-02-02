package org.firstinspires.ftc.robotcontroller.external.samples

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

@TeleOp
@Disabled
class BaseOpMode : OpMode() {
    private val runtime = ElapsedTime()
    private lateinit var leftDrive: DcMotor
    private lateinit var rightDrive: DcMotor

    override fun init() {

        leftDrive = hardwareMap.get(DcMotor::class.java, "left_drive")
        rightDrive = hardwareMap.get(DcMotor::class.java, "right_drive")

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE)
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD)

        telemetry.addData("Status", "Initialized")
    }

    override fun loop() {
        val leftPower: Double
        val rightPower: Double

        val drive = -gamepad1.left_stick_y.toDouble()
        val turn = gamepad1.right_stick_x.toDouble()
        leftPower = Range.clip(drive + turn, -1.0, 1.0)
        rightPower = Range.clip(drive - turn, -1.0, 1.0)

        leftDrive.power = leftPower
        rightDrive.power = rightPower

        telemetry.addData("Status", "Run Time: $runtime")
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)
    }
}