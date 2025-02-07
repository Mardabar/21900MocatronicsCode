package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
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


import java.lang.Math;

@Config
@Autonomous(name = "DavidRightAuto", group = "autonomous", preselectTeleOp = "StraferOpV3")
public abstract class DavidRightAuto extends LinearOpMode {


    public class LiftSlides {
        private DcMotorEx Llin;
        private DcMotorEx Rlin;

        public LiftSlides(HardwareMap hardwareMap) {
            Llin = hardwareMap.get(DcMotorEx.class, "Llin");
            Rlin = hardwareMap.get(DcMotorEx.class, "Rlin");
            Llin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Rlin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Llin.setDirection(DcMotorSimple.Direction.REVERSE);
            Rlin.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Llin.setPower(0.8);
                    Rlin.setPower(0.8);
                    initialized = true;
                }

                double pos = Rlin.getCurrentPosition();
                packet.put("Rlin", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    Rlin.setPower(0);
                    Llin.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Llin.setPower(-0.8);
                    Rlin.setPower(-0.8);
                    initialized = true;
                }

                double pos = Rlin.getCurrentPosition();
                packet.put("Rlin", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    Rlin.setPower(0);
                    Llin.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }

    // Extendo arm class
    public class Extendo {
        private DcMotorEx pickmeup;

        public Extendo(HardwareMap hardwareMap) {
            pickmeup = hardwareMap.get(DcMotorEx.class, "pickmeup");
            pickmeup.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pickmeup.setDirection(DcMotorSimple.Direction.FORWARD);
        }


        public class ArmOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pickmeup.setPower(0.4);
                    initialized = true;
                }

                double pos = pickmeup.getCurrentPosition();
                packet.put("pickmeup", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    pickmeup.setPower(0);
                    return false;
                }
            }
        }

        public Action armOut() {
            return new ArmOut();
        }

        public class ArmIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pickmeup.setPower(-0.4);
                    initialized = true;
                }

                double pos = pickmeup.getCurrentPosition();
                packet.put("pickmeup", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    pickmeup.setPower(0);
                    return false;
                }
            }
        }

        public Action armIn() {
            return new ArmIn();
        }
    }

    // arm up class
    public class rotat {
        private DcMotorEx rotat;

        public rotat(HardwareMap hardwareMap) {
            rotat = hardwareMap.get(DcMotorEx.class, "rotat");
            rotat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotat.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ArmUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rotat.setPower(0.9);
                    initialized = true;
                }

                double pos = rotat.getCurrentPosition();
                packet.put("rotat", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    rotat.setPower(0);
                    return false;
                }
            }
        }

        public Action armUp() {
            return new ArmUp();
        }

        public class ArmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rotat.setPower(-0.9);
                    initialized = true;
                }

                double pos = rotat.getCurrentPosition();
                packet.put("rotat", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    rotat.setPower(0);
                    return false;
                }
            }
        }

        public Action armDown() {
            return new ArmDown();
        }
    }

    // close and open claw class
    public class ClampClaw {
        private Servo imaTouchU;

        public ClampClaw(HardwareMap hardwareMap) {
            imaTouchU = hardwareMap.get(Servo.class, "imaTouchU");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                imaTouchU.setPosition(0.13);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                imaTouchU.setPosition(0.52);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

    }

    // move claw up and down class
    public class ClawMove {
        private Servo ankel;

        public ClawMove(HardwareMap hardwareMap) {
            ankel = hardwareMap.get(Servo.class, "ankel");
        }

        public class ClawUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ankel.setPosition(0.592);
                return false;
            }
        }

        public Action clawUp() {
            return new ClawUp();
        }

        public class ClawDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ankel.setPosition(0.567);
                return false;
            }
        }

        public Action clawDown() {
            return new ClawDown();
        }
    }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        // Slides instance
        LiftSlides slides = new LiftSlides(hardwareMap);
        // Extendo instance
        Extendo pickmeup = new Extendo(hardwareMap);

        // clampclaw instance
        ClampClaw imaTouchU = new ClampClaw(hardwareMap);
        // claw verticle instance
        ClawMove ankel = new ClawMove(hardwareMap);


        // vision here that outputs position
        int visionOutputPosition = 1;


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        //    Actions.runBlocking(ClampClaw.CloseClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();


            int startPosition = visionOutputPosition;
            telemetry.addData("Start Position", position);

            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;

            Action trajectoryActionChosen;
            if (startPosition == 1) {
                trajectoryActionChosen = tab1.build();
            } else if (startPosition == 2) {
                trajectoryActionChosen = tab2.build();
            } else {
                trajectoryActionChosen = tab3.build();
            }

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActionChosen,
                            //lift.liftUp(),
                            //          claw.openClaw(),
                            //                lift.liftDown(),
                            trajectoryActionCloseOut
                    )
            );
        }
    }
}