package org.firstinspires.ftc.teamcode.autonomous;

//RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleops.StraferOpV3;

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
        public Action liftDown(){
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
        public Action armIn(){
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

    // close and open claw class
    public class imaTouchU {
        private Servo imaTouchU;

        public imaTouchU(HardwareMap hardwareMap) {
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
    public class ankel {
        private Servo ankel;

        public ankel(HardwareMap hardwareMap) {
            ankel = hardwareMap.get(Servo.class, "ankel");
        }
    }
    }
}






