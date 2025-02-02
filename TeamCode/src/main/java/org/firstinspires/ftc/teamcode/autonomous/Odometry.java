package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Odometry (Blocks to Java)")
public class Odometry extends LinearOpMode {

  private DcMotor leftBack;
  private DcMotor leftFront;
  private DcMotor rightBack;
  private DcMotor rightFront;
  private IMU imu;

  double tempSpeed;
  double max;
  double min;

  /**
   * Describe this function...
   */
  private void initialize() {
    // Recalibrates the Odometry Computer's internal IMU. Robot MUST Be
    // stationary. Device takes a large number of samples, and uses those
    // as the gyroscope zero-offset. This takes approximately 0.25 seconds.
    PinpointBlocks.recalibrateIMU();
    // Resets the current position to 0,0,0 and recalibrates the Odometry Computer's
    // internal IMU. Robot MUST Be stationary. Device takes a large number of samples,
    // and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
    PinpointBlocks.resetPosAndIMU();
    // Setting the Boolean to true reverses the encoder, false leaves it normal
    PinpointBlocks.reverseEncoders(true, true);
    // sets the number of ticks per mm of linear travel for the odometry pod you are using
    // ticks per unit of the goBILDA 4-Bar Odometry Pod
    PinpointBlocks.encoderResolution(PinpointBlocks.FourightBackarOdometryPod(DistanceUnit.INCH), DistanceUnit.INCH);
    // not exact
    // Sets the odometry pod positions relative to the point that the odometry computer
    // tracks around.The X pod offset refers to how far sideways from the tracking point
    // the X (forward) odometry pod is. left is positivethe Y Pod offset refers to how far
    // forward from the tracking point the Y (strafe) odometry pod is. Forward increases
    PinpointBlocks.offsets(DistanceUnit.INCH, -5.1, 1.6);
    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ((DcMotorEx) leftBack).setPositionPIDFCoefficients(5);
    ((DcMotorEx) leftFront).setPositionPIDFCoefficients(5);
    ((DcMotorEx) rightBack).setPositionPIDFCoefficients(5);
    ((DcMotorEx) rightFront).setPositionPIDFCoefficients(5);
    changeMotorPIDF(1.2, 1.2, 0, 12);
    // Initialize the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
    // Expansion Hub, specifying the hub's orientation on the robot via the direction that
    // the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
    imu.resetYaw();
  }

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    leftBack = hardwareMap.get(DcMotor.class, "leftBack");
    leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    rightBack = hardwareMap.get(DcMotor.class, "rightBack");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    imu = hardwareMap.get(IMU.class, "imu");

    initialize();
    waitForStart();
    if (opModeIsActive()) {
      forward(24, 0.6);
      IMUTurn(90, 0.2);
      strafe(12, 0.6);
      telemetry2();
    }
  }

  /**
   * Describe this function...
   */
  private void move(int speed) {
    leftBack.setPower(-speed);
    leftFront.setPower(-speed);
    rightBack.setPower(speed);
    rightFront.setPower(speed);
  }

  /**
   * Describe this function...
   */
  private void strafing(int speed) {
    leftBack.setPower(-speed);
    leftFront.setPower(speed);
    rightBack.setPower(-speed);
    rightFront.setPower(speed);
  }

  /**
   * Describe this function...
   */
  private double getAngle() {
    // gets the angle of the robot
    return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
  }

  /**
   * Describe this function...
   */
  private void changeMotorPIDF(double p, double i, int d, int f) {
    ((DcMotorEx) leftBack).setVelocityPIDFCoefficients(p, i, d, f);
    ((DcMotorEx) leftFront).setVelocityPIDFCoefficients(p, i, d, f);
    ((DcMotorEx) rightBack).setVelocityPIDFCoefficients(p, i, d, f);
    ((DcMotorEx) rightFront).setVelocityPIDFCoefficients(p, i, d, f);
  }

  /**
   * Describe this function...
   */
  private void telemetry2() {
    // Returns x position the unit of your choice
    // Returns y position the unit of your choice
    telemetry.addData("Pose: ", "(" + Double.parseDouble(JavaUtil.formatNumber(PinpointBlocks.xPosition(DistanceUnit.INCH), 3)) + "," + Double.parseDouble(JavaUtil.formatNumber(PinpointBlocks.yPosition(DistanceUnit.INCH), 3)) + ")");
    telemetry.addLine("Forward is X positive, Left is Y positive");
    // Returns the direction your robot is facing the unit of your choice
    telemetry.addData("Orientation using Degrees: ", PinpointBlocks.orientation(AngleUnit.DEGREES));
    telemetry.addData("speed", tempSpeed);
    // Call this once per loop to read new data from the Odometry Computer. Data will only update once this is called.
    PinpointBlocks.update();
    telemetry.update();
  }

  
  // This doesnt work yet
  private void IMUTurn(int angle, double turnSpeed) {
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    max = angle + 0.5;
    min = angle - 0.5;
    if (getAngle() < min || getAngle() > max) {
      // while the angle is less than min, or the angle is more than max, and the op mode is active
      while ((getAngle() < min || getAngle() > max) && opModeIsActive()) {
        // if the current angle is less than min turn left, when the angle is greater than max, stop
        if (getAngle() < min) {
          // Positive is left, negative is right
          // turning with the set speed
          strafing((int) turnSpeed);
          // while the angle is less than min, or the angle is more than max
          while ((getAngle() < min || getAngle() > max) && opModeIsActive()) {
            if (getAngle() > max) {
              break;
            }
          }
          // stops turning
          strafing(0);
        } else if (getAngle() > max) {
          strafing((int) -turnSpeed);
          while ((getAngle() < min || getAngle() > max) && opModeIsActive()) {
            if (getAngle() < min) {
              break;
            }
          }
          // stops turning
          strafing(0);
        }
      }
    }
  }


  private void forward(int dist, double speed) {
    tempSpeed = speed;
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // Angle from 0 to 180 is turning left from the 0 point, Angle from 0 to -180 is turning right from the 0 point
    telemetry2();
    max = dist + 0.5;
    min = dist - 0.5;
    // while the angle is less than min, or the angle is more than max, and the op mode is active
    telemetry2();
    if (PinpointBlocks.xPosition(DistanceUnit.INCH) < min || PinpointBlocks.xPosition(DistanceUnit.INCH) > max) {
      while ((PinpointBlocks.xPosition(DistanceUnit.INCH) < min || PinpointBlocks.xPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
        telemetry2();
        // if the current angle is less than min turn left, when the angle is greater than max, stop
        if (PinpointBlocks.xPosition(DistanceUnit.INCH) < min) {
          // Positive is left, negative is right
          // turning with the set speed
          telemetry2();
          move((int) tempSpeed);
          while ((PinpointBlocks.xPosition(DistanceUnit.INCH) < min || PinpointBlocks.xPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
            telemetry2();
            if (PinpointBlocks.xPosition(DistanceUnit.INCH) > max) {
              telemetry2();
              break;
            }
          }
          // stops turning
          move(0);
          telemetry2();
        } else if (PinpointBlocks.xPosition(DistanceUnit.INCH) > max) {
          telemetry2();
          move((int) -tempSpeed);
          while ((PinpointBlocks.xPosition(DistanceUnit.INCH) < min || PinpointBlocks.xPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
            telemetry2();
            // Returns x position the unit of your choice
            if (PinpointBlocks.xPosition(DistanceUnit.INCH) < min) {
              telemetry2();
              break;
            }
          }
          telemetry2();
          move(0);
        }
        tempSpeed = 0.1;
      }
    }
  }

  /**
   * Describe this function...
   */
  private void strafe(int dist, double speed) {
    tempSpeed = speed;
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // Angle from 0 to 180 is turning left from the 0 point, Angle from 0 to -180 is turning right from the 0 point
    telemetry2();
    max = dist + 0.5;
    min = dist - 0.5;
    // while the angle is less than min, or the angle is more than max, and the op mode is active
    telemetry2();
    if (PinpointBlocks.yPosition(DistanceUnit.INCH) < min || PinpointBlocks.yPosition(DistanceUnit.INCH) > max) {
      while ((PinpointBlocks.yPosition(DistanceUnit.INCH) < min || PinpointBlocks.yPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
        telemetry2();
        if (PinpointBlocks.yPosition(DistanceUnit.INCH) < min) {
          telemetry2();
          strafing((int) tempSpeed);
          while ((PinpointBlocks.yPosition(DistanceUnit.INCH) < min || PinpointBlocks.yPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
            telemetry2();
            if (PinpointBlocks.yPosition(DistanceUnit.INCH) > max) {
              telemetry2();
              break;
            }
          }
          // stops turning
          move(0);
          telemetry2();
        } else if (PinpointBlocks.yPosition(DistanceUnit.INCH) > max) {
          telemetry2();
          strafing((int) -tempSpeed);
          while ((PinpointBlocks.yPosition(DistanceUnit.INCH) < min || PinpointBlocks.yPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
            telemetry2();
            if (PinpointBlocks.yPosition(DistanceUnit.INCH) < min) {
              telemetry2();
              break;
            }
          }
          telemetry2();
          move(0);
        }
        tempSpeed = 0.1;
      }
    }
  }
}
