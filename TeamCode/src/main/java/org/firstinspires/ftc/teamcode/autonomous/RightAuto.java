package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "RightAuto", preselectTeleOp = "StraferOpV3")
public class RightAuto extends LinearOpMode {
  
  private DcMotor leftFront;
  private DcMotor rightFront;
  private DcMotor leftBack;
  private DcMotor rightBack;
  private DcMotor pickmeup;
  private DcMotor Llin;
  private DcMotor Rlin;
  private DcMotor rotat;
  private IMU imu;
  private DistanceSensor sensor;
  
  private Servo imaTouchU;
  private Servo ankel;
  
  double armSpeed = 1.0;
  double power = .6;
  float yeeyaw;
  
  double leftBackSpeed = 0.4;
  double rightBackSpeed = -0.7;
  double leftFrontSpeed = 0.4;
  double rightFrontSpeed = -0.7;
  
  int leftBackPos = 1000;
  int rightBackPos = 1750;
  int leftFrontPos = -1000;
  int rightFrontPos = -1750;
  
  
   
  private void Forward(int _targetPos) {
    rightBack.setTargetPosition(rightBack.getCurrentPosition() + _targetPos);
    rightFront.setTargetPosition(rightFront.getCurrentPosition() + _targetPos);
    leftBack.setTargetPosition(leftBack.getCurrentPosition() - _targetPos);
    leftFront.setTargetPosition(leftFront.getCurrentPosition() - _targetPos);
    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBack.setPower(power);
    leftFront.setPower(power);
    rightBack.setPower(power);
    rightFront.setPower(power);
    while (leftBack.isBusy()) {
    
    telemetry.addData("rightBackCurrentPosition",rightBack.getTargetPosition());
    telemetry.update();
    }
  }
  
  private void WalleftFrontorward(double dist) {
    while (sensor.getDistance(DistanceUnit.CM) > dist + 15){
      leftBack.setPower(-power);
      leftFront.setPower(-power);
      rightBack.setPower(power);
      rightFront.setPower(power);
      telemetry.addData("Sensor", sensor.getDeviceName() );
      telemetry.addData("Distance (cm)", sensor.getDistance(DistanceUnit.CM));
      telemetry.update();
    }
    
    while (sensor.getDistance(DistanceUnit.CM) > dist){
      leftBack.setPower(-power * .3);
      leftFront.setPower(-power * .3);
      rightBack.setPower(power * .3);
      rightFront.setPower(power * .3);
      telemetry.addData("Sensor", sensor.getDeviceName() );
      telemetry.addData("Distance (cm)", sensor.getDistance(DistanceUnit.CM));
      telemetry.update();
    }
    
    leftBack.setPower(0);
    leftFront.setPower(0);
    rightBack.setPower(0);
    rightFront.setPower(0);
  }

// Stops and resets the encoder value stored in the motor

  private void CoolStrafe(int leftBacktp, int rightBacktp, int leftFronttp, int rightFronttp){
    leftBack.setTargetPosition(leftBack.getCurrentPosition() + leftBacktp);
    rightBack.setTargetPosition(leftFront.getCurrentPosition() + rightBacktp);
    leftFront.setTargetPosition(rightBack.getCurrentPosition() + leftFronttp);
    rightFront.setTargetPosition(rightFront.getCurrentPosition() + rightFronttp);
    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    double average = (Math.abs(leftBacktp) + Math.abs(rightBacktp) + Math.abs(leftFronttp) + Math.abs(rightFronttp)) / 4;
    double powMult = average / (average * power);
    leftBack.setPower(power * (average / Math.abs(leftBacktp)));
    rightBack.setPower(power * (average / Math.abs(rightBacktp)));
    leftFront.setPower(power * (average / Math.abs(leftFronttp)));
    rightFront.setPower(power * (average / Math.abs(rightFronttp)));
    while (leftBack.isBusy()) {
    }
  }

  private void Left(int _targetPos) {
    leftBack.setTargetPosition(leftBack.getCurrentPosition() - _targetPos);
    leftFront.setTargetPosition(leftFront.getCurrentPosition() + _targetPos);
    rightBack.setTargetPosition(rightBack.getCurrentPosition() - _targetPos);
    rightFront.setTargetPosition(rightFront.getCurrentPosition() + _targetPos);
    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBack.setPower(power);
    leftFront.setPower(power);
    rightBack.setPower(power);
    rightFront.setPower(power);
    while (leftBack.isBusy()) {
      telemetry.addData("rightBackCurrentPosition",rightBack.getTargetPosition());
      telemetry.update();
    }
  }
  
  private void Right(int _targetPos) {
    yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    leftBack.setTargetPosition(leftBack.getCurrentPosition() + _targetPos - 390);
    leftFront.setTargetPosition(leftFront.getCurrentPosition() - _targetPos);
    rightBack.setTargetPosition(rightBack.getCurrentPosition() + _targetPos - 390);
    rightFront.setTargetPosition(rightFront.getCurrentPosition() - _targetPos);
    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBack.setPower(power - yeeyaw / 15);
    leftFront.setPower(power + yeeyaw / 15);
    rightBack.setPower(power - yeeyaw / 15);
    rightFront.setPower(power + yeeyaw / 15);
    while (leftBack.isBusy()) {
      yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      leftBack.setPower(power - yeeyaw / 15);
      leftFront.setPower(power + yeeyaw / 15);
      rightBack.setPower(power - yeeyaw / 15);
      rightFront.setPower(power + yeeyaw / 15);
      telemetry.addData("Angle: ", yeeyaw);
      telemetry.update();
    }
  }
  
  private void TurnRightC(int _targetPos) {
    leftBack.setTargetPosition(leftBack.getCurrentPosition() - _targetPos); 
    leftFront.setTargetPosition(leftFront.getCurrentPosition() - _targetPos);
    rightBack.setTargetPosition(rightBack.getCurrentPosition() - _targetPos);
    rightFront.setTargetPosition(rightFront.getCurrentPosition() - _targetPos);
    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBack.setPower(power);
    leftFront.setPower(power);
    rightBack.setPower(power);
    rightFront.setPower(power);
    while (leftBack.isBusy()) {
      telemetry.addData("rightBackCurrentPosition",rightBack.getTargetPosition());
      telemetry.update();
    }
  }
  
    private void TurnLeftC(int _targetPos) {
    leftBack.setTargetPosition(leftBack.getCurrentPosition() + _targetPos); 
    leftFront.setTargetPosition(leftFront.getCurrentPosition() + _targetPos);
    rightBack.setTargetPosition(rightBack.getCurrentPosition() + _targetPos);
    rightFront.setTargetPosition(rightFront.getCurrentPosition() + _targetPos);
    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBack.setPower(power);
    leftFront.setPower(power);
    rightBack.setPower(power);
    rightFront.setPower(power);
    while (leftBack.isBusy()) {
      telemetry.addData("rightBackCurrentPosition",rightBack.getTargetPosition());
      telemetry.update();
    }
  }
  
  private void ArmIn(int _targetPos) {
    pickmeup.setTargetPosition(pickmeup.getCurrentPosition() + _targetPos);
    pickmeup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    pickmeup.setPower(armSpeed);
    while (pickmeup.isBusy()) {
    }
  }
  
  private void ArmOut(int _targetPos) {
    pickmeup.setTargetPosition(pickmeup.getCurrentPosition() - _targetPos);
    pickmeup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    pickmeup.setPower(armSpeed);
    while (pickmeup.isBusy()) {
    }
  }
  
  private void ArmUp(int _targetPos) {
    rotat.setTargetPosition(rotat.getCurrentPosition() + _targetPos);
    rotat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rotat.setPower(armSpeed);
    while (rotat.isBusy()) {
    }
  }
  
  private void ArmDown(int _targetPos) {
    rotat.setTargetPosition(rotat.getCurrentPosition() - _targetPos);
    rotat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rotat.setPower(armSpeed);
    while (rotat.isBusy()) {
    }
  }
  
  private void SimulArmUp(int _targetPos) {
    rotat.setTargetPosition(rotat.getCurrentPosition() + _targetPos);
    rotat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rotat.setPower(armSpeed);
  }
  
  private void SimulArmDown(int _targetPos) {
    rotat.setTargetPosition(rotat.getCurrentPosition() - _targetPos);
    rotat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rotat.setPower(armSpeed);
  }
  
  private void SlidesUp(int _targetPos) {
    Llin.setTargetPosition(Llin.getCurrentPosition() - _targetPos);
    Rlin.setTargetPosition(Rlin.getCurrentPosition() + _targetPos);
    Llin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rlin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Llin.setPower(armSpeed);
    Rlin.setPower(armSpeed);
    while (Llin.isBusy()) {
    }
  }
  
  private void SlidesDown(int _targetPos) {
    Llin.setTargetPosition(Llin.getCurrentPosition() + _targetPos);
    Rlin.setTargetPosition(Rlin.getCurrentPosition() - _targetPos);
    Llin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rlin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Llin.setPower(armSpeed);
    Rlin.setPower(armSpeed);
    while (Llin.isBusy()) {
    }
  }
  
  private void SimulSlidesUp(int _targetPos) {
    Llin.setTargetPosition(Llin.getCurrentPosition() - _targetPos);
    Rlin.setTargetPosition(Rlin.getCurrentPosition() + _targetPos);
    Llin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rlin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Llin.setPower(armSpeed);
    Rlin.setPower(armSpeed);
  }
  
  private void SimulSlidesDown(int _targetPos) {
    Llin.setTargetPosition(Llin.getCurrentPosition() + _targetPos);
    Rlin.setTargetPosition(Rlin.getCurrentPosition() - _targetPos);
    Llin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rlin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Llin.setPower(armSpeed);
    Rlin.setPower(armSpeed);
  }
  
  private void CloseClaw() {
    imaTouchU.setPosition(0.16);
    sleep(300);
  }
    
  private void OpenClaw() {
    imaTouchU.setPosition(0.58);
    sleep(300);
  }
  
  private void OpenClaw2() {
    imaTouchU.setPosition(0.6);
    sleep(500);
  }
  
  private void ClawDown() {
    ankel.setPosition(.567);
  }
  
  private void ClawUp() {
    ankel.setPosition(.658);
  }
  
  private void ClawSet(){
    ankel.setPosition(.612);
  }
  
  
  private void TurnRight(double turns) {
    imu.resetYaw();
    yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    while (yeeyaw > -50 * Math.pow(turns, 1.5)) {
      yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      leftBack.setPower(-power * 1.2);
      leftFront.setPower(-power * 1.2);
      rightBack.setPower(-power * 1.2);
      rightFront.setPower(-power * 1.2);
      telemetry.addLine("fast turn: " + yeeyaw);
      telemetry.update();
    }  
    if (yeeyaw > -90 * turns) {
      while (yeeyaw > -90 * turns) {
        yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        leftBack.setPower(-power * 0.3);
        leftFront.setPower(-power * 0.3);
        rightBack.setPower(-power * 0.3);
        rightFront.setPower(-power * 0.3);
        telemetry.addLine("slow turn: " + yeeyaw);
        telemetry.update();
      }
    }
    leftBack.setPower(0);
    leftFront.setPower(0);
    rightBack.setPower(0);
    rightFront.setPower(0);
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    imu.resetYaw();
  }
  
  private void TurnLeft(double turns) {
    imu.resetYaw();
    yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  while (yeeyaw < 50 * Math.pow(turns, 1.5)) {
    yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    leftBack.setPower(-power * -1.2);
    leftFront.setPower(-power * -1.2);
    rightBack.setPower(-power * -1.2);
    rightFront.setPower(-power * -1.2);
    telemetry.addLine("fast turn: " + yeeyaw);
    telemetry.update();
  }  
  if (yeeyaw < 90 * turns) {
    while (yeeyaw < 90 * turns) {
      yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      leftBack.setPower(-power * -0.3);
      leftFront.setPower(-power * -0.3);
      rightBack.setPower(-power * -0.3);
      rightFront.setPower(-power * -0.3);
      telemetry.addLine("slow turn: " + yeeyaw);
      telemetry.update();
    }
  }
    leftBack.setPower(0);
    leftFront.setPower(0);
    rightBack.setPower(0);
    rightFront.setPower(0);
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    imu.resetYaw();
  }
  


  
   @Override
  public void runOpMode() {
    int turnSpeed;
    int currentPos;
    int minRange;
    int maxRange;
    int armSpeed;

 /* IMU.Parameters myIMUparameters;
  myIMUparameters parameters = new IMU.Parameters(
  new RevHubOrientationOnRobot (
          RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
          RevHubOrientationOnRobot.UsbFacingDirection.DOWN
      )
  ); 
  parameters.mode               = BNO055IMU.SensorMode.IMU;
  parameters.angleUnit          = BNO055IMU.AngleUnit.DEGREES;
  parameters.accelUnit          = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; */
    
    rightBack = hardwareMap.get(DcMotor.class, "rightBack");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    leftBack = hardwareMap.get(DcMotor.class, "leftBack");
    leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    Llin = hardwareMap.get(DcMotor.class, "Llin");
    Rlin = hardwareMap.get(DcMotor.class, "Rlin");
    rotat = hardwareMap.get(DcMotor.class, "rotat");
    pickmeup = hardwareMap.get(DcMotor.class, "pickmeup");
    
    imaTouchU = hardwareMap.get(Servo.class, "imaTouchU");
    ankel = hardwareMap.get(Servo.class, "ankel");
    
    sensor = hardwareMap.get(DistanceSensor.class, "Sensor");
    
    imu = hardwareMap.get(IMU.class, "imu");
    
    rightFront.setDirection(DcMotor.Direction.FORWARD);
    rightBack.setDirection(DcMotor.Direction.FORWARD);
    leftBack.setDirection(DcMotor.Direction.FORWARD);
    leftFront.setDirection(DcMotor.Direction.FORWARD);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    imu.resetYaw();
    
    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Llin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Rlin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rotat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Llin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rlin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rotat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    currentPos = 0;
    turnSpeed = 818;
    minRange = 3;
    maxRange = 14;
    imaTouchU.scaleRange(.2, .8);
    ankel.scaleRange(0, 1);
    telemetry.addData("Sensor", sensor.getDeviceName() );
    telemetry.update();
     
    
//    imu.initialize(parameters);

    waitForStart();

while (opModeIsActive()) {
    imu.resetYaw();
    CloseClaw();
    SimulArmUp(670);
    SimulSlidesUp(2000);
    ClawUp();
    Forward(1180);
    ArmOut(1020);
    ArmIn(990);
    OpenClaw();
    sleep(70);
    Forward(-540);
    SimulArmDown(550);
    SimulSlidesDown(1620);
    sleep(200);
    Right(2222);
    //TurnRight(0.98);
    //Forward(1900);
    //TurnLeft(1);
    // power = 0.2;
    // Forward(30);
    // power = 0.6;
    // ArmOut(1020);
    // ClawDown();
    // sleep(300);
    // CloseClaw();
    // sleep(200);
    SimulArmUp(120);
    //ArmIn(990);
    TurnLeft(1.975);
    ClawSet();
    WalleftFrontorward(30);
    OpenClaw();
    power = 0.2;
    Forward(195);
    sleep(300);
    CloseClaw();
    sleep(200);
    power = 0.6;
    //armSpeed = 0.4;
    SimulArmUp(420);
    SimulSlidesUp(1800);
    Forward(-200);
    ClawUp();
    //armSpeed = 1;
    Right(2700);
    TurnRight(1.975);
    Forward(760);
    ArmOut(1020);
    ArmIn(990);
    OpenClaw();
    sleep(70);
    Forward(-400);
    SimulArmDown(310);
    SimulSlidesDown(1670);
    Forward(-500);
    sleep(200);
    imu.resetYaw();
    Right(2700);
    
    ClawSet();
    // TurnLeft(1.975);
    // Forward(120);
    // sleep(300);
    // CloseClaw();
    // sleep(200);
    // imu.resetYaw();
    // ClawUp();
    // SimulArmUp(360);
    // SimulSlidesUp(2000);
    // Forward(-200);
    // Right(2850);
    // TurnRight(1.975);
    // Forward(600);
    // ArmOut(1020);
    // ArmIn(990);
    // OpenClaw();
    // sleep(70);
    // Forward(-800);
    // Right(2850);
    //WalleftFrontorward(30);
    // sleep(200);
    // CloseClaw();
    // sleep(200);
    // SimulArmUp(310);
    // SimulSlidesUp(1500);
    // Forward(-650);
    // ClawUp();
    // TurnRight(1.02);
    // TurnRight(1.02);
    // Right(-2050);
    // WalleftFrontorward(12);
    // ArmOut(900);
    // ArmIn(870);
    // OpenClaw();
    // sleep(70);
    // Forward(-400);
    
    //CoolStrafe(6000, 6000, 1500, 1500);
    /*
    Forward(385);
    sleep(200);
    OpenClaw();
    sleep(166);
    Forward(-200);
    sleep(130);
    power = .86;
    SlidesDown(3900); 
    sleep(85);
    power = .69;
    TurnRightC(1200);
    sleep(100);
    Right(558);
    ArmDown(1310);
    Forward(790);
    ClawDown();
    OpenClaw2();
    if (Sensor.getDistance(DistanceUnit.CM) > minRange && Sensor.getDistance(DistanceUnit.CM) < maxRange) {
      ArmOut(1100);
      CloseClaw();
      sleep(100);
      ArmUp(1320);
       break;
    }  */
      break;
    }
  }
}