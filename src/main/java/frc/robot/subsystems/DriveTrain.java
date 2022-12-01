// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OdometryHandler;
import frc.robot.Shuffleboard;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private double maxV = 1;
  private double maxA = 1;

  private CANSparkMax leftMaster;
  private CANSparkMax leftSlave;
  private CANSparkMax rightMaster;
  private CANSparkMax rightSlave;

  private RelativeEncoder lEncoder;
  private RelativeEncoder rEncoder;

  private PIDController leftPid;
  private PIDController rightPid;

  private SimpleMotorFeedforward feed;

  private Shuffleboard board;

  private AHRS navx;

  private DifferentialDriveKinematics kinematics;

  private static DriveTrain driveTrain;

  private String KPleft = "kp left";
  private String KPright = "kp right";

  private String KIleft = "ki left";
  private String KIright = "ki right";

  private String KDleft = "kd left";
  private String KDright = "kd right";

  private final double radius = 0.0762;
  private final double gearRatio = 9.454;

  private OdometryHandler odometry;

  public DriveTrain() {
    leftMaster = new CANSparkMax(13, MotorType.kBrushless);
    leftSlave = new CANSparkMax(9, MotorType.kBrushless);
    rightMaster = new CANSparkMax(7, MotorType.kBrushless);
    rightSlave = new CANSparkMax(12, MotorType.kBrushless);

    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    changeMode(IdleMode.kBrake);

    rEncoder = rightMaster.getEncoder();
    lEncoder = leftMaster.getEncoder();

    feed = new SimpleMotorFeedforward(0, 0.15);

    navx = new AHRS(Port.kMXP);

    leftPid = new PIDController(0.02, 0, 0);
    rightPid = new PIDController(0.02, 0, 0);

    board = new Shuffleboard("Noya's chasis");
    board.addNum(KPleft, leftPid.getP());
    board.addNum(KPright, rightPid.getP());

    board.addNum(KIleft, leftPid.getI());
    board.addNum(KIright, rightPid.getI());

    board.addNum(KDleft, leftPid.getD());
    board.addNum(KDright, rightPid.getD());
    
    odometry = new OdometryHandler(this::getLeftDistance,
     this::getRightDistance, this::getAngle);
    odometry.reset(new Pose2d());

    kinematics = new DifferentialDriveKinematics(0.71);

    lEncoder.setPosition(0);
    rEncoder.setPosition(0);

    navx.reset();
  }

  public void changeMode(IdleMode mode) {
    rightMaster.setIdleMode(mode);
    leftMaster.setIdleMode(mode);
    leftSlave.setIdleMode(mode);
    rightSlave.setIdleMode(mode);
  }

  public double getLeftDistance() {
    return (lEncoder.getPosition() * 2 * radius * Math.PI) / gearRatio;
  }

  public double getRightDistance() {
    return (rEncoder.getPosition() * 2 * radius * Math.PI) / gearRatio;
  }

  public double getRightV() {
    return rEncoder.getVelocity() * 2 * radius * Math.PI * (1/gearRatio) / 60; 
  }

  public double getLeftV() {
    return lEncoder.getVelocity() * 2 * radius * Math.PI * (1/gearRatio) / 60; 
  }

  public double getAngle() {
    return navx.getYaw();
  }

  public void leftDrive(double power) {
    leftMaster.set(power);
  }

  public void rightDrive(double power) {
    rightMaster.set(power);
  }

  public void output(double leftPower, double rightPower) {
    leftDrive(leftPower);
    rightDrive(rightPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftPid.setPID(board.getNum(KPleft), board.getNum(KIleft), board.getNum(KDleft));
    rightPid.setPID(board.getNum(KPright), board.getNum(KIright), board.getNum(KDright));

    odometry.update();

    board.addNum("angle", getAngle());
    board.addNum("left distance", getLeftDistance());
    board.addNum("right distance", getRightDistance());

    board.addNum("left V", getLeftV());
    board.addNum("righ V", getRightV());


    board.addString("point", "(" + 
    odometry.getCurrentPosition().getTranslation().getX() + ","
    + odometry.getCurrentPosition().getTranslation().getY() + ")");
  }

  public static DriveTrain getInstance() {
    if (driveTrain == null) {
      driveTrain = new DriveTrain();
    }
    return driveTrain;
  }

  public DifferentialDriveWheelSpeeds speedsSupplier() {
    return new DifferentialDriveWheelSpeeds(getLeftDistance(), getRightDistance());
  }
  
  // public Command followTrajectoryCommand(String pathName) {
  //   PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName,
  //    new PathConstraints(maxV, maxA));
    
  //   return new SequentialCommandGroup(
  //     new InstantCommand(() -> {
  //       odometry.reset(trajectory.getInitialPose());
  //     }),
  //     new InstantCommand(() -> {
  //       navx.reset();
  //     }),
  //     new PPRamseteCommand (
  //       trajectory,
  //       odometry::getCurrentPosition,
  //       new RamseteController(),
  //       this.feed,
  //       kinematics,
  //       this::speedsSupplier,
  //       leftPid,
  //       rightPid,
  //       this::output,
  //       this
  //     )
  //   ).andThen(() -> this.output(0.0, 0.0));
  // }
}
