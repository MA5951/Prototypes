package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Collection extends SubsystemBase {

  private TalonSRX collectionOne;
  private TalonSRX collectionTwo;
  private DoubleSolenoid intakePiston;
  
  private static Collection collection;

  public Collection() {
    collectionOne = new TalonSRX(5);
    collectionTwo = new TalonSRX(8);
    intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
  }

  public void setMotors(double power) {
    collectionOne.set(ControlMode.PercentOutput, -power);
    collectionTwo.set(ControlMode.PercentOutput, power);
  }

  public void extendPistons(Value value){
      intakePiston.set(value);
  }

  @Override
  public void periodic() {
  }

  public static Collection getInstance() {
    if (collection == null) {
      collection = new Collection();
    }
    return collection;
  }
}
