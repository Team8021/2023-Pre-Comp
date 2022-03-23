package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  public DrivetrainSubsystem() {

  }

  private CANSparkMax m_leftMotor = new CANSparkMax(Constants.MOTOR_LEFT_ID, MotorType.kBrushless);
  private CANSparkMax m_rightMotor = new CANSparkMax(Constants.MOTOR_RIGHT_ID, MotorType.kBrushless);

  private RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
  private RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();

  private DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  public void invertRightMotor() {
    m_rightMotor.setInverted(true);
  }

  public void setArcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    m_differentialDrive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  public void setCurvatureDrive(double xSpeed, double zRotation) {
    m_differentialDrive.curvatureDrive(xSpeed, zRotation, true);
  }

  public void setTankDrive(double leftSpeed, double rightSpeed) {
    m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stopDrive() {
    m_differentialDrive.stopMotor();
  }

  public double getLeftEncoderVelocity() {
    return m_leftEncoder.getVelocity();
  }
  
  public double getRightEncoderVelocity() {
    return m_rightEncoder.getVelocity();
  }

  public double getLeftEncoderPosition() {
    return m_leftEncoder.getPosition();
  }

  public double getRightEncoderPosition() {
    return m_rightEncoder.getPosition();
  }

  @Override
  public void periodic() {

  }



}