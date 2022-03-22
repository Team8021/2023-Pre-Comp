package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  public DrivetrainSubsystem() {
    SmartDashboard.putData("Field", m_field);
    leftEncoder.reset();
    rightEncoder.reset();
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
    m_odometry.update(m_gyro.getRotation2d(),
                      leftEncoder.getDistance(),
                      rightEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }




  private Encoder leftEncoder = new Encoder(0, 1);
  private Encoder rightEncoder = new Encoder(2, 3);
  private EncoderSim m_leftEncoderSim = new EncoderSim(leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(rightEncoder);
  private AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  private Field2d m_field = new Field2d();
  private DifferentialDrivetrainSim m_DifferentialDrivetrainSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),
    7.29,
    7.5,
    60,
    Units.inchesToMeters(3),
    .7112,
    VecBuilder.fill(.001, .001, .001, .1, .1, .005, .005)
    );
    private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), new Pose2d(5, 13.5, new Rotation2d()));
    
  @Override
  public void simulationPeriodic()
  {
    m_DifferentialDrivetrainSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(), 
                                          m_rightMotor.get() * RobotController.getInputVoltage());
    m_DifferentialDrivetrainSim.update(0.02);


    m_leftEncoderSim.setDistance(m_DifferentialDrivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_DifferentialDrivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_DifferentialDrivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_DifferentialDrivetrainSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_DifferentialDrivetrainSim.getHeading().getDegrees());
  }
}