// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import static frc.robot.Constants.*;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  public static boolean debug = true;
  String name;

  int cnt = 0;

  boolean m_inverted = false;

  // PID controllers for drive and steer motors
  private final PIDController m_drivePIDController = new PIDController(0.1, 0, 0);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      1,
      0,
      0,
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeed, kMaxAngularAcceleration));

  //private final PIDController m_turningPIDController = new PIDController(7, 0, 0);

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.1);

  private int m_motorChannel;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel   PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoder
   */

  private int m_id;
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int id) {
    m_motorChannel = driveMotorChannel;

    m_id = id;

    SmartDashboard.putString("mod" + m_motorChannel, "");
    m_driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);

    name = Drivetrain.chnlnames[m_id - 1];
    SmartDashboard.putString(name, "Working");

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(kDistPerRot); // inches to meters
    m_driveEncoder.setVelocityConversionFactor(kDistPerRot / 60); // convert RPM to meters per second

    // Set the distance (in this case, angle) in radians per pulse for the turning
    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningEncoder.setPositionConversionFactor(kRadiansPerRot); // inches to meters
    m_turningEncoder.setVelocityConversionFactor(kRadiansPerRot / 60); // convert RPM to meters per second

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //TODO check example and see if this is there: 
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

  }

  public void reset(){
    SmartDashboard.putString("mod" + m_motorChannel, " " + m_turningEncoder.getPosition() + " " + heading()); 
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
    cnt=0;
  }
  public double heading(){
    return m_turningEncoder.getPosition();
  }
  
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(heading());
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), getRotation2d());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), getRotation2d());
  }

  public double getDistance(){
    return m_driveEncoder.getPosition();
  }

  public double getVelocity() {
      return m_driveEncoder.getVelocity();
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    //SwerveModuleState state = desiredState;  // don't optimize
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getRotation2d());

    double velocity=getVelocity();
    // Calculate the drive output from the drive PID controller.
    double driveOutput = m_drivePIDController.calculate(velocity, state.speedMetersPerSecond);
    double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    double turn_angle=getRotation2d().getRadians();

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput = -m_turningPIDController.calculate(turn_angle, state.angle.getRadians());
    double turnFeedforward = 0; //-m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    double set_drive=driveOutput+driveFeedforward;
    double set_turn=turnOutput+turnFeedforward;;

    driveForward(set_drive);
    m_turningMotor.setVoltage(set_turn);
    
    if(debug){
      String s = String.format("Vel %-1.2f vs %-1.2f -> %-1.2f Angle %-3.1f vs %-3.1f -> %-1.2f\n", 
      velocity,state.speedMetersPerSecond,set_drive,Math.toDegrees(turn_angle), state.angle.getDegrees(), set_turn); 
      SmartDashboard.putString(name, s);
      // if((cnt%10)==0){
      //     System.out.println(name+" "+s);
      // }
      // cnt++;
    }   
  }

  public void log() {
    String s = String.format("Drive:%-1.2f m Angle:%-4.1f Abs:%-4.1f deg\n", 
    getDistance(), getRotation2d().getDegrees(),Math.toDegrees(heading()));
    SmartDashboard.putString(name, s);
  }

  // public void setOffset(double offset) {
  //   m_turningEncoder.configMagnetOffset(offset,20);
  // }

  public void setInverted(){
    m_inverted = true;
    //m_driveMotor.setInverted(true);
  }
  
  public void driveForward(double dist) {
    dist = m_inverted? -dist: dist;
    m_driveMotor.setVoltage(dist);
  }
  public void turnAround(double dist) {
    dist = m_inverted? -dist: dist;
    m_turningMotor.setVoltage(dist);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
