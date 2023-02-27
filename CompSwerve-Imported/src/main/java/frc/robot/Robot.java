// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.BNO055;
import frc.robot.subsystems.BNO055.BNO055OffsetData;
import frc.robot.subsystems.BNO055.SystemStatus;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.I2C;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private double[] pos = new double[3]; // [x,y,z] position data
	private BNO055.CalData cal;
	private DecimalFormat f = new DecimalFormat("+000.000;-000.000");
  public BNO055 m_Imu;
  private int[] bnoOffsets = {0, -42, -8, -24, -3, 0, 2, 299, -59, -25, 523};


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //m_robotContainer = new RobotContainer();
    //m_robotContainer.robotInit();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_Imu = BNO055.getInstance(
      BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
      BNO055.vector_type_t.VECTOR_EULER,
      I2C.Port.kOnboard,
      BNO055.BNO055_ADDRESS_A,
      bnoOffsets
    );

  }

  @Override
  public void disabledPeriodic() {
        if (!m_Imu.isInitialized()) {
      //System.out.println("COMMS: " + m_Imu.isSensorPresent()
      //      + ", INITIALIZED: " + m_Imu.isInitialized()
      //      + ", CALIBRATED: " + m_Imu.isCalibrated());
      SystemStatus sysStat = m_Imu.getSystemStatus();
      System.out.println(
        String.format(
          "status: %d, test result %d, error %d",
          sysStat.system_status,
          sysStat.self_test_result,
          sysStat.system_error
        )
      );
    } else {
    
      /* Display the floating point data */
      //pos = m_Imu.getVector();
      //System.out.println("\tYaw: " + f.format(pos[0])
      //    + " Pitch: " + f.format(pos[1]) + " Roll: " + f.format(pos[2])
      //    + "  Heading: " + m_Imu.getHeading());

    /* Display calibration status for each sensor. */
      cal = m_Imu.getCalibration();
      System.out.println("\tCALIBRATION: Sys=" + cal.sys
          + " Gyro=" + cal.gyro + " Accel=" + cal.accel
          + " Mag=" + cal.mag + " Heading=" + m_Imu.getRotation2d().getDegrees());
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //BNO055OffsetData offsets = m_Imu.readOffsets();
    m_Imu.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
