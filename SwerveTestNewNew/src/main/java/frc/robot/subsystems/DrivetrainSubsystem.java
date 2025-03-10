// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import org.frcteam2910.c2022.Robot;
import frc.util.Utilities;
import frc.common.src.main.java.org.frcteam2910.common.control.*;
import frc.common.src.main.java.org.frcteam2910.common.math.Vector2;
import frc.common.src.main.java.org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import frc.common.src.main.java.org.frcteam2910.common.util.HolonomicDriveSignal;
import frc.common.src.main.java.org.frcteam2910.common.util.HolonomicFeedforward;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * SdsModuleConfigurations.MK4_L3.getDriveReduction() * SdsModuleConfigurations.MK4_L3.getWheelDiameter()
            * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(0.891,
            0.15, 0.13592);

    public static final double DRIVETRAIN_CURRENT_LIMIT = 50.0;

    public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
            new FeedforwardConstraint(4.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                    FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
            new MaxAccelerationConstraint(5.0), new CentripetalAccelerationConstraint(5.0)};

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(5.0, 0.0, 0.0), new PidConstants(5.0, 0.0, 0.0),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    private final SwerveDrivePoseEstimator estimator;

    private final Pigeon2 pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds currentVelocity = new ChassisSpeeds();
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final NetworkTableEntry motorOutputPercentageLimiterEntry;

    private double motorOutputLimiter;

  public DrivetrainSubsystem() {
    Mk4ModuleConfiguration mk4ModuleConfiguration = new Mk4ModuleConfiguration();
        mk4ModuleConfiguration.setDriveCurrentLimit(DRIVETRAIN_CURRENT_LIMIT);
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3, FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET);
        frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3, FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);
        backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3, BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);
        backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3, BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);
        estimator = new SwerveDrivePoseEstimator(getGyroscopeRotation(), new Pose2d(), kinematics,
                VecBuilder.fill(0.02, 0.02, 0.01), // estimator values (x, y, rotation) std-devs
                VecBuilder.fill(0.01), // Gyroscope rotation std-dev
                VecBuilder.fill(0.1, 0.1, 0.01)); // Vision (x, y, rotation) std-devs
        // motorOutputPercentageLimiterEntry = tab.add("Motor Percentage", 100.0).withWidget(BuiltInWidgets.kNumberSlider)
        //         .withProperties(Map.of("min", 0.0, "max", 100.0, "Block increment", 10.0)).withPosition(0, 3)
        //         .getEntry();

        // tab.addNumber("Odometry X", () -> Units.metersToFeet(getPose().getX()));
        // tab.addNumber("Odometry Y", () -> Units.metersToFeet(getPose().getY()));
        // tab.addNumber("Odometry Angle", () -> getPose().getRotation().getDegrees());
        // tab.addNumber("Velocity X", () -> Units.metersToFeet(getCurrentVelocity().vxMetersPerSecond));
        // tab.addNumber("Trajectory Position X", () -> {
        //     var lastState = follower.getLastState();
        //     if (lastState == null)
        //         return 0;

        //     return Units.metersToFeet(lastState.getPathState().getPosition().x);
        // });
        // tab.addNumber("Trajectory Velocity X", () -> {
        //     var lastState = follower.getLastState();
        //     if (lastState == null)
        //         return 0;

        //     return Units.metersToFeet(lastState.getVelocity());
        // });
        // tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
        // pigeon.configFactoryDefault();
  }

  private Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(pigeon.getYaw());
}

/**
 * Resets the rotation of the drivetrain to zero.
 */
public void zeroRotation() {
    estimator.resetPosition(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d()),
            getGyroscopeRotation());
}

/**
 * Returns the position of the robot
 */
public Pose2d getPose() {
    return estimator.getEstimatedPosition();
}

public double getMotorOutputLimiter() {
    return motorOutputLimiter;
}

public HolonomicMotionProfiledTrajectoryFollower getFollower() {
    return follower;
}

public ChassisSpeeds getCurrentVelocity() {
    return currentVelocity;
}

/**
 * Sets the position of the robot to the position passed in with the current
 * gyroscope rotation.
 */
public void setPose(Pose2d pose) {
    estimator.resetPosition(pose, getGyroscopeRotation());
}

/**
 * Sets the desired chassis speed of the drivetrain.
 */
public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
}

  @Override
  public void periodic() {
    SwerveModuleState currentFrontLeftModuleState = new SwerveModuleState(frontLeftModule.getDriveVelocity(),
                new Rotation2d(frontLeftModule.getSteerAngle()));
        SwerveModuleState currentFrontRightModuleState = new SwerveModuleState(frontRightModule.getDriveVelocity(),
                new Rotation2d(frontRightModule.getSteerAngle()));
        SwerveModuleState currentBackLeftModuleState = new SwerveModuleState(backLeftModule.getDriveVelocity(),
                new Rotation2d(backLeftModule.getSteerAngle()));
        SwerveModuleState currentBackRightModuleState = new SwerveModuleState(backRightModule.getDriveVelocity(),
                new Rotation2d(backRightModule.getSteerAngle()));

        currentVelocity = kinematics.toChassisSpeeds(currentFrontLeftModuleState, currentFrontRightModuleState,
                currentBackLeftModuleState, currentBackRightModuleState);

        estimator.update(getGyroscopeRotation(), currentFrontLeftModuleState, currentFrontRightModuleState,
                currentBackLeftModuleState, currentBackRightModuleState);

        var driveSignalOpt = follower.update(Utilities.poseToRigidTransform(getPose()),
                new Vector2(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond),
                currentVelocity.omegaRadiansPerSecond, Timer.getFPGATimestamp(), Robot.kDefaultPeriod);

        if (driveSignalOpt.isPresent()) {
            HolonomicDriveSignal driveSignal = driveSignalOpt.get();
            if (driveSignalOpt.get().isFieldOriented()) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveSignal.getTranslation().x,
                        driveSignal.getTranslation().y, driveSignal.getRotation(), getPose().getRotation());
            } else {
                chassisSpeeds = new ChassisSpeeds(driveSignal.getTranslation().x, driveSignal.getTranslation().y,
                        driveSignal.getRotation());
            }
        }

        motorOutputLimiter = motorOutputPercentageLimiterEntry.getDouble(0.0) / 100;

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
  }
}
