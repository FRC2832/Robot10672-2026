// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.Launch;
import frc.robot.commands.SnailModeCmd;
import frc.robot.commands.SpinUp;
import frc.robot.commands.TurtleModeCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private static double maxSpeed = 1.0 * TunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                 // desired top
                                                                                                 // speed
    private static double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation
                                                                                             // per second
                                                                                             // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                     // motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(maxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driverController.getLeftY() * maxSpeed) // Drive
                                                                                // forward
                                                                                // with
                        // negative Y
                        // (forward)
                        .withVelocityY(-driverController.getLeftX() * maxSpeed) // Drive left
                                                                                // with negative
                                                                                // X (left)
                        .withRotationalRate(-driverController.getRightX() * maxAngularRate) // Drive
                                                                                            // counterclockwise
                                                                                            // with
                // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final SwerveRequest.Idle idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(
                        new Rotation2d(-driverController.getLeftY(),
                                -driverController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        new Trigger(() -> driverController.getLeftTriggerAxis() > 0.2).whileTrue(new TurtleModeCmd(drivetrain));
        new Trigger(() -> driverController.getLeftTriggerAxis() > 0.6).whileTrue(new SnailModeCmd(drivetrain));

        drivetrain.registerTelemetry(logger::telemeterize);
        SequentialCommandGroup launchSequence = new SequentialCommandGroup(
                new SpinUp(fuelSubsystem).withTimeout(CANFuelSubsystem.SPIN_UP_SECONDS),
                new Launch(fuelSubsystem));

        // Operator
        // While the left bumper on operator controller is held, intake Fuel
        operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
        // While the right bumper on the operator controller is held, spin up for 1
        // second, then launch fuel. When the button is released, stop.
        operatorController.rightBumper().whileTrue(launchSequence);
        // While the A button is held on the operator controller, eject fuel back out
        // the intake
        operatorController.a().whileTrue(new Eject(fuelSubsystem));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final SwerveRequest.Idle idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }

    public static double getMaxSpeed() {
        return maxSpeed;
    }

    public static void setMaxSpeed(double speed) {
        maxSpeed = speed;
    }

    public static double getMaxAngularRate() {
        return maxAngularRate;
    }

    public static void setMaxAngularRate(double angularRate) {
        maxAngularRate = angularRate;
    }
}
