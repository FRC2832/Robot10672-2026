// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private static Alliance lastEnabledAlliance = Alliance.Red;

    private final RobotContainer robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay timeAndJoystickReplay = new HootAutoReplay().withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        robotContainer.drivetrain.resetMaximumSpeed();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        setDrivetrainRotationByAlliance();
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        setDrivetrainRotationByAlliance();
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    private void setDrivetrainRotationByAlliance() {
        Alliance currentAlliance = DriverStation.getAlliance().get();
        if (lastEnabledAlliance != currentAlliance) {
            lastEnabledAlliance = DriverStation.getAlliance().get();
            double rotationDegrees = DriverStation.getAlliance().get() == Alliance.Red ? 0.0 : 180.0;
            robotContainer.drivetrain.resetRotation(Rotation2d.fromDegrees(rotationDegrees));
        }
    }
}
