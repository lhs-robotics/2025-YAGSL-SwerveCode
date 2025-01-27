package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorLevel;

public class elevatorCommand {
    private final Elevator elevatorSubsystem;
    private final ElevatorLevel targetLevel;

    public GoToLevelCommand(Elevator subsystem, ElevatorLevel level) {
        elevator = subsystem;
        targetLevel = level;

        // Declare subsystem dependencies
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // Command the elevator to the target level
        elevator.goToLevel(targetLevel);
    }

    @Override
    public boolean isFinished() {
        // Stop the command when the elevator is within a 1 cm tolerance of the target
        double currentHeight = elevator.getCurrentHeight();
        double targetHeight = targetLevel.getHeightCm();
        return Math.abs(currentHeight - targetHeight) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Optionally stop the motor if the command is interrupted
            elevator.goToHeight(elevator.getCurrentHeight());
        }
    }
}