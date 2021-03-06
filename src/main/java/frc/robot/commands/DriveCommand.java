package frc.robot.commands;

// import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Translation2d;
// import org.frcteam2910.common.robot.Utilities;

public class DriveCommand extends CommandBase {
    // The subsystem to associate the commands with see invocation in
    // RobotContainer.java
    private DrivetrainSubsystem m_subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveCommand(double forward, double strafe, double rotation, DrivetrainSubsystem subsystem, boolean fieldOrientation) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

        // forward = Utilities.deadband(forward);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        // strafe = Utilities.deadband(strafe);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        // rotation = Utilities.deadband(rotation);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        m_subsystem.drive(new Translation2d(forward, strafe), rotation, fieldOrientation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
