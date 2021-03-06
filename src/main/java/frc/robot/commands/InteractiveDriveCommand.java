package frc.robot.commands;

// import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.frcteam2910.common.robot.Utilities;

public class InteractiveDriveCommand extends CommandBase {
    // The subsystem to associate the commands with see invocation in RobotContainer.java
    private DrivetrainSubsystem m_subsystem;
    private XboxController  xbox = new XboxController(0);

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public InteractiveDriveCommand(DrivetrainSubsystem subsystem) {
     m_subsystem = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
    }


    @Override
    public void execute() {
        double forward = -xbox.getRawAxis(1);
        forward = Utilities.deadband(forward);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = -xbox.getRawAxis(0);
        strafe = Utilities.deadband(strafe);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = -xbox.getRawAxis(4);
        rotation = Utilities.deadband(rotation);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        m_subsystem.drive(new Translation2d(forward, strafe), rotation, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
