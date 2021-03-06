package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.RobotMap;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;

public class DrivetrainSubsystem extends SubsystemBase {

    private static final double TRACKWIDTH = 21.25; // width of the robot
    private static final double WHEELBASE = 23.5; // length of the robot

    // private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0);
    // private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0);
    // private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0);
    // private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0);

    private static DrivetrainSubsystem instance;
    private SwerveDriveOdometry m_odometry;

    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                    .angleEncoder(new TalonFX(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR))
                    .angleMotor(new TalonFX(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR))
                    .driveMotor(new TalonFX(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR)).build();

    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                    .angleEncoder(new TalonFX(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR))
                    .angleMotor(new TalonFX(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR))
                    .driveMotor(new TalonFX(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR)).build();

    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                    .angleEncoder(new TalonFX(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR))
                    .angleMotor(new TalonFX(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR))
                    .driveMotor(new TalonFX(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR)).build();

    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                    .angleEncoder(new TalonFX(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR))
                    .angleMotor(new TalonFX(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR))
                    .driveMotor(new TalonFX(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR)).build();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0));

    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

    public DrivetrainSubsystem() {
        gyroscope.calibrate();
        gyroscope.setInverted(true); // You might not need to invert the gyro
        m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, gyroscope.getRotation2d());

        frontLeftModule.setName("Front Left");
        frontRightModule.setName("Front Right");
        backLeftModule.setName("Back Left");
        backRightModule.setName("Back Right");
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }

        System.out.println("ROSS GET INSTANCE");
        return instance;
    }

    @Override
    public void periodic() {
        frontLeftModule.updateSensors();
        frontRightModule.updateSensors();
        backLeftModule.updateSensors();
        backRightModule.updateSensors();

        SmartDashboard.putNumber("Front Left Module Angle", Math.toDegrees(frontLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Front Right Module Angle", Math.toDegrees(frontRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Left Module Angle", Math.toDegrees(backLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Right Module Angle", Math.toDegrees(backRightModule.getCurrentAngle()));

        SmartDashboard.putNumber("Gyroscope Angle", gyroscope.getAngle().toDegrees());

        // stuff for auto
        // Update the odometry in the periodic block
        m_odometry.update(new Rotation2d(getHeading()), frontLeftModule.getState(), backLeftModule.getState(),
                frontRightModule.getState(), backRightModule.getState());

        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
    }

    public void drive(double speed, double strafe, double rotation, boolean fieldOriented) {
        // speed 20%
        Translation2d translation = new Translation2d(speed, strafe);
        drive(translation, rotation, fieldOriented);
    }

    public void resetGyroscope() {
        System.out.println("RESETTING GYRO");
        gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, gyroscope.getRotation2d());
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyroscope.getRotation2d().getDegrees();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        resetGyroscope();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();
        
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        // frontLeftModule.setDesiredState(desiredStates[0]);
        // frontRightModule.setDesiredState(desiredStates[1]);
        // backLeftModule.setDesiredState(desiredStates[2]);
        // backRightModule.setDesiredState(desiredStates[3]);
        frontLeftModule.setTargetVelocity(desiredStates[0].speedMetersPerSecond, desiredStates[0].angle.getRadians());
        frontRightModule.setTargetVelocity(desiredStates[1].speedMetersPerSecond, desiredStates[1].angle.getRadians());
        backLeftModule.setTargetVelocity(desiredStates[2].speedMetersPerSecond, desiredStates[2].angle.getRadians());
        backRightModule.setTargetVelocity(desiredStates[3].speedMetersPerSecond, desiredStates[3].angle.getRadians());
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyroscope.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void autoTest() {
        System.out.println("AUTO TEST");
        drive(0.2, 0, 0, true);

    }

    public void right90Turn() {
        System.out.println("RIGHT 90 TURN");
        Rotation2 initialAngle = gyroscope.getAngle();
        System.out.println("INITIAL ANGLE=" + initialAngle.toDegrees());
        // WHILE LOOP
        Rotation2 currentAngle = gyroscope.getAngle();
        Rotation2 targetAngle = Rotation2.fromDegrees(currentAngle.toDegrees() + 90);
        System.out.println("TARGET ANGLE=" + targetAngle.toDegrees());
        while (currentAngle.toDegrees() < targetAngle.toDegrees()) {
            drive(0, 0, targetAngle.toRadians(), true);
            // System.out.println("CURRENT ANGLE="+currentAngle.toDegrees());
            currentAngle = gyroscope.getAngle();
        }
        System.out.println("FINAL ANGLE=" + currentAngle.toDegrees());
    }

    public void left90Turn() {
        System.out.println("LEFT 90 TURN");
        drive(0, 0, -90, true);
    }

}