package org.frcteam2910.robot.drivers;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class Mk2SwerveModuleBuilder {
    /**
     * The gear ratio of the angle motor that ships with the standard kit.
     */
    private static final double DEFAULT_ANGLE_REDUCTION = 12.63 / 1.0;

    /**
     * The gear ratio of the drive motor that ships with the standard kit.
     */
    private static final double DEFAULT_DRIVE_REDUCTION = 6.86 / 1.0;

    /**
     * The diameter of the standard wheel in inches.
     */
    private static final double DEFAULT_WHEEL_DIAMETER = 4.0;

    /**
     * Default constants for angle pid running on-board with NEOs.
     */
    private static final PidConstants DEFAULT_ONBOARD_NEO_ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);

    /**
     * Default constants for angle pid running on-board with CIMs.
     */
    private static final PidConstants DEFAULT_ONBOARD_CIM_ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);

    /**
     * Default constants for angle pid running on-board with Mini CIMs
     */
    private static final PidConstants DEFAULT_ONBOARD_MINI_CIM_ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);

    private static final PidConstants DEFAULT_FALCON_ANGLE_CONSTANTS = new PidConstants(0.1, 0.0, 0.5);

    private final Vector2 modulePosition;

    private DoubleSupplier angleSupplier;
    private DoubleSupplier currentDrawSupplier;
    private DoubleSupplier distanceSupplier;
    private DoubleSupplier velocitySupplier;

    private DoubleConsumer driveOutputConsumer;
    private DoubleConsumer targetAngleConsumer;

    private DoubleConsumer initializeAngleCallback;
    private List<BiConsumer<SwerveModule, Double>> updateCallbacks = new ArrayList<>();

    public Mk2SwerveModuleBuilder(Vector2 modulePosition) {
        this.modulePosition = modulePosition;
    }

    /**
     * Configures the swerve module to use an analog encoder through one of the RoboRIO's analog input ports.
     *
     * @param encoder The analog input handle to use for the encoder.
     * @param offset  The offset of the encoder in radians. This value is added to the analog encoder reading to obtain
     *                the true module angle.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder angleEncoder(AnalogInput encoder, double offset) {
        angleSupplier = () -> {
            double angle = (1.0 - encoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI;
            angle += offset;
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        };

        return this;
    }

    public Mk2SwerveModuleBuilder angleEncoder(TalonFX motor) {
        return angleEncoder(motor, DEFAULT_FALCON_ANGLE_CONSTANTS, DEFAULT_ANGLE_REDUCTION);
    }

    public Mk2SwerveModuleBuilder angleEncoder(TalonFX motor, PidConstants constants, double reduction) {
        final double sensorCoefficient = (2.0 * Math.PI) / (reduction * 2048.0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = constants.p;
        config.slot0.kI = constants.i;
        config.slot0.kD = constants.d;

        motor.setNeutralMode(NeutralMode.Brake);

        motor.configAllSettings(config);
        
        angleSupplier = () -> {
            double currentAngle = sensorCoefficient * motor.getSensorCollection().getIntegratedSensorPosition();
            // Calculate the current angle in the range [0, 2pi)
            double currentAngleMod = currentAngle % (2.0 * Math.PI);
            if (currentAngleMod < 0.0) {
                currentAngleMod += 2.0 * Math.PI;
            }

            return currentAngle;
        };

        return this;
    }
   

    public Mk2SwerveModuleBuilder angleMotor(TalonFX motor) {
        return angleMotor(motor, DEFAULT_FALCON_ANGLE_CONSTANTS, DEFAULT_ANGLE_REDUCTION);
    }

    public Mk2SwerveModuleBuilder angleMotor(TalonFX motor, PidConstants constants, double reduction) {
        final double sensorCoefficient = (2.0 * Math.PI) / (reduction * 2048.0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = constants.p;
        config.slot0.kI = constants.i;
        config.slot0.kD = constants.d;

        motor.setNeutralMode(NeutralMode.Brake);

        motor.configAllSettings(config);

        targetAngleConsumer = targetAngle -> {
            double currentAngle = sensorCoefficient * motor.getSensorCollection().getIntegratedSensorPosition();
            // Calculate the current angle in the range [0, 2pi)
            double currentAngleMod = currentAngle % (2.0 * Math.PI);
            if (currentAngleMod < 0.0) {
                currentAngleMod += 2.0 * Math.PI;
            }

            // Figure out target to send to TalonFX because the encoder is continuous
            double newTarget = targetAngle + currentAngle - currentAngleMod;
            if (targetAngle - currentAngleMod > Math.PI) {
                newTarget -= 2.0 * Math.PI;
            } else if (targetAngle - currentAngleMod < -Math.PI) {
                newTarget += 2.0 * Math.PI;
            }

            motor.set(TalonFXControlMode.Position, newTarget / sensorCoefficient);
        };
        initializeAngleCallback = angle -> motor.getSensorCollection().setIntegratedSensorPosition(angle / sensorCoefficient, 50);

        return this;
    }

    /**
     * Configures the swerve module to use a PWM Spark MAX driving a NEO as it's angle motor.
     * <p>
     * The default PID constants are used. These values have been determined to work with all Mk2 modules
     * controlled by a NEO using the standard angle reduction.
     *
     * @param motor The PWM Spark MAX to use as the angle motor.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder angleMotor(SpeedController motor) {
        return angleMotor(motor, MotorType.NEO);
    }

    public Mk2SwerveModuleBuilder angleMotor(SpeedController motor, MotorType motorType) {
        switch (motorType) {
            case CIM:
                // Spark MAXs are special and drive brushed motors in the opposite direction of every other motor
                // controller
                if (motor instanceof Spark || motor instanceof CANSparkMax) {
                    motor.setInverted(true);
                }

                return angleMotor(motor, DEFAULT_ONBOARD_CIM_ANGLE_CONSTANTS);
            case MINI_CIM:
                // Spark MAXs are special and drive brushed motors in the opposite direction of every other motor controller
                if (motor instanceof Spark || motor instanceof CANSparkMax) {
                    motor.setInverted(true);
                }

                return angleMotor(motor, DEFAULT_ONBOARD_MINI_CIM_ANGLE_CONSTANTS);
            case NEO:
                return angleMotor(motor, DEFAULT_ONBOARD_NEO_ANGLE_CONSTANTS);
            default:
                throw new IllegalArgumentException("Unknown motor type " + motorType);
        }
    }

    /**
     * Configures the swerve module to use a generic speed controller as it's angle motor.
     * <p>
     * This method is usually used when custom PID tuning is required or a NEO is not used.
     *
     * @param motor     The speed controller to use as the angle motor.
     * @param constants The PID constants to use to control the module's angle (units are in radians).
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder angleMotor(SpeedController motor, PidConstants constants) {
        PidController controller = new PidController(constants);
        controller.setInputRange(0.0, 2.0 * Math.PI);
        controller.setContinuous(true);

        targetAngleConsumer = controller::setSetpoint;
        updateCallbacks.add((module, dt) -> motor.set(controller.calculate(module.getCurrentAngle(), dt)));

        return this;
    }

    
    public Mk2SwerveModuleBuilder driveMotor(TalonFX motor) {
        return driveMotor(motor, DEFAULT_DRIVE_REDUCTION, DEFAULT_WHEEL_DIAMETER);
    }

    public Mk2SwerveModuleBuilder driveMotor(TalonFX motor, double reduction, double wheelDiameter) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.configAllSettings(config);
        motor.setNeutralMode(NeutralMode.Brake);

        currentDrawSupplier = motor::getSupplyCurrent;
        distanceSupplier = () -> (Math.PI * wheelDiameter * motor.getSensorCollection().getIntegratedSensorPosition()) / (2048.0 * reduction);
        velocitySupplier = () -> (10.0 * Math.PI * wheelDiameter * motor.getSensorCollection().getIntegratedSensorVelocity()) / (2048.0 * reduction);
        driveOutputConsumer = output -> motor.set(TalonFXControlMode.PercentOutput, output);

        return this;
    }

    /**
     * Configures the swerve module to use a generic speed controller driving the specified motor.
     *
     * @param motor     The speed controller to use.
     * @param motorType The type of motor used.
     * @return The builder.
     */
    public Mk2SwerveModuleBuilder driveMotor(SpeedController motor, MotorType motorType) {
        // Spark MAXs are special and drive brushed motors in the opposite direction of every other motor controller
        if (motorType != MotorType.NEO && (motor instanceof Spark || motor instanceof CANSparkMax)) {
            motor.setInverted(true);
        }

        driveOutputConsumer = motor::set;

        return this;
    }

    /**
     * Builds and returns a configured swerve module.
     *
     * @return The built swerve module.
     */
    public SwerveModule build() {
        // Verify everything is populated
        if (angleSupplier == null) {
            // Absolute angle encoder not configured
            throw new IllegalStateException("No absolute encoder has been configured! See Mk2SwerveModuleBuilder.angleEncoder");
        } else if (driveOutputConsumer == null) {
            // Drive motor not configured
            throw new IllegalStateException("No drive motor has been configured! See Mk2SwerveModuleBuilder.driveMotor");
        } else if (targetAngleConsumer == null) {
            // Angle motor not configured
            throw new IllegalStateException("No angle motor has been configured! See Mk2SwerveModuleBuilder.angleMotor");
        }

        return new SwerveModuleImpl();
    }

    private final class SwerveModuleImpl extends SwerveModule {
        private final Object sensorLock = new Object();
        private double currentDraw = 0.0;
        private double velocity = 0.0;

        public SwerveModuleImpl() {
            super(modulePosition);

            if (initializeAngleCallback != null) {
                initializeAngleCallback.accept(angleSupplier.getAsDouble());
            }
        }

        @Override
        protected double readAngle() {
            return angleSupplier.getAsDouble();
        }

        protected double readCurrentDraw() {
            if (currentDrawSupplier == null) {
                return Double.NaN;
            }

            return currentDrawSupplier.getAsDouble();
        }

        @Override
        protected double readDistance() {
            if (distanceSupplier == null) {
                return Double.NaN;
            }

            return distanceSupplier.getAsDouble();
        }

        protected double readVelocity() {
            if (velocitySupplier == null) {
                return Double.NaN;
            }

            return velocitySupplier.getAsDouble();
        }

        @Override
        public double getCurrentVelocity() {
            synchronized (sensorLock) {
                return velocity;
            }
        }

        @Override
        public double getDriveCurrent() {
            synchronized (sensorLock) {
                return currentDraw;
            }
        }

        @Override
        protected void setTargetAngle(double angle) {
            targetAngleConsumer.accept(angle);
        }

        @Override
        protected void setDriveOutput(double output) {
            driveOutputConsumer.accept(output);
        }

        @Override
        public void updateSensors() {
            super.updateSensors();

            double newCurrentDraw = readCurrentDraw();
            double newVelocity = readVelocity();

            synchronized (sensorLock) {
                currentDraw = newCurrentDraw;
                velocity = newVelocity;
            }
        }

        @Override
        public void updateState(double dt) {
            super.updateState(dt);

            updateCallbacks.forEach(c -> c.accept(this, dt));
        }
    }

    public enum MotorType {
        CIM,
        MINI_CIM,
        NEO,
        FALCON_500
    }
}