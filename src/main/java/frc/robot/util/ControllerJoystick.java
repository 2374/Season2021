package frc.robot.util;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;

public class ControllerJoystick {

    private Joystick joystick;

    public ControllerJoystick() {
        this.joystick = new Joystick(RobotMap.JOYSTICK_CONTROLLER_PORT);
    }

    public boolean isPressed(int button) { return joystick.getRawButton(button); }

    public double getSlider() { return (0.5+(0.5*-joystick.getRawAxis(RobotMap.JOYSTICK_CONTROLLER_SLIDER))); }

    public double getXAxis() { return Toolkit.deadZone(joystick.getRawAxis(RobotMap.JOYSTICK_CONTROLLER_X_AXIS), 0.2); }

    public double getYAxis() { return Toolkit.deadZone(joystick.getRawAxis(RobotMap.JOYSTICK_CONTROLLER_Y_AXIS), 0.2); }

    public double getZAxis() { return joystick.getRawAxis(RobotMap.JOYSTICK_CONTROLLER_Z_AXIS); }

    public Joystick getJoystick() { return joystick; }

}