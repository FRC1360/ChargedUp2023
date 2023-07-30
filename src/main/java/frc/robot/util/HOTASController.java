package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class HOTASController extends GenericHID {
    private static final int BUTTON_TRIGGER = 1;
    private static final int BUTTON_THUMB = 2;
    private static final int BUTTON_FIRE_A = 3;
    private static final int BUTTON_PRIMARY = 4;
    private static final int BUTTON_SECONDARY = 5;
    private static final int AXIS_TWIST = 6;

    // TODO check the acc values from driver station and shove their values in here.
    // i prolly need to make more and map like 20 buttons.

    private static final int AXIS_X = 0;
    private static final int AXIS_Y = 1;
    private static final int AXIS_THROTTLE = 2;

    private Joystick joystick;

    public HOTASController(int port) {
        joystick = new Joystick(port);
    }

    @Override
    public double getX(Hand hand) {
        if (hand.equals(Hand.kRight)) {
            return joystick.getRawAxis(AXIS_X);
        }
        return 0.0;
    }

    @Override
    public double getY(Hand hand) {
        if (hand.equals(Hand.kRight)) {
            return joystick.getRawAxis(AXIS_Y);
        }
        return 0.0;
    }

    public boolean getTriggerButton() {
        return joystick.getRawButton(BUTTON_TRIGGER);
    }

    public boolean getThumbButton() {
        return joystick.getRawButton(BUTTON_THUMB);
    }

    public boolean getFireAButton() {
        return joystick.getRawButton(BUTTON_FIRE_A);
    }

    public double getThrottleAxis() {
        return joystick.getRawAxis(AXIS_THROTTLE);
    }


    public boolean getPrimaryButton() {
        return joystick.getRawButton(BUTTON_PRIMARY);
    }

    public boolean getSecondaryButton() {
        return joystick.getRawButton(BUTTON_SECONDARY);
    }

    public double getTwistAxis() {
        return joystick.getRawAxis(AXIS_TWIST);
    }
}
