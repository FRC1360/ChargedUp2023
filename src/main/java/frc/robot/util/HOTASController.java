package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class HOTASController extends GenericHID {
    private static final int BUTTON_TRIGGER_HALF = 1;
    private static final int BUTTON_TRIGGER_FULL = 6;

    private static final int BUTTON_RED = 2;
    private static final int BUTTON_PINKY = 3;
    private static final int BUTTON_PINKY_LEVER = 4;
    private static final int BUTTON_INDEX = 5;

    private static final int DPADL_UP = 7;
    private static final int DPADL_RIGHT = 8;
    private static final int DPADL_DOWN = 9;
    private static final int DPADL_LEFT = 10;
    
    private static final int DPADR_UP = 11;
    private static final int DPADR_RIGHT = 12;
    private static final int DPADR_DOWN = 13;
    private static final int DPADR_LEFT = 14;
    
    private static final int DPAD_THUMB_FORWARD = 15;
    private static final int DPAD_THUMB_RIGHT = 16;
    private static final int DPAD_THUMB_BACKWARD = 17;
    private static final int DPAD_THUMB_LEFT = 19; // NEVER EVER USE THIS IT IS VERY HARD TO PRESS
    private static final int DPAD_THUMB_DOWN = 20; // NEVER EVER USE THIS IT IS VERY HARD TO PRESS

    // TODO check the acc values from driver station and shove their values in here.
    // i prolly need to make more and map like 20 buttons.

    private static final int AXIS_X = 0;
    private static final int AXIS_Y = 1;

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

    public boolean getTriggerHalfButton() {
        return joystick.getRawButton(BUTTON_TRIGGER_HALF);
    }

    public boolean getTriggerFullButton() {
        return joystick.getRawButton(BUTTON_TRIGGER_FULL);
    }

    public boolean getThumbButton() {
        return joystick.getRawButton(BUTTON_THUMB);
    }

    public boolean getFireAButton() {
        return joystick.getRawButton(BUTTON_FIRE_A);
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
