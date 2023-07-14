package frc.lib.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.util.OrbitPID;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int magEncoderID;
    public final double angleOffset;
    public final PIDConstants anglePID;
    public final SimpleMotorFeedforward driveSVA;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     *
     * @param driveMotorID drive controller ID
     * @param angleMotorID angle controller ID
     * @param magEncoderID MagEncoder ID (DIO)
     * @param angleOffset  canCoder offset
     * @param anglePID     angle motor PID values
     * @param driveSVA     drive motor SVA values (feed forward)
     */

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int magEncoderID,
            double angleOffset, PIDConstants anglePID, SimpleMotorFeedforward driveSVA) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.magEncoderID = magEncoderID;
        this.angleOffset = angleOffset;
        this.anglePID = anglePID;
        this.driveSVA = driveSVA;
    }

}