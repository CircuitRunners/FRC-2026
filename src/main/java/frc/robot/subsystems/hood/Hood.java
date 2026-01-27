package frc.robot.subsystems.hood;

import frc.lib.bases.ServoMotorSubsystem;

public class Hood extends ServoMotorSubsystem{
    public Hood() {
        super(
            HoodConstants.getMotorIO(),
            "Hood",
            HoodConstants.kEpsilonThreshold,
            HoodConstants.getServoHomingConfig());
    }
}
