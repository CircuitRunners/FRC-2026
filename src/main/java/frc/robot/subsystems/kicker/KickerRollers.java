package frc.robot.subsystems.kicker;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class KickerRollers extends MotorSubsystem<MotorIOTalonFX> {
    // Setpoints are Placeholders (except for idle), once you rename them in here, make sure you rename them in KickerRollerConstants as well.
    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    public static final Setpoint POINT_ONE = Setpoint.withVoltageSetpoint(KickerRollerConstants.kPH_ONE);
    public static final Setpoint POINT_TWO = Setpoint.withVoltageSetpoint(KickerRollerConstants.kPH_TWO);
    public static final Setpoint POINT_THREE = Setpoint.withVoltageSetpoint(KickerRollerConstants.kPH_THREE);

    public static final KickerRollers mInstance = new KickerRollers();

    public KickerRollers() {
        super(KickerRollerConstants.getMotorIO(), "Kicker Rollers");
    }
}
