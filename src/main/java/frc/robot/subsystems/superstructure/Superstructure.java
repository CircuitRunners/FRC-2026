package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.BeamBreakIO;
import frc.robot.subsystems.drive.Drive;

public class Superstructure {
    private final Drive drive;
    public static BeamBreakIO endEffectorVelocityDip;

    public Superstructure(Drive drive) {
        this.drive = drive;
    }


    private boolean isPathFollowing = false;
    private boolean superstructureDone = false;
    private boolean driveReady = false;
    private boolean hasAlgae = false;
    public boolean readyToRaiseElevator = false;

    private State state = State.TUCK;

    // public Command stowCoralHold() {
    //   return Commands.sequence(
    //           Commands.parallel(
    //               endEffector.setpointCommand(EndEffector.CORAL_HOLD),
    //               MotionPlanner.safePivotAndElevatorToPosition(Pivot.CORAL_HOLD, Elevator.CORAL_HOLD)),
    //           setState(State.HOLD_CORAL))
    //       .withName("Stow Coral Hold");
    // }

    public Command setHasAlgaeCommand(boolean hasAlgae) {
		  return Commands.runOnce(() -> setHasAlgae(hasAlgae));
	  }

    public static enum State {
      TUCK,
      SPIT,
      STATION,
      HOLD_CORAL,
      HOLD_ALGAE,
      L1_CORAL,
      L2_CORAL,
      L3_CORAL,
      L4_CORAL,
      L2_ALGAE,
      L3_ALGAE,
      GULP;
    }
  
    public Command setState(State state) {
      return Commands.runOnce(() -> this.state = state);
    }
  
    public State getState() {
      return state;
    }

    public boolean getHasAlgae() {
      return hasAlgae;
    }

    public void setHasAlgae(boolean has) {
      hasAlgae = has;
    }

    public void setPathFollowing(boolean isFollowing) {
		  isPathFollowing = isFollowing;
	  }

    public void setSuperstructureDone(boolean valToSet) {
		  superstructureDone = valToSet;
	  }

    public void setDriveReady(boolean valToSet) {
		  driveReady = valToSet;
	  }

    public void setReadyToRaiseElevator(boolean valToSet) {
      readyToRaiseElevator = valToSet;
    }

    public boolean getSuperstructureDone() {
      return superstructureDone;
    }



    
}
