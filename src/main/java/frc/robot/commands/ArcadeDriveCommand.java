package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AccelerationLimiter;


public class ArcadeDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;

    public AccelerationLimiter accel = new AccelerationLimiter(2, 1);


    public ArcadeDriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ArcadeDriveCmd started!");
    }

    @Override
    public void execute() {

        double speedMult = 1;
        double realTimeSpeed = speedFunction.get();
        double realTimeTurn = .95 * turnFunction.get();

        double AccelDecelSpeed = accel.calculate(realTimeSpeed);

        double left = AccelDecelSpeed - realTimeTurn;
        double right = AccelDecelSpeed + realTimeTurn;
        driveSubsystem.setMotors(left * speedMult, right * speedMult);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ArcadeDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
