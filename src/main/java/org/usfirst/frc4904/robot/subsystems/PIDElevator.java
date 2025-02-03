package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.security.spec.EncodedKeySpec;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.Utils;
import org.usfirst.frc4904.standard.commands.CreateAndDisown;
import org.usfirst.frc4904.standard.commands.WaitUntil;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

public class PIDElevator extends SubsystemBase {

    // TODO: get actual values
    // public static final double kS = 0.00;
    // public static final double kV = 1.3716;
    // public static final double kA = 0.0299;
    // public static final double kG = 0.4126;
    public static final double kS = 0.00;
    public static final double kV = 1.4555;
    public static final double kA = 0.0513;
    public static final double kG = 0.235;
    // public static finl double kG = .5;

    public static final double kP = 0.07;
    public static final double kI = 0.03;
    public static final double kD = 0;

    private static final double OUTTAKE_ANGLE = 90;
    private static final double INTAKE_ANGLE = 0;

    //TODO: this number has been wrong, make sure it is correct before comps
    private static final double ARM_OFFSET = 181;

    public final CANTalonFX ElevatorMotor;
    public final ArmFeedforward feedforward;
    public final Encoder ElevatorEncoder;

    // possible helpful https://www.chiefdelphi.com/t/using-encoder-to-drive-a-certain-distance/147219/2
    public PIDElevator(CANTalonFX ElevatorMotor, Encoder ElevatorEncoder) {
        this.ElevatorMotor = ElevatorMotor;
        this.ElevatorMotor.setBrakeOnNeutral();
        this.feedforward = new ArmFeedforward(kG, kS, kV, kA);
        this.ElevatorEncoder = ElevatorEncoder;
        ElevatorEncoder.setDistancePerPulse(5);
    }

    // public double getCurrentAngleDegrees() {
    //     var thing = armEncoder.getAbsolutePosition() * 360 - ARM_OFFSET;
    //     if (thing < 0) {
    //         return thing + 360;
    //     }
    //     return thing;
    // }
    //Type of encoder we're using: Rev Through Bore
    public double GetDistance() {
        ElevatorEncoder.getDistance();
    }

    public Command c_controlAngularVelocity(DoubleSupplier degPerSecDealer) {
        var cmd =
            this.run(() -> {
                    var ff =
                        this.feedforward.calculate(
                                Units.degreesToRadians(getCurrentAngleDegrees()),
                                Units.degreesToRadians(degPerSecDealer.getAsDouble())
                            );
                    SmartDashboard.putNumber("feedforward", ff);
                    this.armMotor.setVoltage(ff);
                });
        cmd.setName("arm - c_controlAngularVelocity");

        return cmd;
    }

    public Command c_holdIntakeAngle(
        double maxVelDegPerSec,
        double maxAccelDegPerSecSquare,
        Supplier<Command> onArrivalCommandDealer
    ) {
        return c_holdRotation(
            INTAKE_ANGLE,
            maxVelDegPerSec,
            maxAccelDegPerSecSquare,
            onArrivalCommandDealer
        );
    }

    public Command scuffed() {
        var cmd = new InstantCommand(() -> System.out.println("arm working")).andThen(
            new ParallelRaceGroup(
                new RunCommand(() -> armMotor.setVoltage(5.5)),
                new WaitUntilCommand(
                    (() -> (getCurrentAngleDegrees() > 88 && getCurrentAngleDegrees() < 180))
                )
            ).andThen(new InstantCommand(() -> armMotor.setVoltage(0)))
        );
        cmd.addRequirements(this);
        return cmd;
    }

    //  return new WaitCommand(3)
    //  .andThen(c_holdRotation(OUTTAKE_ANGLE, maxVelDegPerSec, maxAccelDegPerSecSquare, onArrivalCommandDealer))
    //  .andThen(new WaitCommand(3))
    //  .andThen(c_holdRotation(OUTTAKE_ANGLE, maxVelDegPerSec, maxAccelDegPerSecSquare, onArrivalCommandDealer));
    //  return c_holdRotation(OUTTAKE_ANGLE, maxVelDegPerSec, maxAccelDegPerSecSquare, () -> c_holdRotation(OUTTAKE_ANGLE, maxVelDegPerSec, maxAccelDegPerSecSquare, onArrivalCommandDealer));
    public Command scuffedback() {
        return new ParallelRaceGroup(
            (new RunCommand(() -> armMotor.setVoltage(-3))),
            new WaitUntilCommand((() -> getCurrentAngleDegrees() < 50))
        ).andThen(new InstantCommand(() -> armMotor.setVoltage(0)));
    }

    public Command c_holdOuttakeAngle(
        double maxVelDegPerSec,
        double maxAccelDegPerSecSquare,
        Supplier<Command> onArrivalCommandDealer
    ) {
        return c_holdRotation(
            OUTTAKE_ANGLE,
            maxVelDegPerSec,
            maxAccelDegPerSecSquare,
            onArrivalCommandDealer
        );
    }

    public Command c_holdRotation(
        double degreesFromHorizontal,
        double maxVelDegPerSec,
        double maxAccelDegPerSecSquare,
        Supplier<Command> onArrivalCommandDealer
    ) {
        ezControl controller = new ezControl(kP, kI, kD, (position, velocityDegPerSec) -> {
            double ff =
                this.feedforward.calculate(
                        Units.degreesToRadians(getCurrentAngleDegrees()),
                        Units.degreesToRadians(velocityDegPerSec)
                    );
            SmartDashboard.putNumber("Intended voltage", maxAccelDegPerSecSquare);
            return ff;
        });

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelDegPerSec, maxAccelDegPerSecSquare)
        );

        var cmd = getEzMotion(
            controller,
            profile,
            new TrapezoidProfile.State(degreesFromHorizontal, 0),
            new TrapezoidProfile.State(getCurrentAngleDegrees(), 0)
        );

        return onArrivalCommandDealer == null
            ? cmd
            : Utils.nameCommand(
                "pivot w/ onArrival: " + cmd.getName(),
                new ParallelCommandGroup(
                    cmd,
                    new SequentialCommandGroup(
                        new WaitCommand(profile.totalTime()),
                        new CreateAndDisown("arm pivot", onArrivalCommandDealer)
                    )
                )
            );
    }

    private ezMotion getEzMotion(
        ezControl controller,
        TrapezoidProfile profile,
        TrapezoidProfile.State current,
        TrapezoidProfile.State goal
    ) {
        var cmd = new ezMotion(
            controller,
            this::getCurrentAngleDegrees,
            (double volts) -> {
                SmartDashboard.putNumber("Arm Volts", volts);
                this.armMotor.setVoltage(volts);
            },
            (double t) -> {
                TrapezoidProfile.State result = profile.calculate(t, current, goal);

                SmartDashboard.putNumber("deg setpoint", result.velocity);
                return new Pair<>(result.position, result.velocity);
            },
            this
        );
        cmd.setName("arm - c_holdRotation");

        return cmd;
    }
}
