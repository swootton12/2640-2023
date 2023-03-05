package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
public class elevatorsubsystem extends SubsystemBase {

    private static CANSparkMax sped1 = new CANSparkMax(8, MotorType.kBrushless);
    private static CANSparkMax sped2 = new CANSparkMax(9, MotorType.kBrushless);

    Joystick subsystem = new Joystick(1);
    JoystickButton up = new JoystickButton(subsystem, 6);
    JoystickButton down = new JoystickButton(subsystem, 4);
    MotorControllerGroup sped = new MotorControllerGroup(sped1, sped2);
    public void periodic(){                     
    up.whileTrue(sped.set(1));
    up.whileFalse(sped.set(0));
    down.whileTrue(sped.set(-1));
    down.whileFalse(sped.set(0));
    }

}