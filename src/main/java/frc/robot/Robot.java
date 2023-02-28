/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private PS4Controller m_stick;
  private static final int deviceID = 1;
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private double voltage;
  private double current;
  private static double last_setvolts;
  private static double filt_current_fast;
  private static double filt_current_slow;
  private static double fast_filtk = 0.05;
  private static double slow_filtk = 0.005;
  private static double diff_weight = 2.0;
  private static int loop_count_on = 0;
  private static int motor_start_loops = 20;

  @Override
  public void robotInit() {
    // initialize SPARK MAX
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();
    m_motor.setSmartCurrentLimit(10);

    /**
    * In order to read encoder values an encoder object is created using the 
    * getEncoder() method from an existing CANSparkMax object
    */
    m_encoder = m_motor.getEncoder();

    m_stick = new PS4Controller(0);

    SmartDashboard.putBoolean("DIR", true);
    SmartDashboard.putBoolean("ON", false);
    SmartDashboard.putNumber("fast_filtk", fast_filtk);
    SmartDashboard.putNumber("slow_filtk",slow_filtk);
    SmartDashboard.putNumber("diff_weight", diff_weight);
    SmartDashboard.putNumber("motor_start_loops", motor_start_loops);
    last_setvolts = 0.0;
    loop_count_on = 0;
    filt_current_fast = 0.0;
    filt_current_slow = 0.0;
  }

  @Override
  public void teleopPeriodic() {

    boolean direction = SmartDashboard.getBoolean("DIR", false);
    boolean onoff = SmartDashboard.getBoolean("ON", false);
    double setvolts;
 
    boolean stall;
    
    motor_start_loops = (int) SmartDashboard.getNumber("motor_start_loops", motor_start_loops);

    if ( onoff ) {
      if ( loop_count_on <= motor_start_loops ) { // measure time motor is on
        loop_count_on++;
      }
      if ( direction ) {
        setvolts = 1.5;
      } else { 
        setvolts = -1.5;
      }
    } else { 
      loop_count_on = 0;
      setvolts = 0.0; 
    }


    if ( setvolts != last_setvolts ) {
      m_motor.setVoltage(setvolts);
      SmartDashboard.putNumber("setvolts", setvolts);
      if ( setvolts != 0.0 )  { 
        // do something here on motor off->on transition
      }
    }
    last_setvolts = setvolts;

    current = m_motor.getOutputCurrent();

    fast_filtk = SmartDashboard.getNumber("fast_filtk", fast_filtk);
    slow_filtk = SmartDashboard.getNumber("slow_filtk", slow_filtk);

    filt_current_fast = fast_filtk*current + (1.0-fast_filtk)*filt_current_fast;

    filt_current_slow = slow_filtk*current + (1.0-slow_filtk)*filt_current_slow;

    double filt_diff = filt_current_fast - filt_current_slow;

    if ( loop_count_on < motor_start_loops ) {    // motor has just started, ignore stall criteria
      stall = false;
    } else {
      diff_weight = SmartDashboard.getNumber("diff_weight", 0.0);
      if ( (filt_diff>0.0) && (filt_diff > diff_weight*filt_current_slow) ) { // diff > multiple of slow current
        stall = true;
      } else {
        stall = false;
      }
    }


    SmartDashboard.putNumber("Islow", filt_current_slow);
    SmartDashboard.putNumber("Ifast", filt_current_fast);
    SmartDashboard.putNumber("Idiff",filt_diff);
    SmartDashboard.putNumber("loop_count_on", loop_count_on);
    SmartDashboard.putBoolean("stall", stall);
  
    // SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("EncoderVelocity", m_encoder.getVelocity());
  }

  
}
