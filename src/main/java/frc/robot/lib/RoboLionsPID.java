package frc.robot.lib;

public class RoboLionsPID {
  //PID Template Code
  //23MARCH2019 Dustin Krack and Ava Byrd
  
  public double proportionalGain = 0;
  public double integralGain = 0;
  public double derivativeGain = 0;
  public double integral_charge = 0;

  //errors outside of the zone result of the cage
  //do not allow the integrator to run
  public double upperCageLimit = 0;
  public double lowerCageLimit = 0;

  public double upperDeadbandLimit = 0;
  public double lowerDeadbandLimit = 0;

  public double maxOutput = 0;
  public double minOutput = 0;

  public double output = 0;
  //public double input = 0;
  public double error = 0;

  public double derivativeCalculation = 0;

  public double previousError = 0;

  public double deltaTime = 0.02;

  public double cmd = 0;
  public double feed = 0;

  public boolean deadband_active = false;

  // These are used to turn off the limiter cage and the deadband 2/15/20
  public boolean enableCage = true;
  public boolean enableDeadBand = true;

  public static int deadband_counter = 0;

  // Called just before this Command runs the first time to set your values
  public void initialize(double _P,double _I, double _D,
                         double Cage_Limit,double Deadband,double MaxOutput) {
    proportionalGain = _P;
    integralGain = _I;
    derivativeGain = _D;
    
    upperCageLimit = +Cage_Limit;
    lowerCageLimit = -Cage_Limit;
  
    upperDeadbandLimit = +Deadband;
    lowerDeadbandLimit = -Deadband;
  
    maxOutput = +MaxOutput;
    minOutput = -MaxOutput;
  
    output = 0;
    //input = 0;
    error = 0;
  
    derivativeCalculation = 0;
    integral_charge = 0;
    previousError = 0;
  
    deltaTime = 0.02;

    deadband_active = false; // TODO Change ARM PID to Initialize2 so that we can set deadband

    enableCage = true;
    enableDeadBand = true;
    
  }

  public void initialize2(double _P,double _I, double _D,
                  double Cage_Limit,double Deadband,double MaxOutput, boolean enable_Cage, boolean enable_DeadBand) { 
    // We are implementing this function to turn off the cage function and the dead band                
    initialize(_P, _I, _D, Cage_Limit, Deadband, MaxOutput);

    enableCage = enable_Cage;
    enableDeadBand = enable_DeadBand;


  }
  // Called repeatedly when this Command is scheduled to run
  // This is the function that takes in what you told the motors to do and 
  // what the motors actually did to create what we know as PID control
  public double execute(double command, double feedback) {
    cmd = command;
    feed = feedback;

    //1. calculate the error
    error = command - feedback;

    // 2. calculate change of error
    derivativeCalculation = (error - previousError) / deltaTime;
    
    // 3. Latch internal error's state for next time
    previousError = error; 

    // 4. If in deadband, don't integrate the error
    if (enableDeadBand == true) {
      if((error < upperDeadbandLimit) && (error > lowerDeadbandLimit)) {
        deadband_counter++;
        if(deadband_counter > 13) { // 13 = 50 / 4 = 50 ms split into 4 quarters
          deadband_active = true;
          deadband_counter = 13;
        }
          // integral_charge = integral_charge + error*deltaTime;
      }
    }
    
    // 5. If in Zone of Contrability, overcome any frictional forces
    if (enableCage == true) {
      if((error < upperCageLimit) && (error > lowerCageLimit)){
        //integrate if error is small and we are close to our command
        deadband_counter = 0;
        integral_charge = integral_charge + error*deltaTime;
        deadband_active = false; //true
      }
    }

    else {
      integral_charge = integral_charge + error*deltaTime;
    }

    // 6. If in Zone of Saturation, you can't move any faster, so scale off the throttle 
    /* else {
       integral_charge = 0;
       deadband_counter = 0;
       deadband_active = false; //true
       //integrate if error is small and we are close to our command
    }
    */

    // 7. Calculate PID Control Equation
    output = error*proportionalGain + integral_charge*integralGain - derivativeCalculation*derivativeGain;
    if(output > maxOutput) {
      output = maxOutput;
    } else if(output < minOutput) {
      output = minOutput;
    }
    return output;
  }

  // for use if in standby and not running any power
  public void reset() {
    output = 0;
    //input = 0;
    error = 0;
  
    derivativeCalculation = 0;
    integral_charge = 0;
    previousError = 0;

    deadband_active = false;
  }
}