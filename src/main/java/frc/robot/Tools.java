package frc.robot;

public class Tools {
	public Tools() {}

	/**
	 * 
	 * @param setSpeed
	 * @param status
	 * @param lowLimit
	 * @param highLimit
	 * @param upRange
	 * @param downRange
	 * @return
	 */
  public double setOnLimit(double setSpeed, double status, double lowLimit, double highLimit, double upRange, double downRange) {
		double range = highLimit - lowLimit;
		upRange = range * upRange;
		downRange = range * downRange;

		if(range < 0) setSpeed = -setSpeed;
		double speed = 
		    highLimit - status < upRange && setSpeed > 0 ? setSpeed * (highLimit - status)/upRange :
		    status - lowLimit < downRange && setSpeed < 0 ? setSpeed * (status - lowLimit)/downRange :
				setSpeed * 1;
		if(range < 0) speed = -speed;
		return speed;
  }

	public double range(double value, double highLimit, double lowLimit) {
		return value > highLimit ? highLimit : 
		    value < lowLimit ? lowLimit : 
			  value;
	}
}
