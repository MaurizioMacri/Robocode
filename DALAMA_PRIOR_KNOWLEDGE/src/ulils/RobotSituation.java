package ulils;

import dalama.Constants;

public class RobotSituation {

	private long time;
	private double firedGuessFactor;
	private double rightGessFsctor;

	private double distance;
	private double bulletFlightTime;
	private double lateralVelocity;
	private double advancingVelocity;
	private double lateralAcceleration;
	private double advancingAcceleration;
	private double velocity;

	private double power;

	
	public RobotSituation(RobotSituation ToBecloned) {
		this.time =ToBecloned.time;
		this.firedGuessFactor =ToBecloned.firedGuessFactor;
		this.rightGessFsctor =ToBecloned.rightGessFsctor;
		this.distance =ToBecloned.distance;
		this.bulletFlightTime =ToBecloned.bulletFlightTime;
		this.lateralVelocity =ToBecloned.lateralVelocity;
		this.advancingVelocity =ToBecloned.advancingVelocity;
		this.lateralAcceleration =ToBecloned.lateralAcceleration;
		this.advancingAcceleration =ToBecloned.advancingAcceleration;
		this.velocity =ToBecloned.velocity;
		this.power =ToBecloned.power;
	}

public RobotSituation() {
	// TODO Auto-generated constructor stub
}
	


	public double[] toKD_Key(){
		double [] key= new double[Constants.dimensions];
		key[0]=distance;
		key[1]=lateralVelocity;
		key[2]=advancingVelocity;
		key[3]=lateralAcceleration;
		key[4]=advancingAcceleration;
		key[5]=bulletFlightTime;
		return key;
	}

	public void printRobotSituation() {
		System.out.println("--ENEMY SITUATION-- : "+time);
		System.out.println("\t Distance           : "+distance);
		System.out.println("\t Bullet Flight time : "+bulletFlightTime);
		System.out.println("\t Lateral Velocity      : "+lateralVelocity);
		System.out.println("\t Advancing Velocity    : "+advancingVelocity);
		System.out.println("\t Lateral Acc      : "+lateralAcceleration);
		System.out.println("\t Advancing Acc    : "+advancingAcceleration);
	}

	public double getPower() {
		return power;
	}

	public void setPower(double power) {
		this.power = power;
	}

	public double getBulletFlightTime() {
		return bulletFlightTime;
	}

	public double getVelocity() {
		return velocity;
	}

	public long getTime() {
		return time;
	}

	public void setTime(long time) {
		this.time = time;
	}

	public double getFiredGuessFactor() {
		return firedGuessFactor;
	}

	public void setFiredGuessFactor(double firedGuessFactor) {
		this.firedGuessFactor = firedGuessFactor;
	}

	public double getRightGessFsctor() {
		return rightGessFsctor;
	}

	public void setRightGessFsctor(double rightGessFsctor) {
		this.rightGessFsctor = rightGessFsctor;
	}

	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}

	public double getLateralVelocity() {
		return lateralVelocity;
	}

	public void setLateralVelocity(double lateralVelocity) {
		this.lateralVelocity = lateralVelocity;
	}

	public double getAdvancingVelocity() {
		return advancingVelocity;
	}

	public void setAdvancingVelocity(double advancingVelocity) {
		this.advancingVelocity = advancingVelocity;
	}

	public double getLateralAcceleration() {
		return lateralAcceleration;
	}

	public void setLateralAcceleration(double lateralAcceleration) {
		this.lateralAcceleration = lateralAcceleration;
	}

	public double getAdvancingAcceleration() {
		return advancingAcceleration;
	}

	public void setAdvancingAcceleration(double advancingAcceleration) {
		this.advancingAcceleration = advancingAcceleration;
	}

	public void setBulletFlightTime(double bulletFlightTime) {
		this.bulletFlightTime = bulletFlightTime;
	}

	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	
}
