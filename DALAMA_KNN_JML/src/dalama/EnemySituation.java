package dalama;

public class EnemySituation {
	private boolean goodCategorized=false;

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


	public double[] toKD_Key(){
		double [] key= new double[Constants.dimensions];
		key[0]=distance;
		key[1]=lateralVelocity;
		key[2]=advancingVelocity;
		return key;
	}

	public void printEnemySituation() {
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

	public double getMaxEscapeAngle(){
		return Math.asin(8/(WaveBullet.getBulletSpeed(power)));
	}

	public double getEuclideanDistance(EnemySituation s){
		double Eudistance =
				Math.sqrt(Math.pow(distance-s.getDistance(), 2)
						+Math.pow(lateralVelocity-s.getLateralVelocity(), 2)
						+ Math.pow(advancingVelocity-s.getAdvancingVelocity(), 2)
						//						+ Math.pow(lateralAcceleration-s.getLateralAcceleration(), 2)
						//						+Math.pow(advancingAcceleration-s.advancingAcceleration, 2)
						);
		return Eudistance;
	}

	public double getManhattanDistance(EnemySituation s){
		double Eudistance =
				Math.abs(distance-s.getDistance())
				+Math.abs(lateralVelocity-s.getLateralVelocity()) 
				+ Math.abs(advancingVelocity-s.getAdvancingVelocity())
				+ Math.abs(lateralAcceleration-s.getLateralAcceleration())
				+Math.abs(bulletFlightTime-s.bulletFlightTime)
				+Math.abs(lateralAcceleration-s.lateralAcceleration)
				+Math.abs(advancingAcceleration-s.advancingAcceleration);
		return Eudistance;
	}


	public boolean isGoodCategorized() {
		return goodCategorized;
	}

	public void setGoodCategorized(boolean goodCategorized) {
		this.goodCategorized = goodCategorized;
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
		goodCategorized=true;
	}
	public double getDistance() {
		return distance;
	}
	public void setDistance(double distance) {
		this.distance = distance;
	}
	public double getBulltFlightTime() {
		return bulletFlightTime;
	}
	public void setBulletFlightTime(double bulletFlightTime) {
		this.bulletFlightTime = bulletFlightTime;
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
	public void setVelocity(double velocity) {
		this.velocity=velocity;

	}
	public long getTime() {
		return time;
	}
	public void setTime(long time) {
		this.time = time;
	}


	//	double distanceRightWall;
	//	double distanceLeftWall;
	//	double distancetTopWall;
	//	double distanceBottomWall;







}
