package dalama;

import java.awt.geom.Point2D;

class Enemy {
	/*
	 * ok, we should really be using accessors and mutators here,
	 * (i.e getName() and setName()) but life's too short.
	 */
	private  String name;
	private double bearing,heading,speed,x,y,distance,changehead;
	private long ctime; 		//game time that the scan was produced
	private boolean live; 	//is the enemy alive?
	
	
	//Given velocity, initial velocity and time calculate the acceleration.
	//a = (v - v0)/t
	public static double returnAcceleration(double oldVelocity, double currVelocity, double deltaTime){
		return (currVelocity-oldVelocity)/deltaTime;
	}
	
	
	

	
	

	public String getName() {
		return name;
	}
	public Point2D.Double guessPosition(long when) {
		double diff = when - ctime;
		double newY = y + Math.cos(heading) * speed * diff;
		double newX = x + Math.sin(heading) * speed * diff;
		
		return new Point2D.Double(newX, newY);
	}

	public void setName(String name) {
		this.name = name;
	}

	public double getBearing() {
		return bearing;
	}

	public void setBearing(double bearing) {
		this.bearing = bearing;
	}

	public double getHeading() {
		return heading;
	}

	public void setHeading(double heading) {
		this.heading = heading;
	}

	public double getSpeed() {
		return speed;
	}

	public void setSpeed(double speed) {
		this.speed = speed;
	}

	public double getX() {
		return x;
	}

	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	public void setY(double y) {
		this.y = y;
	}

	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}

	public double getChangehead() {
		return changehead;
	}

	public void setChangehead(double changehead) {
		this.changehead = changehead;
	}

	public long getCtime() {
		return ctime;
	}

	public void setCtime(long ctime) {
		this.ctime = ctime;
	}

	public boolean isLive() {
		return live;
	}

	public void setLive(boolean live) {
		this.live = live;
	}
	
	
}