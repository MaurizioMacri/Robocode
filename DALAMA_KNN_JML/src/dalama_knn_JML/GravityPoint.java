package dalama_knn_JML;

/**Holds the x, y, and strength info of a gravity point**/
class GravityPoint {
    private double x,y,power;
    
    public GravityPoint(double pX,double pY,double pPower) {
        x = pX;
        y = pY;
        power = pPower;
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
	public double getPower() {
		return power;
	}
	public void setPower(double power) {
		this.power = power;
	}
    
    
}