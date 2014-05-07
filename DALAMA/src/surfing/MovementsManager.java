package surfing;

import java.util.Hashtable;

import dalama.Constants;

import kdtree.KdTree;

import ulils.RobotSituation;

public class MovementsManager {
	
	public KdTree<Double> history;
	
	public MovementsManager() {
		history = new KdTree<Double>(Constants.dimensions);
	}
	
	
	public void addSituation(RobotSituation newSituation,double GF){
		history.addPoint(newSituation.toKD_Key(), GF);
	}
	
	
	
	

}
