package Shortestpath;

public class OpenList implements Comparable<OpenList> {
	double f;
	int x,y;
	
	@Override
	public String toString() {;
		return "{" +f+" ," +x +" ,"+y+"}";
	}
	
	public OpenList(double f, int x, int y) {
		this.f = f;
		this.x = x;
		this.y = y;
	}
	
	public double getF() {
		return f;
	}
	public void setF(double f) {
		this.f = f;
	}
	public int getX() {
		return x;
	}
	public void setX(int x) {
		this.x = x;
	}
	public int getY() {
		return y;
	}
	public void setY(int y) {
		this.y = y;
	}
	
	@Override
	public int compareTo(OpenList o) {
		if (this.getF() > o.getF()) {
			return 1;
		}
		else if(this.getF() < o.getF()){
			return -1;
		}
		else {
			return 0;
		}
	}
}