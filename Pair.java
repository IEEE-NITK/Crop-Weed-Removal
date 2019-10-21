package Shortestpath;

public class Pair{
	//Stores a and y coordinate of that node.
	int row,column;
	
	public Pair(int x, int y) {
		this.row = x;
		this.column = y;
	}

	@Override
	public String toString() {
		return "Pair [row=" + row + ", column=" + column + "]";
	}
}