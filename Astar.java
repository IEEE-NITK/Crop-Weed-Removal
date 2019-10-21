package Shortestpath;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.Stack;

public class Astar {
	private int Row, Col;
	private int grid[][];
	ArrayList<OpenList> Dest = new ArrayList<>();
	
	Astar(int Row, int Col){
		this.Row = Row;
		this.Col = Col;
	}
	
	boolean isValid(int row, int col) {
		return (row>=0) && (row<Row) && (col>=0) && (col<Col);
	}
	
	//Returns true if destination is reached.
	boolean isDestination(int row, int column, ArrayList<OpenList> dest) {
		for (int i = 0; i < dest.size(); i++) {
			if (dest.get(i).x == row && dest.get(i).y == column) 
				return true;
		}
		return false;
	}
	//to check whether we have reached destination or not
	boolean isDestination(int row, int column, OpenList particulardest) {
		if (particulardest.x == row && particulardest.y == column) 
			return true;
		return false;
	}
	
	//Returns true if cell is not blocked.(0 for cell blocked and 1 for not blocked).
	boolean isUnBlocked(int grid[][], int row, int col) {
		if(grid[row][col] == 1)
			return true;
		return false;
	}
	
	//Calculate H value
	double hValue(int row, int col, OpenList particulardest) {
		return (double)Math.sqrt((row - particulardest.x)*(row - particulardest.x) + (col - particulardest.y)*(col - particulardest.y));
	}
	
	//Main function which does A star search.
	void aStarSearch(int grid[][], Pair start, ArrayList<OpenList> dest) {
		this.grid = grid;

		this.Dest = dest;
		calculateDistance(start, Dest);
		Collections.sort(Dest);
		//If given start point is valid or not.
		if(!isValid(start.row, start.column)) {
			System.out.println("Start point is not valid");
			return;
		}
		
		//If given destination point is valid or not.		
		for (int i = 0; i < Dest.size() ; i++) {
			if(!isValid(Dest.get(i).x, Dest.get(i).y)) {
				System.out.println("One of the destination point is not valid");
				//I can just discard that invalid destination and find for remaining.
				return;
			}
		}
		
		//Whether start or destination is blocked or not.
		if (!isUnBlocked(grid, start.row, start.column)) {
			System.out.println("Start point is blocked");
			return;
		}
		
		/*
		 * Setting up open list, closed list and cell details.
		 * cell details stores all the details of cell like its parent node, distance traveled from starting point
		 * closed list contains all the points which we don't need to check.
		 * open list contains points which need to be explored on f(Estimated distance from starting point to destination) value. 
		 */
		
		ArrayList<LinkedList<OpenList>> openlist = new ArrayList<>();
		for (int i=0; i<Dest.size();i++) {
			openlist.add(new LinkedList<OpenList>());
		}
		CellDetails[][][] cellDetails = new CellDetails[Dest.size()][Row][Col];
		
		//Initializing parameters to first node.
		int i = start.row;
		int j = start.column;
		for (int destloc = 0; destloc < Dest.size(); destloc++) {
			cellDetails[destloc][i][j] = new CellDetails();
			
			cellDetails[destloc][i][j].parent_row = i;
			cellDetails[destloc][i][j].parent_column = j;
			cellDetails[destloc][i][j].f = 0.0;
			cellDetails[destloc][i][j].g = 0.0;
			cellDetails[destloc][i][j].h = 0.0;
		}
		
		//Initializing closed list array. Assigns true if it is in closed list.
		boolean[][][] closedList = new boolean[Dest.size()][Row][Col];
		boolean[][][] openList = new boolean[Dest.size()][Row][Col];
		
		for (int q = 0; q < Dest.size(); q++) {
			openlist.get(q).add(new OpenList(0.0, i, j));
			openList[q][i][j] = true;
		}
		
		boolean foundDestination = false;
		//Checking neighbors for elements in openList
		while(!openlistisempty(openlist)) {
			double gnew,hnew,fnew;
			int si,sj;     //Coordinates of successor.
			
			double min = Double.MAX_VALUE;
			int destaddress = 0;
			for (int r=0; r< openlist.size(); r++) {
				if (!openlist.get(r).isEmpty()) {
				double temp = openlist.get(r).getFirst().f;
				if (temp < min) {
					min = temp;
					destaddress = r;
				}
				}
			}
			
			OpenList current = openlist.get(destaddress).poll();
			i = current.x;
			j = current.y;
			OpenList destination = Dest.get(destaddress);
			closedList[destaddress][i][j] = true;
			openList[destaddress][i][j] = false;
			
			//This srow and scol make the path to move in four directions only.
			int[] srow = new int[] {-1, 0, 1, 0};
			int[] scol = new int[]{0, -1, 0 , 1};
			
			/*
			 * Now we will be checking 4 successors of the current vertex.
			 * We can also check 8 successors of the current vertex. If we
			 * do that then we will get a diagonal path as well.
			 * Here we are doing only four directions (N,S,E,W).
			 */
			for (int w = 0; w < 4; w++) {
				si = i + srow[w];
				sj = j + scol[w];
				if (isValid(si, sj)) {
					if (!(openList[destaddress][si][sj] || closedList[destaddress][si][sj])) {
						cellDetails[destaddress][si][sj] = new CellDetails();
						cellDetails[destaddress][si][sj].f = Integer.MAX_VALUE;
					}
					
					//Add one more if statement for which it can be any destination.
					if (isDestination(si, sj, Dest.get(destaddress))) {
						cellDetails[destaddress][si][sj].parent_row = i;
						cellDetails[destaddress][si][sj].parent_column = j;
						foundDestination = true;
						System.out.println("Destination is found");
						CellDetails[][] cell = new CellDetails[Row][Col];
						cell = cellDetails[destaddress];
						printPath(cell, destination, destaddress);
						return;
					}
					else if(closedList[destaddress][si][sj] == false && isUnBlocked(grid, si, sj) == true) {
						gnew = cellDetails[destaddress][i][j].g + 1.0;
						hnew = hValue(si, sj, destination);
						fnew = gnew + hnew;
						if (fnew < cellDetails[destaddress][si][sj].f) {
							if (!openList[destaddress][si][sj]) {
								openlist.get(destaddress).add(new OpenList(fnew, si, sj));
								openList[destaddress][si][sj] = true;
							}
							for (OpenList z : openlist.get(destaddress)) {
								if(z.x == si && z.y == sj) {
									z.f = fnew;
								}
							}
							cellDetails[destaddress][si][sj].f = fnew;
							cellDetails[destaddress][si][sj].g = gnew;
							cellDetails[destaddress][si][sj].h = hnew;
							cellDetails[destaddress][si][sj].parent_row = i;
							cellDetails[destaddress][si][sj].parent_column = j;
						}
					}
				}
			}
			for (int e=0; e< openlist.size(); e++) {
				Collections.sort(openlist.get(e));
			}	
		}
		if(!foundDestination) {
			System.out.println("Destination not found");
		}
	}

	
	//This prints the path.
	private void printPath(CellDetails[][] cellDetails, OpenList dest, int destaddress) {
		int row = dest.x;
		int col = dest.y;
		Stack<Pair> stack = new Stack<Pair>();
		while(!(cellDetails[row][col].parent_row == row && cellDetails[row][col].parent_column == col)) {
			stack.push(new Pair(row, col));
			int temprow = cellDetails[row][col].parent_row;
			int tempcol = cellDetails[row][col].parent_column;
			row = temprow;
			col = tempcol;
		}
		stack.push(new Pair(row, col));
		while(!stack.isEmpty()) {
			Pair p = stack.pop();
			System.out.println("(" + p.row + " ," + p.column + ")");
		}
		Pair start = new Pair(dest.x, dest.y);
		Dest.remove(destaddress);
		if (!Dest.isEmpty()) {
			aStarSearch(grid, start, Dest);
		}
	}
	
	//function to check whether open list is empty or not.
	boolean openlistisempty(ArrayList<LinkedList<OpenList>> a) {
		for (int i=0; i < a.size(); i++) {
			if (!a.get(i).isEmpty())
				return false;
		}
		return true;
	}
	
	//Method to calculate the distance of each destination from starting vertex.
	void calculateDistance(Pair start, ArrayList<OpenList> destination) {
		for (int i=0;i<Dest.size();i++) {
			Dest.get(i).f = hValue(start.row, start.column, destination.get(i));
		}
	}
}