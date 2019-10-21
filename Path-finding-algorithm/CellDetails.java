package Shortestpath;

public class CellDetails {
	//This cells parent x and y coordinates.
	int parent_row,parent_column;
	
	/*g = distance from starting point in the traversed route.
	 * h = heuristic distance of this cell from destination.
	 * f = g + h (We use this f to estimate how far the destination is from this cell and move according to that)
	 */
	double g;
	double f,h;
}
