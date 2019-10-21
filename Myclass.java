package Shortestpath;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;

public class Myclass {

	public static void main(String[] args) {
		Astar a = new Astar(6,6);
		int[][] grid = new int[][] {
			{1, 1, 1, 0, 1, 1},
		    {1, 0, 1, 0, 1, 1},
		    {1, 0, 1, 1, 1, 1},
		    {1, 1, 1, 0, 1, 1},
		    {1, 0, 1, 0, 1, 1},
		    {1, 1, 1, 0, 1, 1},
		};
		/*
		 * 0 for blocked path
		 * 1 for not blocked
		 */
		
		Pair start = new Pair(0, 0);
		ArrayList<OpenList> destination = new ArrayList<OpenList>();
		destination.add(new OpenList(0.0, 3, 1));
		destination.add(new OpenList(0.0, 5, 0));
		destination.add(new OpenList(0.0, 2, 5));
		long starttime = System.currentTimeMillis();
		a.aStarSearch(grid, start, destination);
		long finishtime = System.currentTimeMillis();
		System.out.println(finishtime - starttime);
	}
}