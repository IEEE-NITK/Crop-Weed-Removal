# -*- coding: utf-8 -*-
"""
Created on Sun Oct 13 00:43:41 2019

@author: Saikumar Dande
"""

import math
import collections

class Pair:
    def __init__(self, row, column):
        self.row = row
        self.column = column
        
    def __repr__(self):
        return '({}, {})'.format(self.row, self.column)
    

class point:
    def __init__(self, row, column, visited):
        self.row = row
        self.column = column
        self.visited = visited
        
    def __repr__(self):
        return '({}, {})'.format(self.row, self.column)


#This class holds the details of that particular cell in the grid.
#It contails the details of its parents like where it came from.
#g = distance travelled from starting point to reach that cell.
#h = pythogorous distancce from that cell to destination(direct distance from cell to destination)
#f = g+h(This f is expected distance from starting cell to reach destination)
class CellDetails:
    def __init__(self, parent_row, parent_column, g, f, h):
        self.parent_row = parent_row
        self.parent_column = parent_column
        self.g = g
        self.f = f
        self.h = h

    def __repr__(self):
        return '({}, {}, {}, {}, {})'.format(self.parent_row,self.parent_column,self.g, self.f, self.h)


class OpenList:
    def __init__(self, f, x, y):
        self.f = f
        self.x = x
        self.y = y

    def __repr__(self):
        return '({}, {}, {})'.format(self.f, self.x, self.y)


class Astar:
    def __init__(self, Row, Col):
        self.Row = Row
        self.Col = Col

    def isValid(self, row, col):
        return (row >= 0) & (row <self.Row) & (col >= 0) & (col < self.Col)

    def isDestination(self, row, column, destination = Pair):
        if destination.row == row and destination.column == column:
            return True
        return False

    def isUnblocked(self, grid, row, col):
        if grid[row][col] == 1:
            return True
        return False

    def hValue(self, row, col, dest = Pair):
        return math.sqrt((row - dest.row)*(row - dest.row) + (col - dest.column)*(col - dest.column))
    
    #This does the path finding.
    def aStarSearch(self, grid, start = Pair, dest = Pair):

        if not self.isValid(start.row, start.column):
            print("Start point is not valid")
            return

        if not self.isValid(dest.row, dest.column):
            print("Destination is not valid")
            return

        if self.isDestination(start.row, start.column, dest):
            print("Destination is start point itself")
            return

        #This list holds the cells which need to be evaluated.
        openlist = []
        cellDetails = [[CellDetails for i in range(self.Col)] for j in range(self.Row)]

        i = start.row
        j = start.column
        
        cellDetails[i][j] = CellDetails(i, j, 0.0, 0.0, 0.0)

        #This list holds cells which need not be checked.
        closedList = [[False for x in range(self.Col)] for y in range(self.Row)]
        openList = [[False for x in range(self.Col)] for y in range(self.Row)]

        openlist.append(OpenList(0.0, i, j))
        openlist.sort(key = lambda e : e.f, reverse=True)
        openList[i][j] = True

        foundDestination = False
        while openlist:
            current = openlist.pop()
            i = current.x
            j = current.y
            closedList[i][j] = True
            openList[i][j] = False

            srow = [0, 1, 0, -1]
            scol = [-1, 0, 1, 0]
            for w in range(4):
                si = i + srow[w]
                sj = j + scol[w]

                if self.isValid(si, sj):
                    if (not(openList[si][sj] or closedList[si][sj])):
                        cellDetails[si][sj] = CellDetails(0, 0, 0.0, 10000.0, 0.0)

                    if self.isDestination(si, sj, dest):
                        cellDetails[si][sj] = CellDetails(i, j, 0.0, 0.0, 0.0)
                        foundDestination = True
                        print("Destination is found")
                        return self.printPath(cellDetails, dest)
                        return

                    elif closedList[si][sj] == False and self.isUnblocked(grid, si, sj) == True:
                        gnew  = cellDetails[si][sj].g + 1.0
                        hnew = self.hValue(si, sj, dest)
                        fnew = gnew + hnew
                        if fnew < cellDetails[si][sj].f:
                            if not openList[si][sj]:
                                openlist.append(OpenList(fnew, si, sj))
                                openList[si][sj] = True

                            for z in openlist:
                                if z.x == si and z.y == sj:
                                    z.f = fnew
                            cellDetails[si][sj] = CellDetails(i, j, gnew, fnew,hnew)

            openlist.sort(key = lambda q : q.f, reverse=True)

        if not foundDestination:
            print("Destination not found")

    def printPath(self, cellDetails, dest = Pair):
        row = dest.row
        col = dest.column
        path = []
        while not (cellDetails[row][col].parent_row == row and cellDetails[row][col].parent_column == col):
            path.append(Pair(row, col))
            temprow = cellDetails[row][col].parent_row
            tempcol = cellDetails[row][col].parent_column
            row = temprow
            col = tempcol
        path.append(Pair(row, col))
        return path
        #while path:
         #   p = path.pop()
          #  print(p.row, p.column)


            