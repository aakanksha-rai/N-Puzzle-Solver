# Manhattan A*
import os
import sys
import heapq
import copy
import time

class Node(object):
    def __init__(self, state, parent, action = None):
        self.state = state
        self.parent = parent
        self.emptyCellPosition = self.findEmptyCellPosition()
        self.action = action
        self.key = self.getKey()
        self.h = 0
        self.g = 0
        self.f = 0
    
    def __hash__(self):
        return self.key
        
    def __eq__(self, other):
        return self.key == other.key
        
    def __lt__(self, other):
        return self.f < other.f
    
    def getKey(self):
        n = 0
        for rowNum, row in enumerate(self.state):
            for colNum, item in enumerate(row):
                n = n*10 + item
        return n
            
    
    #Finds position of 0 in the node
    def findEmptyCellPosition(self):
        for rowNum, row in enumerate(self.state):
            for colNum, item in enumerate(row):
                if item == 0:
                    return (rowNum, colNum)
      
    #Calculates Manhattan distance of a number
    def calculateManhattanDistance(self, myRow, myCol, goalRow, goalCol):
        return abs(myRow-goalRow) + abs(myCol - goalCol)
    
    #Finds position of number in the goal grid
    def findGoalPosition(self, n, goal):
        size = len(self.state)
        goalRowNum = (n - 1)//size
        goalColNum = (n-1) % size
        return (goalRowNum, goalColNum)
    
    #Calculates heuristic value = total Manhattan distance
    def calculateH(self, goal):
        total = 0
        for rowNum, row in enumerate(self.state):
            for colNum, item in enumerate(row):
                if item == 0:
                    continue
                goalPos = self.findGoalPosition(item, goal)
                total = total + self.calculateManhattanDistance(rowNum, colNum, goalPos[0], goalPos[1])
        return total


class Puzzle(object):
    def __init__(self, init_state, goal_state):
        # you may add more attributes if you think is useful
        self.init_state = init_state
        self.goal_state = goal_state
        self.actions = list()
        self.N = len(init_state)
        self.isSolvable = True
        self.TIME_TAKEN = 0
        self.FRONTIER = 0
        self.TOTALMOVES = 0
        
        
    #Returns row number of 0 in grid
    def findEmptyCellRow(self, state):
           for rowNum, row in enumerate(state):
               for colNum, item in enumerate(row):
                   if item == 0:
                       return rowNum

    #Checks whether the input is solvable
    def canSolve(self, start, goal):
        startList = []
        invertCount = 0
        
        for row in start:
            startList.extend(row)
            
        for i in range(len(startList) - 1):
            for j in range(i+1, len(startList)):
                if not (startList[i] == 0 or startList[j] == 0) and startList[i] > startList[j]:
                    invertCount = invertCount + 1;
                    
        if self.N%2 == 1:
            if invertCount%2 == 0:
                return True
        
        if self.N%2 == 0:
            emptyRow = self.findEmptyCellRow(self.init_state)
            if emptyRow % 2 == 0:
                if invertCount % 2 == 1:
                    return True
            if emptyRow % 2 == 1:
                if invertCount % 2 == 0:
                    return True
                    
        return False
        
    #creates the successor nodes
    def createSuccessorNodes(self, currNode):
        successors = []
        actions = ["RIGHT", "LEFT", "DOWN", "UP"]
        for i, newPos in enumerate([(0, -1), (0, 1), (-1, 0), (1, 0)]):
            newEmptyCellPosition = (currNode.emptyCellPosition[0] + newPos[0], currNode.emptyCellPosition[1] + newPos[1])
            #checking for out of range
            if newEmptyCellPosition[0] > (self.N -1) or newEmptyCellPosition[0] < 0 or newEmptyCellPosition[1] > (self.N -1) or newEmptyCellPosition[1] < 0:
                continue
                
            #make copy
            newState = copy.deepcopy(currNode.state)
            
            #now swap
            newState[currNode.emptyCellPosition[0]][currNode.emptyCellPosition[1]] = newState[newEmptyCellPosition[0]][newEmptyCellPosition[1]]
            
            newState[newEmptyCellPosition[0]][newEmptyCellPosition[1]] = 0
            
            newNode = Node(newState, currNode, actions[i])
            
            successors.append(newNode)
            
        return successors
    
    #implementation of A Star search
    def search(self, start, goal):
        #If the grid is unsolvable:
        self.isSolvable = self.canSolve(self.init_state, self.goal_state)
        if not self.isSolvable:
            return ["UNSOLVABLE"]
            
        #initializing start and goal nodes
        startNode = Node(start, None)
        goalNode = Node(goal, None)
        frontier = [startNode]
        
        #Frontier is a heap (similar priority queue)
        heapq.heapify(frontier)
        explored = set()
        frontierSize = 1
        
        while frontierSize > 0:
            #finding the next node to expand
            nextNode = heapq.heappop(frontier)
            frontierSize = frontierSize - 1
            
            #Checking whether the node has been explored already
            if nextNode in explored:
                continue
            
            #when you reach the goal state, get the path
            if nextNode == goalNode:
                path = []
                curr = nextNode
                while curr.action is not None:
                    path.append(curr.action)
                    curr = curr.parent
                self.actions = path[::-1]
                self.FRONTIER = frontierSize
                return self.actions
                
            #when you have not reached the goal state
            explored.add(nextNode)
            successors = self.createSuccessorNodes(nextNode)
            frontierSize = frontierSize + len(successors)
            
            for successor in successors:
                successor.g = nextNode.g + 1
                successor.h = successor.calculateH(goalNode)
                successor.f = successor.g + successor.h
                heapq.heappush(frontier, successor)
        
        #if it can't find search values
        self.isSolvable = False
        return ["UNSOLVABLE"]
    
    def solve(self):
        # my search algorithm has been implemented in the search function
        StartTime = time.time()
        a = self.search(self.init_state, self.goal_state)
        EndTime = time.time()
        self.TIME_TAKEN = EndTime - StartTime
        self.TOTALMOVES = len(a)
        print(self.TIME_TAKEN)
        return a


if __name__ == "__main__":
    # do NOT modify below

    # argv[0] represents the name of the file that is being executed
    # argv[1] represents name of input file
    # argv[2] represents name of destination output file
    if len(sys.argv) != 3:
        raise ValueError("Wrong number of arguments!")

    try:
        f = open(sys.argv[1], 'r')
    except IOError:
        raise IOError("Input file not found!")

    lines = f.readlines()
    
    # n = num rows in input file
    n = len(lines)
    # max_num = n to the power of 2 - 1
    max_num = n ** 2 - 1

    # Instantiate a 2D list of size n x n
    init_state = [[0 for i in range(n)] for j in range(n)]
    goal_state = [[0 for i in range(n)] for j in range(n)]
    

    i,j = 0, 0
    for line in lines:
        for number in line.split(" "):
            if number == '':
                continue
            value = int(number , base = 10)
            if  0 <= value <= max_num:
                init_state[i][j] = value
                j += 1
                if j == n:
                    i += 1
                    j = 0

    for i in range(1, max_num + 1):
        goal_state[(i-1)//n][(i-1)%n] = i
    goal_state[n - 1][n - 1] = 0

    puzzle = Puzzle(init_state, goal_state)
    ans = puzzle.solve()
    print(ans)

    with open(sys.argv[2], 'a') as f:
        for answer in ans:
            f.write(answer+'\n')

