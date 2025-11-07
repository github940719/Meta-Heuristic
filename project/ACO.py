import numpy as np

"""
allSol is a list of sol (of all ants in an iteration)
a sol is a list of routes
a route is a list of nodes = [0, customer1, customer2, ..., 0]
"""

class ACO:

    def setAlgorithmParameter(self, m = 20, alpha = 2, beta = 1, gamma = 3, delta = 0.5, rho = 0.85, Q = 1000, r0 = 0.5, maxIter = 200):
        self.m = m           # number of ants
        self.alpha = alpha   # pheromone factor
        self.beta = beta     # heuristic factor for (1 / dist)
        self.gamma = gamma   # heuristic factor for (1 / width of time window)
        self.delta = delta   # heuristic factor for (1 / waiting time)
        self.rho = rho       # pheromone evaporation rate
        self.Q = Q           # pheromone intensity (in equation 20)
        self.r0 = r0         # exploitation threshold (in equation 15)
        self.maxIter = maxIter   # maximum number of iterations

        if not hasattr(self, "customerCnt"):
            raise AttributeError("run setProblemParameter() before setAlgorithmParameter()")
        
        self.pheromonematrix = np.ones((self.customerCnt + 1, self.customerCnt + 1))
        # please help me find the initial value of pheromone in the paper


    def setProblemParameter(self, datasetFilePath, gd = 60, gt = 5, objSigma = 0.6):
        with open(datasetFilePath, "r") as f:
            lines = f.readlines()

        # parse vehicle capacity and customer count from header
        vehInfo = lines[4].split()
        self.customerCnt = int(vehInfo[0])  # a scalar, number of customers (excluding the depot)
        self.vehCapacity = int(vehInfo[2])  # a scalar, vehicle capacity

        # initialize 
        coordsList = []
        self.deliveryDemand = []   # a list, length = customerCnt + 1, deliveryDemand[0] = 0 represents the depot
        self.pickupDemand = []     # a list, length = customerCnt + 1, pickupDemand[0] = 0 represents the depot
        self.serviceTime = []      # a list, length = customerCnt + 1, serviceTime[0] = 0 represents the depot
        self.timeWindow = []       # a list, length = customerCnt + 1, each element is a list of size 2 [lower bound, upper bound]

        # read customer data line by line
        for line in lines[9:10 + self.customerCnt]:
            info = [int(p) for p in line.split()]
            coordsList.append([info[1], info[2]])
            self.deliveryDemand.append(info[3])
            self.pickupDemand.append(info[4])
            self.timeWindow.append([info[5], info[6]])
            self.serviceTime.append(info[7])
        
        # calculate Euclidean distance matrix from the temporary coordsList
        coords = np.array(coordsList)
        distMatrix = np.zeros((self.customerCnt + 1, self.customerCnt + 1))
        for i in range(self.customerCnt + 1):
            for j in range(i, self.customerCnt + 1):
                dist = np.linalg.norm(coords[i] - coords[j])
                distMatrix[i, j] = distMatrix[j, i] = dist
        
        self.distMatrix = distMatrix  # np array, size = (customerCnt+1) x (customerCnt+1)
        self.timeMatrix = distMatrix  # np array, size = (customerCnt+1) x (customerCnt+1), assuming travel time equals distance
        
        # store the obj function parameters
        self.gd = gd              # a scalar, per vehicle dispatch cost
        self.gt = gt              # a scalar, per unit distance travel cost
        self.objSigma = objSigma  # a scalar, weight for veh dispatch cost in obj (eq 1)


    """ === 兩位學姐做 === """
    def run(self):  # main function to run the ACO, return (bestObj, allSol)
        pass  # write the code here
        # you might need to use self.randomTransfer(), self.calculateObj(), self.destroy(), self.repair(), self.updatePheromone()


    def findFeasibleNodes(self, route):  # return a list of "unvisited" "feasible" nodes
        # return [0] if no customer can be found, and thus the ant should return to the depot and restart a route
        # raise an error if even returning to depot is infeasible
        pass # write the code here
        # you might need to use self.checkFeasibility()


    def randomTransfer(self):  # return a sol of an ant (a list of routes, a route is also a list)
        pass  # write the code here
        # you might need to use self.findFeasibleNodes()


    def calculateObj(self, sol):  # return the obj of a sol
        pass  # write the code here

    def updatePheromone(self, allSol, allObj):  # return the updated pheromoneMatrix
        # allSol: a list of sol, a sol is also a list
        # all Obj: a list of objective values
        pass  # write the code here


    """ === 學弟做 === """
    def checkFeasibility(self, route):  # return True or False
        load = sum(self.deliveryDemand[node] for node in route)  # initial load is the sum of delivery demands
        if load > self.vehCapacity:
            return False  # infeasible, since the veh exceeds capacity at the start

        time = self.timeWindow[0][0]  # start from the depot
        for i in range(1, len(route)):  # exclude the starting depot, but include the ending depot
            prev = route[i-1]
            curr = route[i]

            # travel from prev to curr
            time += self.timeMatrix[prev, curr]

            # check time window
            ready, due = self.timeWindow[curr]
            if time < ready:
                time = ready  # wait at this node until ready
            if time > due:
                return False  # infeasible, since the veh arrives later than the due time

            # add service time
            time += self.serviceTime[curr]

            # update load (no pickup or delivery at the ending depot)
            load += (self.pickupDemand[curr] - self.deliveryDemand[curr])
            if load > self.vehCapacity:
                return False  # infeasible, since the veh exceeds capacity

        return True


    def destroy(self, sol):  # return new sol, R (a list of removed nodes)
        pass  # write the code here


    """ === 學長做 === """
    def repair(self, sol, R):  # return (new sol, new obj)
        # R: a list of removed nodes (to be inserted back somewhere)
        pass  # write the code here
        # you might need to use self.checkFeasibility() and self.calculateObj()



if __name__ == "__main__":
    # datasetFilePath = "Wang_Chen_data/cdp101.txt"  # please put the folder "Wang_Chen_data" in the same directory as ACO.py
    datasetFilePath = "anEasyExample.txt"  # just for testing
    solver = ACO()
    solver.setProblemParameter(datasetFilePath) 
    solver.setAlgorithmParameter()

    "do not run the below green code until we all finish our sub-functions"
    # bestObj, bestSol = solver.run()
    # print("bestObj:", bestObj)
    # print("bestSol:", bestSol)

    "test your sub-functions here, the following is just an example of checkFeasibility()"
    route1 = [0, 1, 2, 0]
    print(solver.checkFeasibility(route1))  # expect True
    route2 = [0, 2, 1, 0]
    print(solver.checkFeasibility(route2))  # expect False, since the time window of a customer is violated
    route3 = [0, 3, 2, 0]
    print(solver.checkFeasibility(route3))  # expect False, since the time window of the depot is violated
    route4 = [0, 1, 4, 0]
    print(solver.checkFeasibility(route4))  # expect False, since the vehicle capacity is exceeded
