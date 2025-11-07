import numpy as np
from gurobipy import GRB, Model, quicksum

class GurobiForVRPSPDTW:

    def __init__(self, datasetFilePath, gd = 60, gt = 5, objSigma = 0.6):
        with open(datasetFilePath, "r") as f:
            lines = f.readlines()

        # parse vehicle capacity and customer count from header
        vehInfo = lines[4].split()
        self.customerCnt = int(vehInfo[0])  # a scalar, number of customers (excluding the depot)
        self.vehCnt = int(vehInfo[1])       # a scalar, number of vehicles
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
        num_nodes = self.customerCnt + 1
        distMatrix = np.zeros((num_nodes, num_nodes))
        for i in range(num_nodes):
            for j in range(i, num_nodes):
                dist = np.linalg.norm(coords[i] - coords[j])
                distMatrix[i, j] = distMatrix[j, i] = dist
        
        self.distMatrix = distMatrix  # np array, size = (customerCnt+1) x (customerCnt+1)
        self.timeMatrix = distMatrix  # np array, size = (customerCnt+1) x (customerCnt+1), assuming travel time equals distance
        
        # store the obj function parameters
        self.gd = gd              # a scalar, per vehicle dispatch cost
        self.gt = gt              # a scalar, per unit distance travel cost
        self.objSigma = objSigma  # a scalar, weight for veh dispatch cost in obj (eq 1)


    def solve(self):
        model = Model("VRPSPDTW")

        # === Sets and Big-M ===
        V = range(1, self.customerCnt + 1) # V = {1, ..., n}, set of customers excluding depot
        V0 = range(self.customerCnt + 1)   # V0 = {0, 1, ..., n}, set of all nodes including depot
        K = range(self.vehCnt)             # K = {1, ..., k}, set of vehicles
        M = max(self.timeWindow[0][1], self.vehCapacity)  # Big-M value for time and load constraints

        # === Decision Variables ===
        # X[i, j, k] = 1 if veh k travels from node i to node j, 0 otherwise
        X = model.addVars([(i, j, k) for i in V0 for j in V0 if i != j for k in K], vtype = GRB.BINARY)
        
        # S[i] = service start time at node i
        S = model.addVars(V, vtype = GRB.CONTINUOUS, lb = 0)
        
        # L[i, k] = load upon departure from node i
        L = model.addVars(V, vtype = GRB.CONTINUOUS, lb = 0)

        # L0[k] = initial load of vehicle k
        L0 = model.addVars(K, vtype = GRB.CONTINUOUS, lb = 0)

        # === Objective Function ===
        # eq (1): minimize the weighted sum of vehicle dispatch cost and travel cost
        dispatchCost = quicksum(self.gd * X[0, j, k] for j in V for k in K)
        travelCost = quicksum(self.gt * self.distMatrix[i, j] * X[i, j, k] for i in V0 for j in V0 for k in K if i != j)
        model.setObjective(self.objSigma * dispatchCost + (1 - self.objSigma) * travelCost, GRB.MINIMIZE)

        # === Constraints ===
        # eq (2): each customer (j) must be visited exactly once
        for j in V:
            model.addConstr(quicksum(X[i, j, k] for i in V0 for k in K if i != j) == 1)

        # eq (3): flow conservation for each vehicle at each customer node (h)
        for h in V:
            for k in K:
                model.addConstr(quicksum(X[i, h, k] for i in V0 if i != h) == quicksum(X[h, j, k] for j in V0 if j != h))
        
        # eq (4) & (6): each vehicle must leaves from and return to the depot at most once
        for k in K:
            model.addConstr(quicksum(X[0, j, k] for j in V) <= 1)
            model.addConstr(quicksum(X[i, 0, k] for i in V) <= 1)
            
        # eq (5): if a vehicle leaves the depot, it must return to the depot
        for k in K:
            model.addConstr(quicksum(X[0, j, k] for j in V) == quicksum(X[i, 0, k] for i in V))

        # eq (7): time cummulation: for customer pair (i, j) excluding the depot
        for i in V:
            for j in V:
                if i != j:
                    model.addConstr(S[i] + self.serviceTime[i] + self.timeMatrix[i, j] - M * (1 - quicksum(X[i, j, k] for k in K)) <= S[j])

        # eq (8): time window constraints for each customer (i)
        for i in V:
            model.addConstr(S[i] >= self.timeWindow[i][0])
            model.addConstr(S[i] <= self.timeWindow[i][1])
        
        # time window constraint for the depot
        for i in V:
            # for the first customer visited after depot
            model.addConstr(S[i] >= self.timeWindow[0][0] + self.distMatrix[0, i] * quicksum(X[0, i, k] for k in K))
            
            # for the last customer before returning back to depot
            model.addConstr(S[i] <= self.timeWindow[0][1] - self.distMatrix[i, 0] * quicksum(X[i, 0, k] for k in K))

        
        # eq (9) initial load of a vehicle: sum of delivery demands for each customer (j) on its route
        for k in K:
            model.addConstr(L0[k] == quicksum(self.deliveryDemand[j] * quicksum(X[i, j, k] for i in V0 if i != j) for j in V))
        
        # eq (10) cummulate load of the first customer visited after depot
        for j in V:
            model.addConstr(L[j] >= L0[k] - self.deliveryDemand[j] + self.pickupDemand[j] - M * (1 - quicksum(X[0, j, k] for k in K)))

        # eq (11) cummulate load along the route
        for i in V:
            for j in V:
                if i != j:
                    model.addConstr(L[j] >= L[i] - self.deliveryDemand[j] + self.pickupDemand[j] - M * (1 - quicksum(X[i, j, k] for k in K)))
        
        # eq (12) & (13) vehicle capacity constraints
        for k in K:
            model.addConstr(L0[k] <= self.vehCapacity)
        for j in V:
            model.addConstr(L[j] <= self.vehCapacity)

        # === Solve and Output ===
        model.optimize()
        if model.Status != GRB.OPTIMAL:
            raise Exception("No optimal solution found.")
        
        routeList = []
        totalDist = 0
        
        for k in K:
            route = [0]
            currentNode = 0
            while True:
                for j in V:
                    if j == currentNode: continue
                    if X[currentNode, j, k].X > 0.5:
                        route.append(j)
                        totalDist += self.distMatrix[currentNode, j]
                        currentNode = j
                        break

                if currentNode == 0:
                    break  # no further node to visit
                
                if X[currentNode, 0, k].X > 0.5:
                    route.append(0)  # back to depot
                    totalDist += self.distMatrix[currentNode, 0]
                    break
            
            if len(route) > 2:  # vehicle k has served at least one customer
                routeList.append(route)

        return model.ObjVal, len(routeList), totalDist, routeList


if __name__ == "__main__":
    datasetFilePath = "Wang_Chen_data/rdp101.txt"
    solver = GurobiForVRPSPDTW(datasetFilePath)

    obj, vehCnt, totalDist, routeList = solver.solve()
    print("\n===== Optimal Solution =====")
    print(f"obj: {obj:.2f}, number of vehicles used: {vehCnt}, total distance: {totalDist:.2f}")
    for idx, route in enumerate(routeList):
        print(f"Vehicle {idx + 1} route: {route}")
