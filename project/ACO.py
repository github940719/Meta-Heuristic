import numpy as np
import random
import copy
import heapq
import time
import os

"""
a sol is a list of routes
a route is a list of nodes = [0, customer1, customer2, ..., 0]
"""

EPS = 1e-10   # 極小常數，避免除以 0

class ACO:

    def setAlgorithmParameter(self, m, alpha, beta, gamma, delta, rho, Q, r0, maxIter, L, D):
        self.m = m           # number of ants
        self.alpha = alpha   # pheromone factor
        self.beta = beta     # heuristic factor for (1 / dist)
        self.gamma = gamma   # heuristic factor for (1 / width of time window)
        self.delta = delta   # heuristic factor for (1 / waiting time)
        self.rho = rho       # pheromone evaporation rate
        self.Q = Q           # pheromone intensity (in equation 20)
        self.r0 = r0         # exploitation threshold (in equation 15)
        self.maxIter = maxIter   # maximum number of iterations
        self.L = L  # number of customers to be removed in destroy operator
        self.D = D  # larger D -> customers with higher relevance are more likely to be removed(in destroy operator)
        self.pheromonematrix = None


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
            for j in range(i + 1, self.customerCnt + 1):
                dist = np.linalg.norm(coords[i] - coords[j])
                distMatrix[i, j] = distMatrix[j, i] = dist
        
        self.distMatrix = distMatrix  # np array, size = (customerCnt+1) x (customerCnt+1)
        self.timeMatrix = distMatrix  # np array, size = (customerCnt+1) x (customerCnt+1), assuming travel time equals distance
        
        # store the obj function parameters
        self.gd = gd              # a scalar, per vehicle dispatch cost
        self.gt = gt              # a scalar, per unit distance travel cost
        self.objSigma = objSigma  # a scalar, weight for veh dispatch cost in obj (eq 1)

        # calculate the normalized dist (for destroy operator)
        maxDist = np.max(self.distMatrix)
        self.normalizedDistMatrix = self.distMatrix / maxDist


    def run(self):  # main function to run the ACO, return (bestObj, best sol)

        # initialize
        self.pheromonematrix = np.ones((self.customerCnt + 1, self.customerCnt + 1))
        bestSol = None
        bestObj = float("inf")

        # main loop
        for iter in range(self.maxIter):

            # initialize for this iteration
            Sbest = None  # best sol found in this iteration
            SbestObj = float("inf")

            # each ant constructs a sol
            for ant in range(self.m):
                sol = self.randomTransfer()   # generate a sol for this ant
                obj, _, _ = self.calculateObj(sol)  # calculate the obj of this sol

                # update best solution found so far
                if obj < SbestObj:
                    Sbest = copy.deepcopy(sol)
                    SbestObj = obj

            # destroy and repair
            Sd, R = self.destroy(sol = Sbest)  
            Sr, SrObj = self.repair(Sd, R)

            # global update
            if SrObj < SbestObj:
                Sbest = copy.deepcopy(Sr)
                SbestObj = SrObj
            if SbestObj < bestObj:
                bestObj = SbestObj
                bestSol = copy.deepcopy(Sbest)
            self.pheromonematrix = self.updatePheromone(Sbest)

            print("iter", iter, "bestObj", bestObj)  # 沒有要 trace 程式碼可以關掉
        
        # destroy and repair "local search again"
        Sd, R = self.destroy(sol = bestSol)  
        Sr, SrObj = self.repair(Sd, R)
        if SrObj < bestObj:
            bestSol = copy.deepcopy(Sr)
            bestObj = SrObj
        return bestSol, self.calculateObj(bestSol)


    def findFeasibleNodes(self, visitedSet, route, bottleNeckLoad, currentLoad, currentTime):  # return a list of "unvisited" "feasible" nodes

        # check feasibility given bottleNeckLoad and currentTime
        def checkFeasibilityWithCummulation(nextNode):   # return True or False

            # arrival time
            arrivalTime = currentTime + self.timeMatrix[route[-1], nextNode]
            if arrivalTime > self.timeWindow[nextNode][1]:
                return False  # arrival later than the due time
            
            # ensure that we can back to depot within due time
            startTime = max(arrivalTime, self.timeWindow[nextNode][0])  # starts no earlier than the ready time
            endTime = startTime + self.serviceTime[nextNode]
            if (endTime + self.timeMatrix[nextNode, 0] > self.timeWindow[0][1]):  # returning time to depot > due time
                return False  # cannot return to depot within due time

            # load
            bottleNeckTemp = bottleNeckLoad + self.deliveryDemand[nextNode]  # this delivery would travels from depot to this node
            if bottleNeckTemp > self.vehCapacity:
                return False  # exceed capacity
            
            currentLoadTemp = currentLoad + self.pickupDemand[nextNode]  # no need to re-calculate the deliveryDemand, as it is dropped off here
            bottleNeckTemp = max(bottleNeckTemp, currentLoadTemp)
            if bottleNeckTemp > self.vehCapacity:
                return False  # exceed capacity

            return True  # feasible


        feasible = []
        for i in range(1, self.customerCnt + 1):
            if i in visitedSet:
                continue  # this node has been visited

            """
            方式一: 直接用 bottleNeck, currentLoad, currentTime 檢查加入 i 之後的可行性 (目前寫法)
            方式二: 重新檢查整條路線的可行性  (註解寫法)
            設定相同的 random.seed(數字 e.g. 0)、np.random.seed(數字 e.g. 0) ，答案應相同 (不一樣請告訴我!)，比較程式執行時間差異
            """
            
            if checkFeasibilityWithCummulation(i):  # 方式一
            # if self.checkFeasibility(route + [i, 0]):   # 方式二
                feasible.append(i)  # this node can be visited next
        
        if len(feasible) == 0:  # no feasible node or all nodes have been visited
            feasible = [0]  # the only choice now is returning to depot

        return feasible
    

    def randomTransfer(self):  # return a sol of an ant (a list of routes, a route is also a list)
        sol = []     
        visitedSet = set()  # 已拜訪客戶的集合

        while len(visitedSet) < self.customerCnt:
            route = [0] # 如果還有顧客沒拜訪，就再從倉庫出發
            currentTime = self.timeWindow[0][0]  # 當前時間
            currentLoad = 0  # 當前車輛的載重
            bottleNeckLoad = 0  # 到目前為止的容量瓶頸
            currNode = 0  # 目前的客戶編號

            while True:  
                # 找這條路線下一個可以拜訪的顧客候選
                feasibleNodes = self.findFeasibleNodes(visitedSet, route, bottleNeckLoad, currentLoad, currentTime)
                if feasibleNodes == [0]:  # 若沒有可行節點，回倉庫結束
                    sol.append(route + [0])
                    break
                
                # 算每個可行客戶的分數
                scores = []  # scores[i] = feasibleNodes[i] 的分數
                for j in feasibleNodes:
                    tau = self.pheromonematrix[currNode][j] ** self.alpha
                    eta = (1.0 / (self.distMatrix[currNode][j] + EPS)) ** self.beta
                    ready, due = self.timeWindow[j]

                    """
                    測試論文跟我們發想的 random transfer rule 解's 差異
                    """
                    tw_term = (1.0 / max(due - ready, EPS)) ** self.gamma
                    wt_term = (1.0 / max(self.serviceTime[j], EPS)) ** self.delta
                    # tw_term = (1.0 / max(due - (currentTime + self.timeMatrix[currNode, j]), EPS)) ** self.delta
                    # wt_term = 1.0  # 不考慮的意思
                    score = tau * eta * tw_term * wt_term
                    scores.append(score)

                # 決定下一個節點（r0 控制貪婪 or 隨機）
                r = random.random()
                scores = np.array(scores, dtype = float) 
                if r <= self.r0:  # exploitation
                    maxScore = np.max(scores)
                    maxIndices = np.where(scores == maxScore)[0]   # 找出所有並列最大的位置
                    nextNode = feasibleNodes[np.random.choice(maxIndices)]  # 從並列最大的客戶中隨機挑一個
                else:  # exploration, 依照分數計算機率
                    sum_scores = scores.sum()
                    if sum_scores < EPS:  # 若總和為 0，均勻分布
                        probs = np.ones_like(scores) / len(scores)
                    else:
                        probs = scores / sum_scores
                    nextNode = int(np.random.choice(feasibleNodes, p = probs))


                # 更新當前參數
                arrival = currentTime + self.timeMatrix[currNode, nextNode]
                ready, due = self.timeWindow[nextNode]
                currentTime = max(arrival, ready) + self.serviceTime[nextNode] # 更新當前時間
                bottleNeckLoad += self.deliveryDemand[nextNode]  # 更新裝卸載前的容量瓶頸
                currentLoad += self.pickupDemand[nextNode]  # 更新當前載重
                bottleNeckLoad = max(bottleNeckLoad, currentLoad)  # 更新裝卸載後的容量瓶頸
                route.append(nextNode)
                visitedSet.add(nextNode) # 紀錄已拜訪
                currNode = nextNode

        return sol


    def calculateObj(self, sol):  # return the obj of a sol
        total_distance = 0.0
        for route in sol:
            for i in range(len(route) - 1):
                total_distance += self.distMatrix[route[i]][route[i + 1]] # 累加路徑距離
        NV = len(sol) # 車輛數量
        f = self.objSigma * self.gd * NV + (1.0 - self.objSigma) * self.gt * total_distance
        return f, NV, total_distance    # 目標函數 f(S), 車輛數, 總距離


    def updatePheromone(self, Sbest):  # return the updated pheromoneMatrix
        self.pheromonematrix *= (1.0 - self.rho)  # 費洛蒙蒸發

        _, _, length = self.calculateObj(Sbest)
        delta_tau = self.Q / (length + EPS) # 計算該路徑增加的費洛蒙量

        for route in Sbest:
            for i in range(len(route) - 1):
                u, v = route[i], route[i + 1]
                self.pheromonematrix[u][v] += delta_tau # 更新費洛蒙
                self.pheromonematrix[v][u] = self.pheromonematrix[u][v]  # only print on the terminal, do not write to the outputFile

        return self.pheromonematrix


    def checkFeasibility(self, route):  # return True or False
        load = sum(self.deliveryDemand[node] for node in route[1:-1])  # initial load is the sum of delivery demands
        if load > self.vehCapacity:
            return False  # infeasible, since the veh exceeds capacity at the start

        time = self.timeWindow[0][0]  # start from the depot
        for i in range(len(route) - 1):  # exclude the starting depot, but include the ending depot
            prev = route[i]
            curr = route[i+1]

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
        
        # helper function: calculate relevance between 2 customers i and j
        def calculateRelevance(i, j):
            
            # V_ij = 0 if i and j are in the same route; 1 otherwise
            sameRoute = any(i in route and j in route for route in sol)
            V_ij = 0 if sameRoute else 1
            
            # relevance R_ij = 1 / (C'_ij + V_ij, C'_ij is the normalized distance between i and j)
            denom = self.normalizedDistMatrix[i][j] + V_ij
            return 1 / denom if denom != 0 else float("inf")  # avoid division by zero


        # main logic of destroy operator
        R = []  # a list of removed customers
        U = set([i for i in range(1, self.customerCnt + 1)])  # a set of unremoved customers

        # randomly remove a seed customer
        seed = random.choice(list(U))
        R.append(seed)
        U.remove(seed)

        # remove the second to the L-th customer based on relatedness
        while len(R) <= self.L:
            r = random.choice(R)  # randomly select a seed customer from R
            relevanceScores = [(c, calculateRelevance(r, c)) for c in U]
            relevanceScores.sort(key = lambda x: x[1], reverse = True)  # sort relevance from high to low
            
            idx = int((random.random() ** self.D) * len(U))  # idx is the index of customer to remove in relevanceScores
            removedCustomer = relevanceScores[idx][0]
            """ if D is larger, customers with higher relevance scores are more likely to be removed. """
            
            R.append(removedCustomer)
            U.remove(removedCustomer)

        # re-construct the new sol without the removed customers
        newSol = []
        for route in sol:
            newRoute = [node for node in route if node not in R]  # remove the customers in R
            if len(newRoute) > 2:  # route is not [0, 0]
                newSol.append(newRoute[:])  # deep copy newRoute into newSol

        return newSol, R


    def repair(self, sol, R):
        while R :  # while R is not empty
            insertionCost = {node: [] for node in R}  # insertionCost[node] = a list of (routeID, position, delta)

            # 1. try each possible place for insertion
            for node in R:
                for j, route in enumerate(sol):  # for each route (idx = j) 
                    for pos in range(1, len(route)):  # for each possible position (pos, cannot insert before the starting depot)
                        candRoute = route[:]
                        candRoute.insert(pos, node)
                        if not self.checkFeasibility(candRoute):  # insert at this pos is infeasible
                            continue         
                        
                        # calculate delta : change of obj
                        prev = route[pos-1]
                        next = route[pos]
                        distIncrease = self.distMatrix[prev, node] + self.distMatrix[node, next] - self.distMatrix[prev, next]
                        delta = (1 - self.objSigma) * self.gt * distIncrease
                        insertionCost[node].append((j, pos, delta))

                # add an alternative : open a new route, change of obj [1] veh dispatch + 1 [2] dist + 2 * dist[0][node]
                insertionCost[node].append((None, None, self.objSigma * self.gd + (1 - self.objSigma) *  self.gt * 2 * self.distMatrix[0, node]))

            # 2. calculate regret value
            regret = {}  # regred[node] = (regret value, best (routeID, position, delta))
            for node, lst in insertionCost.items():
                if len(lst) == 1:  # this node can only be inserted to a new route
                    regret[node] = (-1, lst[0])  # give it a small regret value s.t. very unlikely to be chosen in this iteration
                else:
                    # best, second = (routeIdx, pos, delta), key = delta
                    best, second = heapq.nsmallest(2, lst, key = lambda x: x[2])
                    regret[node] = (second[2] - best[2], best)
            
            # 3. choose the node and its corresponding pos with the largest regret value
            nodeToInsert = max(regret.keys(), key = lambda n: regret[n][0])  # key = regret value
            R.remove(nodeToInsert)
            bestRouteIdx, bestPos, _ = regret[nodeToInsert][1]
            if bestRouteIdx is None:  # open a new route for this nodeToInsert
                newRoute = [0, nodeToInsert, 0]
                sol.append(newRoute)
            else:
                sol[bestRouteIdx].insert(bestPos, nodeToInsert)

        obj, _, _ = self.calculateObj(sol)
        return sol, obj



if __name__ == "__main__":  # 測試 L, D 的範例程式碼
    solver = ACO()
    datasetFilePath = "Wang_Chen_data/rdp101.txt"
    solver.setProblemParameter(datasetFilePath)

    with open("testResult.txt", "w") as outputFile:
        outputFile.write("L, D, minObj, NV, Dist, avgObj\n")
        outputFile.flush()

        for testL in [0.05, 0.1, 0.15, 0.2, 0.25]:
            for testD in [1, 2, 3, 4, 5]:

                results = []  # (obj, NV, dist)

                for _ in range(10):  # run 10 times
                    solver.setAlgorithmParameter(
                        m = 20, alpha = 2, beta = 1, gamma = 2, delta = 3, 
                        rho = 0.85,Q = 1000, r0 = 0.5, maxIter = 200,
                        L = testL * solver.customerCnt, D = testD
                    )

                    startTime = time.perf_counter()
                    bestSol, bestObjInfo = solver.run()
                    endTime = time.perf_counter()

                    print("bestSol:")
                    for route in bestSol:
                        print(route)
                    print("bestObj:", bestObjInfo[0])
                    print("number of vehicle", bestObjInfo[1])
                    print("total dist", bestObjInfo[2])
                    print(f"execution time {endTime - startTime:.2f}")

                    # append (obj, NV, Dist)
                    results.append((bestObjInfo[0], bestObjInfo[1], bestObjInfo[2]))

                minObj, vehCnt, Dist = min(results, key = lambda x: x[0])
                avgObj = sum(r[0] for r in results) / len(results)

                # write
                outputFile.write(f"{testL}, {testD}, {minObj}, {vehCnt}, {Dist}, {avgObj}\n")
                outputFile.flush()
