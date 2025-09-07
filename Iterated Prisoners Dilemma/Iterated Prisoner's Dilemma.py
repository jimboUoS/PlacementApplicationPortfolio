#this code seeks to determine the minimum proportion of an initial population made up of tit for tat and always defect agents 
#in order for a steady population of tit for tat agents to remain after a certain number of rounds
import matplotlib.pyplot as plt
import random

class Agent:
    def __init__(self, roundOfBirth, strategy):
        self.roundOfBirth = roundOfBirth
        #strategy: False = Always Defect, True = Tit For Tat
        self.strategy = strategy
        #previous move: False = Defect, True = Cooperate
        self.opponentPreviousMove = True
        self.points = 0
    def getStrategy(self):
        return self.strategy
    def getRoundOfBirth(self):
        return self.roundOfBirth
    def getOpponentPreviousMove(self):
        return self.opponentPreviousMove
    def setOpponentPreviousMove(self,move):
        self.opponentPreviousMove=move
        return self.opponentPreviousMove
    def addPoints(self,pointsThisRound):
        self.points += pointsThisRound
    def getPoints(self):
        return self.points
    def chooseMove(self):
        if self.strategy:
            if self.opponentPreviousMove:
                return True
            else:
                return False
        else:
            return False

def interact(agent1, agent2):
    '''
    agent1Choice = agent1.chooseMove()
    agent2Choice = agent2.chooseMove()
    if agent1Choice and agent2Choice:
        agent1.addPoints(3)
        agent2.addPoints(3)
        agent1.setOpponentPreviousMove(True)
        agent2.setOpponentPreviousMove(True)
    elif agent1Choice != agent2Choice:
        if agent1Choice:
            agent2.addPoints(5)
            agent1.setOpponentPreviousMove(False)
            agent2.setOpponentPreviousMove(True)
        else:
            agent1.addPoints(5)
            agent1.setOpponentPreviousMove(True)
            agent2.setOpponentPreviousMove(False)
    else:
        agent1.addPoints(1)
        agent2.addPoints(1)
        agent1.setOpponentPreviousMove(False)
        agent2.setOpponentPreviousMove(False)
    '''
    if agent1.getStrategy() and agent2.getStrategy():
        agent1.addPoints(30)
        agent2.addPoints(30)
    elif agent1.getStrategy() and not agent2.getStrategy():
        agent1.addPoints(9)
        agent2.addPoints(14)
    elif not agent1.getStrategy and agent2.getStrategy():
        agent1.addPoints(14)
        agent2.addPoints(9)
    else:
        agent1.addPoints(10)
        agent2.addPoints(10)


def runMultipleRounds(activatePlot, initTftPopulation,initADPopulation, nRounds, roundSize, reproductionThreshold, agentLifespan):
    def killElderlyAgents(r):
        to_remove = []
        for a in agents:
            if r - a.getRoundOfBirth() >= agentLifespan:
                to_remove.append(a)
        for a in to_remove:
            agents.remove(a)
    def produceOffspring(r):
        for a in agents:
            if a.getPoints() >= reproductionThreshold:
                agents.append(Agent(r+1, a.getStrategy()))
                a.addPoints(-50)
    def resetAgentMemory():
        for a in agents:
            a.setOpponentPreviousMove(True)
    tFtPopulation = initTftPopulation
    aDPopulation = initADPopulation
    rounds = range(1,nRounds+1)
    tFtPopulationPoints = []
    aDPopulationPoints = []
    proportionPoints = []
    agents = []
    for i in range(tFtPopulation):
        agents.append(Agent(1,1))
    for i in range(aDPopulation):
        agents.append(Agent(1,0))

    for r in rounds:
        agentIndexList = range(len(agents))
        agentColumn1 = []
        agentColumn2 = []
        agentColumn1 = random.sample(agentIndexList, k=len(agents)//2)
        for i in agentIndexList:
            if i not in agentColumn1:
                agentColumn2.append(i)
        tFtPopulation = 0
        aDPopulation = 0
        for a in agents:
            if a.getStrategy():
                tFtPopulation +=1
            else:
                aDPopulation += 1
        #print(f"Tit for Tat Population: {tFtPopulation}")
        #print(f"Always Defect Population: {aDPopulation}")
        tFtPopulationPoints.append(tFtPopulation)
        aDPopulationPoints.append(aDPopulation)

        
        for i in range(len(agentColumn1)):
            '''
            for m in range(roundSize):
                interact(agents[agentColumn1[i]],agents[agentColumn2[i]])
            '''
            interact(agents[agentColumn1[i]],agents[agentColumn2[i]])
        produceOffspring(r)
        killElderlyAgents(r)
        resetAgentMemory()

    for r in rounds:
        proportionPoints.append(100*tFtPopulationPoints[r-1]/(tFtPopulationPoints[r-1]+aDPopulationPoints[r-1]))

    if activatePlot:
        figtext_args = (0.1, 0, 
                        f"TfT Initial Population = {initTftPopulation}\n AD Initial Population = {initADPopulation}\nMoves per round = {roundSize}") 
        
        figtext_kwargs = dict(ha ="left",va="bottom",  
                            fontsize = 10, wrap = True) 
    
        graph1 = plt.figure(1)
        plt.plot(rounds,tFtPopulationPoints, label = "Tit for Tat")
        plt.plot(rounds,aDPopulationPoints, label ="Always Defect")
        plt.xlabel("Round Number")
        plt.ylabel("Population")
        plt.legend(loc="upper left")
        plt.title("Population")
        plt.subplots_adjust(bottom=0.2)
        plt.figtext(*figtext_args,**figtext_kwargs)

        graph2 = plt.figure(2)
        plt.xlabel("Round Number")
        plt.ylabel("Proportion of Population(%)")
        plt.plot(rounds,proportionPoints)
        plt.title("Tit for Tat Proportion of Total Population")
        plt.subplots_adjust(bottom=0.2)
        plt.figtext(*figtext_args,**figtext_kwargs)
        plt.show()
    if proportionPoints[-1]>=proportionPoints[0]:
        return True
    else:
        return False

def runSingleRound():
    nice = Agent(1,True)
    nasty = Agent(1,False)

    for i in range(10):
        interact(nice,nasty)
        print(f"Tit for Tat: {nice.getPoints()}, Always Defect: {nasty.getPoints()}")

def runStochasticModel(activatePlot):
    probabilityPoints = []
    for i in range(1,16):
        successCounter = 0
        for j in range(1000):
            if runMultipleRounds(False,i,100-i,30,10,50,10):
                successCounter += 1
        probabilityPoints.append(successCounter/10)
        print(f"Initial Population: {i}\nChance of Survival: {successCounter/10}%")
    if activatePlot:
        figtext_args = (0.1, 0, 
                        f"Moves per round = 10\nNB: Survival means that the proportion of Tit for Tat is not decreasing over 30 rounds") 
        
        figtext_kwargs = dict(ha ="left",va="bottom",  
                            fontsize = 10, wrap = True) 
    
        graph1 = plt.figure(1)
        plt.plot(range(1,16),probabilityPoints)
        plt.xlabel("Initial Proportion of Population (%)")
        plt.ylabel("Probability (%)")
        plt.legend(loc="upper left")
        plt.title("Probability Tit for Tat Survival Given Different Starting Proportions")
        plt.subplots_adjust(bottom=0.2)
        plt.figtext(*figtext_args,**figtext_kwargs)
        plt.show()


#runMultipleRounds(True,1,99,30,10,50,10)
runStochasticModel(True)
#for i in range(roundSize):
#    interact(agent1,agent2)       
    
