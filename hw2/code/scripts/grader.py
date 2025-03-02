
import numpy as np
import pdb
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer

plannerList = ["RRT", "RRTCONNECT", "RRTSTAR", "PRM"]

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################


def graderMain(executablePath, gradingCSV):
    problems = [["map1.txt", "1.570796,0.785398,1.570796,0.785398,1.570796",
                                # "0.392699,2.356194,3.141592,2.8274328,4.712388"],
                                "0.392699,2.356194,3.141592,2.8274328,4.712388"],
            ["map2.txt", "0.392699,2.356194,3.141592",
                                "1.570796,0.785398,1.570796"],
            ["map2.txt", "1.24125,2.10627,4.82693,1.74531,3.4807", "1.84826,1.45934,3.67245,1.53569,0.957493"],
            ["map2.txt", "0.491547,0.439235,1.28589,2.89919,5.15018", "1.81125,2.08216,0.572703,2.68498,5.8716"],
            ["map2.txt", "1.38864,1.2442,3.84809,0.689471,4.23867", "0.0358716,1.91787,1.64349,4.1178,5.38818"],
            ["map2.txt", "0.689754,2.34712,1.25037,4.06221,3.72399", "1.77364,1.05033,1.27534,3.93406,1.10734"],
            ["map2.txt", "1.84209,1.53353,5.73272,3.55732,1.19826", "1.69847,1.2549,1.8079,4.13209,5.95018"],
            ["map2.txt", "1.78624,0.482256,5.52926,1.08525,1.12461", "1.84205,1.15973,0.0177676,5.65972,1.81428"],
            ["map2.txt", "1.55799,1.2162,0.198589,0.7047,4.56961", "0.000294541,0.973433,1.93373,0.198883,1.67813"],
            ["map2.txt", "1.64546,5.92556,0.5441,1.83599,4.70755", "1.2286,2.13644,3.37822,0.906326,1.33909"],
            ["map2.txt", "1.68143,5.41703,1.61216,2.62316,4.20011", "1.47304,5.63362,6.1285,0.967125,3.74454"],
            ["map2.txt", "0.735098,1.82959,3.60075,5.31327,0.0164781", "1.23207,2.01547,5.92914,4.85466,0.589285"],
            ["map2.txt", "0.596419,6.27151,1.50089,0.994727,3.55602", "1.06656,2.67597,6.10127,1.55625,3.41281"],
            ["map2.txt", "1.07045,1.6861,1.02147,4.30331,0.32284", "1.25423,4.25586,1.46991,0.310849,1.53353"],
            ["map2.txt", "1.86598,1.19004,4.21352,1.27125,1.62951", "1.08842,0.98179,1.01243,4.52012,5.53932"],
            ["map2.txt", "1.29426,5.64784,1.87211,3.98346,2.016", "1.82715,1.85292,1.86426,1.26334,0.599044"],
            ["map2.txt", "0.171429,2.30706,1.07353,3.01397,2.61083", "0.893626,1.46367,0.970082,2.68004,3.73744"],
            ["map2.txt", "1.6874,0.870468,0.560215,3.7729,3.23021", "0.525147,0.194549,3.18225,3.54627,6.09731"],
            ["map2.txt", "0.201302,2.02362,4.41482,1.93504,0.90576", "1.01757,2.44974,1.88284,3.04771,5.23451"],
            ["map2.txt", "0.992425,0.117963,5.24151,1.50628,1.38653", "1.78992,1.64852,6.02113,2.30289,1.34454"],
            ["map2.txt", "0.725333,1.5947,1.02194,4.75951,0.329795", "1.05041,2.44314,2.47034,5.62618,6.22759"],
            ["map2.txt", "0.141967,1.03525,1.24993,4.69453,0.837635", "1.87184,1.49113,1.37792,6.15462,3.15319"]]
    scores = []
    for aPlanner in [0, 1, 2, 3]:
    # for aPlanner in [2]:
    
        print("\nTESTING " + plannerList[aPlanner] + "\n")
        for i, data in enumerate(problems):
            inputMap, startPos, goalPos = [*data]
            numDOFs = len(startPos.split(","))
            outputSolutionFile = "../output/grader_out/tmp.txt"
            commandPlan = "{} {} {} {} {} {} {}".format(
                executablePath,
                inputMap, numDOFs, startPos, goalPos,
                aPlanner, outputSolutionFile)
            print("EXECUTING: " + str(commandPlan))
            commandVerify = "./../build/verifier {} {} {} {} {}".format(
                inputMap, numDOFs, startPos, goalPos,
                outputSolutionFile)
            print("EXECUTING: " + str(commandVerify))
            try:
                start = timer()
                subprocess.run(commandPlan.split(" "), check=True) # True if want to see failure errors
                timespent = timer() - start
                returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                if returncode != 0:
                    print("Returned an invalid solution")
                
                ### Calculate the cost from their solution
                with open(outputSolutionFile) as f:
                    line = f.readline().rstrip()  # filepath of the map
                    solution = []
                    for line in f:
                        solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                    solution = np.asarray(solution).astype(float)
                    numSteps = solution.shape[0]


                    ## Cost is sum of all joint angle movements
                    difsPos = np.abs(solution[1:,]-solution[:-1,])
                    cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()



                    success = returncode == 0
                    scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success])
            
                # ### Visualize their results
                # commandViz = "python visualizer.py ../output/grader_out/tmp.txt --gifFilepath=../output/grader_out/grader_{}{}.gif".format(plannerList[aPlanner], i)
                # commandViz += " --incPrev=1"
                # subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
            except Exception as exc:
                print("Failed: {} !!".format(exc))
                scores.append([aPlanner, inputMap, i, -1, -1, timespent, False])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success"])
    df.to_csv(gradingCSV, index=False)
            

if __name__ == "__main__":
    graderMain("./../build/planner", "../output/grader_out/grader_results.csv")