//============================================================================
// Name        : main.cpp
// Author      : André Luyde
// Description : FLexi-VNS
//============================================================================

#include "solution/solution.h"

using namespace std;


int compare(const void * a, const void * b);

int main(int argc, char* argv[]) {
    double finalTime;
	vector<string> arguments(argv + 1, argv + argc);
    cout << fixed << setprecision(2);
    string inputFileName = arguments[0];
    string problem = argv[2];

    Data data(inputFileName, problem);

    int vns_max = stoi(argv[3]);

    // cout << "Aqui: " << argv[0] << " - " << argv[1] << " - " << argv[2] << " - " << argv[3] << endl;
    // cout << inputFileName << " - " << problem << vns_max << endl;
    // exit(1);

    for(int i = 0; i < (int)data.orderNodes.size(); i++){
        qsort(&data.orderNodes[i].front(), data.orderNodes[i].size(), sizeof(Request), compare);
    }
    
    double timeMax = 1000000;
    string inst = inputFileName.substr(5);
    if(problem == "EVRP"){
        if(inst == "Large_VA_Input_111c_21s.txt"){
            timeMax = 1305.60 * 0.6;
        }else if(inst == "Large_VA_Input_111c_22s.txt"){
            timeMax = 1413.60 * 0.6;
        }else if(inst == "Large_VA_Input_111c_24s.txt"){
            timeMax = 1314.00 * 0.6;
        }else if(inst == "Large_VA_Input_111c_26s.txt"){
            timeMax = 1507.20 * 0.6;
        }else if(inst == "Large_VA_Input_111c_28s.txt"){
            timeMax = 1450.20 * 0.6;
        }else if(inst == "Large_VA_Input_200c_21s.txt"){
            timeMax = 4599.00 * 0.6;
        }else if(inst == "Large_VA_Input_250c_21s.txt"){
            timeMax = 7254.00 * 0.6;
        }else if(inst == "Large_VA_Input_300c_21s.txt"){
            timeMax = 10933.80 * 0.6;
        }else if(inst == "Large_VA_Input_350c_21s.txt"){
            timeMax = 13921.80 * 0.6;
        }else if(inst == "Large_VA_Input_400c_21s.txt"){
            timeMax = 18307.20 * 0.6;
        }else if(inst == "Large_VA_Input_450c_21s.txt"){
            timeMax = 31531.20 * 0.6;
        }else if(inst == "Large_VA_Input_500c_21s.txt"){
            timeMax = 21360.60 * 0.6;
        }
    }else if(problem == "BSS"){
        if(inst == "P-n6-k2.vrp"){
            timeMax = 0.67 * 0.6;
        }else if(inst == "P-n7-k3.vrp"){
            timeMax = 0.38 * 0.6;
        }else if(inst == "P-n8-k3.vrp"){
            timeMax = 0.47 * 0.6;
        }else if(inst == "P-n16-k8.vrp"){
            timeMax = 1.66 * 0.6;
        }else if(inst == "P-n19-k2.vrp"){
            timeMax = 1.98 * 0.6;
        }else if(inst == "P-n21-k2.vrp"){
            timeMax = 1.93 * 0.6;
        }else if(inst == "P-n23-k8.vrp"){
            timeMax = 4.15 * 0.6;
        }else if(inst == "P-n40-k5.vrp"){
            timeMax = 8.42 * 0.6;
        }else if(inst == "P-n45-k5.vrp"){
            timeMax = 12.12 * 0.6;
        }else if(inst == "P-n50-k7.vrp"){
            timeMax = 15.18 * 0.6;
        }else if(inst == "P-n55-k8.vrp"){
            timeMax = 16.25 * 0.6;
        }else if(inst == "P-n60-k10.vrp"){
            timeMax = 33.57 * 0.6;
        }else if(inst == "P-n70-k10.vrp"){
            timeMax = 21.3 * 0.6;
        }
    }

    int cont = 0, cont2 = 0;
    double somaObjective = 0, bestTime, sumTime = 0;
    Solution bestSolution;
    bestSolution.objective = INF;
    vector < int >  saveUsedStations = data.usedStations;

    clock_t start = clock();
    Solution solution;
    data.usedStations = saveUsedStations;
    if(problem == "BSS"){
        solution.VNSBSS(data, vns_max, timeMax);
    }else if(problem == "EVRP"){
        solution.VNS(data, vns_max, timeMax);
    }else{
        solution.VNSTW(data, vns_max, timeMax);
    }
    
    clock_t end = clock();
    finalTime = ((double) end - start) / ((double) CLOCKS_PER_SEC);
    
    cout << "Final Solution" << endl;
    if (problem == "BSS"){
        solution.showSolutionBSS();
    }else if(problem == "EVRP"){
        solution.showSolution();
    }else{
        solution.showSolutionTW();
    }
    
    cout << "Execution time:  " << finalTime << "s." << endl;
    
	return 0;
}

int compare(const void * a, const void * b) {
    Request *idA = (Request *) a;
    Request *idB = (Request *) b;
    if (idA->distance == idB->distance)
        return 0;
    return (idA->distance > idB->distance) ? -1 : 1;
}
