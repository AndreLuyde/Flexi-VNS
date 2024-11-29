#include "solution.h"

void Solution::VNS(Data &data, int vnsMax) {
    Solution bestSolution, currentSolution, validSolution, smallVehiclesSolution;
    int iter = 0, lastShake = 0;
    double bestObjective;
    vector < int > saveUsedStations;
    
    inicialSolutionAvailable(data);
    
    Evaluation ***saveEval;
    saveEval = new Evaluation**[data.numVehicles];
    for(int k = 0; k < data.numVehicles; k++){
        saveEval[k] = new Evaluation*[data.numTotalPoints + 2];
        for(int i = 0; i < data.numTotalPoints + 2; i++){
            saveEval[k][i] = new Evaluation[data.numTotalPoints + 2];
        }
    }
    buildData(data);
    
    bestObjective       = objective;
    currentSolution     = *this;
    validSolution       = *this;
    saveUsedStations    = data.usedStations;
    
    for(int k = 0; k < data.numVehicles; k++){
        for(int i = 0; i < (int)vehicles[k].route.size(); i++){
            for(int j = 0; j < (int)vehicles[k].route.size(); j++){
                saveEval[k][i][j] = data.eval[k][i][j];
            }
        }
    }
    
    while (iter < vnsMax) {
        shake(data, iter, lastShake);

        interRVND(data, lastShake);

        if(violationTimeTour < EPS && violationBattery < EPS){
            if(validSolution.objective - objective > EPS){
                validSolution = *this;
                iter = 0;    
            }
        }

        if(bestObjective - objective > EPS ){
            bestObjective       = objective;
            currentSolution     = *this;
            saveUsedStations    = data.usedStations;
            for(int k = 0; k < data.numVehicles; k++){
                for(int i = 0; i < (int)vehicles[k].route.size(); i++){
                    for(int j = 0; j < (int)vehicles[k].route.size(); j++){
                        saveEval[k][i][j] = data.eval[k][i][j];
                    }
                }
            }
            iter            = 0;
        }else{
            iter++;
            *this               = currentSolution;
            data.usedStations   = saveUsedStations;
            for(int k = 0; k < data.numVehicles; k++){
                for(int i = 0; i < (int)vehicles[k].route.size(); i++){
                    for(int j = 0; j < (int)vehicles[k].route.size(); j++){
                        data.eval[k][i][j] = saveEval[k][i][j];
                    }
                }
            }
        }
    }
    
    int contV = 0;
    for(int i = 0; i < (int)vehicles.size(); i++){
        if(vehicles[i].route.size() > 2){
            contV++;
        }
    }
    amountVehicles = contV;
    
    for(int k = 0; k < data.numVehicles; k++){
        for(int i = 0; i < data.numTotalPoints + 2; i++){
            delete[] data.eval[k][i];
            delete[] saveEval[k][i];
        }
        delete[] data.eval[k];
        delete[] saveEval[k];
    }
    delete[] data.eval;
    delete[] saveEval;
}

void Solution::VNSBSS(Data &data, int vnsMax, double timeMax) {
    Solution bestSolution, currentSolution, validSolution, smallVehiclesSolution;
    int iter = 0, lastShake = 0;
    double bestObjective, time = 0;
    vector < int > saveUsedStations;
    clock_t start = clock();
    clock_t end;
    
    inicialSolutionAvailableBSS(data);
    
    Evaluation ***saveEval;
    saveEval = new Evaluation**[data.numVehicles];
    for(int k = 0; k < data.numVehicles; k++){
        saveEval[k] = new Evaluation*[data.numTotalPoints];
        for(int i = 0; i < data.numTotalPoints; i++){
            saveEval[k][i] = new Evaluation[data.numTotalPoints];
        }
    }
    
    buildDataBSS(data);
    
    bestObjective       = objective;
    currentSolution     = *this;
    validSolution       = *this;
    saveUsedStations    = data.usedStations;
    
    for(int k = 0; k < data.numVehicles; k++){
        for(int i = 0; i < (int)vehicles[k].route.size(); i++){
            for(int j = 0; j < (int)vehicles[k].route.size(); j++){
                saveEval[k][i][j] = data.eval[k][i][j];
            }
        }
    }
    
    while (iter < vnsMax && time < timeMax) {
        
        shakeBSS(data, iter, lastShake);

        interRVNDBSS(data, lastShake);
        
        if(violationDemand < EPS && violationBattery < EPS){
            if(validSolution.objective - objective > EPS){
                validSolution = *this;
                iter = 0;    
            }
        }

        if(bestObjective - objective > EPS ){
            bestObjective       = objective;
            currentSolution     = *this;
            saveUsedStations    = data.usedStations;
            for(int k = 0; k < data.numVehicles; k++){
                for(int i = 0; i < (int)vehicles[k].route.size(); i++){
                    for(int j = 0; j < (int)vehicles[k].route.size(); j++){
                        saveEval[k][i][j] = data.eval[k][i][j];
                    }
                }
            }
            iter            = 0;
        }else{
            iter++;
            *this               = currentSolution;
            data.usedStations   = saveUsedStations;
            for(int k = 0; k < data.numVehicles; k++){
                for(int i = 0; i < (int)vehicles[k].route.size(); i++){
                    for(int j = 0; j < (int)vehicles[k].route.size(); j++){
                        data.eval[k][i][j] = saveEval[k][i][j];
                    }
                }
            }
        }
        end     = clock();
        time = ((double) end - start) / ((double) CLOCKS_PER_SEC);
    }
    
    if(violationDemand > EPS || violationBattery > EPS){
        *this = validSolution;
    }
    
    for(int i = 0; i < (int)vehicles.size(); i++){
        if(vehicles[i].route.size() > 2){
            objective = objective - vehicles[i].vehicleCost;
        }
    }

    for(int k = 0; k < data.numVehicles; k++){
        for(int i = 0; i < data.numTotalPoints; i++){
            delete[] data.eval[k][i];
            delete[] saveEval[k][i];
        }
        delete[] data.eval[k];
        delete[] saveEval[k];
    }
    delete[] data.eval;
    delete[] saveEval;
}

void Solution::VNS(Data &data, int vnsMax, double timeMax) {
    Solution bestSolution, currentSolution, validSolution, smallVehiclesSolution;
    int iter = 0, lastShake = 0;
    double bestObjective, time = 0;
    vector < int > saveUsedStations;
    clock_t start = clock();
    clock_t end;
    
    inicialSolutionAvailable(data);

    Evaluation ***saveEval;
    saveEval = new Evaluation**[data.numVehicles];
    for(int k = 0; k < data.numVehicles; k++){
        saveEval[k] = new Evaluation*[data.numTotalPoints + 2];
        for(int i = 0; i < data.numTotalPoints + 2; i++){
            saveEval[k][i] = new Evaluation[data.numTotalPoints + 2];
        }
    }

    buildData(data);
    
    bestObjective       = objective;
    currentSolution     = *this;
    validSolution       = *this;
    saveUsedStations    = data.usedStations;
    
    for(int k = 0; k < data.numVehicles; k++){
        for(int i = 0; i < (int)vehicles[k].route.size(); i++){
            for(int j = 0; j < (int)vehicles[k].route.size(); j++){
                saveEval[k][i][j] = data.eval[k][i][j];
            }
        }
    }
    
    while (iter < vnsMax && time < timeMax) {
        
        shake(data, iter, lastShake);
        
        interRVND(data, lastShake);
        
        if(violationTimeTour < EPS && violationBattery < EPS){
            if(validSolution.objective - objective > EPS){
                validSolution = *this;
                iter = 0;    
            }
        }

        if(bestObjective - objective > EPS ){
            bestObjective       = objective;
            currentSolution     = *this;
            saveUsedStations    = data.usedStations;
            for(int k = 0; k < data.numVehicles; k++){
                for(int i = 0; i < (int)vehicles[k].route.size(); i++){
                    for(int j = 0; j < (int)vehicles[k].route.size(); j++){
                        saveEval[k][i][j] = data.eval[k][i][j];
                    }
                }
            }
            iter            = 0;
        }else{
            iter++;
            *this               = currentSolution;
            data.usedStations   = saveUsedStations;
            for(int k = 0; k < data.numVehicles; k++){
                for(int i = 0; i < (int)vehicles[k].route.size(); i++){
                    for(int j = 0; j < (int)vehicles[k].route.size(); j++){
                        data.eval[k][i][j] = saveEval[k][i][j];
                    }
                }
            }
        }
        end     = clock();
        time = ((double) end - start) / ((double) CLOCKS_PER_SEC);
    }
    
    int contV = 0;
    for(int i = 0; i < (int)vehicles.size(); i++){
        if(vehicles[i].route.size() > 2){
            contV++;
        }
    }
    amountVehicles = contV;
    for(int k = 0; k < data.numVehicles; k++){
        for(int i = 0; i < data.numTotalPoints + 2; i++){
            delete[] data.eval[k][i];
            delete[] saveEval[k][i];
        }
        delete[] data.eval[k];
        delete[] saveEval[k];
    }
    delete[] data.eval;
    delete[] saveEval;
}

void Solution::inicialSolutionAvailableBSS(Data &data){
    double batterConsumption;

    vehicles.resize(data.numVehicles);                                          //quantidade de veiculos    
    amountVehicles = 1;
    
    for (int i = 0; i < (int)vehicles.size(); i++){
        vehicles[i].demandCapacity          = data.demandCapacity;       //limite de demanda
        vehicles[i].batteryCapacity         = data.batteryCapacity;      //limite de bateria
        vehicles[i].rateConsumption         = data.rateConsumption;      //consumo de bateria por KM
        vehicles[i].vehicleCost             = 0;                  //custo do Veículo
        vehicles[i].batteryKm               = data.batteryCapacity; //data.batteryCapacity / vehicles[i].rateConsumption;
        vehicles[i].batteryUsed             = 0;
        vehicles[i].demand                  = 0;
        vehicles[i].violationTw             = 0;
        vehicles[i].violationBattery        = 0;
        vehicles[i].violationDemand         = 0;
        vehicles[i].violationBattery        = 0;
        vehicles[i].objective               = 0;
        vehicles[i].distance                = 0;
        vehicles[i].numberStationsVisited   = 0;
        vehicles[i].penality                = 0;

        vehicles[i].route.push_back(data.requests[0]);                              //iniciar com deposito
    }
    int k = 0;
    for(int i = data.numBatteryStations; i < (int)data.requests.size()-1; i++){
        if(vehicles[k].demand + data.requests[i].demand < vehicles[k].demandCapacity){
            batterConsumption = data.distances[vehicles[k].route.back().id][data.requests[i].id]; // Consumo de bateria até o cliente
            if(vehicles[k].batteryUsed + batterConsumption > vehicles[k].batteryKm){
                vehicles[k].route.push_back(data.requests[vehicles[k].route.back().id - data.numBatteryStations +2]);
                if(data.usedStations[vehicles[k].route.back().id] == 0){
                    vehicles[k].stationCost += vehicles[k].route.back().stationCost;
                }
                data.usedStations[vehicles[k].route.back().id] += 1;
                vehicles[k].batteryUsed = 0;
            }
            vehicles[k].distance        += data.distances[vehicles[k].route.back().id][data.requests[i].id];
            vehicles[k].batteryUsed         += data.distances[vehicles[k].route.back().id][data.requests[i].id];
            vehicles[k].route.push_back(data.requests[i]);
            vehicles[k].demand              += data.requests[i].demand;
        }else{
            if(k == data.numVehicles - 1){
                data.numVehicles += 1;
                Vehicle car;
                car.route.push_back(data.requests[0]);
                vehicles.push_back(car);
            }
            k++;
            amountVehicles++;
            vehicles[k].distance            += data.distances[vehicles[k].route.back().id][data.requests[i].id];
            vehicles[k].batteryUsed         += data.distances[vehicles[k].route.back().id][data.requests[i].id];
            vehicles[k].route.push_back(data.requests[i]);
            vehicles[k].demand              += data.requests[i].demand;
        }
    }

    for (int i = 0; i < (int)vehicles.size(); i++){
        batterConsumption = data.distances[vehicles[i].route.back().id][data.requests[data.numTotalPoints-1].id]; // Consumo de bateria até o deposito
        if(vehicles[i].batteryUsed + batterConsumption > vehicles[i].batteryKm){
            vehicles[i].route.push_back(data.requests[vehicles[i].route.back().id - data.numBatteryStations +2]);
            if(data.usedStations[vehicles[i].route.back().id] == 0){
                vehicles[i].stationCost += vehicles[i].route.back().stationCost;
            }
            data.usedStations[vehicles[i].route.back().id] += 1;
            vehicles[i].batteryUsed = 0;
        }

        vehicles[i].penality            = (vehicles[i].violationTw * data.betaTw) + (vehicles[i].violationDemand * data.betaDemand) + (vehicles[i].violationBattery * data.betaBattery);
        vehicles[i].distance            += data.distances[vehicles[i].route.back().id][data.numTotalPoints-1];
        vehicles[i].batteryUsed         += data.distances[vehicles[i].route.back().id][data.numTotalPoints-1];
        vehicles[i].route.push_back(data.requests[data.copyDepot]);                          //volta pro deposito com deposito

        distance                        += vehicles[i].distance;
        demand                          += vehicles[i].demand;
        penality                        += vehicles[i].penality;
        stationCost 					+= vehicles[i].stationCost;

        if((int)vehicles[i].route.size() > 2){
            vehicleCost                    += vehicles[i].vehicleCost;
        }

        vehicles[i].objective += vehicles[i].penality + vehicles[i].vehicleCost + vehicles[i].distance + vehicles[i].stationCost;
    }
    objective = distance + vehicleCost + penality + stationCost;
}

void Solution::inicialSolutionAvailable(Data &data){
    double batterConsumption;
    bool finish = false;
    vector < bool > usedCustomer;
    Vehicle car;
    car.demandCapacity          = data.demandCapacity;       //limite de demanda
    car.rateConsumption         = data.rateConsumption;      //consumo de bateria por KM
    car.batteryCapacity         = data.batteryCapacity;      //limite de bateria
    car.vehicleCost             = 0;                  //custo do Veículo
    car.batteryKm               = data.batteryCapacity;
    car.batteryUsed             = 0;
    car.demand                  = 0;
    car.violationTw             = 0;
    car.violationBattery        = 0;
    car.violationDemand         = 0;
    car.violationBattery        = 0;
    car.violationTimeTour       = 0;
    car.objective               = 0;
    car.distance            = 0;
    car.numberStationsVisited   = 0;
    car.penality                = 0;
    car.ride                    = 0;
    car.route.push_back(data.requests[0]);                              //iniciar com deposito

    vehicles.resize(data.numVehicles);                                          //quantidade de veiculos    
    amountVehicles = 1;
    
    for (int i = 0; i < (int)vehicles.size(); i++){
        vehicles[i].demandCapacity          = data.demandCapacity;       //limite de demanda
        vehicles[i].rateConsumption         = data.rateConsumption;      //consumo de bateria por KM
        vehicles[i].batteryCapacity         = data.batteryCapacity;      //limite de bateria
        vehicles[i].vehicleCost             = 0;                  //custo do Veículo
        vehicles[i].batteryKm               = data.batteryCapacity;
        vehicles[i].batteryUsed             = 0;
        vehicles[i].demand                  = 0;
        vehicles[i].violationTw             = 0;
        vehicles[i].violationBattery        = 0;
        vehicles[i].violationDemand         = 0;
        vehicles[i].violationBattery        = 0;
        vehicles[i].violationTimeTour       = 0;
        vehicles[i].objective               = 0;
        vehicles[i].distance            = 0;
        vehicles[i].numberStationsVisited   = 0;
        vehicles[i].penality                = 0;
        vehicles[i].ride                    = 0;

        vehicles[i].route.push_back(data.requests[0]);                              //iniciar com deposito
    }
    int k = 0, idCustomer = -1, idLast = 0;
    for(int i = 0; i < (int)data.requests.size(); i++){
        if(!data.requests[i].batteryStation){
            usedCustomer.push_back(false);
        }else{
            usedCustomer.push_back(true);
        }
    }
    usedCustomer[0] = true;
    usedCustomer[usedCustomer.size()-1] = true;
    
    while (!finish){
        for(int j = 0; j < (int)data.orderNodes[idLast].size(); j++){
            if(!usedCustomer[data.orderNodes[idLast][j].id] && data.requests[idLast].id != data.orderNodes[idLast][j].id){
                idCustomer = data.orderNodes[idLast][j].id;
                break;
            }
        }
        
        if(idCustomer == -1){
            finish = true;
            break;
        }

        if(vehicles[k].ride + (data.distances[vehicles[k].route.back().id][data.requests[idCustomer].id] / data.milhasMinuto) < data.maxTour){

            batterConsumption = data.distances[vehicles[k].route.back().id][data.requests[idCustomer].id]; // Consumo de bateria até o cliente
            if(vehicles[k].batteryUsed + batterConsumption > vehicles[k].batteryKm){                
                Request station = data.requests[data.closerStation[data.requests[idCustomer].id].first.second];
                vehicles[k].ride                += data.distances[vehicles[k].route.back().id][station.id] / data.milhasMinuto;
                vehicles[k].ride                += station.serviceTime;
                vehicles[k].distance        += data.distances[vehicles[k].route.back().id][station.id];
                vehicles[k].batteryUsed         += data.distances[vehicles[k].route.back().id][station.id];
                if(vehicles[k].batteryUsed > data.batteryCapacity){
                    vehicles[k].violationBattery = vehicles[k].violationBattery + (vehicles[k].batteryUsed - data.batteryCapacity);
                }
                vehicles[k].route.push_back(station);
                vehicles[k].numStations++;
                vehicles[k].batteryUsed = 0;
                idLast = station.id;
                idCustomer = -1;
            }else{
                vehicles[k].ride                += data.distances[vehicles[k].route.back().id][data.requests[idCustomer].id] / data.milhasMinuto;
                vehicles[k].ride                += data.requests[idCustomer].serviceTime;
                vehicles[k].distance        += data.distances[vehicles[k].route.back().id][data.requests[idCustomer].id];
                vehicles[k].batteryUsed         += data.distances[vehicles[k].route.back().id][data.requests[idCustomer].id];
                vehicles[k].route.push_back(data.requests[idCustomer]);
                vehicles[k].numCustomers++;
                idLast = data.requests[idCustomer].id;
                idCustomer = -1;
                usedCustomer[idLast] = true;
            }
        }else{
            if(k == data.numVehicles - 1){
                data.numVehicles += 1;
                vehicles.push_back(car);
            }
            k++;
            amountVehicles++;
            vehicles[k].ride                += data.distances[vehicles[k].route.back().id][data.requests[idCustomer].id] / data.milhasMinuto;
            vehicles[k].ride                += data.requests[idCustomer].serviceTime;
            vehicles[k].distance        += data.distances[vehicles[k].route.back().id][data.requests[idCustomer].id];
            vehicles[k].batteryUsed         += data.distances[vehicles[k].route.back().id][data.requests[idCustomer].id];
            vehicles[k].route.push_back(data.requests[idCustomer]);
            vehicles[k].numCustomers++;
            idLast = data.requests[idCustomer].id;
            idCustomer = -1;
            usedCustomer[idLast] = true;
        }
    }
    
    for (int i = 0; i < (int)vehicles.size(); i++){
        batterConsumption = data.distances[vehicles[i].route.back().id][data.requests[data.numTotalPoints-1].id]; // Consumo de bateria até o deposito
        if(vehicles[i].batteryUsed + batterConsumption > vehicles[i].batteryKm){
            Request station = data.requests[data.closerStation[vehicles[i].route.back().id].first.second];
            vehicles[i].ride                += data.distances[vehicles[i].route.back().id][station.id] / data.milhasMinuto;
            vehicles[i].ride                += station.serviceTime;
            vehicles[i].distance        += data.distances[vehicles[i].route.back().id][station.id];
            vehicles[i].batteryUsed         += data.distances[vehicles[i].route.back().id][station.id];
            if(vehicles[i].batteryUsed > data.batteryCapacity){
                vehicles[i].violationBattery = vehicles[i].violationBattery + (vehicles[i].batteryUsed - data.batteryCapacity);
            }
            vehicles[i].route.push_back(station);
            vehicles[i].numStations++;
            vehicles[i].batteryUsed = 0;
        }
        vehicles[i].ride                += data.distances[vehicles[i].route.back().id][data.requests[data.numTotalPoints-1].id] / data.milhasMinuto;
        vehicles[i].distance        += data.distances[vehicles[i].route.back().id][data.requests[data.numTotalPoints-1].id];
        vehicles[i].batteryUsed         += data.distances[vehicles[i].route.back().id][data.requests[data.numTotalPoints-1].id];

        if(vehicles[i].ride > data.maxTour){
            vehicles[i].violationTimeTour   = vehicles[i].ride - data.maxTour;
        }
        vehicles[i].penality            = (vehicles[i].violationTw * data.betaTw) + (vehicles[i].violationDemand * data.betaDemand) + (vehicles[i].violationBattery * data.betaBattery) + (vehicles[i].violationTimeTour * data.betaTour);
        
        vehicles[i].route.push_back(data.requests[data.copyDepot]);                          //volta pro deposito com deposito

        ride               				+= vehicles[i].ride;
        distance                    += vehicles[i].distance;
        penality                        += vehicles[i].penality;
        violationTimeTour               += vehicles[i].violationTimeTour;
        violationBattery                += vehicles[i].violationBattery;

        vehicles[i].objective += vehicles[i].penality + vehicles[i].vehicleCost + vehicles[i].distance + vehicles[i].stationCost;
    }
    objective = distance + vehicleCost + penality + stationCost;
}

void Solution::buildDataBSS(Data &data){
    double distance = 0, battery = 0, distanceLastStation = 0, distanceFirstStation = 0, violation = 0, violAlreadyCalc = 0;
    int j = 0, i = 0, idStation = -1;
    bool thereAreBattery = false;
    
    data.eval = new Evaluation**[data.numVehicles];
    for(int k = 0; k < data.numVehicles; k++){
        data.eval[k] = new Evaluation*[data.numTotalPoints];
        for(int i = 0; i < data.numTotalPoints; i++){
            data.eval[k][i] = new Evaluation[data.numTotalPoints];
        }
    }
    
    for(int k = 0; k < data.numVehicles; k++){
        for(i = 0; i < (int)vehicles[k].route.size() - 1; i++){
            distance            = 0;
            battery             = 0;
            distanceLastStation = 0;
            violation           = 0;
            thereAreBattery    = false;
            if(vehicles[k].route[i].batteryStation){
                idStation       = i;
            }else{
                idStation       = -1;
            }
            violAlreadyCalc = 0;
            for(j = i; j < (int)vehicles[k].route.size() - 1; j++){
                
                if(battery > data.batteryCapacity){
                    violation       += (battery - data.batteryCapacity) - violAlreadyCalc;
                    violAlreadyCalc += ((battery - data.batteryCapacity) - violAlreadyCalc);
                }else{
                    violAlreadyCalc = 0;
                }

                if(vehicles[k].route[j].batteryStation){
                    thereAreBattery    = true;
                    battery             = 0;
                    distanceLastStation = 0;
                    idStation           = j;
                }

                data.eval[k][i][j].violation        = violation;
                data.eval[k][i][j].distance         = distance;
                data.eval[k][i][j].batteryStation   = thereAreBattery;
                data.eval[k][i][j].stationToEnd     = distanceLastStation;
                data.eval[k][i][j].idStationToEnd   = idStation;
                
                distance            += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];
                battery             += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];
                distanceLastStation += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];                    
            }
            if(battery > data.batteryCapacity){
                violation       += (battery - data.batteryCapacity) - violAlreadyCalc;
                violAlreadyCalc += ((battery - data.batteryCapacity) - violAlreadyCalc);
            }else{
                violAlreadyCalc = 0;
            }

            data.eval[k][i][j].violation        = violation;
            data.eval[k][i][j].distance         = distance;
            data.eval[k][i][j].batteryStation   = thereAreBattery;
            data.eval[k][i][j].stationToEnd     = distanceLastStation;
            data.eval[k][i][j].idStationToEnd   = idStation;
        }
        data.eval[k][i][i].violation        = 0;
        data.eval[k][i][i].distance         = 0;
        data.eval[k][i][i].batteryStation   = false;
        data.eval[k][i][i].stationToEnd     = 0;
        data.eval[k][i][i].idStationToEnd   = -1;

        for(i = (int)vehicles[k].route.size() -1; i > 0; i--){
            distanceFirstStation    = 0;
            thereAreBattery        = false;
            if(vehicles[k].route[i].batteryStation){
                idStation           = i;
            }else{
                idStation           = -1;
            }
            for(j = i; j > 0; j--){
                if(vehicles[k].route[j].batteryStation){
                    distanceFirstStation    = 0;
                    idStation               = j;
                }

                data.eval[k][j][i].beginToStation   = distanceFirstStation;
                data.eval[k][j][i].idBeginToStation = idStation;
                
                distanceFirstStation    += data.distances[vehicles[k].route[j].id][vehicles[k].route[j-1].id];
            }
            data.eval[k][j][i].beginToStation   = distanceFirstStation;
            data.eval[k][j][i].idBeginToStation = idStation;
        }
        data.eval[k][i][i].beginToStation   = 0;
        data.eval[k][i][i].idBeginToStation = -1;
    }
}

void Solution::updateDataBSS(Data &data, int k){
    double distance = 0, battery = 0, distanceLastStation = 0, distanceFirstStation = 0, violation = 0, violAlreadyCalc = 0;
    bool thereAreBattery = false;
    int idStation = -1, j = 0, i = 0;
    
    for(i = 0; i < (int)vehicles[k].route.size() - 1; i++){
        distance            = 0;
        battery             = 0;
        violation           = 0;
        distanceLastStation = 0;
        thereAreBattery     = false;
        if(vehicles[k].route[i].batteryStation){
            idStation       = i;
        }else{
            idStation       = -1;
        }
        violAlreadyCalc = 0;
        for(j = i; j < (int)vehicles[k].route.size() - 1; j++){
            if(battery > data.batteryCapacity){
                violation       += (battery - data.batteryCapacity) - violAlreadyCalc;
                violAlreadyCalc += ((battery - data.batteryCapacity) - violAlreadyCalc);
            }else{
                violAlreadyCalc = 0;
            }
            
            if(vehicles[k].route[j].batteryStation){
                thereAreBattery    = true;
                battery             = 0;
                distanceLastStation = 0;
                idStation           = j;
            }

            data.eval[k][i][j].violation        = violation;
            data.eval[k][i][j].distance         = distance;
            data.eval[k][i][j].batteryStation   = thereAreBattery;
            data.eval[k][i][j].stationToEnd     = distanceLastStation;
            data.eval[k][i][j].idStationToEnd   = idStation;
            
            distance            += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];
            battery             += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];
            distanceLastStation += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];                    
        }
        if(battery > data.batteryCapacity){
            violation       += (battery - data.batteryCapacity) - violAlreadyCalc;
            violAlreadyCalc += ((battery - data.batteryCapacity) - violAlreadyCalc);
        }else{
            violAlreadyCalc = 0;
        }

        data.eval[k][i][j].violation        = violation;
        data.eval[k][i][j].distance         = distance;
        data.eval[k][i][j].batteryStation   = thereAreBattery;                
        data.eval[k][i][j].stationToEnd     = distanceLastStation;
        data.eval[k][i][j].idStationToEnd   = idStation;
    }
    data.eval[k][i][i].violation        = 0;
    data.eval[k][i][i].distance         = 0;
    data.eval[k][i][i].batteryStation   = false;
    data.eval[k][i][i].stationToEnd     = 0;
    data.eval[k][i][i].idStationToEnd   = -1;

    for(i = (int)vehicles[k].route.size() -1; i > 0; i--){
        distanceFirstStation    = 0;

        if(vehicles[k].route[i].batteryStation){
            idStation           = i;
        }else{
            idStation           = -1;
        }
        for(j = i; j > 0; j--){
            if(vehicles[k].route[j].batteryStation){
                distanceFirstStation    = 0;
                idStation               = j;
            }

            data.eval[k][j][i].beginToStation   = distanceFirstStation;
            data.eval[k][j][i].idBeginToStation = idStation;

            distanceFirstStation    += data.distances[vehicles[k].route[j].id][vehicles[k].route[j-1].id];
        }
        data.eval[k][j][i].beginToStation   = distanceFirstStation;
        data.eval[k][j][i].idBeginToStation = idStation;
    }
    data.eval[k][i][i].beginToStation   = 0;
    data.eval[k][i][i].idBeginToStation = -1;
}

void Solution::buildData(Data &data){
    double distance = 0, battery = 0, distanceLastStation = 0, distanceFirstStation = 0, violation = 0, violAlreadyCalc = 0, totalViolation = 0;
    int j = 0, i = 0, idStation = -1;
    bool thereAreBattery = false;
    
    data.eval = new Evaluation**[data.numVehicles];
    for(int k = 0; k < data.numVehicles; k++){
        data.eval[k] = new Evaluation*[data.numTotalPoints + 2];
        for(int i = 0; i < data.numTotalPoints + 2; i++){
            data.eval[k][i] = new Evaluation[data.numTotalPoints + 2];
        }
    }
    
    for(int k = 0; k < data.numVehicles; k++){
        for(i = 0; i < (int)vehicles[k].route.size() - 1; i++){
            distance            = 0;
            battery             = 0;
            distanceLastStation = 0;
            violation           = 0;
            thereAreBattery    = false;
            if(vehicles[k].route[i].batteryStation){
                idStation       = i;
            }else{
                idStation       = -1;
            }

            for(j = i; j < (int)vehicles[k].route.size() - 1; j++){
            
                if(battery > data.batteryCapacity){
                    violation       = (battery - data.batteryCapacity) - violAlreadyCalc;
                    violAlreadyCalc += violation;
                    totalViolation  += violation;
                }else{
                    violAlreadyCalc = 0;
                }

                if(vehicles[k].route[j].batteryStation){
                    thereAreBattery    = true;
                    battery             = 0;
                    distanceLastStation = 0;
                    idStation           = j;
                }

                data.eval[k][i][j].violation        = totalViolation;
                data.eval[k][i][j].distance         = distance;
                data.eval[k][i][j].batteryStation   = thereAreBattery;
                data.eval[k][i][j].stationToEnd     = distanceLastStation;
                data.eval[k][i][j].idStationToEnd   = idStation;
                
                distance            += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];
                battery             += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];
                distanceLastStation += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];                    
            }
            if(battery > data.batteryCapacity){
                violation       = (battery - data.batteryCapacity) - violAlreadyCalc;
                violAlreadyCalc += violation;
                totalViolation  += violation;
            }else{
                violAlreadyCalc = 0;
            }

            data.eval[k][i][j].violation        = totalViolation;
            data.eval[k][i][j].distance         = distance;
            data.eval[k][i][j].batteryStation   = thereAreBattery;
            data.eval[k][i][j].stationToEnd     = distanceLastStation;
            data.eval[k][i][j].idStationToEnd   = idStation;
        }
        data.eval[k][i][i].violation        = 0;
        data.eval[k][i][i].distance         = 0;
        data.eval[k][i][i].batteryStation   = false;
        data.eval[k][i][i].stationToEnd     = 0;
        data.eval[k][i][i].idStationToEnd   = -1;

        for(i = (int)vehicles[k].route.size() -1; i > 0; i--){
            distanceFirstStation    = 0;
            thereAreBattery        = false;
            if(vehicles[k].route[i].batteryStation){
                idStation           = i;
            }else{
                idStation           = -1;
            }
            for(j = i; j > 0; j--){
                if(vehicles[k].route[j].batteryStation){
                    distanceFirstStation    = 0;
                    idStation               = j;
                }

                data.eval[k][j][i].beginToStation   = distanceFirstStation;
                data.eval[k][j][i].idBeginToStation = idStation;
                
                distanceFirstStation    += data.distances[vehicles[k].route[j].id][vehicles[k].route[j-1].id];
            }
            data.eval[k][j][i].beginToStation   = distanceFirstStation;
            data.eval[k][j][i].idBeginToStation = idStation;
        }
        data.eval[k][i][i].beginToStation   = 0;
        data.eval[k][i][i].idBeginToStation = -1;
    }
}

void Solution::updateData(Data &data, int k){
    double distance = 0, battery = 0, distanceLastStation = 0, distanceFirstStation = 0, violation = 0, violAlreadyCalc = 0, totalViolation = 0;
    bool thereAreBattery = false;
    int idStation = -1, j = 0, i = 0;
    
    for(i = 0; i < (int)vehicles[k].route.size() - 1; i++){
        distance            = 0;
        battery             = 0;
        violation           = 0;
        distanceLastStation = 0;
        thereAreBattery     = false;
        if(vehicles[k].route[i].batteryStation){
            idStation       = i;
        }else{
            idStation       = -1;
        }
        for(j = i; j < (int)vehicles[k].route.size() - 1; j++){
            if(battery > data.batteryCapacity){
                violation       = (battery - data.batteryCapacity) - violAlreadyCalc;
                violAlreadyCalc += violation;
                totalViolation  += violation;
            }else{
                violAlreadyCalc = 0;
            }
            
            if(vehicles[k].route[j].batteryStation){
                thereAreBattery    = true;
                battery             = 0;
                distanceLastStation = 0;
                idStation           = j;
            }

            data.eval[k][i][j].violation        = totalViolation;
            data.eval[k][i][j].distance         = distance;
            data.eval[k][i][j].batteryStation   = thereAreBattery;
            data.eval[k][i][j].stationToEnd     = distanceLastStation;
            data.eval[k][i][j].idStationToEnd   = idStation;
            
            distance            += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];
            battery             += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];
            distanceLastStation += data.distances[vehicles[k].route[j].id][vehicles[k].route[j+1].id];                    
        }
        if(battery > data.batteryCapacity){
            violation       = (battery - data.batteryCapacity) - violAlreadyCalc;
            violAlreadyCalc += violation;
            totalViolation  += violation;
        }else{
            violAlreadyCalc = 0;
        }

        data.eval[k][i][j].violation        = totalViolation;
        data.eval[k][i][j].distance         = distance;
        data.eval[k][i][j].batteryStation   = thereAreBattery;                
        data.eval[k][i][j].stationToEnd     = distanceLastStation;
        data.eval[k][i][j].idStationToEnd   = idStation;
    }
    data.eval[k][i][i].violation        = 0;
    data.eval[k][i][i].distance         = 0;
    data.eval[k][i][i].batteryStation   = false;
    data.eval[k][i][i].stationToEnd     = 0;
    data.eval[k][i][i].idStationToEnd   = -1;

    for(i = (int)vehicles[k].route.size() -1; i > 0; i--){
        distanceFirstStation    = 0;

        if(vehicles[k].route[i].batteryStation){
            idStation           = i;
        }else{
            idStation           = -1;
        }
        for(j = i; j > 0; j--){
            if(vehicles[k].route[j].batteryStation){
                distanceFirstStation    = 0;
                idStation               = j;
            }

            data.eval[k][j][i].beginToStation   = distanceFirstStation;
            data.eval[k][j][i].idBeginToStation = idStation;

            distanceFirstStation    += data.distances[vehicles[k].route[j].id][vehicles[k].route[j-1].id];
        }
        data.eval[k][j][i].beginToStation   = distanceFirstStation;
        data.eval[k][j][i].idBeginToStation = idStation;
    }
    data.eval[k][i][i].beginToStation   = 0;
    data.eval[k][i][i].idBeginToStation = -1;
}

void Solution::shake(Data &data, int intensity, int &lastShake){
    int iter = 0, maxIntensity = 3;
    
    if(intensity > maxIntensity){
        intensity   = maxIntensity;
    }

    while(iter < intensity){
            uniform_int_distribution<int> distribution(0, 100);
            int select = distribution(generator);

            if(select < 33.33){
                shakeRelocation(data);
                lastShake = 1;
            }else if(select < 66.66){
                shakeSwap(data);
                lastShake = 3;
            }else{
                shakeRemoveStations(data);
            }
        iter++;
    }
}

void Solution::shakeSwap(Data &data){
    int v1, v2, r1, r2, k = 0;
    Vehicle car1, car2;

    if((int)vehicles.size() > 1){
    	uniform_int_distribution<int> distribution(0, (int)vehicles.size()-1);
    	do{
    		v1  = distribution(generator);
    		v2  = distribution(generator);
    		if(k > 20){
    			shakeRelocation(data);
    			return;
    		}
    		k++;
    	}while(v1 == v2 || (int)vehicles[v1].route.size() < 3 || (int)vehicles[v2].route.size() < 3);

    	uniform_int_distribution<int> distribution2(1,(int)vehicles[v1].route.size()-2);
	    r1      = distribution2(generator);
	    uniform_int_distribution<int> distribution3(1,(int)vehicles[v2].route.size()-2);
	    r2      = distribution3(generator);
		
        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;
        // =========================================Update Distance==========================================================================
        distance                = distance - (vehicles[v1].distance + vehicles[v2].distance);
        vehicles[v1].distance   = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].distance;
        vehicles[v2].distance   = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].distance;
        distance                = distance + vehicles[v1].distance + vehicles[v2].distance;

        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || vehicles[v2].route[r2].batteryStation || data.eval[v1][r1+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            if(!vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].beginToStation;
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                
                bat = data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].beginToStation;
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery1 = max(vehicles[v1].distance - data.batteryCapacity, 0.0);
        }
        
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || data.eval[v2][r2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(!vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                
                bat = data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(vehicles[v2].distance - data.batteryCapacity, 0.0);
        }

        violationBattery = violationBattery + (vBattery1 + vBattery2) - (vehicles[v1].violationBattery + vehicles[v2].violationBattery);

        vehicles[v1].violationBattery = vBattery1;
        vehicles[v2].violationBattery = vBattery2;

        // ======================================Update Ride =====================================================
        if(vehicles[v1].route[r1].batteryStation){
            vehicles[v1].numStations--;
            vehicles[v2].numStations++;
        }else{
            vehicles[v1].numCustomers--;
            vehicles[v2].numCustomers++;
        }
        
        if(vehicles[v2].route[r2].batteryStation){
            vehicles[v2].numStations--;
            vehicles[v1].numStations++;
        }else{
            vehicles[v2].numCustomers--;
            vehicles[v1].numCustomers++;
        }

        double oldRide1 = vehicles[v1].ride;
        double oldRide2 = vehicles[v2].ride;

        vehicles[v1].ride = (vehicles[v1].distance / data.milhasMinuto) + (vehicles[v1].numCustomers * data.customerService) + (vehicles[v1].numStations * data.stationService);
        vehicles[v2].ride = (vehicles[v2].distance / data.milhasMinuto) + (vehicles[v2].numCustomers * data.customerService) + (vehicles[v2].numStations * data.stationService);
        
        ride = ride - (oldRide1 + oldRide2) + (vehicles[v1].ride + vehicles[v2].ride);

        double oldViolationTimeTour1 = vehicles[v1].violationTimeTour;
        double oldViolationTimeTour2 = vehicles[v2].violationTimeTour;
        
        vehicles[v1].violationTimeTour = max(vehicles[v1].ride - data.maxTour, 0.0);
        vehicles[v2].violationTimeTour = max(vehicles[v2].ride - data.maxTour, 0.0);

        violationTimeTour = violationTimeTour - (oldViolationTimeTour1 + oldViolationTimeTour2) + (vehicles[v1].violationTimeTour + vehicles[v2].violationTimeTour);

        // =========================================================================================================

        penality    = (violationTimeTour * data.betaTour) + (violationBattery * data.betaBattery);
        objective   = distance + penality + stationCost;

        swap(vehicles[v1].route[r1], vehicles[v2].route[r2]);
        
        //======================================Update data===================================================================
        updateData(data, v1);
        updateData(data, v2);
    }
}

void Solution::shakeRelocation(Data &data){
    int v1, v2, r1, r2;
    Vehicle car1, car2;
    bool moreThanOne = false;

    if((int)vehicles.size() > 1){
        uniform_int_distribution<int> distribution(0, (int)vehicles.size()-1);
        do{
	        v1  = distribution(generator);
	    }while((int)vehicles[v1].route.size() < 3);
        if(v1 == 0){
            v2 = 1;
        }else{
            v2 = v1-1;
        }
        moreThanOne = true;
    }else{
    	uniform_int_distribution<int> distribution(0, (int)vehicles.size()-1);
    	
    	do{
	        v1  = distribution(generator);
	        v2  = distribution(generator);
	    }while(v1 == v2 || (int)vehicles[v1].route.size() < 3);
    }

    uniform_int_distribution<int> distribution2(1,(int)vehicles[v1].route.size()-2);
    r1 = distribution2(generator);

    if((int)vehicles[v2].route.size() < 3){
        r2 = 1;
    }else{
        uniform_int_distribution<int> distribution3(1,(int)vehicles[v2].route.size()-2);
        r2 = distribution3(generator);
    }

    int depot2V1 = vehicles[v1].route.size()-1;
    int depot2V2 = vehicles[v2].route.size()-1;
    // =========================================Update Distance==========================================================================
    distance                = distance - (vehicles[v1].distance + vehicles[v2].distance);
    vehicles[v1].distance   = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].distance;
    vehicles[v2].distance   = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].distance;
    distance                = distance + vehicles[v1].distance + vehicles[v2].distance;

    //===========================================Update Battery======================================================================
    double vBattery1 = 0;
    double vBattery2 = 0;
    if(data.eval[v1][0][r1-1].batteryStation || data.eval[v1][r1+1][depot2V1].batteryStation){
        if(data.eval[v1][0][r1-1].batteryStation){
            vBattery1   += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
        }
        if(data.eval[v1][r1+1][depot2V1].batteryStation){
            vBattery1   += data.eval[v1][data.eval[v1][r1+1][depot2V1].idBeginToStation][depot2V1].violation;
        }
        double bat      = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].beginToStation;
        vBattery1       += max(bat - data.batteryCapacity, 0.0);
    }else{
        vBattery1       = max(vehicles[v1].distance - data.batteryCapacity, 0.0);
    }

    if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || data.eval[v2][r2][depot2V2].batteryStation){
        if(data.eval[v2][0][r2-1].batteryStation){
            vBattery2   += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
        }
        if(data.eval[v2][r2][depot2V2].batteryStation){
            vBattery2   += data.eval[v2][data.eval[v2][r2][depot2V2].idBeginToStation][depot2V2].violation;
        }
        if(!vehicles[v1].route[r1].batteryStation){
            double bat  = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
            vBattery2   += max(bat - data.batteryCapacity, 0.0);
        }else{
            double bat  = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
            vBattery2   += max(bat - data.batteryCapacity, 0.0);
                        
            bat         = data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
            vBattery2   += max(bat - data.batteryCapacity, 0.0);
        }
    }else{
        vBattery2       = max(vehicles[v2].distance - data.batteryCapacity, 0.0);
    }

    violationBattery                = violationBattery + (vBattery1 + vBattery2) - (vehicles[v1].violationBattery + vehicles[v2].violationBattery);
    vehicles[v1].violationBattery   = vBattery1;
    vehicles[v2].violationBattery   = vBattery2;

    // ======================================Update Ride =====================================================
    double oldRide1 = vehicles[v1].ride;
    double oldRide2 = vehicles[v2].ride;

    if(vehicles[v1].route[r1].batteryStation){
        vehicles[v1].numStations--;
        vehicles[v2].numStations++;
    }else{
        vehicles[v1].numCustomers--;
        vehicles[v2].numCustomers++;
    }

    vehicles[v1].ride = (vehicles[v1].distance / data.milhasMinuto) + (vehicles[v1].numCustomers * data.customerService) + (vehicles[v1].numStations * data.stationService);
    vehicles[v2].ride = (vehicles[v2].distance / data.milhasMinuto) + (vehicles[v2].numCustomers * data.customerService) + (vehicles[v2].numStations * data.stationService);
    
    ride = ride - (oldRide1 + oldRide2) + (vehicles[v1].ride + vehicles[v2].ride);

    double oldViolationTimeTour1 = vehicles[v1].violationTimeTour;
    double oldViolationTimeTour2 = vehicles[v2].violationTimeTour;

    vehicles[v1].violationTimeTour = max(vehicles[v1].ride - data.maxTour, 0.0);
    vehicles[v2].violationTimeTour = max(vehicles[v2].ride - data.maxTour, 0.0);

    violationTimeTour = violationTimeTour - (oldViolationTimeTour1 + oldViolationTimeTour2) + (vehicles[v1].violationTimeTour + vehicles[v2].violationTimeTour);

    // =========================================================================================================
    vehicles[v2].route.insert(vehicles[v2].route.begin() + r2, vehicles[v1].route[r1]);
    vehicles[v1].route.erase(vehicles[v1].route.begin() + r1);
    
    if(moreThanOne){
        if((int)vehicles[v1].route.size() < 3){
            amountVehicles--;
        }
        if((int)vehicles[v2].route.size() == 3){
            amountVehicles++;
        }
    }

    penality    = (violationTimeTour * data.betaTour) + (violationBattery * data.betaBattery);
    objective   = distance + penality + stationCost;

    //======================================Update data===================================================================
    updateData(data, v1);
    updateData(data, v2);
}

void Solution::shakeRemoveStations(Data &data){
    int idStationRemove;
	vector<int> stations;

    for(int i = 0; i < (int)vehicles.size(); i++){
        if(vehicles[i].route.size() > 2){
            if(data.eval[i][0][vehicles[i].route.size()-1].batteryStation){
                for(int j = 1; j < (int)vehicles[i].route.size() -1; j++){
                    if(vehicles[i].route[j].batteryStation){
                        stations.push_back(vehicles[i].route[j].id);
                    }
                }
            }
        }
    }
    if(stations.size() == 0){
        uniform_int_distribution<int> distribution(0, 100);
        int select = distribution(generator);

        if(select < 50){
            shakeRelocation(data);
        }else{
            shakeSwap(data);
        }
        return;
    }
    uniform_int_distribution<int> distribution(0, stations.size()-1);

    idStationRemove = distribution(generator);
    vector < pair < int, int > > idRemove;

    for(int i = 0; i < (int)vehicles.size(); i++){
        if((int)vehicles[i].route.size() > 2){
            if(data.eval[i][0][(int)vehicles[i].route.size()-1].batteryStation){
                for(int j = 1; j < (int)vehicles[i].route.size() - 1; j++){
                    if(vehicles[i].route[j].id == stations[idStationRemove]){
                        idRemove.push_back(make_pair(i, j));
                    }
                }
            }
        }
    }

    for(int k = (int)idRemove.size() -1; k >= 0; k--){
        int v1      = idRemove[k].first;
        int r1      = idRemove[k].second;
        int depotV1 = vehicles[v1].route.size() - 1;

        double oldDistance          = vehicles[v1].distance;
        vehicles[v1].distance   = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depotV1].distance;
        distance                = distance + vehicles[v1].distance - oldDistance;
        
        double vBattery1 = 0;
        if(data.eval[v1][0][r1-1].batteryStation){
            vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
        }
        if(data.eval[v1][r1+1][depotV1].batteryStation){
            vBattery1 += data.eval[v1][data.eval[v1][r1+1][depotV1].idBeginToStation][depotV1].violation;
        }
        double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depotV1].beginToStation;
        vBattery1 += max(bat - data.batteryCapacity, 0.0);

        violationBattery                = violationBattery + vBattery1 - vehicles[v1].violationBattery;
        vehicles[v1].violationBattery   = vBattery1;
        
        // ======================================Update Ride =====================================================
        double oldRide1 = vehicles[v1].ride;

        vehicles[v1].numStations--;
        vehicles[v1].ride = (vehicles[v1].distance / data.milhasMinuto) + (vehicles[v1].numCustomers * data.customerService) + (vehicles[v1].numStations * data.stationService);
        
        ride = ride - oldRide1 + vehicles[v1].ride;

        double oldViolationTimeTour1 = vehicles[v1].violationTimeTour;

        vehicles[v1].violationTimeTour = max(vehicles[v1].ride - data.maxTour, 0.0);

        violationTimeTour = violationTimeTour - oldViolationTimeTour1 + vehicles[v1].violationTimeTour;

        // =========================================================================================================

        penality     = (violationTimeTour * data.betaTour)  + (violationBattery * data.betaBattery);
        objective    = distance + penality + stationCost;

        vehicles[v1].route.erase(vehicles[v1].route.begin() + r1);

        if((int)vehicles[v1].route.size() < 3){
            amountVehicles--;
        }

        updateData(data, v1);
    }
}

void Solution::interRVND(Data &data, int lastShake){
    double bestObjective = objective;
    int k = 0;
    vector < pair < int, int > > idRemove;
    vector<int> neighbors;
    neighbors.push_back(1);      //Realocação 1
    neighbors.push_back(2);      //Realocação 2
    neighbors.push_back(3);      //Swap
    neighbors.push_back(4);      //Swap 2
    neighbors.push_back(5);      //remove Station
    neighbors.push_back(6);      // Swap 2x1
    neighbors.push_back(7);      // Adicionar Estação
    if(lastShake != 0){
        neighbors.erase(neighbors.begin() + lastShake-1);
    }
    shuffle(neighbors.begin(), neighbors.end(), generator);
    while(true){
        if (k >= (int)neighbors.size()){break;}

        if(neighbors[k] == 1){
            interRelocation(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if(neighbors[k] == 2){
            interRelocation2(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if(neighbors[k] == 3){
            interSwap(data);
            
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 4){
            interSwap2(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 5){
            removeStation(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 6){
            interSwap2x1(data);
            
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 7){
            addStation(data);
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else{
            k++;
        }
    }
}

void Solution::addStation(Data &data){
    bool improve = false;
    double bestObjective = objective, newObjective, newPenality;
    double newViolationBattery = violationBattery, newRide = ride, bestVBt1 = 0, newDistance = distance, newViolationTimeTour = violationTimeTour;
    double bestRide1 = 0, bestDistance1 = 0, bestVTimeT1 = 0;
    int v1, r1, r2, bestV1 = 0, bestR1 = 0, bestR2 = 0;
    
    for(int i = 1; i <= data.numBatteryStations; i++){
        for(int k = 0; k < (int)vehicles.size(); k++){
            if((int)vehicles[k].route.size() > 2){
                for(int j = 1; j < (int)vehicles[k].route.size() - 1; j++){
                    if((data.requests[i].id != vehicles[k].route[j-1].id) && (data.requests[i].id != vehicles[k].route[j].id)){
                        newRide                 = ride;
                        newDistance             = distance;
                        newViolationBattery     = violationBattery;
                        newViolationTimeTour    = violationTimeTour;
                        r2 = i;
                        v1 = k;
                        r1 = j;
                        int depotV1 = vehicles[v1].route.size() - 1;
                        
                        double newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][data.requests[r2].id] + data.distances[data.requests[r2].id][vehicles[v1].route[r1].id] + data.eval[v1][r1][depotV1].distance;
                        newDistance = newDistance + newDistance1 - vehicles[v1].distance;
                        
                        double vBattery1 = 0;
                        if(data.eval[v1][0][r1-1].batteryStation){
                            vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
                        }
                        if(data.eval[v1][r1][depotV1].batteryStation){
                            vBattery1 += data.eval[v1][data.eval[v1][r1][depotV1].idBeginToStation][depotV1].violation;
                        }
                        double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][data.requests[r2].id];
                        vBattery1 += max(bat - data.batteryCapacity, 0.0);

                        bat = data.eval[v1][r1][depotV1].beginToStation + data.distances[data.requests[r2].id][vehicles[v1].route[r1].id];
                        vBattery1 += max(bat - data.batteryCapacity, 0.0);
                        
                        newViolationBattery = newViolationBattery + vBattery1 - vehicles[v1].violationBattery;
                        
                        // ======================================Update Ride =====================================================
                        double newRide1;

                        int numStations1     = vehicles[v1].numStations;
                        numStations1++;
                        
                        newRide1 = (newDistance1 / data.milhasMinuto) + (vehicles[v1].numCustomers * data.customerService) + (numStations1 * data.stationService);
                        
                        newRide = newRide + newRide1 - vehicles[v1].ride;

                        double newViolationTimeTour1;
                        newViolationTimeTour1 = max(newRide1 - data.maxTour, 0.0);

                        newViolationTimeTour = newViolationTimeTour - vehicles[v1].violationTimeTour + newViolationTimeTour1;

                        // ===========================================Update custSolution====================================================================
                        newPenality     = (newViolationBattery * data.betaBattery)  + (violationTimeTour * data.betaTour);
                        newObjective    = newDistance + vehicleCost + newPenality + stationCost;

                        if(bestObjective - newObjective > EPS){
                            bestRide1       = newRide1;
                            bestDistance1   = newDistance1;
                            bestVTimeT1     = newViolationTimeTour1;
                            bestV1          = v1;
                            bestR1          = r1;
                            bestR2          = r2;
                            bestVBt1        = vBattery1;
                            improve         = true;
                            bestObjective   = newObjective;
                        }
                    }
                }
            }
        }
    }

    if(improve){
        distance                    = distance - vehicles[bestV1].distance;
        vehicles[bestV1].distance   = bestDistance1;
        distance                    = distance + vehicles[bestV1].distance;

        violationBattery                    = violationBattery - vehicles[bestV1].violationBattery;
        vehicles[bestV1].violationBattery   = bestVBt1;
        violationBattery                    = violationBattery + bestVBt1;

        ride                    = ride - vehicles[bestV1].ride;
        vehicles[bestV1].ride   = bestRide1;
        ride                    = ride + vehicles[bestV1].ride;
        
        vehicles[bestV1].numStations++;

        violationTimeTour                    = violationTimeTour - vehicles[bestV1].violationTimeTour;
        vehicles[bestV1].violationTimeTour   = bestVTimeT1;
        violationTimeTour                    = violationTimeTour + vehicles[bestV1].violationTimeTour;

        penality = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective = distance + penality + vehicleCost + stationCost;

        vehicles[bestV1].route.insert(vehicles[bestV1].route.begin() + bestR1, data.requests[bestR2]);
        
        //======================================Update data===================================================================
        updateData(data, bestV1);

        double oldRide              = vehicles[bestV1].ride;
        double oldDistance          = vehicles[bestV1].distance;
        double oldViolationBattery  = vehicles[bestV1].violationBattery;
        double oldViolationTimeTour = vehicles[bestV1].violationTimeTour;

        vehicles[bestV1].intraRVND(data, bestV1);

        ride                = ride              + vehicles[bestV1].ride              - oldRide;
        distance        = distance      + vehicles[bestV1].distance      - oldDistance;
        violationBattery    = violationBattery  + vehicles[bestV1].violationBattery  - oldViolationBattery;
        violationTimeTour   = violationTimeTour + vehicles[bestV1].violationTimeTour - oldViolationTimeTour;
        
        penality            = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::removeStation(Data &data){
    int v1, r1, bestV1, bestR1;
    bool improve = false;
    double newObjective, bestObjective = objective, newPenality, newVehiclesCost = vehicleCost, newStationCost = stationCost;
    double newViolationBattery = violationBattery, newRide = ride, bestVBt1, newViolationTimeTour = violationTimeTour, newDistance = distance;
    double bestRide1 = 0, bestDistance1 = 0, bestVTimeT1 = 0;
    vector < pair < int, int > > idRemove;
    for(int i = 0; i < (int)vehicles.size(); i++){
        if((int)vehicles[i].route.size() > 2){
            for(int j = 1; j < (int)vehicles[i].route.size() - 1; j++){
                if(vehicles[i].route[j].batteryStation){
                    if(vehicles[i].route[j-1].id == vehicles[i].route[j].id){
                        idRemove.push_back(make_pair(i, j-1));
                    }
                }
            }
        }
    }
    
    if(idRemove.size() > 0){
        int v = idRemove[(int)idRemove.size() -1].first;
        for(int k = (int)idRemove.size() -1; k >= 0; k--){
            if(v != idRemove[k].first){
                updateData(data, v);
                v = idRemove[k].first;
            }

            data.usedStations[vehicles[idRemove[k].first].route[idRemove[k].second].id] = data.usedStations[vehicles[idRemove[k].first].route[idRemove[k].second].id] - 1;
            vehicles[idRemove[k].first].route.erase(vehicles[idRemove[k].first].route.begin() + idRemove[k].second);
            if(vehicles[idRemove[k].first].route.size() < 3){
                amountVehicles--;
            }
        }
        updateData(data, v);
    }
    
    for(int i = 0; i < (int)vehicles.size(); i++){
        if((int)vehicles[i].route.size() > 2){
            for(int j = 1; j < (int)vehicles[i].route.size() - 1; j++){
                newRide = ride;
                newDistance = distance;
                newViolationBattery = violationBattery;
                newViolationTimeTour = violationTimeTour;

                if(vehicles[i].route[j].batteryStation){
                    v1 = i;
                    r1 = j;
                    int depotV1 = vehicles[v1].route.size() - 1;
                    
                    double newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depotV1].distance;
                    newDistance         = newDistance + newDistance1 - vehicles[v1].distance;
                    
                    double vBattery1 = 0;
                    if(data.eval[v1][0][r1-1].batteryStation){
                        vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
                    }
                    if(data.eval[v1][r1+1][depotV1].batteryStation){
                        vBattery1 += data.eval[v1][data.eval[v1][r1+1][depotV1].idBeginToStation][depotV1].violation;
                    }
                    double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depotV1].beginToStation;
                    vBattery1 += max(bat - data.batteryCapacity, 0.0);

                    newViolationBattery = newViolationBattery + vBattery1 - vehicles[v1].violationBattery;

                    // ======================================Update Ride =====================================================
                    double newRide1;

                    int numStations1     = vehicles[v1].numStations;
                    numStations1--;
                    
                    newRide1 = (newDistance1 / data.milhasMinuto) + (vehicles[v1].numCustomers * data.customerService) + (numStations1 * data.stationService);
                    
                    newRide = newRide + newRide1 - vehicles[v1].ride;

                    double newViolationTimeTour1;
                    newViolationTimeTour1 = max(newRide1 - data.maxTour, 0.0);
                    
                    newViolationTimeTour = newViolationTimeTour - vehicles[v1].violationTimeTour + newViolationTimeTour1;

                    // ===========================================Update custSolution====================================================================
                    newPenality     = (newViolationBattery * data.betaBattery)  + (violationTimeTour * data.betaTour);
                    newObjective    = newDistance + newVehiclesCost + newPenality + newStationCost;

                    if(bestObjective - newObjective > EPS){
                        bestRide1       = newRide1;
                        bestDistance1   = newDistance1;
                        bestVTimeT1     = newViolationTimeTour1;
                        bestV1          = i;
                        bestR1          = j;
                        bestVBt1        = vBattery1;
                        improve         = true;
                        bestObjective   = newObjective;
                    }
                }
            }
        }
    }

    if(improve){
        distance                    = distance - vehicles[bestV1].distance;
        vehicles[bestV1].distance   = bestDistance1;
        distance                    = distance + vehicles[bestV1].distance;

        violationBattery                    = violationBattery - vehicles[bestV1].violationBattery;
        vehicles[bestV1].violationBattery   = bestVBt1;
        violationBattery                    = violationBattery + bestVBt1;

        ride                    = ride - vehicles[bestV1].ride;
        vehicles[bestV1].ride   = bestRide1;
        ride                    = ride + vehicles[bestV1].ride;
        
        vehicles[bestV1].numStations--;

        violationTimeTour                    = violationTimeTour - vehicles[bestV1].violationTimeTour;
        vehicles[bestV1].violationTimeTour   = bestVTimeT1;
        violationTimeTour                    = violationTimeTour + vehicles[bestV1].violationTimeTour;

        penality  = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective = distance + penality + stationCost + vehicleCost;

        vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1);
        
        if((int)vehicles[bestV1].route.size() < 3){
            amountVehicles--;
        }

        //======================================Update data===================================================================
        updateData(data, bestV1);

        double oldRide              = vehicles[bestV1].ride;
        double oldDistance          = vehicles[bestV1].distance;
        double oldViolationBattery  = vehicles[bestV1].violationBattery;
        double oldViolationTimeTour = vehicles[bestV1].violationTimeTour;
        
        vehicles[bestV1].intraRVND(data, bestV1);
        
        ride                = ride              + vehicles[bestV1].ride              - oldRide;
        distance        = distance      + vehicles[bestV1].distance      - oldDistance;
        violationBattery    = violationBattery  + vehicles[bestV1].violationBattery  - oldViolationBattery;
        violationTimeTour   = violationTimeTour + vehicles[bestV1].violationTimeTour - oldViolationTimeTour;

        penality            = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::interRelocation(Data &data){
    int bestV1, bestV2, bestR1, bestR2;
    double bestVTimeT1, bestVTimeT2, bestDistance1, bestDistance2, bestVBt1 = 0, bestVBt2 = 0, bestRide1 = 0, bestRide2 = 0;
    double newPenality, bestObjective = objective, newObjective;
    double newViolationBattery  = violationBattery;
    double newViolationTimeTour = violationTimeTour;
    double newVehicleCost       = vehicleCost;
    double newDistance          = distance;
    double newRide              = ride;
    bool improve = true;

    vector < pair < pair < int, int >, pair < int, int > > > vt;
    improve = false;
    for(int i = 0; i < (int)vehicles.size();i++) {
        if((int)vehicles[i].route.size() > 2){
            for(int j = 0; j < (int)vehicles.size(); j++) {
                if(i != j){
                    for(int r1 = 1; r1 < (int)vehicles[i].route.size()-1; r1++) {
                        if((int)vehicles[j].route.size() < 3){
                            vt.push_back(make_pair(make_pair(i, r1), make_pair(j, 1)));
                        }else{
                            for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                                vt.push_back(make_pair(make_pair(i, r1), make_pair(j, r2)));
                            }                        
                        }
                    }
                }
            }
        }
    }

    shuffle(vt.begin(), vt.end(), generator);
    for(int i = 0; i < (int)vt.size(); i++){
        newViolationBattery     = violationBattery;
        newViolationTimeTour    = violationTimeTour;
        newDistance             = distance;
        newRide                 = ride;

        int v1 = vt[i].first.first;
        int v2 = vt[i].second.first;
        int r1 = vt[i].first.second;
        int r2 = vt[i].second.second;
        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;

        // =========================================Update Distance==========================================================================
        double newDistance1;
        double newDistance2;
        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id]   + data.eval[v1][r1+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id]     + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || data.eval[v1][r1+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].beginToStation;
            vBattery1 += max(bat - data.batteryCapacity, 0.0);
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }

        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || data.eval[v2][r2][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(!vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;

        // ======================================Update Ride =====================================================
        double newRide1;
        double newRide2;

        int numCustomers1    = vehicles[v1].numCustomers;
        int numStations1     = vehicles[v1].numStations;
        int numCustomers2    = vehicles[v2].numCustomers;
        int numStations2     = vehicles[v2].numStations;

        if(vehicles[v1].route[r1].batteryStation){
            numStations1--;
            numStations2++;
        }else{
            numCustomers1--;
            numCustomers2++;
        }

        newRide1 = (newDistance1 / data.milhasMinuto) + (numCustomers1 * data.customerService) + (numStations1 * data.stationService);
        newRide2 = (newDistance2 / data.milhasMinuto) + (numCustomers2 * data.customerService) + (numStations2 * data.stationService);
        
        newRide = newRide + (newRide1 + newRide2) - (vehicles[v1].ride + vehicles[v2].ride);

        double newViolationTimeTour1;
        double newViolationTimeTour2;

        newViolationTimeTour1 = max(newRide1 - data.maxTour, 0.0);
        newViolationTimeTour2 = max(newRide2 - data.maxTour, 0.0);

        newViolationTimeTour = newViolationTimeTour - (vehicles[v1].violationTimeTour + vehicles[v2].violationTimeTour) + (newViolationTimeTour1 + newViolationTimeTour2);

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationTimeTour * data.betaTour);
        newObjective    = newDistance + newVehicleCost + newPenality + stationCost;

        if(bestObjective - newObjective > EPS){
            bestV1          = v1;
            bestV2          = v2;
            bestR1          = r1;
            bestR2          = r2;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            bestRide1       = newRide1;
            bestRide2       = newRide2;
            bestVTimeT1      = newViolationTimeTour1;
            bestVTimeT2      = newViolationTimeTour2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
        }
    }
    
    if(improve){
        distance                    = distance - (vehicles[bestV1].distance + vehicles[bestV2].distance);
        vehicles[bestV1].distance   = bestDistance1;
        vehicles[bestV2].distance   = bestDistance2;
        distance                    = distance + vehicles[bestV1].distance + vehicles[bestV2].distance;
        
        violationBattery                    = violationBattery - (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery);
        vehicles[bestV1].violationBattery   = bestVBt1;
        vehicles[bestV2].violationBattery   = bestVBt2;
        violationBattery                    = violationBattery + bestVBt1 + bestVBt2;

        ride                    = ride - (vehicles[bestV1].ride + vehicles[bestV2].ride);
        vehicles[bestV1].ride   = bestRide1;
        vehicles[bestV2].ride   = bestRide2;
        ride                    = ride + vehicles[bestV1].ride + vehicles[bestV2].ride;

        if(vehicles[bestV1].route[bestR1].batteryStation){
            vehicles[bestV1].numStations--;
            vehicles[bestV2].numStations++;
        }else{
            vehicles[bestV1].numCustomers--;
            vehicles[bestV2].numCustomers++;
        }

        violationTimeTour                    = violationTimeTour - (vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour);
        vehicles[bestV1].violationTimeTour   = bestVTimeT1;
        vehicles[bestV2].violationTimeTour   = bestVTimeT2;
        violationTimeTour                    = violationTimeTour + vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour;

        penality    = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective   = distance + penality + stationCost + vehicleCost;

        vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1]);
        vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1);
        
        if((int)vehicles[bestV1].route.size() < 3){
            amountVehicles--;
        }
        if((int)vehicles[bestV2].route.size() == 3){
            amountVehicles++;
        }
        
        //======================================Update data===================================================================
        updateData(data, bestV1);
        updateData(data, bestV2);
        
        double oldDistance1          = vehicles[bestV1].distance;
        double oldRide1              = vehicles[bestV1].ride;
        double oldViolationBattery1  = vehicles[bestV1].violationBattery;
        double oldViolationTimeTour1 = vehicles[bestV1].violationTimeTour;
        double oldDistance2          = vehicles[bestV2].distance;
        double oldRide2              = vehicles[bestV2].ride;
        double oldViolationBattery2  = vehicles[bestV2].violationBattery;
        double oldViolationTimeTour2 = vehicles[bestV2].violationTimeTour;
        
        vehicles[bestV1].intraRVND(data, bestV1);
        vehicles[bestV2].intraRVND(data, bestV2);
        
        distance        = distance      + (vehicles[bestV1].distance        + vehicles[bestV2].distance)        - (oldDistance1 + oldDistance2);
        ride                = ride              + (vehicles[bestV1].ride                + vehicles[bestV2].ride)                - (oldRide1 + oldRide2);
        violationBattery    = violationBattery  + (vehicles[bestV1].violationBattery    + vehicles[bestV2].violationBattery)    - (oldViolationBattery1 + oldViolationBattery2);
        violationTimeTour   = violationTimeTour + (vehicles[bestV1].violationTimeTour   + vehicles[bestV2].violationTimeTour)   - (oldViolationTimeTour1 + oldViolationTimeTour2);

        penality            = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::interRelocation2(Data &data){
    int bestV1, bestV2, bestR1, bestR1_2, bestR2, flag = 0;
    double bestVBt1, bestVBt2;
    double bestRide1 = 0, bestRide2 = 0, bestDistance1 = 0, bestDistance2 = 0, bestVTimeT1 = 0, bestVTimeT2 = 0;
    double newPenality, newObjective;
    double newVehicleCost       = vehicleCost;
    double bestObjective        = objective;
    double newViolationBattery  = violationBattery;
    double newViolationTimeTour = violationTimeTour;
    double newRide              = ride;
    double newDistance          = distance;
    bool improve = true;

    vector < pair < pair < pair < int, int >, int >, pair < int, int > > > vt;
    improve = false;
    for(int i = 0; i < (int)vehicles.size();i++) {
        if((int)vehicles[i].route.size() > 3){
            for(int j = 0; j < (int)vehicles.size(); j++) {
                if(i != j){
                    for(int r1 = 1; r1 < (int)vehicles[i].route.size()-2; r1++) {
                        if((int)vehicles[j].route.size() < 3){
                            vt.push_back(make_pair(make_pair(make_pair(i, r1), r1+1), make_pair(j, 1)));
                        }else{
                            for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                                vt.push_back(make_pair(make_pair(make_pair(i, r1), r1+1), make_pair(j, r2)));
                            }                        
                        }
                    }
                }
            }
        }
    }

    shuffle(vt.begin(), vt.end(), generator);
    for(int i = 0; i < (int)vt.size(); i++){
        newViolationBattery     = violationBattery;
        newViolationTimeTour    = violationTimeTour;
        newRide                 = ride;
        newDistance             = distance;

        int v1      = vt[i].first.first.first;
        int v2      = vt[i].second.first;
        int r1      = vt[i].first.first.second;
        int r1_2    = vt[i].first.second;
        int r2      = vt[i].second.second;

        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;

        // =========================================Update Ride==========================================================================
        double newDistance1;
        double newDistance2;
        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || data.eval[v1][r1_2+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1_2+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1_2+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].beginToStation;
            vBattery1 += max(bat - data.batteryCapacity, 0.0);
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }

        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1].batteryStation && !vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;

        // ======================================Update Ride =====================================================
        double newRide1;
        double newRide2;

        int numCustomers1    = vehicles[v1].numCustomers;
        int numStations1     = vehicles[v1].numStations;
        int numCustomers2    = vehicles[v2].numCustomers;
        int numStations2     = vehicles[v2].numStations;

        if(vehicles[v1].route[r1].batteryStation){
            numStations1--;
            numStations2++;
        }else{
            numCustomers1--;
            numCustomers2++;
        }
        if(vehicles[v1].route[r1_2].batteryStation){
            numStations1--;
            numStations2++;
        }else{
            numCustomers1--;
            numCustomers2++;
        }

        newRide1 = (newDistance1 / data.milhasMinuto) + (numCustomers1 * data.customerService) + (numStations1 * data.stationService);
        newRide2 = (newDistance2 / data.milhasMinuto) + (numCustomers2 * data.customerService) + (numStations2 * data.stationService);
        
        newRide = newRide + (newRide1 + newRide2) - (vehicles[v1].ride + vehicles[v2].ride);

        double newViolationTimeTour1;
        double newViolationTimeTour2;
        
        newViolationTimeTour1 = max(newRide1 - data.maxTour, 0.0);
        newViolationTimeTour2 = max(newRide2 - data.maxTour, 0.0);
        
        newViolationTimeTour = newViolationTimeTour - (vehicles[v1].violationTimeTour + vehicles[v2].violationTimeTour) + (newViolationTimeTour1 + newViolationTimeTour2);

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationTimeTour * data.betaTour);
        newObjective    = newDistance + newVehicleCost + newPenality + stationCost;

        if(bestObjective - newObjective > EPS){
            bestV1          = v1;
            bestV2          = v2;
            bestR1          = r1;
            bestR1_2        = r1_2;
            bestR2          = r2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            flag            = 1;
            bestRide1       = newRide1;
            bestRide2       = newRide2;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            bestVTimeT1     = newViolationTimeTour1;
            bestVTimeT2     = newViolationTimeTour2;
        }

        //=============================================Invertendo ================================================
        newViolationBattery  = violationBattery;
        newViolationTimeTour = violationTimeTour;
        newRide              = ride;
        newDistance          = distance;

        // =========================================Update Ride==========================================================================
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        vBattery2 = 0;
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1_2].batteryStation && !vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - (vehicles[v1].violationBattery + vehicles[v2].violationBattery);

        // ======================================Update Ride =====================================================
        newRide2 = (newDistance2 / data.milhasMinuto) + (numCustomers2 * data.customerService) + (numStations2 * data.stationService);
        
        newRide = newRide + (newRide1 + newRide2) - (vehicles[v1].ride + vehicles[v2].ride);

        newViolationTimeTour2 = max(newRide2 - data.maxTour, 0.0);
        
        newViolationTimeTour = newViolationTimeTour - (vehicles[v1].violationTimeTour + vehicles[v2].violationTimeTour) + (newViolationTimeTour1 + newViolationTimeTour2);

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationTimeTour * data.betaTour);
        newObjective    = newDistance + newVehicleCost + newPenality + stationCost;
        
        if(bestObjective - newObjective > EPS){
            bestV1          = v1;
            bestV2          = v2;
            bestR1          = r1;
            bestR1_2        = r1_2;
            bestR2          = r2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            flag            = 2;
            bestRide1       = newRide1;
            bestRide2       = newRide2;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            bestVTimeT1     = newViolationTimeTour1;
            bestVTimeT2     = newViolationTimeTour2;
        }
    }

    if(improve){
        distance                    = distance - (vehicles[bestV1].distance + vehicles[bestV2].distance);
        vehicles[bestV1].distance   = bestDistance1;
        vehicles[bestV2].distance   = bestDistance2;
        distance                    = distance + vehicles[bestV1].distance + vehicles[bestV2].distance;
        
        violationBattery                    = violationBattery - (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery);
        vehicles[bestV1].violationBattery   = bestVBt1;
        vehicles[bestV2].violationBattery   = bestVBt2;
        violationBattery                    = violationBattery + bestVBt1 + bestVBt2;

        ride                    = ride - (vehicles[bestV1].ride + vehicles[bestV2].ride);
        vehicles[bestV1].ride   = bestRide1;
        vehicles[bestV2].ride   = bestRide2;
        ride                    = ride + vehicles[bestV1].ride + vehicles[bestV2].ride;

        if(vehicles[bestV1].route[bestR1].batteryStation){
            vehicles[bestV1].numStations--;
            vehicles[bestV2].numStations++;
        }else{
            vehicles[bestV1].numCustomers--;
            vehicles[bestV2].numCustomers++;
        }
        if(vehicles[bestV1].route[bestR1_2].batteryStation){
            vehicles[bestV1].numStations--;
            vehicles[bestV2].numStations++;
        }else{
            vehicles[bestV1].numCustomers--;
            vehicles[bestV2].numCustomers++;
        }

        violationTimeTour                    = violationTimeTour - (vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour);
        vehicles[bestV1].violationTimeTour   = bestVTimeT1;
        vehicles[bestV2].violationTimeTour   = bestVTimeT2;
        violationTimeTour                    = violationTimeTour + vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour;

        penality = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);

        objective = distance + penality + stationCost + vehicleCost;
        
        if(flag == 1){
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1_2]);
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1]);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1_2);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1);
        }else{
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1]);
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1_2]);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1_2);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1);
        }
        
        if((int)vehicles[bestV1].route.size() < 3){
            amountVehicles--;
        }
        if((int)vehicles[bestV2].route.size() == 4){
            amountVehicles++;
        }
        
        //======================================Update data===================================================================
        updateData(data, bestV1);
        updateData(data, bestV2);

        double oldRide1                 = vehicles[bestV1].ride;
        double oldDistance1             = vehicles[bestV1].distance;
        double oldViolationBattery1     = vehicles[bestV1].violationBattery;
        double oldViolationTimeTour1    = vehicles[bestV1].violationTimeTour;
        double oldRide2                 = vehicles[bestV2].ride;
        double oldDistance2             = vehicles[bestV2].distance;
        double oldViolationBattery2     = vehicles[bestV2].violationBattery;
        double oldViolationTimeTour2    = vehicles[bestV2].violationTimeTour;
        
        vehicles[bestV1].intraRVND(data, bestV1);
        vehicles[bestV2].intraRVND(data, bestV2);
        
        ride                = ride              + (vehicles[bestV1].ride + vehicles[bestV2].ride)                           - (oldRide1 + oldRide2);
        distance        = distance      + (vehicles[bestV1].distance + vehicles[bestV2].distance)           - (oldDistance1 + oldDistance2);
        violationBattery    = violationBattery  + (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery)   - (oldViolationBattery1 + oldViolationBattery2);
        violationTimeTour   = violationTimeTour + (vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour) - (oldViolationTimeTour1 + oldViolationTimeTour2);
        
        penality            = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::interSwap(Data &data){
    int bestV1 = 0, bestV2 = 0, bestR1 = 0, bestR2 = 0;
    double bestVBt1 = 0, bestVBt2 = 0, bestRide1 = 0, bestRide2 = 0, bestVTimeT1 = 0, bestVTimeT2 = 0, bestDistance1 = 0, bestDistance2 = 0;
    double newPenality, bestObjective = objective, newObjective;
    double newViolationBattery  = violationBattery;
    double newViolationTimeTour = violationTimeTour;
    double newRide              = ride;
    double newDistance          = distance;
    bool improve = true;

    vector < pair < pair < int, int >, pair < int, int > > > vt;
    improve = false;
    for(int i = 0; i < (int)vehicles.size();i++) {
        if((int)vehicles[i].route.size() > 2){
            for(int j = 0; j < (int)vehicles.size(); j++) {
                if(i != j){
                    for(int r1 = 1; r1 < (int)vehicles[i].route.size()-1; r1++) {
                        if((int)vehicles[j].route.size() > 2){
                            for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                                vt.push_back(make_pair(make_pair(i, r1), make_pair(j, r2)));
                            }                        
                        }
                    }
                }
            }
        }
    }

    shuffle(vt.begin(), vt.end(), generator);
    for(int i = 0; i < (int)vt.size(); i++){
        newViolationBattery = violationBattery;
        newViolationTimeTour = violationTimeTour;
        newRide             = ride;
        newDistance         = distance;

        int v1 = vt[i].first.first;
        int v2 = vt[i].second.first;
        int r1 = vt[i].first.second;
        int r2 = vt[i].second.second;

        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;
        
        // =========================================Update Ride==========================================================================
        double newDistance1;
        double newDistance2;
        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].distance;
        newDistance = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;

        if(data.eval[v1][0][r1-1].batteryStation || vehicles[v2].route[r2].batteryStation || data.eval[v1][r1+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            if(!vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].beginToStation;
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                
                bat = data.eval[v1][r1+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }
        
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || data.eval[v2][r2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(!vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;

        // ======================================Update Ride =====================================================
        double newRide1;
        double newRide2;

        int numCustomers1    = vehicles[v1].numCustomers;
        int numStations1     = vehicles[v1].numStations;
        int numCustomers2    = vehicles[v2].numCustomers;
        int numStations2     = vehicles[v2].numStations;

        if(vehicles[v1].route[r1].batteryStation){
            numStations1--;
            numStations2++;
        }else{
            numCustomers1--;
            numCustomers2++;
        }
        if(vehicles[v2].route[r2].batteryStation){
            numStations2--;
            numStations1++;
        }else{
            numCustomers2--;
            numCustomers1++;
        }

        newRide1 = (newDistance1 / data.milhasMinuto) + (numCustomers1 * data.customerService) + (numStations1 * data.stationService);
        newRide2 = (newDistance2 / data.milhasMinuto) + (numCustomers2 * data.customerService) + (numStations2 * data.stationService);
        
        newRide = newRide + (newRide1 + newRide2) - (vehicles[v1].ride + vehicles[v2].ride);

        double newViolationTimeTour1;
        double newViolationTimeTour2;
        
        newViolationTimeTour1 = max(newRide1 - data.maxTour, 0.0);
        newViolationTimeTour2 = max(newRide2 - data.maxTour, 0.0);

        newViolationTimeTour = newViolationTimeTour - (vehicles[v1].violationTimeTour + vehicles[v2].violationTimeTour) + (newViolationTimeTour1 + newViolationTimeTour2);

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationTimeTour * data.betaTour);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;

        if(bestObjective - newObjective > EPS){
            bestV1          = v1;
            bestV2          = v2;
            bestR1          = r1;
            bestR2          = r2;
            bestRide1       = newRide1;
            bestRide2       = newRide2;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            bestVTimeT1     = newViolationTimeTour1;
            bestVTimeT2     = newViolationTimeTour2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
        }
    }

    if(improve){
        distance                    = distance - (vehicles[bestV1].distance + vehicles[bestV2].distance);
        vehicles[bestV1].distance   = bestDistance1;
        vehicles[bestV2].distance   = bestDistance2;
        distance                    = distance + vehicles[bestV1].distance + vehicles[bestV2].distance;

        violationBattery                    = violationBattery - (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery);
        vehicles[bestV1].violationBattery   = bestVBt1;
        vehicles[bestV2].violationBattery   = bestVBt2;
        violationBattery                    = violationBattery + bestVBt1 + bestVBt2;

        ride                    = ride - (vehicles[bestV1].ride + vehicles[bestV2].ride);
        vehicles[bestV1].ride   = bestRide1;
        vehicles[bestV2].ride   = bestRide2;
        ride                    = ride + vehicles[bestV1].ride + vehicles[bestV2].ride;
        
        if(vehicles[bestV1].route[bestR1].batteryStation){
            vehicles[bestV1].numStations--;
            vehicles[bestV2].numStations++;
        }else{
            vehicles[bestV1].numCustomers--;
            vehicles[bestV2].numCustomers++;
        }
        if(vehicles[bestV2].route[bestR2].batteryStation){
            vehicles[bestV2].numStations--;
            vehicles[bestV1].numStations++;
        }else{
            vehicles[bestV2].numCustomers--;
            vehicles[bestV1].numCustomers++;
        }

        violationTimeTour                    = violationTimeTour - (vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour);
        vehicles[bestV1].violationTimeTour   = bestVTimeT1;
        vehicles[bestV2].violationTimeTour   = bestVTimeT2;
        violationTimeTour                    = violationTimeTour + vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour;

        penality  = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective = distance + penality + stationCost + vehicleCost;

        swap(vehicles[bestV1].route[bestR1], vehicles[bestV2].route[bestR2]);
        //======================================Update data===================================================================

        updateData(data, bestV1);
        updateData(data, bestV2);

        double oldRide1                 = vehicles[bestV1].ride;
        double oldDistance1             = vehicles[bestV1].distance;
        double oldViolationBattery1     = vehicles[bestV1].violationBattery;
        double oldViolationTimeTour1    = vehicles[bestV1].violationTimeTour;
        double oldRide2                 = vehicles[bestV2].ride;
        double oldDistance2             = vehicles[bestV2].distance;
        double oldViolationBattery2     = vehicles[bestV2].violationBattery;
        double oldViolationTimeTour2    = vehicles[bestV2].violationTimeTour;
        
        vehicles[bestV1].intraRVND(data, bestV1);
        vehicles[bestV2].intraRVND(data, bestV2);
        
        ride                = ride              + (vehicles[bestV1].ride + vehicles[bestV2].ride)                           - (oldRide1 + oldRide2);
        distance        = distance      + (vehicles[bestV1].distance + vehicles[bestV2].distance)           - (oldDistance1 + oldDistance2);
        violationBattery    = violationBattery  + (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery)   - (oldViolationBattery1 + oldViolationBattery2);
        violationTimeTour   = violationTimeTour + (vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour) - (oldViolationTimeTour1 + oldViolationTimeTour2);

        penality            = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::interSwap2(Data &data){
    int bestV1 = 0, bestV2 = 0, bestR1 = 0, bestR1_2 = 0, bestR2 = 0, bestR2_2 = 0, flag = 0;
    double bestVBt1 = 0, bestVBt2 = 0, bestRide1 = 0, bestRide2 = 0, bestVTimeT1 = 0, bestVTimeT2 = 0, bestDistance1 = 0, bestDistance2 = 0;
    double newPenality, newObjective;
    double bestObjective        = objective;
    double newViolationBattery  = violationBattery;
    double newViolationTimeTour = violationTimeTour;
    double newRide              = ride;
    double newDistance          = distance;
    bool improve = false;

    vector < pair < pair < pair < int, int >, int >, pair < pair < int, int >, int > > > vt;

    for(int i = 0; i < (int)vehicles.size(); i++) {
        for(int j = i; j < (int)vehicles.size(); j++) {
            if((int)vehicles[i].route.size() > 3 && (int)vehicles[j].route.size() > 3 && (i != j)){
                for(int r1 = 1; r1 < (int)vehicles[i].route.size()-2; r1++) {                    
                    for(int r2 = 1; r2 < (int)vehicles[j].route.size()-2; r2++) {
                        vt.push_back( make_pair( make_pair( make_pair(i, r1), r1+1), make_pair( make_pair(j, r2), r2+1)));
                    }
                }
            }
        }
    }

    
    shuffle(vt.begin(), vt.end(), generator);
    for(int i = 0; i < (int)vt.size(); i++){
        newViolationBattery = violationBattery;
        newViolationTimeTour = violationTimeTour;
        newRide             = ride;
        newDistance         = distance;

        int v1      = vt[i].first.first.first;
        int v2      = vt[i].second.first.first;
        int r1      = vt[i].first.first.second;
        int r1_2    = vt[i].first.second;
        int r2      = vt[i].second.first.second;
        int r2_2    = vt[i].second.second;

        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;
        
        // =========================================Update Ride==========================================================================
        double newDistance1;
        double newDistance2;
        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v2].route[r2_2].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || vehicles[v2].route[r2].batteryStation || vehicles[v2].route[r2_2].batteryStation || data.eval[v1][r1_2+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1_2+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1_2+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            if(vehicles[v2].route[r2].batteryStation && vehicles[v2].route[r2_2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2_2].id][vehicles[v1].route[r1_2+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v2].route[r2].batteryStation && vehicles[v2].route[r2_2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v2].route[r2_2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2_2].id][vehicles[v1].route[r1_2+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v2].route[r2].batteryStation && !vehicles[v2].route[r2_2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2_2].id][vehicles[v1].route[r1_2+1].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v2].route[r2_2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v2].route[r2_2].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].beginToStation;
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2_2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2_2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2_2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1].batteryStation && !vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;
        
        // ======================================Update Ride =====================================================
        double newRide1;
        double newRide2;

        int numCustomers1    = vehicles[v1].numCustomers;
        int numStations1     = vehicles[v1].numStations;
        int numCustomers2    = vehicles[v2].numCustomers;
        int numStations2     = vehicles[v2].numStations;

        if(vehicles[v1].route[r1].batteryStation){
            numStations1--;
            numStations2++;
        }else{
            numCustomers1--;
            numCustomers2++;
        }
        if(vehicles[v1].route[r1_2].batteryStation){
            numStations1--;
            numStations2++;
        }else{
            numCustomers1--;
            numCustomers2++;
        }
        if(vehicles[v2].route[r2].batteryStation){
            numStations2--;
            numStations1++;
        }else{
            numCustomers2--;
            numCustomers1++;
        }
        if(vehicles[v2].route[r2_2].batteryStation){
            numStations2--;
            numStations1++;
        }else{
            numCustomers2--;
            numCustomers1++;
        }

        newRide1 = (newDistance1 / data.milhasMinuto) + (numCustomers1 * data.customerService) + (numStations1 * data.stationService);
        newRide2 = (newDistance2 / data.milhasMinuto) + (numCustomers2 * data.customerService) + (numStations2 * data.stationService);
        
        newRide = newRide + (newRide1 + newRide2) - (vehicles[v1].ride + vehicles[v2].ride);

        double newViolationTimeTour1;
        double newViolationTimeTour2;
        
        newViolationTimeTour1 = max(newRide1 - data.maxTour, 0.0);
        newViolationTimeTour2 = max(newRide2 - data.maxTour, 0.0);

        newViolationTimeTour = newViolationTimeTour - (vehicles[v1].violationTimeTour + vehicles[v2].violationTimeTour) + (newViolationTimeTour1 + newViolationTimeTour2);

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationTimeTour * data.betaTour);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;
        
        if(bestObjective - newObjective > EPS){
            bestV1          = v1;
            bestV2          = v2;
            bestR1          = r1;
            bestR1_2        = r1_2;
            bestR2          = r2;
            bestR2_2        = r2_2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            bestRide1       = newRide1;
            bestRide2       = newRide2;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            bestVTimeT1     = newViolationTimeTour1;
            bestVTimeT2     = newViolationTimeTour2;
            flag            = 1;
        }
            
        //=====================================================Inverte V2==================================================================
        newViolationBattery = violationBattery;
        newViolationTimeTour = violationTimeTour;
        newRide             = ride;
        newDistance         = distance;

        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].distance;
        newDistance = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        vBattery2 = 0;
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2_2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2_2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2_2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1_2].batteryStation && !vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2_2+1].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;

        // ======================================Update Ride =====================================================

        newRide2 = (newDistance2 / data.milhasMinuto) + (numCustomers2 * data.customerService) + (numStations2 * data.stationService);
        
        newRide = newRide + (newRide1 + newRide2) - (vehicles[v1].ride + vehicles[v2].ride);
        
        newViolationTimeTour2 = max(newRide2 - data.maxTour, 0.0);

        newViolationTimeTour = newViolationTimeTour - (vehicles[v1].violationTimeTour + vehicles[v2].violationTimeTour) + (newViolationTimeTour1 + newViolationTimeTour2);

        // ===========================================Update custSolution====================================================================
        
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationTimeTour * data.betaTour);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;
        
        if(bestObjective - newObjective > EPS){
            bestV1          = v1;
            bestV2          = v2;
            bestR1          = r1;
            bestR1_2        = r1_2;
            bestR2          = r2;
            bestR2_2        = r2_2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            bestRide1       = newRide1;
            bestRide2       = newRide2;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            bestVTimeT1     = newViolationTimeTour1;
            bestVTimeT2     = newViolationTimeTour2;
            flag            = 2;
        }
        
        //====================================================Inverte V1=====================================================]
        newViolationBattery  = violationBattery;
        newViolationTimeTour = violationTimeTour;
        newRide              = ride;
        newDistance          = distance;

        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2_2].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        vBattery1 = 0;
        vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || vehicles[v2].route[r2].batteryStation || vehicles[v2].route[r2_2].batteryStation || data.eval[v1][r1_2+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1_2+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1_2+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            if(vehicles[v2].route[r2_2].batteryStation && vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2_2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v2].route[r2_2].batteryStation && vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2_2].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v2].route[r2_2].batteryStation && !vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2_2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2_2].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].beginToStation;
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2_2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2_2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2_2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1].batteryStation && !vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
            
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;

        // ======================================Update Ride =====================================================
        
        newRide1 = (newDistance1 / data.milhasMinuto) + (numCustomers1 * data.customerService) + (numStations1 * data.stationService);
        newRide2 = (newDistance2 / data.milhasMinuto) + (numCustomers2 * data.customerService) + (numStations2 * data.stationService);
        
        newRide = newRide + (newRide1 + newRide2) - (vehicles[v1].ride + vehicles[v2].ride);

        newViolationTimeTour1 = max(newRide1 - data.maxTour, 0.0);
        newViolationTimeTour2 = max(newRide2 - data.maxTour, 0.0);

        newViolationTimeTour = newViolationTimeTour - (vehicles[v1].violationTimeTour + vehicles[v2].violationTimeTour) + (newViolationTimeTour1 + newViolationTimeTour2);

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationTimeTour * data.betaTour);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;

        if(bestObjective - newObjective > EPS){
            bestV1          = v1;
            bestV2          = v2;
            bestR1          = r1;
            bestR1_2        = r1_2;
            bestR2          = r2;
            bestR2_2        = r2_2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            bestRide1       = newRide1;
            bestRide2       = newRide2;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            bestVTimeT1     = newViolationTimeTour1;
            bestVTimeT2     = newViolationTimeTour2;
            flag            = 3;
        }
    }

    if(improve){
        distance                    = distance - (vehicles[bestV1].distance + vehicles[bestV2].distance);
        vehicles[bestV1].distance   = bestDistance1;
        vehicles[bestV2].distance   = bestDistance2;
        distance                    = distance + vehicles[bestV1].distance + vehicles[bestV2].distance;

        violationBattery                    = violationBattery - (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery);
        vehicles[bestV1].violationBattery   = bestVBt1;
        vehicles[bestV2].violationBattery   = bestVBt2;
        violationBattery                    = violationBattery + bestVBt1 + bestVBt2;

        ride                    = ride - (vehicles[bestV1].ride + vehicles[bestV2].ride);
        vehicles[bestV1].ride   = bestRide1;
        vehicles[bestV2].ride   = bestRide2;
        ride                    = ride + vehicles[bestV1].ride + vehicles[bestV2].ride;

        if(vehicles[bestV1].route[bestR1].batteryStation){
            vehicles[bestV1].numStations--;
            vehicles[bestV2].numStations++;
        }else{
            vehicles[bestV1].numCustomers--;
            vehicles[bestV2].numCustomers++;
        }
        if(vehicles[bestV1].route[bestR1_2].batteryStation){
            vehicles[bestV1].numStations--;
            vehicles[bestV2].numStations++;
        }else{
            vehicles[bestV1].numCustomers--;
            vehicles[bestV2].numCustomers++;
        }
        if(vehicles[bestV2].route[bestR2].batteryStation){
            vehicles[bestV2].numStations--;
            vehicles[bestV1].numStations++;
        }else{
            vehicles[bestV2].numCustomers--;
            vehicles[bestV1].numCustomers++;
        }
        if(vehicles[bestV2].route[bestR2_2].batteryStation){
            vehicles[bestV2].numStations--;
            vehicles[bestV1].numStations++;
        }else{
            vehicles[bestV2].numCustomers--;
            vehicles[bestV1].numCustomers++;
        }

        violationTimeTour                    = violationTimeTour - (vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour);
        vehicles[bestV1].violationTimeTour   = bestVTimeT1;
        vehicles[bestV2].violationTimeTour   = bestVTimeT2;
        violationTimeTour                    = violationTimeTour + vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour;

        penality  = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective = distance + penality + stationCost + vehicleCost;

        if(flag == 1){
            swap(vehicles[bestV1].route[bestR1], vehicles[bestV2].route[bestR2]);
            swap(vehicles[bestV1].route[bestR1_2], vehicles[bestV2].route[bestR2_2]);
        }else if(flag == 2){
            swap(vehicles[bestV1].route[bestR1_2], vehicles[bestV2].route[bestR2]);
            swap(vehicles[bestV1].route[bestR1], vehicles[bestV2].route[bestR2_2]);
        }else{
            swap(vehicles[bestV1].route[bestR1], vehicles[bestV2].route[bestR2_2]);
            swap(vehicles[bestV1].route[bestR1_2], vehicles[bestV2].route[bestR2]);
        }

        //======================================Update data===================================================================
        updateData(data, bestV1);
        updateData(data, bestV2);

        double oldRide1                 = vehicles[bestV1].ride;
        double oldDistance1             = vehicles[bestV1].distance;
        double oldViolationBattery1     = vehicles[bestV1].violationBattery;
        double oldViolationTimeTour1    = vehicles[bestV1].violationTimeTour;
        double oldRide2                 = vehicles[bestV2].ride;
        double oldDistance2             = vehicles[bestV2].distance;
        double oldViolationBattery2     = vehicles[bestV2].violationBattery;
        double oldViolationTimeTour2    = vehicles[bestV2].violationTimeTour;
        
        vehicles[bestV1].intraRVND(data, bestV1);
        vehicles[bestV2].intraRVND(data, bestV2);
        
        ride                = ride              + (vehicles[bestV1].ride + vehicles[bestV2].ride)                           - (oldRide1 + oldRide2);
        distance        = distance      + (vehicles[bestV1].distance + vehicles[bestV2].distance)           - (oldDistance1 + oldDistance2);
        violationBattery    = violationBattery  + (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery)   - (oldViolationBattery1 + oldViolationBattery2);
        violationTimeTour   = violationTimeTour + (vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour) - (oldViolationTimeTour1 + oldViolationTimeTour2);

        penality            = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::interSwap2x1(Data &data){
    int bestV1 = 0, bestV2 = 0, bestR1 = 0, bestR1_2 = 0, bestR2 = 0, flag = 0;
    double bestVBt1 = 0, bestVBt2 = 0, bestDistance1 = 0, bestDistance2 = 0, bestVTimeT1 = 0, bestVTimeT2 = 0, bestRide1 = 0, bestRide2 = 0;
    double newPenality, newObjective;
    double bestObjective        = objective;
    double newViolationBattery  = violationBattery;
    double newViolationTimeTour = violationTimeTour;
    double newRide              = ride;
    double newDistance          = distance;
    bool improve = false;

    vector < pair < pair < pair < int, int >, int >, pair < int, int > > > vt;

    for(int i = 0; i < (int)vehicles.size(); i++) {
        for(int j = 0; j < (int)vehicles.size(); j++) {
            if((int)vehicles[i].route.size() > 3 && (int)vehicles[j].route.size() > 2 && i != j){
                for(int r1 = 1; r1 < (int)vehicles[i].route.size()-2; r1++) {                    
                    for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                        vt.push_back( make_pair( make_pair( make_pair(i, r1), r1+1), make_pair(j, r2)));
                    }
                }
            }
        }
    }

    improve = false;
    shuffle(vt.begin(), vt.end(), generator);
    for(int i = 0; i < (int)vt.size(); i++){
        newViolationBattery  = violationBattery;
        newViolationTimeTour = violationTimeTour;
        newRide              = ride;
        newDistance          = distance;

        int v1      = vt[i].first.first.first;
        int v2      = vt[i].second.first;
        int r1      = vt[i].first.first.second;
        int r1_2    = vt[i].first.second;
        int r2      = vt[i].second.second;

        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;
        
        // =========================================Update Ride==========================================================================
        double newDistance1;
        double newDistance2;
        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || vehicles[v2].route[r2].batteryStation || data.eval[v1][r1_2+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1_2+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1_2+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            if(vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].beginToStation;
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1].batteryStation && !vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2+1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;
        
        // ======================================Update Ride =====================================================
        double newRide1;
        double newRide2;

        int numCustomers1    = vehicles[v1].numCustomers;
        int numStations1     = vehicles[v1].numStations;
        int numCustomers2    = vehicles[v2].numCustomers;
        int numStations2     = vehicles[v2].numStations;

        if(vehicles[v1].route[r1].batteryStation){
            numStations1--;
            numStations2++;
        }else{
            numCustomers1--;
            numCustomers2++;
        }
        if(vehicles[v1].route[r1_2].batteryStation){
            numStations1--;
            numStations2++;
        }else{
            numCustomers1--;
            numCustomers2++;
        }
        if(vehicles[v2].route[r2].batteryStation){
            numStations2--;
            numStations1++;
        }else{
            numCustomers2--;
            numCustomers1++;
        }

        newRide1 = (newDistance1 / data.milhasMinuto) + (numCustomers1 * data.customerService) + (numStations1 * data.stationService);
        newRide2 = (newDistance2 / data.milhasMinuto) + (numCustomers2 * data.customerService) + (numStations2 * data.stationService);
        
        newRide = newRide + (newRide1 + newRide2) - (vehicles[v1].ride + vehicles[v2].ride);

        double newViolationTimeTour1;
        double newViolationTimeTour2;
        
        newViolationTimeTour1 = max(newRide1 - data.maxTour, 0.0);
        newViolationTimeTour2 = max(newRide2 - data.maxTour, 0.0);

        newViolationTimeTour = newViolationTimeTour - (vehicles[v1].violationTimeTour + vehicles[v2].violationTimeTour) + (newViolationTimeTour1 + newViolationTimeTour2);

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationTimeTour * data.betaTour);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;
        
        if(bestObjective - newObjective > EPS){
            bestV1          = v1;
            bestV2          = v2;
            bestR1          = r1;
            bestR1_2        = r1_2;
            bestR2          = r2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            bestRide1       = newRide1;
            bestRide2       = newRide2;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            bestVTimeT1     = newViolationTimeTour1;
            bestVTimeT2     = newViolationTimeTour2;
            flag            = 1;
        }
            
        //=====================================================Inverte==================================================================
        newViolationBattery  = violationBattery;
        newViolationTimeTour = violationTimeTour;
        newRide              = ride;
        newDistance          = distance;

        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        vBattery2 = 0;
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1_2].batteryStation && !vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;

        // ======================================Update Ride =====================================================
        newRide2 = (newDistance2 / data.milhasMinuto) + (numCustomers2 * data.customerService) + (numStations2 * data.stationService);
        
        newRide = newRide + (newRide1 + newRide2) - (vehicles[v1].ride + vehicles[v2].ride);
        
        newViolationTimeTour2 = max(newRide2 - data.maxTour, 0.0);

        newViolationTimeTour = newViolationTimeTour - (vehicles[v1].violationTimeTour + vehicles[v2].violationTimeTour) + (newViolationTimeTour1 + newViolationTimeTour2);

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationTimeTour * data.betaTour);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;
        
        if(bestObjective - newObjective > EPS){
            bestV1          = v1;
            bestV2          = v2;
            bestR1          = r1;
            bestR1_2        = r1_2;
            bestR2          = r2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            bestRide1       = newRide1;
            bestRide2       = newRide2;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            bestVTimeT1     = newViolationTimeTour1;
            bestVTimeT2     = newViolationTimeTour2;
            flag            = 2;
        }
    }

    if(improve){
        distance                    = distance - (vehicles[bestV1].distance + vehicles[bestV2].distance);
        vehicles[bestV1].distance   = bestDistance1;
        vehicles[bestV2].distance   = bestDistance2;
        distance                    = distance + vehicles[bestV1].distance + vehicles[bestV2].distance;

        violationBattery                    = violationBattery - (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery);
        vehicles[bestV1].violationBattery   = bestVBt1;
        vehicles[bestV2].violationBattery   = bestVBt2;
        violationBattery                    = violationBattery + bestVBt1 + bestVBt2;

        ride                    = ride - (vehicles[bestV1].ride + vehicles[bestV2].ride);
        vehicles[bestV1].ride   = bestRide1;
        vehicles[bestV2].ride   = bestRide2;
        ride                    = ride + vehicles[bestV1].ride + vehicles[bestV2].ride;

        if(vehicles[bestV1].route[bestR1].batteryStation){
            vehicles[bestV1].numStations--;
            vehicles[bestV2].numStations++;
        }else{
            vehicles[bestV1].numCustomers--;
            vehicles[bestV2].numCustomers++;
        }
        if(vehicles[bestV1].route[bestR1_2].batteryStation){
            vehicles[bestV1].numStations--;
            vehicles[bestV2].numStations++;
        }else{
            vehicles[bestV1].numCustomers--;
            vehicles[bestV2].numCustomers++;
        }
        if(vehicles[bestV2].route[bestR2].batteryStation){
            vehicles[bestV2].numStations--;
            vehicles[bestV1].numStations++;
        }else{
            vehicles[bestV2].numCustomers--;
            vehicles[bestV1].numCustomers++;
        }

        violationTimeTour                    = violationTimeTour - (vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour);
        vehicles[bestV1].violationTimeTour   = bestVTimeT1;
        vehicles[bestV2].violationTimeTour   = bestVTimeT2;
        violationTimeTour                    = violationTimeTour + vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour;

        penality = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);

        objective = distance + penality + stationCost + vehicleCost;

        if(flag == 1){
            swap(vehicles[bestV1].route[bestR1], vehicles[bestV2].route[bestR2]);
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1_2]);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1_2);    
        }else{
            swap(vehicles[bestV1].route[bestR1_2], vehicles[bestV2].route[bestR2]);
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1]);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1);
        }

        //======================================Update data===================================================================
        updateData(data, bestV1);
        updateData(data, bestV2);

        double oldRide1                 = vehicles[bestV1].ride;
        double oldDistance1             = vehicles[bestV1].distance;
        double oldViolationBattery1     = vehicles[bestV1].violationBattery;
        double oldViolationTimeTour1    = vehicles[bestV1].violationTimeTour;
        double oldRide2                 = vehicles[bestV2].ride;
        double oldDistance2             = vehicles[bestV2].distance;
        double oldViolationBattery2     = vehicles[bestV2].violationBattery;
        double oldViolationTimeTour2    = vehicles[bestV2].violationTimeTour;
        
        vehicles[bestV1].intraRVND(data, bestV1);
        vehicles[bestV2].intraRVND(data, bestV2);
        
        ride                = ride              + (vehicles[bestV1].ride + vehicles[bestV2].ride)                           - (oldRide1 + oldRide2);
        distance        = distance      + (vehicles[bestV1].distance + vehicles[bestV2].distance)           - (oldDistance1 + oldDistance2);
        violationBattery    = violationBattery  + (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery)   - (oldViolationBattery1 + oldViolationBattery2);
        violationTimeTour   = violationTimeTour + (vehicles[bestV1].violationTimeTour + vehicles[bestV2].violationTimeTour) - (oldViolationTimeTour1 + oldViolationTimeTour2);

        penality            = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Vehicle::intraRVND(Data &data, int v){
    penality = violationBattery * data.betaBattery;
    objective = ride + penality;
    double bestObjective = objective;
    int count = 3, i;
    vector<int> vR;

    for(i = 0; i < count; i++){
        vR.push_back(i);
    }
    
    shuffle(vR.begin(), vR.end(), generator);
    count = 0;
    
    while(1){
        if (count >= (int)vR.size()){break;}
        
        if(vR[count] == 0){
            intraRealocation(data, v);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(vR.begin(), vR.end(), generator);
                count = 0;
            }else{
                count++;
            }
         }else if (vR[count] == 1){
            intraShift2(data, v);
            
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(vR.begin(), vR.end(), generator);
                count = 0;
            }else{
                count++;
            }
        }else if (vR[count] == 2){            
            intraSwap(data, v);
            
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(vR.begin(), vR.end(), generator);
                count = 0;
            }else{
                count++;
            }
        }
    }
}

void Vehicle::intraSwap(Data &data, int v) {
    int i, j, bestR1 = -1, bestR2 = -1;
    penality = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
    double bestObjective = distance + penality, newObjective, newPenality, bestVB = 0, bestRide = 0, bestDistance = 0, bestVTimeT = 0;
    vector < pair < int, int > > vt; 
    
    for (i = 1; i < (int)route.size()-1; i++) {
        for (j = i+1; j < (int)route.size()-1; j++) {
            vt.push_back(make_pair(i, j));
        }
    }

    int depot2 = route.size() - 1;
    for (i = 0; i < (int)vt.size(); i++) {
        int r1 = vt[i].first;
        int r2 = vt[i].second;
        double newRide              = 0;
        double newDistance          = 0;
        double vBattery             = 0;
        double newViolationTimeTour = 0;

        if(r1 == r2-1){
            newDistance = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].distance;

            if(data.eval[v][0][depot2].batteryStation){
                if(data.eval[v][0][r1-1].batteryStation){
                    vBattery += data.eval[v][0][data.eval[v][0][r1-1].idStationToEnd].violation;
                }

                if(route[r2].batteryStation && route[r1].batteryStation){
                    double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id];
                    vBattery += max(bat - data.batteryCapacity, 0.0);

                    bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                    vBattery += max(bat - data.batteryCapacity, 0.0);
                }else if(!route[r2].batteryStation && route[r1].batteryStation){
                    double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1].id];
                    vBattery += max(bat - data.batteryCapacity, 0.0);

                    bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                    vBattery += max(bat - data.batteryCapacity, 0.0);
                }else if(route[r2].batteryStation && !route[r1].batteryStation){
                    double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id];
                    vBattery += max(bat - data.batteryCapacity, 0.0);

                    bat = data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                    vBattery += max(bat - data.batteryCapacity, 0.0);
                }else{
                    double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                    vBattery += max(bat - data.batteryCapacity, 0.0);
                }
                
                if(data.eval[v][r2+1][depot2].batteryStation){
                    vBattery += data.eval[v][data.eval[v][r2+1][depot2].idBeginToStation][depot2].violation;
                }
            }else{
                vBattery = max(newDistance - data.batteryCapacity, 0.0);
            }
        }else{
            newDistance = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].distance;

            // ==================Update Battery============================
            if(data.eval[v][0][depot2].batteryStation){
                if(data.eval[v][0][r1-1].batteryStation){
                    vBattery += data.eval[v][0][data.eval[v][0][r1-1].idStationToEnd].violation;
                }

                if(route[r2].batteryStation){
                    double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id];
                    vBattery += max(bat - data.batteryCapacity, 0.0);

                    if(data.eval[v][r1+1][r2-1].batteryStation){
                        bat = data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        if(route[r1].batteryStation){
                            bat = data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                }else{
                    if(data.eval[v][r1+1][r2-1].batteryStation){
                        double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        if(route[r1].batteryStation){
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                }
                
                if(data.eval[v][r1+1][r2-1].batteryStation){
                    vBattery += data.eval[v][data.eval[v][r1+1][r2+1].idBeginToStation][data.eval[v][r1+1][r2-1].idStationToEnd].violation;
                }else{
                    vBattery += max(data.eval[v][r1+1][r2-1].distance - data.batteryCapacity, 0.0);
                }
                
                if(route[r1].batteryStation){
                    if(data.eval[v][r1+1][r2-1].batteryStation){
                        double bat = data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        if(route[r2].batteryStation){
                            double bat = data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                }else{
                    if(data.eval[v][r1+1][r2-1].batteryStation){
                        double bat = data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        if(route[r2].batteryStation){
                            double bat = data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                }
                
                if(data.eval[v][r2+1][depot2].batteryStation){
                    vBattery += data.eval[v][data.eval[v][r2+1][depot2].idBeginToStation][depot2].violation;
                }
            }else{
                vBattery = max(newDistance - data.batteryCapacity, 0.0);
            }
        }
        
        //=====================================Update Ride ===================================================
        newRide = (newDistance / data.milhasMinuto) + (numCustomers * data.customerService) + (numStations * data.stationService);

        newViolationTimeTour = max(newRide - data.maxTour, 0.0);

        //===========================================================================================
        newPenality  = (vBattery * data.betaBattery) + (newViolationTimeTour * data.betaTour);
        newObjective = newDistance + newPenality;

        if(bestObjective - newObjective > EPS){
            bestObjective   = newObjective;
            bestVB          = vBattery;
            bestRide        = newRide;
            bestDistance    = newDistance;
            bestVTimeT      = newViolationTimeTour;
            bestR1          = r1;
            bestR2          = r2;
        }
    }

    if(bestR1 != -1){
        ride                = bestRide;
        distance        = bestDistance;
        violationBattery    = bestVB;
        violationTimeTour   = bestVTimeT;

        penality    = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective   = distance + penality + stationCost + vehicleCost;

        swap(route[bestR1], route[bestR2]);

        double distance, battery, violation, distanceLastStation, distanceFirstStation;
        bool thereAreBattery;
        int idStation;
        for(i = 0; i < (int)route.size() - 1; i++){
            distance            = 0;
            battery             = 0;
            violation           = 0;
            distanceLastStation = 0;
            thereAreBattery    = false;
            if(route[i].batteryStation){
                idStation       = i;
            }else{
                idStation       = -1;
            }

            for(j = i; j < (int)route.size() - 1; j++){
                violation += max(battery - data.batteryCapacity, 0.0);

                if(route[j].batteryStation){
                    thereAreBattery    = true;
                    battery             = 0;
                    distanceLastStation = 0;
                    idStation           = j;
                }

                data.eval[v][i][j].violation        = violation;
                data.eval[v][i][j].distance         = distance;
                data.eval[v][i][j].batteryStation   = thereAreBattery;
                data.eval[v][i][j].stationToEnd     = distanceLastStation;
                data.eval[v][i][j].idStationToEnd   = idStation;
                
                distance            += data.distances[route[j].id][route[j+1].id];
                battery             += data.distances[route[j].id][route[j+1].id];
                distanceLastStation += data.distances[route[j].id][route[j+1].id];                    
            }
            violation += max(battery - data.batteryCapacity, 0.0);
            
            data.eval[v][i][j].violation        = violation;
            data.eval[v][i][j].distance         = distance;
            data.eval[v][i][j].batteryStation   = thereAreBattery;                
            data.eval[v][i][j].stationToEnd     = distanceLastStation;
            data.eval[v][i][j].idStationToEnd   = idStation;
        }
        data.eval[v][i][i].violation        = 0;
        data.eval[v][i][i].distance         = 0;
        data.eval[v][i][i].batteryStation   = false;
        data.eval[v][i][i].stationToEnd     = 0;
        data.eval[v][i][i].idStationToEnd   = -1;

        for(i = (int)route.size() -1; i > 0; i--){
            distanceFirstStation    = 0;
            if(route[i].batteryStation){
                idStation           = i;
            }else{
                idStation           = -1;
            }
            for(j = i; j > 0; j--){
                
                if(route[j].batteryStation){
                    distanceFirstStation    = 0;
                    idStation               = j;
                }

                data.eval[v][j][i].beginToStation   = distanceFirstStation;
                data.eval[v][j][i].idBeginToStation = idStation;
                
                distanceFirstStation    += data.distances[route[j].id][route[j-1].id];
            }
            data.eval[v][j][i].beginToStation   = distanceFirstStation;
            data.eval[v][j][i].idBeginToStation = idStation;
        }
        data.eval[v][i][i].beginToStation   = 0;
        data.eval[v][i][i].idBeginToStation = -1;
    }
}

void Vehicle::intraShift2(Data &data, int v) {
    int i, j, bestR1 = -1, bestR2 = -1, bestR1_2 = -1;
    penality = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
    double bestObjective = distance + penality, newObjective, newPenality, bestVB = 0, bestRide = 0, bestDistance = 0, bestVTimeT = 0;
    bool invert = false;
    vector < pair < int, int > > vt; 
    
    if((int)route.size() > 4){
        for (i = 1; i < (int)route.size()-2; i++) {
            for (j = 1; j < (int)route.size()-2; j++) {
                if(i != j && i != j+1 && i != j-1){
                    vt.push_back(make_pair(i, j));
                }
            }
        }

        int depot2 = route.size() - 1;
        for (i = 0; i < (int)vt.size(); i++) {
            //==============================Distance==================================
            int r1      = vt[i].first;
            int r1_2    = vt[i].first+1;
            int r2      = vt[i].second;
            double newRide              = 0;
            double newDistance          = 0;
            double vBattery             = 0;
            double newViolationTimeTour = 0;
            
            if(r1 > r2){
                newDistance = data.eval[v][0][r2].distance + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance +   data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].distance;
                
                // ==================Update Battery============================
                vBattery = 0;
                if(data.eval[v][0][depot2].batteryStation){
                    if(data.eval[v][0][r2].batteryStation){
                        vBattery += data.eval[v][0][data.eval[v][0][r2].idStationToEnd].violation;
                    }
                    
                    if(route[r1].batteryStation && route[r1_2].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(!route[r1].batteryStation && route[r1_2].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(route[r1].batteryStation && !route[r1_2].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else{
                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }

                    if(data.eval[v][r2+1][r1-1].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r2+1][r1-1].idBeginToStation][data.eval[v][r2+1][r1-1].idStationToEnd].violation;
                    }else{
                        vBattery += max(data.eval[v][r2+1][r1-1].distance - data.batteryCapacity, 0.0);
                    }
                    
                    if(data.eval[v][r1_2+1][depot2].batteryStation){
                        vBattery = vBattery + data.eval[v][data.eval[v][r1_2+1][depot2].idBeginToStation][depot2].violation;
                        double bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        double bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].distance;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }
                }else{
                    vBattery = max(newDistance - data.batteryCapacity, 0.0);
                }
               
               //=====================================Update Ride ===================================================
                newRide = (newDistance / data.milhasMinuto) + (numCustomers * data.customerService) + (numStations * data.stationService);

                newViolationTimeTour = max(newRide - data.maxTour, 0.0);

                //===========================================================================================
                newPenality  = (vBattery * data.betaBattery) + (newViolationTimeTour * data.betaTour);
                newObjective = newDistance + newPenality;
        
                if(bestObjective - newObjective > EPS){
                    bestObjective   = newObjective;
                    bestVB          = vBattery;
                    bestRide        = newRide;
                    bestDistance    = newDistance;
                    bestVTimeT      = newViolationTimeTour;
                    bestR1          = r1;
                    bestR1_2        = r1_2;
                    bestR2          = r2;
                    invert          = false;
                }
                
                //=========================================Inverte========================================
                newDistance = data.eval[v][0][r2].distance + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance +   data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].distance;
                
                // ==================Update Battery============================
                vBattery = 0;
                if(data.eval[v][0][depot2].batteryStation){
                    if(data.eval[v][0][r2].batteryStation){
                        vBattery += data.eval[v][0][data.eval[v][0][r2].idStationToEnd].violation;
                    }
                    
                    if(route[r1_2].batteryStation && route[r1].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(!route[r1_2].batteryStation && route[r1].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(route[r1_2].batteryStation && !route[r1].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else{
                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }

                    if(data.eval[v][r2+1][r1-1].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r2+1][r1-1].idBeginToStation][data.eval[v][r2+1][r1-1].idStationToEnd].violation;
                    }else{
                        vBattery += max(data.eval[v][r2+1][r1-1].distance - data.batteryCapacity, 0.0);
                    }
                    
                    if(data.eval[v][r1_2+1][depot2].batteryStation){
                        vBattery = vBattery + data.eval[v][data.eval[v][r1_2+1][depot2].idBeginToStation][depot2].violation;
                        double bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        double bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].distance;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }
                }else{
                    vBattery = max(newDistance - data.batteryCapacity, 0.0);
                }
               
               //=====================================Update Ride ===================================================
                newRide = (newDistance / data.milhasMinuto) + (numCustomers * data.customerService) + (numStations * data.stationService);

                newViolationTimeTour = max(newRide - data.maxTour, 0.0);

                //===========================================================================================
                newPenality  = (vBattery * data.betaBattery) + (newViolationTimeTour * data.betaTour);
                newObjective = newDistance + newPenality;
        
                if(bestObjective - newObjective > EPS){
                    bestObjective   = newObjective;
                    bestVB          = vBattery;
                    bestRide        = newRide;
                    bestDistance    = newDistance;
                    bestVTimeT      = newViolationTimeTour;
                    bestR1          = r1;
                    bestR1_2        = r1_2;
                    bestR2          = r2;
                    invert          = true;
                }
            }else{
                newDistance = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].distance;

                // ==================Update Battery============================
                vBattery = 0;
                if(data.eval[v][0][depot2].batteryStation){
                    if(data.eval[v][0][r1-1].batteryStation){
                        vBattery += data.eval[v][0][data.eval[v][0][r1-1].idStationToEnd].violation;
                        double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        double bat = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }
                    
                    if(data.eval[v][r1_2+1][r2].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r1_2+1][r2].idBeginToStation][data.eval[v][r1_2+1][r2].idStationToEnd].violation;
                    }else{
                        vBattery += max(data.eval[v][r1_2+1][r2].distance - data.batteryCapacity, 0.0);
                    }
                    
                    if(route[r1].batteryStation && route[r1_2].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(!route[r1].batteryStation && route[r1_2].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(route[r1].batteryStation && !route[r1_2].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else{
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                    if(data.eval[v][r2+1][depot2].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r2+1][depot2].idBeginToStation][depot2].violation;
                    }
                }else{
                    vBattery = max(newDistance - data.batteryCapacity, 0.0);
                }
                
                //=====================================Update Ride ===================================================
                newRide = (newDistance / data.milhasMinuto) + (numCustomers * data.customerService) + (numStations * data.stationService);

                newViolationTimeTour = max(newRide - data.maxTour, 0.0);

                //===========================================================================================
                newPenality = (vBattery * data.betaBattery) + (newViolationTimeTour * data.betaTour);
                newObjective = newDistance + newPenality;
        
                if(bestObjective - newObjective > EPS){
                    bestObjective   = newObjective;
                    bestVB          = vBattery;
                    bestRide        = newRide;
                    bestDistance    = newDistance;
                    bestVTimeT      = newViolationTimeTour;
                    bestR1          = r1;
                    bestR1_2        = r1_2;
                    bestR2          = r2;
                    invert          = false;
                }
                
                //=========================================Inverte========================================
                newDistance = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].distance;

                // ==================Update Battery============================
                vBattery = 0;
                if(data.eval[v][0][depot2].batteryStation){
                    if(data.eval[v][0][r1-1].batteryStation){
                        vBattery += data.eval[v][0][data.eval[v][0][r1-1].idStationToEnd].violation;
                        double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        double bat = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }
                    
                    if(data.eval[v][r1_2+1][r2].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r1_2+1][r2].idBeginToStation][data.eval[v][r1_2+1][r2].idStationToEnd].violation;
                    }else{
                        vBattery += max(data.eval[v][r1_2+1][r2].distance - data.batteryCapacity, 0.0);
                    }
                    
                    if(route[r1_2].batteryStation && route[r1].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(!route[r1_2].batteryStation && route[r1].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(route[r1_2].batteryStation && !route[r1].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else{
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                    if(data.eval[v][r2+1][depot2].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r2+1][depot2].idBeginToStation][depot2].violation;
                    }
                }else{
                    vBattery = max(newDistance - data.batteryCapacity, 0.0);
                }
                
                //=====================================Update Ride ===================================================
                newRide = (newDistance / data.milhasMinuto) + (numCustomers * data.customerService) + (numStations * data.stationService);

                newViolationTimeTour = max(newRide - data.maxTour, 0.0);

                //===========================================================================================
                newPenality  = (vBattery * data.betaBattery) + (newViolationTimeTour * data.betaTour);
                newObjective = newDistance + newPenality;
        
                if(bestObjective - newObjective > EPS){
                    bestObjective   = newObjective;
                    bestVB          = vBattery;
                    bestRide        = newRide;
                    bestDistance    = newDistance;
                    bestVTimeT      = newViolationTimeTour;
                    bestR1          = r1;
                    bestR1_2        = r1_2;
                    bestR2          = r2;
                    invert          = true;
                }
            }
        }

        if(bestR1 != -1){
            ride                = bestRide;
            distance        = bestDistance;
            violationBattery    = bestVB;
            violationTimeTour   = bestVTimeT;

            penality    = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
            objective   = distance + penality + stationCost + vehicleCost;
            
            if(bestR1 < bestR2){
                if(!invert){
                    route.insert(route.begin()+bestR2+1, route[bestR1_2]);
                    route.insert(route.begin()+bestR2+1, route[bestR1]);
                    route.erase(route.begin()+bestR1_2);
                    route.erase(route.begin()+bestR1);
                }else{
                    route.insert(route.begin()+bestR2+1, route[bestR1]);
                    route.insert(route.begin()+bestR2+1, route[bestR1_2]);
                    route.erase(route.begin()+bestR1_2);
                    route.erase(route.begin()+bestR1);
                }
            }else{
                if(!invert){
                    route.insert(route.begin()+bestR2+1, route[bestR1_2]);
                    route.insert(route.begin()+bestR2+1, route[bestR1+1]);
                    route.erase(route.begin()+bestR1_2+2);
                    route.erase(route.begin()+bestR1+2);
                }else{
                    route.insert(route.begin()+bestR2+1, route[bestR1]);
                    route.insert(route.begin()+bestR2+1, route[bestR1_2+1]);
                    route.erase(route.begin()+bestR1_2+2);
                    route.erase(route.begin()+bestR1+2);
                }
            }
            
            double distance, battery, violation, distanceLastStation, distanceFirstStation;
            bool thereAreBattery;
            int idStation;
            for(i = 0; i < (int)route.size() - 1; i++){
                distance            = 0;
                battery             = 0;
                violation           = 0;
                distanceLastStation = 0;
                thereAreBattery    = false;
                if(route[i].batteryStation){
                    idStation       = i;
                }else{
                    idStation       = -1;
                }

                for(j = i; j < (int)route.size() - 1; j++){
                    violation += max(battery - data.batteryCapacity, 0.0);

                    if(route[j].batteryStation){
                        thereAreBattery    = true;
                        battery             = 0;
                        distanceLastStation = 0;
                        idStation           = j;
                    }

                    data.eval[v][i][j].violation        = violation;
                    data.eval[v][i][j].distance         = distance;
                    data.eval[v][i][j].batteryStation   = thereAreBattery;
                    data.eval[v][i][j].stationToEnd     = distanceLastStation;
                    data.eval[v][i][j].idStationToEnd   = idStation;
                    
                    distance            += data.distances[route[j].id][route[j+1].id];
                    battery             += data.distances[route[j].id][route[j+1].id];
                    distanceLastStation += data.distances[route[j].id][route[j+1].id];                    
                }
                violation += max(battery - data.batteryCapacity, 0.0);

                data.eval[v][i][j].violation        = violation;
                data.eval[v][i][j].distance         = distance;
                data.eval[v][i][j].batteryStation   = thereAreBattery;                
                data.eval[v][i][j].stationToEnd     = distanceLastStation;
                data.eval[v][i][j].idStationToEnd   = idStation;
            }
            data.eval[v][i][i].violation        = 0;
            data.eval[v][i][i].distance         = 0;
            data.eval[v][i][i].batteryStation   = false;
            data.eval[v][i][i].stationToEnd     = 0;
            data.eval[v][i][i].idStationToEnd   = -1;

            for(i = (int)route.size() -1; i > 0; i--){
                distanceFirstStation    = 0;
                if(route[i].batteryStation){
                    idStation           = i;
                }else{
                    idStation           = -1;
                }
                for(j = i; j > 0; j--){
                    
                    if(route[j].batteryStation){
                        distanceFirstStation    = 0;
                        idStation               = j;
                    }

                    data.eval[v][j][i].beginToStation   = distanceFirstStation;
                    data.eval[v][j][i].idBeginToStation = idStation;
                    
                    distanceFirstStation    += data.distances[route[j].id][route[j-1].id];
                }
                data.eval[v][j][i].beginToStation   = distanceFirstStation;
                data.eval[v][j][i].idBeginToStation = idStation;
            }
            data.eval[v][i][i].beginToStation   = 0;
            data.eval[v][i][i].idBeginToStation = -1;
        }
    }
}

void Vehicle::intraRealocation(Data &data, int v) {
    int i, j, bestR1 = -1, bestR2 = -1;
    penality = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
    double bestObjective = distance + penality, newObjective, newPenality, bestVB = 0, bestRide = 0, bestDistance = 0, bestVTimeT = 0;
    vector < pair < int, int > > vt; 

    for (i = 1; i < (int)route.size()-1; i++) {
        for (j = 1; j < (int)route.size()-1; j++) {
            if(i != j && i != j-1 && i != j+1){
                vt.push_back(make_pair(i, j));
            }
        }
    }

    int depot2 = route.size() - 1;
    for (i = 0; i < (int)vt.size(); i++) {
        int r1 = vt[i].first;
        int r2 = vt[i].second;
        double newDistance  = 0;
        double newRide      = 0;
        double vBattery     = 0;
        double newViolationTimeTour;

        if(r1 < r2){
            newDistance = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance +  data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2].id] + data.eval[v][r2][depot2].distance;

            // ==================Update Battery============================
            vBattery = 0;
            if(data.eval[v][0][depot2].batteryStation){
                if(data.eval[v][0][r1-1].batteryStation){
                    vBattery += data.eval[v][0][data.eval[v][0][r1-1].idStationToEnd].violation;
                }
                if(data.eval[v][r1+1][r2-1].batteryStation){
                    double bat  = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].beginToStation;
                    vBattery    += max(bat - data.batteryCapacity, 0.0);

                    if(route[r1].batteryStation){
                        bat = data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id];
                        vBattery    += max(bat - data.batteryCapacity, 0.0);

                        bat = data.distances[route[r1].id][route[r2].id] + data.eval[v][r2][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);                    
                    }else{
                        bat = data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2].id] + data.eval[v][r2][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }
                }else{
                    double bat = 0;
                    if(route[r1].batteryStation){
                        bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id];
                        vBattery    += max(bat - data.batteryCapacity, 0.0);

                        bat = data.distances[route[r1].id][route[r2].id] + data.eval[v][r2][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);                    
                    }else{
                        bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2].id] + data.eval[v][r2][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }
                }
                if(data.eval[v][r2][depot2].batteryStation){
                    vBattery += data.eval[v][data.eval[v][r2][depot2].idBeginToStation][depot2].violation;
                }
            }else{
                vBattery = max(newDistance - data.batteryCapacity, 0.0);
            }
        }else{
            newDistance = data.eval[v][0][r2].distance + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][depot2].distance;

            // ==================Update Battery============================
            vBattery = 0;
            if(data.eval[v][0][depot2].batteryStation){
                if(data.eval[v][0][r2].batteryStation){
                    vBattery += data.eval[v][0][data.eval[v][0][r2].idStationToEnd].violation;
                }
                if(route[r1].batteryStation){
                    double bat  = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id];
                    vBattery    += max(bat - data.batteryCapacity, 0.0);

                    if(data.eval[v][r2+1][r1-1].batteryStation){
                        bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);

                        bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }
                }else{
                    double bat = 0;
                    if(data.eval[v][r2+1][r1-1].batteryStation){
                        bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);

                        bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }
                }
                if(data.eval[v][r1+1][depot2].batteryStation){
                    vBattery += data.eval[v][data.eval[v][r1+1][depot2].idBeginToStation][depot2].violation;
                }
            }else{
                vBattery = max(newDistance - data.batteryCapacity, 0.0);
            }
        }

        //=====================================Update Ride ===================================================
        newRide = (newDistance / data.milhasMinuto) + (numCustomers * data.customerService) + (numStations * data.stationService);

        newViolationTimeTour = max(newRide - data.maxTour, 0.0);

        newPenality  = (vBattery * data.betaBattery) + (newViolationTimeTour * data.betaTour);
        newObjective = newDistance + newPenality;

        if(bestObjective - newObjective > EPS){
            bestObjective   = newObjective;
            bestVB          = vBattery;
            bestRide        = newRide;
            bestDistance    = newDistance;
            bestVTimeT      = newViolationTimeTour;
            bestR1          = r1;
            bestR2          = r2;
        }
    }

    if(bestR1 != -1){
        ride                = bestRide;
        distance        = bestDistance;
        violationBattery    = bestVB;
        violationTimeTour   = bestVTimeT;

        penality            = (violationBattery * data.betaBattery) + (violationTimeTour * data.betaTour);
        objective           = distance + penality + stationCost + vehicleCost;

        if(bestR1 < bestR2){
            route.insert(route.begin()+bestR2, route[bestR1]);
            route.erase(route.begin()+bestR1);
        }else{
            route.insert(route.begin()+bestR2+1, route[bestR1]);
            route.erase(route.begin()+bestR1+1);
        }

        double distance, battery, violation, distanceLastStation, distanceFirstStation;
        bool thereAreBattery;
        int idStation;
        for(i = 0; i < (int)route.size() - 1; i++){
            distance            = 0;
            battery             = 0;
            violation           = 0;
            distanceLastStation = 0;
            thereAreBattery    = false;
            if(route[i].batteryStation){
                idStation       = i;
            }else{
                idStation       = -1;
            }

            for(j = i; j < (int)route.size() - 1; j++){
                violation += max(battery - data.batteryCapacity, 0.0);

                if(route[j].batteryStation){
                    thereAreBattery    = true;
                    battery             = 0;
                    distanceLastStation = 0;
                    idStation           = j;
                }

                data.eval[v][i][j].violation        = violation;
                data.eval[v][i][j].distance         = distance;
                data.eval[v][i][j].batteryStation   = thereAreBattery;
                data.eval[v][i][j].stationToEnd     = distanceLastStation;
                data.eval[v][i][j].idStationToEnd   = idStation;
                
                distance            += data.distances[route[j].id][route[j+1].id];
                battery             += data.distances[route[j].id][route[j+1].id];
                distanceLastStation += data.distances[route[j].id][route[j+1].id];                    
            }
            violation += max(battery - data.batteryCapacity, 0.0);

            data.eval[v][i][j].violation        = violation;
            data.eval[v][i][j].distance         = distance;
            data.eval[v][i][j].batteryStation   = thereAreBattery;                
            data.eval[v][i][j].stationToEnd     = distanceLastStation;
            data.eval[v][i][j].idStationToEnd   = idStation;
        }
        data.eval[v][i][i].violation        = 0;
        data.eval[v][i][i].distance         = 0;
        data.eval[v][i][i].batteryStation   = false;
        data.eval[v][i][i].stationToEnd     = 0;
        data.eval[v][i][i].idStationToEnd   = -1;

        for(i = (int)route.size() -1; i > 0; i--){
            distanceFirstStation    = 0;
            if(route[i].batteryStation){
                idStation           = i;
            }else{
                idStation           = -1;
            }
            for(j = i; j > 0; j--){
                
                if(route[j].batteryStation){
                    distanceFirstStation    = 0;
                    idStation               = j;
                }

                data.eval[v][j][i].beginToStation   = distanceFirstStation;
                data.eval[v][j][i].idBeginToStation = idStation;
                
                distanceFirstStation    += data.distances[route[j].id][route[j-1].id];
            }
            data.eval[v][j][i].beginToStation   = distanceFirstStation;
            data.eval[v][j][i].idBeginToStation = idStation;
        }
        data.eval[v][i][i].beginToStation   = 0;
        data.eval[v][i][i].idBeginToStation = -1;
    }
}

void Solution::showSolution() {
    for (int i = 0; i < (int)vehicles.size(); i++) {
        if((int)vehicles[i].route.size() < 3){
            continue;
        }
        for (int j = 0; j < (int)vehicles[i].route.size(); j++) {
            if(vehicles[i].route[j].batteryStation){
                cout << vehicles[i].route[j].sigla << "(S) - ";
            }else{
                cout << vehicles[i].route[j].sigla << " - ";
            }
        }
        cout << endl;
        cout << "     VB: " << vehicles[i].violationBattery << " | " << "     VTT: " << vehicles[i].violationTimeTour << " | ";
        cout << " Distance: " << vehicles[i].distance << " | ";
        cout << " Ride: " << vehicles[i].ride << endl;
    }
    cout << "Objective Solution: " << objective << endl;
    cout << "Distance: " << distance << endl;
    cout << "Amount Vehicles: " << amountVehicles << endl;
}

void Vehicle::showRoute() {
    for (int j = 0; j < (int)route.size(); j++) {
        if(route[j].batteryStation){
            cout << route[j].sigla << "(S) - ";
        }else{
            cout << route[j].sigla << " - ";
        }
    }
    cout << endl;

    cout << "   VTW: " << violationTw << " | " << "     VD: " << violationDemand << " | " << "  VB: " << violationBattery << " | " << "  VTT: " << violationTimeTour << " | ";
    cout << "Distance: " << distance << endl;
    cout << "Ride: " << ride << endl;
    cout << "Objectivo: " << objective;
}

void Solution::shakeBSS(Data &data, int intensity, int &lastShake){
    int iter = 0, maxIntensity = 3;
    
    if(intensity > maxIntensity){
        intensity   = maxIntensity;
    }

    while(iter < intensity){
            uniform_int_distribution<int> distribution(0, 100);
            int select = distribution(generator);

            if(select < 33.33){
                shakeRelocationBSS(data);
                lastShake = 1;
            }else if(select < 66.66){
                shakeSwapBSS(data);
                lastShake = 3;
            }else{
                shakeRemoveStationsBSS(data);
            }
        iter++;
    }
}

void Solution::shakeSwapBSS(Data &data){
    int v1, v2, r1, r2, k = 0;
    Vehicle car1, car2;

    if((int)vehicles.size() > 1){
    	uniform_int_distribution<int> distribution(0, (int)vehicles.size()-1);
    	do{
    		v1  = distribution(generator);
    		v2  = distribution(generator);
    		if(k > 20){
    			shakeRelocationBSS(data);
    			return;
    		}
    		k++;
    	}while(v1 == v2 || (int)vehicles[v1].route.size() < 3 || (int)vehicles[v2].route.size() < 3);

    	uniform_int_distribution<int> distribution2(1,(int)vehicles[v1].route.size()-2);
	    r1      = distribution2(generator);
	    uniform_int_distribution<int> distribution3(1,(int)vehicles[v2].route.size()-2);
	    r2      = distribution3(generator);
		
        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;
        // =========================================Update Distance==========================================================================
        distance                = distance - (vehicles[v1].distance + vehicles[v2].distance);
        vehicles[v1].distance   = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].distance;
        vehicles[v2].distance   = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].distance;
        distance                = distance + vehicles[v1].distance + vehicles[v2].distance;

        // ========================================Update Demand=======================================================================
        vehicles[v1].demand             = vehicles[v1].demand - vehicles[v1].route[r1].demand + vehicles[v2].route[r2].demand;
        vehicles[v2].demand             = vehicles[v2].demand - vehicles[v2].route[r2].demand + vehicles[v1].route[r1].demand;
                
        double oldVDemand1 = vehicles[v1].violationDemand;
        double oldVDemand2 = vehicles[v2].violationDemand;

        vehicles[v1].violationDemand = max(vehicles[v1].demand - data.demandCapacity, 0.0);
        vehicles[v2].violationDemand = max(vehicles[v2].demand - data.demandCapacity, 0.0);

        violationDemand = violationDemand - oldVDemand1 + vehicles[v1].violationDemand;
        violationDemand = violationDemand - oldVDemand2 + vehicles[v2].violationDemand;
        
        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || vehicles[v2].route[r2].batteryStation || data.eval[v1][r1+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            if(!vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].beginToStation;
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                
                bat = data.eval[v1][r1+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery1 = max(vehicles[v1].distance - data.batteryCapacity, 0.0);
        }
        
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || data.eval[v2][r2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(!vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(vehicles[v2].distance - data.batteryCapacity, 0.0);
        }

        violationBattery = violationBattery + (vBattery1 + vBattery2) - (vehicles[v1].violationBattery + vehicles[v2].violationBattery);

        vehicles[v1].violationBattery = vBattery1;
        vehicles[v2].violationBattery = vBattery2;

        penality = (violationDemand * data.betaDemand) + (violationBattery * data.betaBattery);

        objective = distance + vehicleCost + penality + stationCost;

        swap(vehicles[v1].route[r1], vehicles[v2].route[r2]);
        
        //======================================Update data===================================================================
        updateDataBSS(data, v1);
        updateDataBSS(data, v2);
    }
}

void Solution::shakeRelocationBSS(Data &data){
    int v1, v2, r1, r2;
    Vehicle car1, car2;

    if((int)vehicles.size() > 1){
        uniform_int_distribution<int> distribution(0, (int)vehicles.size()-1);
        do{
	        v1  = distribution(generator);
	    }while((int)vehicles[v1].route.size() < 3);
        if(v1 == 0){
            v2 = 1;
        }else{
            v2 = v1-1;
        }
    }else{
    	uniform_int_distribution<int> distribution(0, (int)vehicles.size()-1);
    	
    	do{
	        v1  = distribution(generator);
	        v2  = distribution(generator);
	    }while(v1 == v2 || (int)vehicles[v1].route.size() < 3);
    }

    uniform_int_distribution<int> distribution2(1,(int)vehicles[v1].route.size()-2);
    r1      = distribution2(generator);

    if((int)vehicles[v2].route.size() < 3){
        r2 = 1;
    }else{
        uniform_int_distribution<int> distribution3(1,(int)vehicles[v2].route.size()-2);
        r2      = distribution3(generator);
    }

    int depot2V1 = vehicles[v1].route.size()-1;
    int depot2V2 = vehicles[v2].route.size()-1;
    // =========================================Update Distance==========================================================================
    distance                = distance - (vehicles[v1].distance + vehicles[v2].distance);
    vehicles[v1].distance   = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].distance;
    vehicles[v2].distance   = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].distance;
    distance                = distance + vehicles[v1].distance + vehicles[v2].distance;

    // ========================================Update Demand=======================================================================
    vehicles[v1].demand             = vehicles[v1].demand - vehicles[v1].route[r1].demand;
    vehicles[v2].demand             = vehicles[v2].demand + vehicles[v1].route[r1].demand;
            
    double oldVDemand1 = vehicles[v1].violationDemand;
    double oldVDemand2 = vehicles[v2].violationDemand;

    vehicles[v1].violationDemand = max(vehicles[v1].demand - data.demandCapacity, 0.0);
    vehicles[v2].violationDemand = max(vehicles[v2].demand - data.demandCapacity, 0.0);

    violationDemand = violationDemand - oldVDemand1 + vehicles[v1].violationDemand;
    violationDemand = violationDemand - oldVDemand2 + vehicles[v2].violationDemand;
    
    //===========================================Update Battery======================================================================
    double vBattery1 = 0;
    double vBattery2 = 0;
    if(data.eval[v1][0][r1-1].batteryStation || vehicles[v1].route[r1].batteryStation || data.eval[v1][r1+1][depot2V1].batteryStation){
        double bat      = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].beginToStation;
        if(data.eval[v1][0][r1-1].batteryStation){
            vBattery1   += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
        }
        if(data.eval[v1][r1+1][depot2V1].batteryStation){
            vBattery1   += data.eval[v1][data.eval[v1][r1+1][depot2V1].idBeginToStation][depot2V1].violation;
        }
        vBattery1       += max(bat - data.batteryCapacity, 0.0);
    }else{
        vBattery1       = max(vehicles[v1].distance - data.batteryCapacity, 0.0);
    }

    if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || data.eval[v2][r2][depot2V2].batteryStation){
        if(data.eval[v2][0][r2-1].batteryStation){
            vBattery2   += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
        }
        if(data.eval[v2][r2][depot2V2].batteryStation){
            vBattery2   += data.eval[v2][data.eval[v2][r2][depot2V2].idBeginToStation][depot2V2].violation;
        }
        if(!vehicles[v1].route[r1].batteryStation){
            double bat  = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
            vBattery2   += max(bat - data.batteryCapacity, 0.0);
        }else{
            double bat  = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
            vBattery2   += max(bat - data.batteryCapacity, 0.0);
                        
            bat         = data.eval[v2][r2][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id];
            vBattery2   += max(bat - data.batteryCapacity, 0.0);
        }
    }else{
        vBattery2       = max(vehicles[v2].distance - data.batteryCapacity, 0.0);
    }

    violationBattery = violationBattery + (vBattery1 + vBattery2) - (vehicles[v1].violationBattery + vehicles[v2].violationBattery);
    vehicles[v1].violationBattery = vBattery1;
    vehicles[v2].violationBattery = vBattery2;

    vehicles[v2].route.insert(vehicles[v2].route.begin() + r2, vehicles[v1].route[r1]);
    vehicles[v1].route.erase(vehicles[v1].route.begin() + r1);
    
    if((int)vehicles[v1].route.size() < 3){
        vehicleCost = vehicleCost - vehicles[v1].vehicleCost;
        amountVehicles--;
    }
    if((int)vehicles[v2].route.size() == 3){
        vehicleCost = vehicleCost + vehicles[v2].vehicleCost;
        amountVehicles++;
    }

    penality = (violationDemand * data.betaDemand) + (violationBattery * data.betaBattery);
    objective = distance + vehicleCost + penality + stationCost;

    //======================================Update data===================================================================
    updateDataBSS(data, v1);
    updateDataBSS(data, v2);
}

void Solution::shakeRemoveStationsBSS(Data &data){
    int idStationRemove;
	vector<int> stations;

    for(int i = 0; i < (int)vehicles.size(); i++){
        if(vehicles[i].route.size() > 2){
            if(data.eval[i][0][vehicles[i].route.size()-1].batteryStation){
                for(int j = 1; j < (int)vehicles[i].route.size() -1; j++){
                    if(vehicles[i].route[j].batteryStation){
                        stations.push_back(vehicles[i].route[j].id);
                    }
                }
            }
        }
    }

    if(stations.size() == 0){
        uniform_int_distribution<int> distribution(0, 100);
        int select = distribution(generator);

        if(select < 50){
            shakeRelocationBSS(data);
        }else{
            shakeSwapBSS(data);
        }
        return;
    }
    uniform_int_distribution<int> distribution(0, stations.size()-1);

    idStationRemove = distribution(generator);
    vector < pair < int, int > > idRemove;

    for(int i = 0; i < (int)vehicles.size(); i++){
        if((int)vehicles[i].route.size() > 2){
            if(data.eval[i][0][(int)vehicles[i].route.size()-1].batteryStation){
                for(int j = 1; j < (int)vehicles[i].route.size() - 1; j++){
                    if(vehicles[i].route[j].id == stations[idStationRemove]){
                        idRemove.push_back(make_pair(i, j));
                    }
                }
            }
        }
    }

    for(int k = (int)idRemove.size() -1; k >= 0; k--){
        int v1 = idRemove[k].first;
        int r1 = idRemove[k].second;
        int depotV1 = vehicles[v1].route.size() - 1;

        double oldDistance      = vehicles[v1].distance;
        vehicles[v1].distance   = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depotV1].distance;
        distance                = distance + vehicles[v1].distance - oldDistance;
        
        double vBattery1 = 0;
        if(data.eval[v1][0][r1-1].batteryStation){
            vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
        }
        if(data.eval[v1][r1+1][depotV1].batteryStation){
            vBattery1 += data.eval[v1][data.eval[v1][r1+1][depotV1].idBeginToStation][depotV1].violation;
        }
        double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depotV1].beginToStation;
        vBattery1 += max(bat - data.batteryCapacity, 0.0);

        violationBattery = violationBattery + vBattery1 - vehicles[v1].violationBattery;
        vehicles[v1].violationBattery = vBattery1;
        
        if(data.usedStations[vehicles[v1].route[r1].id] == 1){
            stationCost                 = stationCost - vehicles[v1].route[r1].stationCost;
            vehicles[v1].stationCost    = vehicles[v1].stationCost - vehicles[v1].route[r1].stationCost;
        }
        data.usedStations[vehicles[v1].route[r1].id] = data.usedStations[vehicles[v1].route[r1].id] - 1;
        vehicles[v1].route.erase(vehicles[v1].route.begin() + r1);

        if((int)vehicles[v1].route.size() < 3){
            vehicleCost = vehicleCost - vehicles[v1].vehicleCost;
            amountVehicles--;
        }

        penality     = (violationBattery * data.betaBattery)  + (violationDemand * data.betaDemand);
        objective    = distance + vehicleCost + penality + stationCost;

        updateDataBSS(data, v1);
    }
}

void Solution::interRVNDBSS(Data &data, int lastShake){
    double bestObjective = objective;
    int k = 0;
    bool removeAddStation = false;
    vector < pair < int, int > > idRemove;
    vector<int> neighbors;
    for(int i = 0; i < (int)data.usedStations.size(); i++){
        if (data.usedStations[i] > 0){
            removeAddStation = true;
            break;
        }
    }
    neighbors.push_back(1);      //Realocação 1
    neighbors.push_back(2);      //Realocação 2
    neighbors.push_back(3);      //Swap
    neighbors.push_back(4);      //Swap 2
    neighbors.push_back(5);      //remove Station
    neighbors.push_back(6);      // Swap 2x1
    if(!removeAddStation){
        neighbors.push_back(7);      // Adicionar Estação
    }
    if(lastShake != 0){
        neighbors.erase(neighbors.begin() + lastShake-1);
    }


    shuffle(neighbors.begin(), neighbors.end(), generator);
    while(true){
        if (k >= (int)neighbors.size()){break;}

        if(neighbors[k] == 1){
            interRelocationBSS(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if(neighbors[k] == 2){
            interRelocation2BSS(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if(neighbors[k] == 3){
            interSwapBSS(data);
            
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 4){
            interSwap2BSS(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 5){
            removeStationBSS(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 6){
            interSwap2x1BSS(data);
            
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 7){
            addStationBSS(data);
            
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else{
            k++;
        }
    }
}

void Solution::addStationBSS(Data &data){
    bool improve = false;
    double bestObjective = objective, newObjective, newPenality, newStationCost = stationCost;
    double newViolationBattery = violationBattery, newDistance = distance, bestVBt1 = 0;
    double bestDistance1 = 0;
    int v1, r1, r2, bestV1 = 0, bestR1 = 0, bestR2 = 0;
    
    for(int i = 2; i < data.numBatteryStations; i++){
        for(int k = 0; k < (int)vehicles.size(); k++){
            if((int)vehicles[k].route.size() > 2){
                for(int j = 1; j < (int)vehicles[k].route.size() - 1; j++){
                	newStationCost = stationCost;
                    if((data.requests[i].id != vehicles[k].route[j-1].id) && (data.requests[i].id != vehicles[k].route[j+1].id)){
                        newDistance = distance;
                        newStationCost = stationCost;
                        newViolationBattery = violationBattery;
                        r2 = i;
                        v1 = k;
                        r1 = j;
                        int depotV1     = vehicles[v1].route.size() - 1;
                        double newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][data.requests[r2].id] + data.distances[data.requests[r2].id][vehicles[v1].route[r1].id] + data.eval[v1][r1][depotV1].distance;
                        newDistance         = newDistance + newDistance1 - vehicles[v1].distance;
                        
                        double vBattery1 = 0;
                        if(data.eval[v1][0][r1-1].batteryStation){
                            vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
                        }
                        if(data.eval[v1][r1][depotV1].batteryStation){
                            vBattery1 += data.eval[v1][data.eval[v1][r1][depotV1].idBeginToStation][depotV1].violation;
                        }
                        double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][data.requests[r2].id];
                        vBattery1 += max(bat - data.batteryCapacity, 0.0);
                        
                        bat = data.eval[v1][r1][depotV1].beginToStation + data.distances[data.requests[r2].id][vehicles[v1].route[r1].id];
                        vBattery1 += max(bat - data.batteryCapacity, 0.0);
                        
                        newViolationBattery = newViolationBattery + vBattery1 - vehicles[v1].violationBattery;
                        if(data.usedStations[data.requests[r2].id] == 0){
                            newStationCost = newStationCost + data.requests[r2].stationCost;
                        }

                        // ===========================================Update custSolution====================================================================
                        newPenality     = (newViolationBattery * data.betaBattery)  + (violationDemand * data.betaDemand);
                        newObjective    = newDistance + vehicleCost + newPenality + newStationCost;
                        
                        if(bestObjective - newObjective > EPS){
                            bestDistance1   = newDistance1;
                            bestV1          = v1;
                            bestR1          = r1;
                            bestR2          = r2;
                            bestVBt1        = vBattery1;
                            improve         = true;
                            bestObjective   = newObjective;
                        }
                    }
                }
            }
        }
    }

    if(improve){
        distance                    = distance - vehicles[bestV1].distance;
        vehicles[bestV1].distance   = bestDistance1;
        distance                    = distance + vehicles[bestV1].distance;

        violationBattery                    = violationBattery - vehicles[bestV1].violationBattery;
        vehicles[bestV1].violationBattery   = bestVBt1;
        violationBattery                    = violationBattery + bestVBt1;

        penality = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);

        if(data.usedStations[data.requests[bestR2].id] == 0){
            stationCost                                    = stationCost + data.requests[bestR2].stationCost;
            vehicles[bestV1].stationCost                   = vehicles[bestV1].stationCost + data.requests[bestR2].stationCost;
            data.usedStations[data.requests[bestR2].id]    = data.usedStations[data.requests[bestR2].id] + 1;
        }else{
            data.usedStations[data.requests[bestR2].id]    = data.usedStations[data.requests[bestR2].id] + 1;
        }

        objective = distance + vehicleCost + penality + stationCost;

        vehicles[bestV1].route.insert(vehicles[bestV1].route.begin() + bestR1, data.requests[bestR2]);
        
        //======================================Update data===================================================================
        updateDataBSS(data, bestV1);

        double oldDistance              = vehicles[bestV1].distance;
        double oldViolationBattery  = vehicles[bestV1].violationBattery;

        vehicles[bestV1].intraRVNDBSS(data, bestV1);

        distance            = distance + vehicles[bestV1].distance - oldDistance;
        violationBattery    = violationBattery + vehicles[bestV1].violationBattery - oldViolationBattery;

        penality            = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::removeStationBSS(Data &data){
    int v1, r1, bestV1, bestR1;
    bool improve = false;
    double newObjective, bestObjective = objective, newPenality, newVehiclesCost, newStationCost = stationCost;
    double newViolationBattery = violationBattery, newDistance = distance, bestVBt1;
    double bestDistance1 = 0;
    vector < pair < int, int > > idRemove;

    for(int i = 0; i < (int)vehicles.size(); i++){
        if((int)vehicles[i].route.size() > 2){
            for(int j = 1; j < (int)vehicles[i].route.size() - 1; j++){
                if(vehicles[i].route[j].batteryStation){
                    if(vehicles[i].route[j-1].id == vehicles[i].route[j].id){
                        idRemove.push_back(make_pair(i, j-1));
                    }
                }
            }
        }
    }
    
    if(idRemove.size() > 0){
        int v = idRemove[(int)idRemove.size() -1].first;
        for(int k = (int)idRemove.size() -1; k >= 0; k--){
            if(v != idRemove[k].first){
                updateDataBSS(data, v);
                v = idRemove[k].first;
            }

            data.usedStations[vehicles[idRemove[k].first].route[idRemove[k].second].id] = data.usedStations[vehicles[idRemove[k].first].route[idRemove[k].second].id] - 1;
            vehicles[idRemove[k].first].route.erase(vehicles[idRemove[k].first].route.begin() + idRemove[k].second);
            if(vehicles[idRemove[k].first].route.size() < 3){
                amountVehicles--;
            }
        }
        updateDataBSS(data, v);
    }
    
    for(int i = 0; i < (int)vehicles.size(); i++){
        if((int)vehicles[i].route.size() > 2){
            for(int j = 1; j < (int)vehicles[i].route.size() - 1; j++){
                newDistance         = distance;
                newStationCost      = stationCost;
                newVehiclesCost     = vehicleCost;
                newViolationBattery = violationBattery;

                if(vehicles[i].route[j].batteryStation){
                    v1 = i;
                    r1 = j;
                    int depotV1         = vehicles[v1].route.size() - 1;
                    double newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depotV1].distance;
                    newDistance         = newDistance + newDistance1 - vehicles[v1].distance;
                    
                    double vBattery1 = 0;
                    if(data.eval[v1][0][r1-1].batteryStation){
                        vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
                    }
                    if(data.eval[v1][r1+1][depotV1].batteryStation){
                        vBattery1 += data.eval[v1][data.eval[v1][r1+1][depotV1].idBeginToStation][depotV1].violation;
                    }
                    double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depotV1].beginToStation;
                    vBattery1 += max(bat - data.batteryCapacity, 0.0);

                    newViolationBattery = newViolationBattery + vBattery1 - vehicles[v1].violationBattery;
                    
                    if(data.usedStations[vehicles[v1].route[r1].id] == 1){
                        newStationCost = newStationCost - vehicles[v1].route[r1].stationCost;
                    }

                    if((int)vehicles[v1].route.size() < 3){
                        newVehiclesCost = newVehiclesCost - vehicles[v1].vehicleCost;
                    }

                    // ===========================================Update custSolution====================================================================
                    newPenality     = (newViolationBattery * data.betaBattery)  + (violationDemand * data.betaDemand);
                    newObjective    = newDistance + newVehiclesCost + newPenality + newStationCost;
                    
                    if(bestObjective - newObjective > EPS){
                        bestDistance1   = newDistance1;
                        bestV1          = i;
                        bestR1          = j;
                        bestVBt1        = vBattery1;
                        improve         = true;
                        bestObjective   = newObjective;
                    }
                }
            }
        }
    }

    if(improve){
        distance                    = distance - vehicles[bestV1].distance;
        vehicles[bestV1].distance   = bestDistance1;
        distance                    = distance + vehicles[bestV1].distance;

        violationBattery                    = violationBattery - vehicles[bestV1].violationBattery;
        vehicles[bestV1].violationBattery   = bestVBt1;
        violationBattery                    = violationBattery + bestVBt1;

        penality = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);

        if(data.usedStations[vehicles[bestV1].route[bestR1].id] == 1){
            stationCost                                             = stationCost - vehicles[bestV1].route[bestR1].stationCost;
            vehicles[bestV1].stationCost                            = vehicles[bestV1].stationCost - vehicles[bestV1].route[bestR1].stationCost;
            data.usedStations[vehicles[bestV1].route[bestR1].id]    = data.usedStations[vehicles[bestV1].route[bestR1].id] -1;
        }else{
            data.usedStations[vehicles[bestV1].route[bestR1].id]    = data.usedStations[vehicles[bestV1].route[bestR1].id] -1;
        }

        vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1);
        
        if((int)vehicles[bestV1].route.size() < 3){
            vehicleCost = vehicleCost - vehicles[bestV1].vehicleCost;
            amountVehicles--;
        }

        objective = distance + vehicleCost + penality + stationCost;
        //======================================Update data===================================================================
        updateDataBSS(data, bestV1);

        double oldDistance              = vehicles[bestV1].distance;
        double oldViolationBattery  = vehicles[bestV1].violationBattery;
        
        vehicles[bestV1].intraRVNDBSS(data, bestV1);
        
        distance            = distance + vehicles[bestV1].distance - oldDistance;
        violationBattery    = violationBattery + vehicles[bestV1].violationBattery - oldViolationBattery;
        
        penality            = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::interRelocationBSS(Data &data){
    int bestV1, bestV2, bestR1, bestR2;
    double newPenality, bestObjective = objective, newObjective;
    double newViolationBattery = violationBattery;
    double newViolationDemand = violationDemand;
    double newVehicleCost = vehicleCost;
    double newDistance = distance;
    double bestVBt1 = 0;
    double bestVBt2 = 0;
    double bestVD1 = 0;
    double bestVD2 = 0;
    double bestDistance1 = 0, bestDistance2 = 0;
    bool improve = true;

    vector < pair < pair < int, int >, pair < int, int > > > vt;
    improve = false;
    for(int i = 0; i < (int)vehicles.size();i++) {
        if((int)vehicles[i].route.size() > 2){
            for(int j = 0; j < (int)vehicles.size(); j++) {
                if(i != j){
                    for(int r1 = 1; r1 < (int)vehicles[i].route.size()-1; r1++) {
                        if((int)vehicles[j].route.size() < 3){
                            vt.push_back(make_pair(make_pair(i, r1), make_pair(j, 1)));
                        }else{
                            for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                                vt.push_back(make_pair(make_pair(i, r1), make_pair(j, r2)));
                            }                        
                        }
                    }
                }
            }
        }
    }

    shuffle(vt.begin(), vt.end(), generator);
    for(int i = 0; i < (int)vt.size(); i++){
        newVehicleCost      = vehicleCost;
        newViolationBattery = violationBattery;
        newViolationDemand  = violationDemand;
        newDistance             = distance;

        int v1 = vt[i].first.first;
        int v2 = vt[i].second.first;
        int r1 = vt[i].first.second;
        int r2 = vt[i].second.second;
        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;

        // =========================================Update Distance==========================================================================
        double newDistance1;
        double newDistance2;
        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        // ========================================Update Demand=======================================================================
        double vDemand1 = 0;
        double vDemand2 = 0;
        double newDemand1 = vehicles[v1].demand - vehicles[v1].route[r1].demand;
        double newDemand2 = vehicles[v2].demand + vehicles[v1].route[r1].demand;
        
        vDemand1 = max(newDemand1 - data.demandCapacity, 0.0);
        vDemand2 = max(newDemand2 - data.demandCapacity, 0.0);

        newViolationDemand = newViolationDemand + (vDemand1 + vDemand2) - (vehicles[v2].violationDemand + vehicles[v1].violationDemand);

        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || vehicles[v1].route[r1].batteryStation || data.eval[v1][r1+1][depot2V1].batteryStation){
            double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].beginToStation;
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            vBattery1 += max(bat - data.batteryCapacity, 0.0);
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }

        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || data.eval[v2][r2][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(!vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;

        // ===========================================Update Vehicles cost====================================================================
        if((int)vehicles[v1].route.size() < 3){
            newVehicleCost = newVehicleCost - vehicles[v1].vehicleCost;
        }
        if((int)vehicles[v2].route.size() == 3){
            newVehicleCost = newVehicleCost + vehicles[v2].vehicleCost;
        }

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationDemand * data.betaDemand);
        newObjective    = newDistance + newVehicleCost + newPenality + stationCost;
        
        if(bestObjective - newObjective > EPS){
            bestV1          = vt[i].first.first;
            bestV2          = vt[i].second.first;
            bestR1          = vt[i].first.second;
            bestR2          = vt[i].second.second;
            bestDistance1       = newDistance1;
            bestDistance2       = newDistance2;
            bestVD1         = vDemand1;
            bestVD2         = vDemand2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
        }
    }
    
    if(improve){
        distance                    = distance - (vehicles[bestV1].distance + vehicles[bestV2].distance);
        vehicles[bestV1].distance   = bestDistance1;
        vehicles[bestV2].distance   = bestDistance2;
        distance                    = distance + vehicles[bestV1].distance + vehicles[bestV2].distance;
        
        vehicles[bestV1].demand             = vehicles[bestV1].demand - vehicles[bestV1].route[bestR1].demand;
        vehicles[bestV2].demand             = vehicles[bestV2].demand + vehicles[bestV1].route[bestR1].demand;
        violationDemand                     = violationDemand - (vehicles[bestV1].violationDemand + vehicles[bestV2].violationDemand);
        vehicles[bestV1].violationDemand    = bestVD1;
        vehicles[bestV2].violationDemand    = bestVD2;
        violationDemand                     = violationDemand + (bestVD1 + bestVD2);
        
        violationBattery                    = violationBattery - (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery);
        vehicles[bestV1].violationBattery   = bestVBt1;
        vehicles[bestV2].violationBattery   = bestVBt2;
        violationBattery                    = violationBattery + bestVBt1 + bestVBt2;

        penality = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);

        vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1]);
        vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1);
        
        if((int)vehicles[bestV1].route.size() < 3){
            vehicleCost = vehicleCost - vehicles[bestV1].vehicleCost;
            amountVehicles--;
        }
        if((int)vehicles[bestV2].route.size() == 3){
            vehicleCost = vehicleCost + vehicles[bestV2].vehicleCost;
            amountVehicles++;
        }

        objective = distance + vehicleCost + penality + stationCost;
        //======================================Update data===================================================================
        updateDataBSS(data, bestV1);
        updateDataBSS(data, bestV2);

        double oldDistance1             = vehicles[bestV1].distance;
        double oldViolationBattery1     = vehicles[bestV1].violationBattery;
        double oldDistance2             = vehicles[bestV2].distance;
        double oldViolationBattery2     = vehicles[bestV2].violationBattery;
        
        vehicles[bestV1].intraRVNDBSS(data, bestV1);
        vehicles[bestV2].intraRVNDBSS(data, bestV2);
        
        distance            = distance + (vehicles[bestV1].distance + vehicles[bestV2].distance) - (oldDistance1 + oldDistance2);
        violationBattery    = violationBattery + (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery) - (oldViolationBattery1 + oldViolationBattery2);

        penality            = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::interRelocation2BSS(Data &data){
    int bestV1, bestV2, bestR1, bestR1_2, bestR2, flag = 0;
    double bestVD1, bestVD2, bestVBt1, bestVBt2;
    double newPenality, newObjective;
    double newVehicleCost = vehicleCost;
    double bestObjective = objective;
    double newViolationBattery = violationBattery;
    double newViolationDemand = violationDemand;
    double newDistance = distance;
    double bestDistance1 = 0, bestDistance2 = 0;
    bool improve = true;

    vector < pair < pair < pair < int, int >, int >, pair < int, int > > > vt;
    improve = false;
    for(int i = 0; i < (int)vehicles.size();i++) {
        if((int)vehicles[i].route.size() > 3){
            for(int j = 0; j < (int)vehicles.size(); j++) {
                if(i != j){
                    for(int r1 = 1; r1 < (int)vehicles[i].route.size()-2; r1++) {
                        if((int)vehicles[j].route.size() < 3){
                            vt.push_back(make_pair(make_pair(make_pair(i, r1), r1+1), make_pair(j, 1)));
                        }else{
                            for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                                vt.push_back(make_pair(make_pair(make_pair(i, r1), r1+1), make_pair(j, r2)));
                            }                        
                        }
                    }
                }
            }
        }
    }

    shuffle(vt.begin(), vt.end(), generator);
    for(int i = 0; i < (int)vt.size(); i++){
        newVehicleCost      = vehicleCost;
        newViolationBattery = violationBattery;
        newViolationDemand  = violationDemand;
        newDistance         = distance;

        int v1      = vt[i].first.first.first;
        int v2      = vt[i].second.first;
        int r1      = vt[i].first.first.second;
        int r1_2    = vt[i].first.second;
        int r2      = vt[i].second.second;

        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;

        // =========================================Update Distance==========================================================================
        double newDistance1;
        double newDistance2;
        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        // ========================================Update Demand=======================================================================
        double vDemand1 = 0;
        double vDemand2 = 0;
        double newDemand1 = vehicles[v1].demand - (vehicles[v1].route[r1].demand + vehicles[v1].route[r1_2].demand);
        double newDemand2 = vehicles[v2].demand + (vehicles[v1].route[r1].demand + vehicles[v1].route[r1_2].demand);
        
        vDemand1 = max(newDemand1 - data.demandCapacity, 0.0);
        vDemand2 = max(newDemand2 - data.demandCapacity, 0.0);

        newViolationDemand = newViolationDemand + (vDemand1 + vDemand2) - (vehicles[v2].violationDemand + vehicles[v1].violationDemand);

        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || data.eval[v1][r1_2+1][depot2V1].batteryStation){
            double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].beginToStation;
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1_2+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1_2+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            vBattery1 += max(bat - data.batteryCapacity, 0.0);
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }

        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1].batteryStation && !vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + (vBattery1 + vBattery2) - (vehicles[v1].violationBattery + vehicles[v2].violationBattery);

        // ===========================================Update Vehicles cost====================================================================
        if((int)vehicles[v1].route.size() < 3){
            newVehicleCost = newVehicleCost - vehicles[v1].vehicleCost;
        }
        if((int)vehicles[v2].route.size() == 4){
            newVehicleCost = newVehicleCost + vehicles[v2].vehicleCost;
        }

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationDemand * data.betaDemand);
        newObjective    = newDistance + newVehicleCost + newPenality + stationCost;

        if(bestObjective - newObjective > EPS){
            bestV1          = vt[i].first.first.first;
            bestV2          = vt[i].second.first;
            bestR1          = vt[i].first.first.second;
            bestR1_2        = vt[i].first.second;
            bestR2          = vt[i].second.second;
            bestVD1         = vDemand1;
            bestVD2         = vDemand2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            flag            = 1;
            bestDistance1       = newDistance1;
            bestDistance2       = newDistance2;
        }

        //=============================================Invertendo ================================================
        newViolationBattery = violationBattery;
        newDistance         = distance;

        // =========================================Update Distance==========================================================================
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        vBattery2 = 0;
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1_2].batteryStation && !vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2].id] + data.eval[v2][r2][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - (vehicles[v1].violationBattery + vehicles[v2].violationBattery);

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationDemand * data.betaDemand);
        newObjective    = newDistance + newVehicleCost + newPenality + stationCost;
        
        if(bestObjective - newObjective > EPS){
            bestV1          = vt[i].first.first.first;
            bestV2          = vt[i].second.first;
            bestR1          = vt[i].first.first.second;
            bestR1_2        = vt[i].first.second;
            bestR2          = vt[i].second.second;
            bestVD1         = vDemand1;
            bestVD2         = vDemand2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            flag            = 2;
            bestDistance1       = newDistance1;
            bestDistance2       = newDistance2;
            // break;
        }
    }

    if(improve){
        distance                    = distance - (vehicles[bestV1].distance + vehicles[bestV2].distance);
        vehicles[bestV1].distance   = bestDistance1;
        vehicles[bestV2].distance   = bestDistance2;
        distance                    = distance + vehicles[bestV1].distance + vehicles[bestV2].distance;
        
        vehicles[bestV1].demand             = vehicles[bestV1].demand - (vehicles[bestV1].route[bestR1].demand + vehicles[bestV1].route[bestR1_2].demand);
        vehicles[bestV2].demand             = vehicles[bestV2].demand + (vehicles[bestV1].route[bestR1].demand + vehicles[bestV1].route[bestR1_2].demand);
        violationDemand                     = violationDemand - (vehicles[bestV1].violationDemand + vehicles[bestV2].violationDemand);
        vehicles[bestV1].violationDemand    = bestVD1;
        vehicles[bestV2].violationDemand    = bestVD2;
        violationDemand                     = violationDemand + (bestVD1 + bestVD2);
        
        violationBattery                    = violationBattery - (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery);
        vehicles[bestV1].violationBattery   = bestVBt1;
        vehicles[bestV2].violationBattery   = bestVBt2;
        violationBattery                    = violationBattery + bestVBt1 + bestVBt2;

        penality = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);

        if(flag == 1){
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1_2]);
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1]);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1_2);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1);
        }else{
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1]);
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1_2]);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1_2);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1);
        }
        
        if((int)vehicles[bestV1].route.size() < 3){
            vehicleCost = vehicleCost - vehicles[bestV1].vehicleCost;
            amountVehicles--;
        }
        if((int)vehicles[bestV2].route.size() == 4){
            vehicleCost = vehicleCost + vehicles[bestV2].vehicleCost;
            amountVehicles++;
        }
        
        objective = distance + vehicleCost + penality + stationCost;
        //======================================Update data===================================================================
        updateDataBSS(data, bestV1);
        updateDataBSS(data, bestV2);

        double oldDistance1             = vehicles[bestV1].distance;
        double oldViolationBattery1     = vehicles[bestV1].violationBattery;
        double oldDistance2             = vehicles[bestV2].distance;
        double oldViolationBattery2     = vehicles[bestV2].violationBattery;
        
        vehicles[bestV1].intraRVNDBSS(data, bestV1);
        vehicles[bestV2].intraRVNDBSS(data, bestV2);
        
        distance            = distance + (vehicles[bestV1].distance + vehicles[bestV2].distance) - (oldDistance1 + oldDistance2);
        violationBattery    = violationBattery + (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery) - (oldViolationBattery1 + oldViolationBattery2);

        penality            = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::interSwapBSS(Data &data){
    int bestV1 = 0, bestV2 = 0, bestR1 = 0, bestR2 = 0;
    double newPenality, bestObjective = objective, newObjective;
    double newViolationBattery = violationBattery;
    double newViolationDemand = violationDemand;
    double newDistance = distance;
    double bestVBt1 = 0;
    double bestVBt2 = 0;
    double bestVD1 = 0;
    double bestVD2 = 0;
    double bestDistance1 = 0, bestDistance2 = 0;
    bool improve = true;

    vector < pair < pair < int, int >, pair < int, int > > > vt;
    improve = false;
    for(int i = 0; i < (int)vehicles.size();i++) {
        if((int)vehicles[i].route.size() > 2){
            for(int j = 0; j < (int)vehicles.size(); j++) {
                if(i != j){
                    for(int r1 = 1; r1 < (int)vehicles[i].route.size()-1; r1++) {
                        if((int)vehicles[j].route.size() > 2){
                            for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                                vt.push_back(make_pair(make_pair(i, r1), make_pair(j, r2)));
                            }                        
                        }
                    }
                }
            }
        }
    }

    shuffle(vt.begin(), vt.end(), generator);
    for(int i = 0; i < (int)vt.size(); i++){
        newViolationBattery = violationBattery;
        newViolationDemand  = violationDemand;
        newDistance         = distance;

        int v1 = vt[i].first.first;
        int v2 = vt[i].second.first;
        int r1 = vt[i].first.second;
        int r2 = vt[i].second.second;

        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;
        
        // =========================================Update Distance==========================================================================
        double newDistance1;
        double newDistance2;
        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        // ========================================Update Demand=======================================================================
        double vDemand1 = 0;
        double vDemand2 = 0;
        double newDemand1 = vehicles[v1].demand - vehicles[v1].route[r1].demand + vehicles[v2].route[r2].demand;
        double newDemand2 = vehicles[v2].demand - vehicles[v2].route[r2].demand + vehicles[v1].route[r1].demand;
        
        vDemand1 = max(newDemand1 - data.demandCapacity, 0.0);
        vDemand2 = max(newDemand2 - data.demandCapacity, 0.0);

        newViolationDemand = newViolationDemand + (vDemand1 + vDemand2) - (vehicles[v2].violationDemand + vehicles[v1].violationDemand);

        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || vehicles[v2].route[r2].batteryStation || data.eval[v1][r1+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            if(!vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id] + data.eval[v1][r1+1][depot2V1].beginToStation;
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                
                bat = data.eval[v1][r1+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }
        
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || data.eval[v2][r2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(!vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationDemand * data.betaDemand);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;

        if(bestObjective - newObjective > EPS){
            bestV1          = vt[i].first.first;
            bestV2          = vt[i].second.first;
            bestR1          = vt[i].first.second;
            bestR2          = vt[i].second.second;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            bestVD1         = vDemand1;
            bestVD2         = vDemand2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            // break;
        }
    }

    if(improve){
        distance                    = distance - (vehicles[bestV1].distance + vehicles[bestV2].distance);
        vehicles[bestV1].distance   = bestDistance1;
        vehicles[bestV2].distance   = bestDistance2;
        distance                    = distance + vehicles[bestV1].distance + vehicles[bestV2].distance;

        vehicles[bestV1].demand             = vehicles[bestV1].demand - vehicles[bestV1].route[bestR1].demand + vehicles[bestV2].route[bestR2].demand;
        vehicles[bestV2].demand             = vehicles[bestV2].demand - vehicles[bestV2].route[bestR2].demand + vehicles[bestV1].route[bestR1].demand;
        violationDemand                     = violationDemand - (vehicles[bestV1].violationDemand + vehicles[bestV2].violationDemand);
        vehicles[bestV1].violationDemand    = bestVD1;
        vehicles[bestV2].violationDemand    = bestVD2;
        violationDemand                     = violationDemand + (bestVD1 + bestVD2);
        
        violationBattery                    = violationBattery - (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery);
        vehicles[bestV1].violationBattery   = bestVBt1;
        vehicles[bestV2].violationBattery   = bestVBt2;
        violationBattery                    = violationBattery + bestVBt1 + bestVBt2;

        penality = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);

        swap(vehicles[bestV1].route[bestR1], vehicles[bestV2].route[bestR2]);

        objective = distance + vehicleCost + penality + stationCost;

        //======================================Update data===================================================================
        updateDataBSS(data, bestV1);
        updateDataBSS(data, bestV2);

        double oldDistance1             = vehicles[bestV1].distance;
        double oldViolationBattery1 = vehicles[bestV1].violationBattery;
        double oldDistance2             = vehicles[bestV2].distance;
        double oldViolationBattery2 = vehicles[bestV2].violationBattery;
        
        vehicles[bestV1].intraRVNDBSS(data, bestV1);
        vehicles[bestV2].intraRVNDBSS(data, bestV2);
        
        distance            = distance + (vehicles[bestV1].distance + vehicles[bestV2].distance) - (oldDistance1 + oldDistance2);
        violationBattery    = violationBattery + (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery) - (oldViolationBattery1 + oldViolationBattery2);
        
        penality            = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::interSwap2BSS(Data &data){
    int bestV1 = 0, bestV2 = 0, bestR1 = 0, bestR1_2 = 0, bestR2 = 0, bestR2_2 = 0, flag = 0;
    double bestVD1 = 0, bestVD2 = 0, bestVBt1 = 0, bestVBt2 = 0;
    double newPenality, newObjective;
    double bestObjective = objective;
    double newViolationBattery = violationBattery;
    double newViolationDemand = violationDemand;
    double newDistance = distance;
    double bestDistance1 = 0, bestDistance2 = 0;
    bool improve = false;

    vector < pair < pair < pair < int, int >, int >, pair < pair < int, int >, int > > > vt;

    for(int i = 0; i < (int)vehicles.size(); i++) {
        for(int j = i; j < (int)vehicles.size(); j++) {
            if((int)vehicles[i].route.size() > 3 && (int)vehicles[j].route.size() > 3 && (i != j)){
                for(int r1 = 1; r1 < (int)vehicles[i].route.size()-2; r1++) {                    
                    for(int r2 = 1; r2 < (int)vehicles[j].route.size()-2; r2++) {
                        vt.push_back( make_pair( make_pair( make_pair(i, r1), r1+1), make_pair( make_pair(j, r2), r2+1)));
                    }
                }
            }
        }
    }

    
    shuffle(vt.begin(), vt.end(), generator);
    for(int i = 0; i < (int)vt.size(); i++){
        newViolationBattery = violationBattery;
        newViolationDemand  = violationDemand;
        newDistance         = distance;

        int v1      = vt[i].first.first.first;
        int v2      = vt[i].second.first.first;
        int r1      = vt[i].first.first.second;
        int r1_2    = vt[i].first.second;
        int r2      = vt[i].second.first.second;
        int r2_2    = vt[i].second.second;

        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;
        
        // =========================================Update Distance==========================================================================
        double newDistance1;
        double newDistance2;
        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v2].route[r2_2].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        // ========================================Update Demand=======================================================================
        double vDemand1 = 0;
        double vDemand2 = 0;
        double newDemand1 = vehicles[v1].demand - (vehicles[v1].route[r1].demand + vehicles[v1].route[r1_2].demand) + (vehicles[v2].route[r2].demand + vehicles[v2].route[r2_2].demand);
        double newDemand2 = vehicles[v2].demand - (vehicles[v2].route[r2].demand + vehicles[v2].route[r2_2].demand) + (vehicles[v1].route[r1].demand + vehicles[v1].route[r1_2].demand);
        
        vDemand1 = max(newDemand1 - data.demandCapacity, 0.0);
        vDemand2 = max(newDemand2 - data.demandCapacity, 0.0);

        newViolationDemand = newViolationDemand + (vDemand1 + vDemand2) - (vehicles[v2].violationDemand + vehicles[v1].violationDemand);

        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || vehicles[v2].route[r2].batteryStation || vehicles[v2].route[r2_2].batteryStation || data.eval[v1][r1_2+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1_2+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1_2+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            if(vehicles[v2].route[r2].batteryStation && vehicles[v2].route[r2_2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2_2].id][vehicles[v1].route[r1_2+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v2].route[r2].batteryStation && vehicles[v2].route[r2_2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v2].route[r2_2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2_2].id][vehicles[v1].route[r1_2+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v2].route[r2].batteryStation && !vehicles[v2].route[r2_2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2_2].id][vehicles[v1].route[r1_2+1].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v2].route[r2_2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v2].route[r2_2].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].beginToStation;
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2_2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2_2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2_2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1].batteryStation && !vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;
        
        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationDemand * data.betaDemand);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;
        
        if(bestObjective - newObjective > EPS){
            bestV1          = vt[i].first.first.first;
            bestV2          = vt[i].second.first.first;
            bestR1          = vt[i].first.first.second;
            bestR1_2        = vt[i].first.second;
            bestR2          = vt[i].second.first.second;
            bestR2_2        = vt[i].second.second;
            bestVD1         = vDemand1;
            bestVD2         = vDemand2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            flag            = 1;
        }
            
        //=====================================================Inverte V2==================================================================
        newViolationBattery = violationBattery;
        newDistance         = distance;

        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        vBattery2 = 0;
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2_2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2_2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2_2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1_2].batteryStation && !vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2_2+1].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;

        // ===========================================Update custSolution====================================================================
        
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationDemand * data.betaDemand);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;
        
        if(bestObjective - newObjective > EPS){
            bestV1          = vt[i].first.first.first;
            bestV2          = vt[i].second.first.first;
            bestR1          = vt[i].first.first.second;
            bestR1_2        = vt[i].first.second;
            bestR2          = vt[i].second.first.second;
            bestR2_2        = vt[i].second.second;
            bestVD1         = vDemand1;
            bestVD2         = vDemand2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            flag            = 2;
        }
        
        //====================================================Inverte V1=====================================================]
        newViolationBattery = violationBattery;
        newDistance         = distance;

        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2_2].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        vBattery1 = 0;
        vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || vehicles[v2].route[r2].batteryStation || vehicles[v2].route[r2_2].batteryStation || data.eval[v1][r1_2+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1_2+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1_2+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            if(vehicles[v2].route[r2_2].batteryStation && vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2_2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v2].route[r2_2].batteryStation && vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2_2].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v2].route[r2_2].batteryStation && !vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2_2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2_2].id] + data.distances[vehicles[v2].route[r2_2].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].beginToStation;
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }
        
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2_2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2_2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2_2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1].batteryStation && !vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2_2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2_2+1].id] + data.eval[v2][r2_2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
            
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationDemand * data.betaDemand);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;

        if(bestObjective - newObjective > EPS){
            bestV1          = vt[i].first.first.first;
            bestV2          = vt[i].second.first.first;
            bestR1          = vt[i].first.first.second;
            bestR1_2        = vt[i].first.second;
            bestR2          = vt[i].second.first.second;
            bestR2_2        = vt[i].second.second;
            bestVD1         = vDemand1;
            bestVD2         = vDemand2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            flag            = 3;
        }
    }

    if(improve){
        distance                    = distance - (vehicles[bestV1].distance + vehicles[bestV2].distance);
        vehicles[bestV1].distance   = bestDistance1;
        vehicles[bestV2].distance   = bestDistance2;
        distance                    = distance + vehicles[bestV1].distance + vehicles[bestV2].distance;

        vehicles[bestV1].demand             = vehicles[bestV1].demand - (vehicles[bestV1].route[bestR1].demand + vehicles[bestV1].route[bestR1_2].demand) + (vehicles[bestV2].route[bestR2].demand + vehicles[bestV2].route[bestR2_2].demand);
        vehicles[bestV2].demand             = vehicles[bestV2].demand - (vehicles[bestV2].route[bestR2].demand + vehicles[bestV2].route[bestR2_2].demand) + (vehicles[bestV1].route[bestR1].demand + vehicles[bestV1].route[bestR1_2].demand);
        violationDemand                     = violationDemand - (vehicles[bestV1].violationDemand + vehicles[bestV2].violationDemand);
        vehicles[bestV1].violationDemand    = bestVD1;
        vehicles[bestV2].violationDemand    = bestVD2;
        violationDemand                     = violationDemand + (bestVD1 + bestVD2);
        
        violationBattery                    = violationBattery - (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery);
        vehicles[bestV1].violationBattery   = bestVBt1;
        vehicles[bestV2].violationBattery   = bestVBt2;
        violationBattery                    = violationBattery + bestVBt1 + bestVBt2;

        penality = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);

        if(flag == 1){
            swap(vehicles[bestV1].route[bestR1], vehicles[bestV2].route[bestR2]);
            swap(vehicles[bestV1].route[bestR1_2], vehicles[bestV2].route[bestR2_2]);
        }else if(flag == 2){
            swap(vehicles[bestV1].route[bestR1_2], vehicles[bestV2].route[bestR2]);
            swap(vehicles[bestV1].route[bestR1], vehicles[bestV2].route[bestR2_2]);
        }else{
            swap(vehicles[bestV1].route[bestR1], vehicles[bestV2].route[bestR2_2]);
            swap(vehicles[bestV1].route[bestR1_2], vehicles[bestV2].route[bestR2]);
        }

        objective = distance + vehicleCost + penality + stationCost;

        //======================================Update data===================================================================
        updateDataBSS(data, bestV1);
        updateDataBSS(data, bestV2);

        double oldDistance1         = vehicles[bestV1].distance;
        double oldViolationBattery1 = vehicles[bestV1].violationBattery;
        double oldDistance2         = vehicles[bestV2].distance;
        double oldViolationBattery2 = vehicles[bestV2].violationBattery;
        
        vehicles[bestV1].intraRVNDBSS(data, bestV1);
        vehicles[bestV2].intraRVNDBSS(data, bestV2);
        
        distance            = distance + (vehicles[bestV1].distance + vehicles[bestV2].distance) - (oldDistance1 + oldDistance2);
        violationBattery    = violationBattery + (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery) - (oldViolationBattery1 + oldViolationBattery2);
        
        penality            = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Solution::interSwap2x1BSS(Data &data){
    int bestV1 = 0, bestV2 = 0, bestR1 = 0, bestR1_2 = 0, bestR2 = 0, flag = 0;
    double bestVD1 = 0, bestVD2 = 0, bestVBt1 = 0, bestVBt2 = 0;
    double newPenality, newObjective;
    double bestObjective = objective;
    double newViolationBattery = violationBattery;
    double newViolationDemand = violationDemand;
    double newDistance = distance;
    double bestDistance1 = 0, bestDistance2 = 0;
    bool improve = false;

    vector < pair < pair < pair < int, int >, int >, pair < int, int > > > vt;

    for(int i = 0; i < (int)vehicles.size(); i++) {
        for(int j = 0; j < (int)vehicles.size(); j++) {
            if((int)vehicles[i].route.size() > 3 && (int)vehicles[j].route.size() > 2 && i != j){
                for(int r1 = 1; r1 < (int)vehicles[i].route.size()-2; r1++) {                    
                    for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                        vt.push_back( make_pair( make_pair( make_pair(i, r1), r1+1), make_pair(j, r2)));
                    }
                }
            }
        }
    }

    improve = false;
    shuffle(vt.begin(), vt.end(), generator);
    for(int i = 0; i < (int)vt.size(); i++){
        newViolationBattery = violationBattery;
        newViolationDemand  = violationDemand;
        newDistance         = distance;

        int v1      = vt[i].first.first.first;
        int v2      = vt[i].second.first;
        int r1      = vt[i].first.first.second;
        int r1_2    = vt[i].first.second;
        int r2      = vt[i].second.second;

        int depot2V1 = vehicles[v1].route.size() - 1;
        int depot2V2 = vehicles[v2].route.size() - 1;
        
        // =========================================Update Distance==========================================================================
        double newDistance1;
        double newDistance2;
        newDistance1 = data.eval[v1][0][r1-1].distance + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].distance;
        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        // ========================================Update Demand=======================================================================
        double vDemand1 = 0;
        double vDemand2 = 0;
        double newDemand1 = vehicles[v1].demand - (vehicles[v1].route[r1].demand + vehicles[v1].route[r1_2].demand) + vehicles[v2].route[r2].demand;
        double newDemand2 = vehicles[v2].demand - vehicles[v2].route[r2].demand + (vehicles[v1].route[r1].demand + vehicles[v1].route[r1_2].demand);
        
        vDemand1 = max(newDemand1 - data.demandCapacity, 0.0);
        vDemand2 = max(newDemand2 - data.demandCapacity, 0.0);

        newViolationDemand = newViolationDemand + (vDemand1 + vDemand2) - (vehicles[v2].violationDemand + vehicles[v1].violationDemand);

        //===========================================Update Battery======================================================================
        double vBattery1 = 0;
        double vBattery2 = 0;
        if(data.eval[v1][0][r1-1].batteryStation || vehicles[v2].route[r2].batteryStation || data.eval[v1][r1_2+1][depot2V1].batteryStation){
            if(data.eval[v1][0][r1-1].batteryStation){
                vBattery1 += data.eval[v1][0][data.eval[v1][0][r1-1].idStationToEnd].violation;
            }
            if(data.eval[v1][r1_2+1][depot2V1].batteryStation){
                vBattery1 += data.eval[v1][data.eval[v1][r1_2+1][depot2V1].idBeginToStation][depot2V1].violation;
            }
            if(vehicles[v2].route[r2].batteryStation){
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v1][r1_2+1][depot2V1].beginToStation + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id];
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v1][0][r1-1].stationToEnd + data.distances[vehicles[v1].route[r1-1].id][vehicles[v2].route[r2].id] + data.distances[vehicles[v2].route[r2].id][vehicles[v1].route[r1_2+1].id] + data.eval[v1][r1_2+1][depot2V1].beginToStation;
                vBattery1 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery1 = max(newDistance1 - data.batteryCapacity, 0.0);
        }
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1].batteryStation && vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1].batteryStation && !vehicles[v1].route[r1_2].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2+1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;
        
        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationDemand * data.betaDemand);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;
        
        if(bestObjective - newObjective > EPS){
            bestV1          = vt[i].first.first.first;
            bestV2          = vt[i].second.first;
            bestR1          = vt[i].first.first.second;
            bestR1_2        = vt[i].first.second;
            bestR2          = vt[i].second.second;
            bestVD1         = vDemand1;
            bestVD2         = vDemand2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            flag            = 1;
        }
            
        //=====================================================Inverte==================================================================
        newViolationBattery = violationBattery;
        newDistance         = distance;

        newDistance2 = data.eval[v2][0][r2-1].distance + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].distance;
        newDistance  = newDistance + (newDistance1 + newDistance2) - (vehicles[v1].distance + vehicles[v2].distance);

        //===========================================Update Battery======================================================================
        vBattery2 = 0;
        if(data.eval[v2][0][r2-1].batteryStation || vehicles[v1].route[r1].batteryStation || vehicles[v1].route[r1_2].batteryStation || data.eval[v2][r2+1][depot2V2].batteryStation){
            if(data.eval[v2][0][r2-1].batteryStation){
                vBattery2 += data.eval[v2][0][data.eval[v2][0][r2-1].idStationToEnd].violation;
            }
            if(data.eval[v2][r2+1][depot2V2].batteryStation){
                vBattery2 += data.eval[v2][data.eval[v2][r2+1][depot2V2].idBeginToStation][depot2V2].violation;
            }
            if(vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(!vehicles[v1].route[r1_2].batteryStation && vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else if(vehicles[v1].route[r1_2].batteryStation && !vehicles[v1].route[r1].batteryStation){
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
                            
                bat = data.eval[v2][r2+1][depot2V2].beginToStation + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id];
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }else{
                double bat = data.eval[v2][0][r2-1].stationToEnd + data.distances[vehicles[v2].route[r2-1].id][vehicles[v1].route[r1_2].id] + data.distances[vehicles[v1].route[r1_2].id][vehicles[v1].route[r1].id] + data.distances[vehicles[v1].route[r1].id][vehicles[v2].route[r2+1].id] + data.eval[v2][r2+1][depot2V2].beginToStation;
                vBattery2 += max(bat - data.batteryCapacity, 0.0);
            }
        }else{
            vBattery2 = max(newDistance2 - data.batteryCapacity, 0.0);
        }

        newViolationBattery = newViolationBattery + vBattery1 + vBattery2 - vehicles[v1].violationBattery - vehicles[v2].violationBattery;

        // ===========================================Update custSolution====================================================================
        newPenality     = (newViolationBattery * data.betaBattery)  + (newViolationDemand * data.betaDemand);
        newObjective    = newDistance + vehicleCost + newPenality + stationCost;
        
        if(bestObjective - newObjective > EPS){
            bestV1          = vt[i].first.first.first;
            bestV2          = vt[i].second.first;
            bestR1          = vt[i].first.first.second;
            bestR1_2        = vt[i].first.second;
            bestR2          = vt[i].second.second;
            bestVD1         = vDemand1;
            bestVD2         = vDemand2;
            bestVBt1        = vBattery1;
            bestVBt2        = vBattery2;
            improve         = true;
            bestObjective   = newObjective;
            bestDistance1   = newDistance1;
            bestDistance2   = newDistance2;
            flag            = 2;
        }
    }

    if(improve){
        distance                    = distance - (vehicles[bestV1].distance + vehicles[bestV2].distance);
        vehicles[bestV1].distance   = bestDistance1;
        vehicles[bestV2].distance   = bestDistance2;
        distance                    = distance + vehicles[bestV1].distance + vehicles[bestV2].distance;

        vehicles[bestV1].demand             = vehicles[bestV1].demand - (vehicles[bestV1].route[bestR1].demand + vehicles[bestV1].route[bestR1_2].demand) + vehicles[bestV2].route[bestR2].demand;
        vehicles[bestV2].demand             = vehicles[bestV2].demand - vehicles[bestV2].route[bestR2].demand + (vehicles[bestV1].route[bestR1].demand + vehicles[bestV1].route[bestR1_2].demand);
        violationDemand                     = violationDemand - (vehicles[bestV1].violationDemand + vehicles[bestV2].violationDemand);
        vehicles[bestV1].violationDemand    = bestVD1;
        vehicles[bestV2].violationDemand    = bestVD2;
        violationDemand                     = violationDemand + (bestVD1 + bestVD2);
        
        violationBattery                    = violationBattery - (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery);
        vehicles[bestV1].violationBattery   = bestVBt1;
        vehicles[bestV2].violationBattery   = bestVBt2;
        violationBattery                    = violationBattery + bestVBt1 + bestVBt2;

        penality = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);

        if(flag == 1){
            swap(vehicles[bestV1].route[bestR1], vehicles[bestV2].route[bestR2]);
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1_2]);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1_2);    
        }else{
            swap(vehicles[bestV1].route[bestR1_2], vehicles[bestV2].route[bestR2]);
            vehicles[bestV2].route.insert(vehicles[bestV2].route.begin() + bestR2, vehicles[bestV1].route[bestR1]);
            vehicles[bestV1].route.erase(vehicles[bestV1].route.begin() + bestR1);
        }

        objective = distance + vehicleCost + penality + stationCost;

        //======================================Update data===================================================================
        updateDataBSS(data, bestV1);
        updateDataBSS(data, bestV2);

        double oldDistance1             = vehicles[bestV1].distance;
        double oldViolationBattery1     = vehicles[bestV1].violationBattery;
        double oldDistance2             = vehicles[bestV2].distance;
        double oldViolationBattery2     = vehicles[bestV2].violationBattery;
        
        vehicles[bestV1].intraRVNDBSS(data, bestV1);
        vehicles[bestV2].intraRVNDBSS(data, bestV2);
        
        distance            = distance + (vehicles[bestV1].distance + vehicles[bestV2].distance) - (oldDistance1 + oldDistance2);
        violationBattery    = violationBattery + (vehicles[bestV1].violationBattery + vehicles[bestV2].violationBattery) - (oldViolationBattery1 + oldViolationBattery2);
        
        penality            = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);
        objective           = distance + penality + vehicleCost + stationCost;
    }
}

void Vehicle::intraRVNDBSS(Data &data, int v){
    penality = violationBattery * data.betaBattery;
    objective = distance + penality;
    double bestObjective = objective;
    int count = 3, i;
    vector<int> vR;

    for(i = 0; i < count; i++){
        vR.push_back(i);
    }
    
    shuffle(vR.begin(), vR.end(), generator);
    count = 0;
    
    while(1){
        if (count >= (int)vR.size()){break;}
        
        if(vR[count] == 0){
            intraRealocationBSS(data, v);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(vR.begin(), vR.end(), generator);
                count = 0;
            }else{
                count++;
            }
         }else if (vR[count] == 1){
            intraShift2BSS(data, v);
            
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(vR.begin(), vR.end(), generator);
                count = 0;
            }else{
                count++;
            }
        }else if (vR[count] == 2){
            intraSwapBSS(data, v);
            
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(vR.begin(), vR.end(), generator);
                count = 0;
            }else{
                count++;
            }
        }
    }
}

void Vehicle::intraSwapBSS(Data &data, int v) {
    int i, j, bestR1 = -1, bestR2 = -1;
    penality = violationBattery * data.betaBattery;
    double bestObjective = distance + penality, newObjective, newPenality, bestVB = 0, bestDistance = 0;
    vector < pair < int, int > > vt; 
    
    for (i = 1; i < (int)route.size()-1; i++) {
        for (j = i+1; j < (int)route.size()-1; j++) {
            vt.push_back(make_pair(i, j));
        }
    }

    int depot2 = route.size() - 1;
    for (i = 0; i < (int)vt.size(); i++) {
        int r1 = vt[i].first;
        int r2 = vt[i].second;
        double newDistance = 0;
        double vBattery = 0;

        // ==================Update Battery============================
        if(r1 == r2-1){
            newDistance = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].distance;

            if(data.eval[v][0][depot2].batteryStation){
                if(data.eval[v][0][r1-1].batteryStation){
                    vBattery += data.eval[v][0][data.eval[v][0][r1-1].idStationToEnd].violation;
                }

                if(route[r2].batteryStation && route[r1].batteryStation){
                    double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id];
                    vBattery += max(bat - data.batteryCapacity, 0.0);

                    bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                    vBattery += max(bat - data.batteryCapacity, 0.0);
                }else if(!route[r2].batteryStation && route[r1].batteryStation){
                    double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1].id];
                    vBattery += max(bat - data.batteryCapacity, 0.0);

                    bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                    vBattery += max(bat - data.batteryCapacity, 0.0);
                }else if(route[r2].batteryStation && !route[r1].batteryStation){
                    double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id];
                    vBattery += max(bat - data.batteryCapacity, 0.0);

                    bat = data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                    vBattery += max(bat - data.batteryCapacity, 0.0);
                }else{
                    double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                    vBattery += max(bat - data.batteryCapacity, 0.0);
                }
                
                if(data.eval[v][r2+1][depot2].batteryStation){
                    vBattery += data.eval[v][data.eval[v][r2+1][depot2].idBeginToStation][depot2].violation;
                }
            }else{
                vBattery = max(newDistance - data.batteryCapacity, 0.0);
            }
        }else{
            newDistance = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].distance;

            if(data.eval[v][0][depot2].batteryStation){
                if(data.eval[v][0][r1-1].batteryStation){
                    vBattery += data.eval[v][0][data.eval[v][0][r1-1].idStationToEnd].violation;
                }

                if(route[r2].batteryStation){
                    double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id];
                    vBattery += max(bat - data.batteryCapacity, 0.0);

                    if(data.eval[v][r1+1][r2-1].batteryStation){
                        bat = data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        if(route[r1].batteryStation){
                            bat = data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                }else{
                    if(data.eval[v][r1+1][r2-1].batteryStation){
                        double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        if(route[r1].batteryStation){
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                }
                
                if(data.eval[v][r1+1][r2-1].batteryStation){
                    vBattery += data.eval[v][data.eval[v][r1+1][r2+1].idBeginToStation][data.eval[v][r1+1][r2-1].idStationToEnd].violation;
                }else{
                    vBattery += max(data.eval[v][r1+1][r2-1].distance - data.batteryCapacity, 0.0);
                }
                
                if(route[r1].batteryStation){
                    if(data.eval[v][r1+1][r2-1].batteryStation){
                        double bat = data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        if(route[r2].batteryStation){
                            double bat = data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                }else{
                    if(data.eval[v][r1+1][r2-1].batteryStation){
                        double bat = data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        if(route[r2].batteryStation){
                            double bat = data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r2].id] + data.distances[route[r2].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                }
                
                if(data.eval[v][r2+1][depot2].batteryStation){
                    vBattery += data.eval[v][data.eval[v][r2+1][depot2].idBeginToStation][depot2].violation;
                }
            }else{
                vBattery = max(newDistance - data.batteryCapacity, 0.0);
            }
        }
        
        newPenality     = vBattery * data.betaBattery;
        newObjective    = newDistance + newPenality;
   
        if(bestObjective - newObjective > EPS){
            bestObjective   = newObjective;
            bestVB          = vBattery;
            bestDistance    = newDistance;
            bestR1          = vt[i].first;
            bestR2          = vt[i].second;
        }
    }

    if(bestR1 != -1){
        distance = bestDistance;
        
        violationBattery = bestVB;
        
        penality = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);
        
        objective = distance + penality + vehicleCost + stationCost;

        swap(route[bestR1], route[bestR2]);

        double distance, battery, violation, distanceLastStation, distanceFirstStation, violAlreadyCalc = 0;
        bool thereAreBattery;
        int idStation;
        for(i = 0; i < (int)route.size() - 1; i++){
            distance            = 0;
            battery             = 0;
            violation           = 0;
            distanceLastStation = 0;
            thereAreBattery    = false;
            if(route[i].batteryStation){
                idStation       = i;
            }else{
                idStation       = -1;
            }
            violAlreadyCalc = 0;
            for(j = i; j < (int)route.size() - 1; j++){
                if(battery > data.batteryCapacity){
                    violation       += (battery - data.batteryCapacity) - violAlreadyCalc;
                    violAlreadyCalc += ((battery - data.batteryCapacity) - violAlreadyCalc);
                }else{
                    violAlreadyCalc = 0;
                }

                if(route[j].batteryStation){
                    thereAreBattery    = true;
                    battery             = 0;
                    distanceLastStation = 0;
                    idStation           = j;
                }

                data.eval[v][i][j].violation        = violation;
                data.eval[v][i][j].distance         = distance;
                data.eval[v][i][j].batteryStation   = thereAreBattery;
                data.eval[v][i][j].stationToEnd     = distanceLastStation;
                data.eval[v][i][j].idStationToEnd   = idStation;
                
                distance            += data.distances[route[j].id][route[j+1].id];
                battery             += data.distances[route[j].id][route[j+1].id];
                distanceLastStation += data.distances[route[j].id][route[j+1].id];                    
            }
            if(battery > data.batteryCapacity){
                violation       += (battery - data.batteryCapacity) - violAlreadyCalc;
                violAlreadyCalc += ((battery - data.batteryCapacity) - violAlreadyCalc);
            }else{
                violAlreadyCalc = 0;
            }
            data.eval[v][i][j].violation        = violation;
            data.eval[v][i][j].distance         = distance;
            data.eval[v][i][j].batteryStation   = thereAreBattery;                
            data.eval[v][i][j].stationToEnd     = distanceLastStation;
            data.eval[v][i][j].idStationToEnd   = idStation;
        }
        data.eval[v][i][i].violation        = 0;
        data.eval[v][i][i].distance         = 0;
        data.eval[v][i][i].batteryStation   = false;
        data.eval[v][i][i].stationToEnd     = 0;
        data.eval[v][i][i].idStationToEnd   = -1;

        for(i = (int)route.size() -1; i > 0; i--){
            distanceFirstStation    = 0;
            if(route[i].batteryStation){
                idStation           = i;
            }else{
                idStation           = -1;
            }
            for(j = i; j > 0; j--){
                
                if(route[j].batteryStation){
                    distanceFirstStation    = 0;
                    idStation               = j;
                }

                data.eval[v][j][i].beginToStation   = distanceFirstStation;
                data.eval[v][j][i].idBeginToStation = idStation;
                
                distanceFirstStation    += data.distances[route[j].id][route[j-1].id];
            }
            data.eval[v][j][i].beginToStation   = distanceFirstStation;
            data.eval[v][j][i].idBeginToStation = idStation;
        }
        data.eval[v][i][i].beginToStation   = 0;
        data.eval[v][i][i].idBeginToStation = -1;
    }
}

void Vehicle::intraShift2BSS(Data &data, int v) {
    int i, j, bestR1 = -1, bestR2 = -1, bestR1_2 = -1;
    penality = violationBattery * data.betaBattery;
    double bestObjective = objective, newObjective, newPenality, bestVB = 0, bestDistance = 0;
    bool invert = false;
    vector < pair < int, int > > vt; 
    

    if((int)route.size() > 4){
        for (i = 1; i < (int)route.size()-2; i++) {
            for (j = 1; j < (int)route.size()-2; j++) {
                if(i != j && i != j+1 && i != j-1){
                    vt.push_back(make_pair(i, j));
                }
            }
        }

        int depot2 = route.size() - 1;
        for (i = 0; i < (int)vt.size(); i++) {
            int r1      = vt[i].first;
            int r1_2    = vt[i].first+1;
            int r2      = vt[i].second;

            double newDistance  = 0;
            double vBattery     = 0;
            
            if(r1 > r2){
                newDistance = data.eval[v][0][r2].distance + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance +   data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].distance;
                // ==================Update Battery============================
                vBattery = 0;
                if(data.eval[v][0][depot2].batteryStation){
                    if(data.eval[v][0][r2].batteryStation){
                        vBattery += data.eval[v][0][data.eval[v][0][r2].idStationToEnd].violation;
                    }
                    if(route[r1].batteryStation && route[r1_2].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(!route[r1].batteryStation && route[r1_2].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(route[r1].batteryStation && !route[r1_2].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else{
                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                    if(data.eval[v][r2+1][r1-1].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r2+1][r1-1].idBeginToStation][data.eval[v][r2+1][r1-1].idStationToEnd].violation;
                    }else{
                        vBattery += max(data.eval[v][r2+1][r1-1].distance - data.batteryCapacity, 0.0);
                    }
                    if(data.eval[v][r1_2+1][depot2].batteryStation){
                        vBattery = vBattery + data.eval[v][data.eval[v][r1_2+1][depot2].idBeginToStation][depot2].violation;
                        double bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        double bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].distance;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }
                }else{
                    vBattery = max(newDistance - data.batteryCapacity, 0.0);
                }
               
                newPenality  = vBattery * data.betaBattery;
                newObjective = newDistance + newPenality;
        
                if(bestObjective - newObjective > EPS){
                    bestObjective   = newObjective;
                    bestVB          = vBattery;
                    bestDistance    = newDistance;
                    bestR1          = vt[i].first;
                    bestR1_2        = vt[i].first + 1;
                    bestR2          = vt[i].second;
                    invert          = false;
                }
                
                //=========================================Inverte========================================
                newDistance = data.eval[v][0][r2].distance + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance +   data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].distance;
                
                // ==================Update Battery============================
                vBattery = 0;
                if(data.eval[v][0][depot2].batteryStation){
                    if(data.eval[v][0][r2].batteryStation){
                        vBattery += data.eval[v][0][data.eval[v][0][r2].idStationToEnd].violation;
                    }
                    
                    if(route[r1_2].batteryStation && route[r1].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(!route[r1_2].batteryStation && route[r1].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(route[r1_2].batteryStation && !route[r1].batteryStation){
                        double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id];
                        vBattery += max(bat - data.batteryCapacity, 0.0);

                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            bat = data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            bat = data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else{
                        if(data.eval[v][r2+1][r1-1].batteryStation){
                            double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }

                    if(data.eval[v][r2+1][r1-1].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r2+1][r1-1].idBeginToStation][data.eval[v][r2+1][r1-1].idStationToEnd].violation;
                    }else{
                        vBattery += max(data.eval[v][r2+1][r1-1].distance - data.batteryCapacity, 0.0);
                    }
                    
                    if(data.eval[v][r1_2+1][depot2].batteryStation){
                        vBattery = vBattery + data.eval[v][data.eval[v][r1_2+1][depot2].idBeginToStation][depot2].violation;
                        double bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        double bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][depot2].distance;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }
                }else{
                    vBattery = max(newDistance - data.batteryCapacity, 0.0);
                }
               
                newPenality = vBattery * data.betaBattery;
                newObjective = newDistance + newPenality;
        
                if(bestObjective - newObjective > EPS){
                    bestObjective   = newObjective;
                    bestVB          = vBattery;
                    bestDistance    = newDistance;
                    bestR1          = vt[i].first;
                    bestR1_2        = vt[i].first + 1;
                    bestR2          = vt[i].second;
                    invert          = true;
                }
            }else{
                newDistance = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].distance;

                // ==================Update Battery============================
                vBattery = 0;
                if(data.eval[v][0][depot2].batteryStation){
                    if(data.eval[v][0][r1-1].batteryStation){
                        vBattery += data.eval[v][0][data.eval[v][0][r1-1].idStationToEnd].violation;
                        double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        double bat = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }
                    
                    if(data.eval[v][r1_2+1][r2].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r1_2+1][r2].idBeginToStation][data.eval[v][r1_2+1][r2].idStationToEnd].violation;
                    }else{
                        vBattery += max(data.eval[v][r1_2+1][r2].distance - data.batteryCapacity, 0.0);
                    }
                    
                    if(route[r1].batteryStation && route[r1_2].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(!route[r1].batteryStation && route[r1_2].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(route[r1].batteryStation && !route[r1_2].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else{
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                    if(data.eval[v][r2+1][depot2].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r2+1][depot2].idBeginToStation][depot2].violation;
                    }
                }else{
                    vBattery = max(newDistance - data.batteryCapacity, 0.0);
                }
                
                newPenality  = vBattery * data.betaBattery;
                newObjective = newDistance + newPenality;
        
                if(bestObjective - newObjective > EPS){
                    bestObjective   = newObjective;
                    bestVB          = vBattery;
                    bestDistance    = newDistance;
                    bestR1          = r1;
                    bestR1_2        = r1_2;
                    bestR2          = r2;
                    invert          = false;
                }
                
                //=========================================Inverte========================================
                newDistance = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].distance;

                // ==================Update Battery============================
                vBattery = 0;
                if(data.eval[v][0][depot2].batteryStation){
                    if(data.eval[v][0][r1-1].batteryStation){
                        vBattery += data.eval[v][0][data.eval[v][0][r1-1].idStationToEnd].violation;
                        double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        double bat = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].beginToStation;
                        vBattery += max(bat - data.batteryCapacity, 0.0);
                    }
                    
                    if(data.eval[v][r1_2+1][r2].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r1_2+1][r2].idBeginToStation][data.eval[v][r1_2+1][r2].idStationToEnd].violation;
                    }else{
                        vBattery += max(data.eval[v][r1_2+1][r2].distance - data.batteryCapacity, 0.0);
                    }
                    
                    if(route[r1_2].batteryStation && route[r1].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(!route[r1_2].batteryStation && route[r1].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else if(route[r1_2].batteryStation && !route[r1].batteryStation){
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1_2].id];
                            vBattery += max(bat - data.batteryCapacity, 0.0);

                            bat = data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }else{
                        if(data.eval[v][r1_2+1][r2].batteryStation){
                            double bat = data.eval[v][r1_2+1][r2].stationToEnd + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }else{
                            double bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1_2+1].id] + data.eval[v][r1_2+1][r2].distance + data.distances[route[r2].id][route[r1_2].id] + data.distances[route[r1_2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][depot2].beginToStation;
                            vBattery += max(bat - data.batteryCapacity, 0.0);
                        }
                    }
                    if(data.eval[v][r2+1][depot2].batteryStation){
                        vBattery += data.eval[v][data.eval[v][r2+1][depot2].idBeginToStation][depot2].violation;
                    }
                }else{
                    vBattery = max(newDistance - data.batteryCapacity, 0.0);
                }
                
                newPenality  = vBattery * data.betaBattery;
                newObjective = newDistance + newPenality;
        
                if(bestObjective - newObjective > EPS){
                    bestObjective   = newObjective;
                    bestVB          = vBattery;
                    bestDistance    = newDistance;
                    bestR1          = r1;
                    bestR1_2        = r1_2;
                    bestR2          = r2;
                    invert          = true;
                }
            }
        }

        if(bestR1 != -1){
            distance            = bestDistance;
            violationBattery    = bestVB;
            
            penality    = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);
            objective   = distance + penality + vehicleCost + stationCost;
            
            if(bestR1 < bestR2){
                if(!invert){
                    route.insert(route.begin()+bestR2+1, route[bestR1_2]);
                    route.insert(route.begin()+bestR2+1, route[bestR1]);
                    route.erase(route.begin()+bestR1_2);
                    route.erase(route.begin()+bestR1);
                }else{
                    route.insert(route.begin()+bestR2+1, route[bestR1]);
                    route.insert(route.begin()+bestR2+1, route[bestR1_2]);
                    route.erase(route.begin()+bestR1_2);
                    route.erase(route.begin()+bestR1);
                }
            }else{
                if(!invert){
                    route.insert(route.begin()+bestR2+1, route[bestR1_2]);
                    route.insert(route.begin()+bestR2+1, route[bestR1+1]);
                    route.erase(route.begin()+bestR1_2+2);
                    route.erase(route.begin()+bestR1+2);
                }else{
                    route.insert(route.begin()+bestR2+1, route[bestR1]);
                    route.insert(route.begin()+bestR2+1, route[bestR1_2+1]);
                    route.erase(route.begin()+bestR1_2+2);
                    route.erase(route.begin()+bestR1+2);
                }
            }
            
            double distance, battery, violation, distanceLastStation, distanceFirstStation, violAlreadyCalc = 0;
            bool thereAreBattery;
            int idStation;
            for(i = 0; i < (int)route.size() - 1; i++){
                distance            = 0;
                battery             = 0;
                violation           = 0;
                distanceLastStation = 0;
                thereAreBattery    = false;
                if(route[i].batteryStation){
                    idStation       = i;
                }else{
                    idStation       = -1;
                }
                violAlreadyCalc = 0;
                for(j = i; j < (int)route.size() - 1; j++){
                    if(battery > data.batteryCapacity){
                        violation       += (battery - data.batteryCapacity) - violAlreadyCalc;
                        violAlreadyCalc += ((battery - data.batteryCapacity) - violAlreadyCalc);
                    }else{
                        violAlreadyCalc = 0;
                    }

                    if(route[j].batteryStation){
                        thereAreBattery    = true;
                        battery             = 0;
                        distanceLastStation = 0;
                        idStation           = j;
                    }

                    data.eval[v][i][j].violation        = violation;
                    data.eval[v][i][j].distance         = distance;
                    data.eval[v][i][j].batteryStation   = thereAreBattery;
                    data.eval[v][i][j].stationToEnd     = distanceLastStation;
                    data.eval[v][i][j].idStationToEnd   = idStation;
                    
                    distance            += data.distances[route[j].id][route[j+1].id];
                    battery             += data.distances[route[j].id][route[j+1].id];
                    distanceLastStation += data.distances[route[j].id][route[j+1].id];                    
                }
                if(battery > data.batteryCapacity){
                    violation       += (battery - data.batteryCapacity) - violAlreadyCalc;
                    violAlreadyCalc += ((battery - data.batteryCapacity) - violAlreadyCalc);
                }else{
                    violAlreadyCalc = 0;
                }
                data.eval[v][i][j].violation        = violation;
                data.eval[v][i][j].distance         = distance;
                data.eval[v][i][j].batteryStation   = thereAreBattery;                
                data.eval[v][i][j].stationToEnd     = distanceLastStation;
                data.eval[v][i][j].idStationToEnd   = idStation;
            }
            data.eval[v][i][i].violation        = 0;
            data.eval[v][i][i].distance         = 0;
            data.eval[v][i][i].batteryStation   = false;
            data.eval[v][i][i].stationToEnd     = 0;
            data.eval[v][i][i].idStationToEnd   = -1;

            for(i = (int)route.size() -1; i > 0; i--){
                distanceFirstStation    = 0;
                if(route[i].batteryStation){
                    idStation           = i;
                }else{
                    idStation           = -1;
                }
                for(j = i; j > 0; j--){
                    
                    if(route[j].batteryStation){
                        distanceFirstStation    = 0;
                        idStation               = j;
                    }

                    data.eval[v][j][i].beginToStation   = distanceFirstStation;
                    data.eval[v][j][i].idBeginToStation = idStation;
                    
                    distanceFirstStation    += data.distances[route[j].id][route[j-1].id];
                }
                data.eval[v][j][i].beginToStation   = distanceFirstStation;
                data.eval[v][j][i].idBeginToStation = idStation;
            }
            data.eval[v][i][i].beginToStation   = 0;
            data.eval[v][i][i].idBeginToStation = -1;
        }
    }
}

void Vehicle::intraRealocationBSS(Data &data, int v) {
    int i, j, bestR1 = -1, bestR2 = -1;
    penality = violationBattery * data.betaBattery;
    double bestObjective = objective, newObjective, newPenality, bestVB = 0, bestDistance = 0;
    vector < pair < int, int > > vt; 

    for (i = 1; i < (int)route.size()-1; i++) {
        for (j = 1; j < (int)route.size()-1; j++) {
            if(i != j && i != j-1 && i != j+1){
                vt.push_back(make_pair(i, j));
            }
        }
    }

    int depot2 = route.size() - 1;
    for (i = 0; i < (int)vt.size(); i++) {
        //==============================Distance==================================
        int r1 = vt[i].first;
        int r2 = vt[i].second;
        double newDistance = 0;
        double vBattery = 0;
        
        if(r1 < r2){
            newDistance = data.eval[v][0][r1-1].distance + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance +  data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2].id] + data.eval[v][r2][depot2].distance;

            // ==================Update Battery============================
            vBattery = 0;
            if(data.eval[v][0][depot2].batteryStation){
                if(data.eval[v][0][r1-1].batteryStation){
                    vBattery += data.eval[v][0][data.eval[v][0][r1-1].idStationToEnd].violation;
                }
                if(data.eval[v][r1+1][r2-1].batteryStation){
                    double bat  = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].beginToStation;
                    vBattery    += max(bat - data.batteryCapacity, 0.0);

                    if(route[r1].batteryStation){
                        bat = data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id];
                        vBattery    += max(bat - data.batteryCapacity, 0.0);

                        bat = data.distances[route[r1].id][route[r2].id] + data.eval[v][r2][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);                    
                    }else{
                        bat = data.eval[v][r1+1][r2-1].stationToEnd + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2].id] + data.eval[v][r2][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }
                }else{
                    double bat = 0;
                    if(route[r1].batteryStation){
                        bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id];
                        vBattery    += max(bat - data.batteryCapacity, 0.0);

                        bat = data.distances[route[r1].id][route[r2].id] + data.eval[v][r2][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);                    
                    }else{
                        bat = data.eval[v][0][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][r2-1].distance + data.distances[route[r2-1].id][route[r1].id] + data.distances[route[r1].id][route[r2].id] + data.eval[v][r2][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }
                }
                if(data.eval[v][r2][depot2].batteryStation){
                    vBattery += data.eval[v][data.eval[v][r2][depot2].idBeginToStation][depot2].violation;
                }
            }else{
                vBattery = max(newDistance - data.batteryCapacity, 0.0);
            }
        }else{
            newDistance = data.eval[v][0][r2].distance + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][depot2].distance;

            // ==================Update Battery============================
            vBattery = 0;
            if(data.eval[v][0][depot2].batteryStation){
                if(data.eval[v][0][r2].batteryStation){
                    vBattery += data.eval[v][0][data.eval[v][0][r2].idStationToEnd].violation;
                }
                if(route[r1].batteryStation){
                    double bat  = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id];
                    vBattery    += max(bat - data.batteryCapacity, 0.0);

                    if(data.eval[v][r2+1][r1-1].batteryStation){
                        bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);

                        bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        bat = data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }
                }else{
                    double bat = 0;
                    if(data.eval[v][r2+1][r1-1].batteryStation){
                        bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);

                        bat = data.eval[v][r2+1][r1-1].stationToEnd + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }else{
                        bat = data.eval[v][0][r2].stationToEnd + data.distances[route[r2].id][route[r1].id] + data.distances[route[r1].id][route[r2+1].id] + data.eval[v][r2+1][r1-1].distance + data.distances[route[r1-1].id][route[r1+1].id] + data.eval[v][r1+1][depot2].beginToStation;
                        vBattery    += max(bat - data.batteryCapacity, 0.0);
                    }
                }
                if(data.eval[v][r1+1][depot2].batteryStation){
                    vBattery += data.eval[v][data.eval[v][r1+1][depot2].idBeginToStation][depot2].violation;
                }
            }else{
                vBattery = max(newDistance - data.batteryCapacity, 0.0);
            }
        }

        newPenality = vBattery * data.betaBattery;
        newObjective = newDistance + newPenality;
   
        if(bestObjective - newObjective > EPS){
            bestObjective   = newObjective;
            bestVB          = vBattery;
            bestDistance    = newDistance;
            bestR1          = r1;
            bestR2          = r2;
        }
    }

    if(bestR1 != -1){
        distance            = bestDistance;
        violationBattery    = bestVB;
        
        penality    = (violationBattery * data.betaBattery) + (violationDemand * data.betaDemand);
        objective   = distance + penality + vehicleCost + stationCost;
        
        if(bestR1 < bestR2){
            route.insert(route.begin()+bestR2+1, route[bestR1]);
            route.erase(route.begin()+bestR1);
        }else{
            route.insert(route.begin()+bestR2, route[bestR1]);
            route.erase(route.begin()+bestR1+1);
        }

        double distance, battery, violation, distanceLastStation, distanceFirstStation, violAlreadyCalc = 0;
        bool thereAreBattery;
        int idStation;
        for(i = 0; i < (int)route.size() - 1; i++){
            distance            = 0;
            battery             = 0;
            violation           = 0;
            distanceLastStation = 0;
            thereAreBattery    = false;
            if(route[i].batteryStation){
                idStation       = i;
            }else{
                idStation       = -1;
            }
            violAlreadyCalc = 0;
            for(j = i; j < (int)route.size() - 1; j++){
                if(battery > data.batteryCapacity){
                    violation       += (battery - data.batteryCapacity) - violAlreadyCalc;
                    violAlreadyCalc += ((battery - data.batteryCapacity) - violAlreadyCalc);
                }else{
                    violAlreadyCalc = 0;
                }

                if(route[j].batteryStation){
                    thereAreBattery    = true;
                    battery             = 0;
                    distanceLastStation = 0;
                    idStation           = j;
                }

                data.eval[v][i][j].violation        = violation;
                data.eval[v][i][j].distance         = distance;
                data.eval[v][i][j].batteryStation   = thereAreBattery;
                data.eval[v][i][j].stationToEnd     = distanceLastStation;
                data.eval[v][i][j].idStationToEnd   = idStation;
                
                distance            += data.distances[route[j].id][route[j+1].id];
                battery             += data.distances[route[j].id][route[j+1].id];
                distanceLastStation += data.distances[route[j].id][route[j+1].id];                    
            }
            if(battery > data.batteryCapacity){
                violation       += (battery - data.batteryCapacity) - violAlreadyCalc;
                violAlreadyCalc += ((battery - data.batteryCapacity) - violAlreadyCalc);
            }else{
                violAlreadyCalc = 0;
            }
            data.eval[v][i][j].violation        = violation;
            data.eval[v][i][j].distance         = distance;
            data.eval[v][i][j].batteryStation   = thereAreBattery;                
            data.eval[v][i][j].stationToEnd     = distanceLastStation;
            data.eval[v][i][j].idStationToEnd   = idStation;
        }
        data.eval[v][i][i].violation        = 0;
        data.eval[v][i][i].distance         = 0;
        data.eval[v][i][i].batteryStation   = false;
        data.eval[v][i][i].stationToEnd     = 0;
        data.eval[v][i][i].idStationToEnd   = -1;

        for(i = (int)route.size() -1; i > 0; i--){
            distanceFirstStation    = 0;
            if(route[i].batteryStation){
                idStation           = i;
            }else{
                idStation           = -1;
            }
            for(j = i; j > 0; j--){
                
                if(route[j].batteryStation){
                    distanceFirstStation    = 0;
                    idStation               = j;
                }

                data.eval[v][j][i].beginToStation   = distanceFirstStation;
                data.eval[v][j][i].idBeginToStation = idStation;
                
                distanceFirstStation    += data.distances[route[j].id][route[j-1].id];
            }
            data.eval[v][j][i].beginToStation   = distanceFirstStation;
            data.eval[v][j][i].idBeginToStation = idStation;
        }
        data.eval[v][i][i].beginToStation   = 0;
        data.eval[v][i][i].idBeginToStation = -1;
    }
}

void Solution::showSolutionBSS() {
    for (int i = 0; i < (int)vehicles.size(); i++) {
        if((int)vehicles[i].route.size() < 3){
            continue;
        }
        for (int j = 0; j < (int)vehicles[i].route.size(); j++) {
            if(vehicles[i].route[j].batteryStation){
                cout << vehicles[i].route[j].sigla << "(S) - ";
            }else{
                cout << vehicles[i].route[j].sigla << " - ";
            }
        }
        cout << endl;
        cout << "Demand: " << vehicles[i].demand << " | "; 
        cout << "   Capacity: " << vehicles[i].demandCapacity << " | ";
        cout << "   VTW: " << vehicles[i].violationTw << " | " << "     VD: " << vehicles[i].violationDemand << " | " << "     VB: " << vehicles[i].violationBattery << " | ";
        cout << "   Distance: " << vehicles[i].distance << endl;
    }
    cout << "Objective Solution: " << objective << endl;
    cout << "Amount Vehicles: " << amountVehicles << endl;
    cout << "Station builted: " << stationCost << endl;
}

void Solution::VNSTW(Data &data, int vnsMax, double timeMax) {
    Solution bestSolution, currentSolution, validSolution, smallVehiclesSolution;
    smallVehiclesSolution.amountVehicles = -1;
    int iter = 0, bestNumVehicles = 99, lastShake = 0;
    double bestObjective;

    generateGreedySolutionTW(data);
    evaluateSolutionTW(data);
    
    bestObjective = objective;
    currentSolution = *this;
    validSolution = *this;
    
    while (iter < vnsMax) {
        shakeTW(data, iter, lastShake);
        
        interRVNDTW(data, lastShake);
        
        if(violationDemand < EPS && violationBattery < EPS && violationTw < EPS){
            if(validSolution.objective - objective > EPS){
                validSolution = *this;
            }
        
            if(amountVehicles < bestNumVehicles){
                bestNumVehicles = amountVehicles;
                smallVehiclesSolution = *this;
            }
        }

        if(bestObjective - objective > EPS ){
            bestObjective   = objective;
            currentSolution = *this;
            iter            = 0;
        }else{
            iter++;
            *this           = currentSolution;
        }
    }
    
    if(violationDemand > EPS || violationBattery > EPS || violationTw > EPS){
        *this = validSolution;
    }

    if(smallVehiclesSolution.amountVehicles != -1 && smallVehiclesSolution.amountVehicles < amountVehicles){
        *this = smallVehiclesSolution;
    }
    
    vector < pair < int, int > > idRemove;
    Vehicle car;
    car.route.push_back(data.requests[0]);
    car.route.push_back(data.requests[0]);
    bool onliBat = true;
    for(int i = 0; i < (int)vehicles.size(); i++){
        vehicles[i].vehicleCost = 0;
        if(vehicles[i].route.size() > 2){
            onliBat = true;
            for(int j = 0; j < (int)vehicles[i].route.size(); j++){
                if(!vehicles[i].route[j].batteryStation){
                    onliBat = false;
                    break;
                }
            }
            if(onliBat){
                vehicles[i].route = car.route;
            }
        }
    }

    evaluateSolutionTW(data);
}

void Solution::generateGreedySolutionTW(Data &data) {
    int k = data.numBatteryStations;
    double batterConsumption, wait = 0, vTw = 0;

    if(data.numRequests >= 100){
        vehicles.resize(20);                                          //quantidade de veiculos    
        amountVehicles = 20;
    }else if(data.numRequests >= 15){
        vehicles.resize(10);                                          //quantidade de veiculos    
        amountVehicles = 10;
    }else if(data.numRequests >= 10){
        vehicles.resize(10);                                          //quantidade de veiculos    
        amountVehicles = 10;
    }else{
        vehicles.resize(10);                                          //quantidade de veiculos    
        amountVehicles = 10;
    }
    
    for (int i = 0; i < (int)vehicles.size(); i++){
        vehicles[i].demandCapacity          = data.demandCapacity;       //limite de demanda
        vehicles[i].batteryCapacity         = data.batteryCapacity;      //limite de bateria
        vehicles[i].rateConsumption         = data.rateConsumption;      //consumo de bateria por KM
        vehicles[i].vehicleCost             = 10000;                  //custo do Veículo
        vehicles[i].batteryKm               = data.batteryCapacity / vehicles[i].rateConsumption;
        vehicles[i].batteryUsed             = 0;
        vehicles[i].demand                  = 0;
        vehicles[i].violationTw             = 0;
        vehicles[i].violationBattery        = 0;
        vehicles[i].violationDemand         = 0;
        vehicles[i].violationBattery        = 0;
        vehicles[i].objective               = 0;
        vehicles[i].justDistance            = 0;
        vehicles[i].numberStationsVisited   = 0;
        vehicles[i].penality                = 0;
        vehicles[i].ride                    = 0;

        vehicles[i].route.push_back(data.requests[0]);                              //iniciar com deposito
    }
    // exit(1);
    int i = 0;
    while(k < (int)data.requests.size()){
        if(!data.requests[k].batteryStation){
            if(vehicles[i].demand < vehicles[i].demandCapacity){
                batterConsumption = data.distances[vehicles[i].route.back().id][data.requests[k].id]; // Consumo de bateria até o cliente
                if(vehicles[i].batteryUsed + batterConsumption > vehicles[i].batteryKm){
                    vehicles[i].violationBattery = (vehicles[i].batteryUsed + batterConsumption) - vehicles[i].batteryKm;
                }

                if((vehicles[i].batteryUsed + batterConsumption > vehicles[i].batteryKm)){
                    if (!vehicles[i].route.back().batteryStation){
                        vehicles[i].ride                    += data.closerStation[vehicles[i].route.back().id].second + data.requests[data.closerStation[vehicles[i].route.back().id].first.second].waitTimeStation;
                        vehicles[i].justDistance            += data.closerStation[vehicles[i].route.back().id].second;
                        vehicles[i].batteryUsed             += data.closerStation[vehicles[i].route.back().id].second;
                        vehicles[i].numberStationsVisited++;
                        vehicles[i].route.push_back(data.requests[data.closerStation[vehicles[i].route.back().id].first.second]);
                        vehicles[i].timeCharging = ((vehicles[i].batteryUsed * vehicles[i].rateConsumption)) * vehicles[i].route.back().rechargeRate;
                        vehicles[i].chargingTimeStation.push_back(make_pair(data.requests[data.closerStation[vehicles[i].route.back().id].first.second].id, vehicles[i].timeCharging));
                        vehicles[i].batteryUsed = 0;
                    }
                }

                if(vehicles[i].demand + data.requests[k].demand <= vehicles[i].demandCapacity){
                    vehicles[i].ride                += data.distances[vehicles[i].route.back().id][data.requests[k].id];
                    vehicles[i].justDistance       += data.distances[vehicles[i].route.back().id][data.requests[k].id];
                    wait = 0, vTw = 0;

                    if(vehicles[i].ride < data.requests[k].twA){
                        wait                        = data.requests[k].twA - vehicles[i].ride;
                        vehicles[i].ride  = data.requests[k].twA;
                    }else if(vehicles[i].ride > data.requests[k].twB){
                        vTw                         = vehicles[i].ride - data.requests[k].twB;
                        vehicles[i].ride  = data.requests[k].twB;
                    }

                    vehicles[i].ride                += data.requests[k].serviceTime;
                    vehicles[i].batteryUsed         += data.distances[vehicles[i].route.back().id][data.requests[k].id];
                    vehicles[i].route.push_back(data.requests[k]);
                    vehicles[i].demand              += data.requests[k].demand;
                    vehicles[i].violationTw         += vTw;
                    k++;
                }else{
                    i++;
                }
            }else{
                if(i == (int)vehicles.size()){
                    cout << "NUMERO DE VEICULOS EXEDIDO" << endl;
                    exit(1);
                }
                i++;
            }
        }else{
            k++;
        }
    }

    for (int i = 0; i < (int)vehicles.size(); i++){
        if(vehicles[i].route.size() < 2){
            vehicles[i].route.push_back(data.requests[0]);                              //finalizar veículos vazios
        }
    }

    for (i = 0; i < (int)vehicles.size(); i++){
        if(vehicles[i].route.back().id != 0){
            vehicles[i].penality = (vehicles[i].violationTw * data.betaTw) + (vehicles[i].violationDemand * data.betaDemand) + (vehicles[i].violationBattery * data.betaBattery);
            vehicles[i].ride   += data.distances[vehicles[i].route.back().id][0];
            vehicles[i].justDistance        += data.distances[vehicles[i].route.back().id][0];
            vehicles[i].batteryUsed         += data.distances[vehicles[i].route.back().id][0];
            vehicles[i].route.push_back(data.requests[0]);                          //volta pro deposito com deposito
        }

        numberStationsVisited           += vehicles[i].numberStationsVisited;
        ride               += vehicles[i].ride;
        justDistance                    += vehicles[i].justDistance;
        demand                          += vehicles[i].demand;
        penality                        += vehicles[i].penality;

        if(vehicles[i].route.size() > 2){
            vehicleCost                    += vehicles[i].vehicleCost;
        }
        vehicles[i].objective += vehicles[i].penality + vehicles[i].vehicleCost + vehicles[i].justDistance;
    }
    objective = justDistance + vehicleCost + penality;
}

void Solution::evaluateSolutionTW(Data &data){
    violationTw             = 0;
    violationDemand         = 0;
    violationBattery        = 0;
    justDistance            = 0;
    numberStationsVisited   = 0;
    penality                = 0;
    vehicleCost            = 0;
    ride                    = 0;
    amountVehicles          = 0;

    for (int i = 0; i < (int)vehicles.size(); i++) {
        vehicles[i].evaluateVehicleTW(data);
        violationTw             += vehicles[i].violationTw;
        violationBattery        += vehicles[i].violationBattery;
        violationDemand         += vehicles[i].violationDemand;
        justDistance            += vehicles[i].justDistance;
        numberStationsVisited   += vehicles[i].numberStationsVisited;
        penality                += vehicles[i].penality;
        ride                    += vehicles[i].ride;

        if((int)vehicles[i].route.size() > 2){
            vehicleCost                    += vehicles[i].vehicleCost;
            amountVehicles++;
        }
    }
    objective = justDistance + vehicleCost + penality;
}

void Vehicle::evaluateVehicleTW(Data &data){
    ride = 0; 
    demand = 0;
    penality = 0;
    violationTw = 0;
    batteryUsed = 0;
    justDistance = 0;
    numberStationsVisited = 0;
    violationBattery = 0;
    violationDemand = 0;

    if(route.size() > 2){
        for(int i = 0; i < (int)route.size()-1; i++){

            if(route[i].batteryStation){                                //É bateria
                numberStationsVisited++;
                if(batteryKm - batteryUsed < EPS){
                    timeCharging        = (batteryKm * rateConsumption) * route[i].rechargeRate;
                    batteryUsed         = 0; //carga total
                }else{
                    timeCharging        = (batteryUsed * rateConsumption) * route[i].rechargeRate;
                    batteryUsed         = 0; //carga suficiente
                }
                ride += timeCharging;
            }else{
                if(ride < route[i].twA){                                //janela A
                    ride = route[i].twA;
                }else if(ride > route[i].twB){                          //janela B
                    violationTw += (ride - route[i].twB);
                    ride = route[i].twB;
                }
    
                ride += route[i].serviceTime;                           //tempo de serviço
                demand += route[i].demand;
            }

            ride            += data.distances[route[i].id][route[i+1].id];
            justDistance    += data.distances[route[i].id][route[i+1].id];
            batteryUsed     += data.distances[route[i].id][route[i+1].id];
            
            if(batteryUsed > batteryKm){
                violationBattery += (batteryUsed - batteryKm);
            }
        }

        if(ride > route[route.size()-1].twB){                          //janela B
            violationTw += (ride - route[route.size()-1].twB);
            ride = route[route.size()-1].twB;
        }
        
        if(demand > demandCapacity){
            violationDemand += (demand - demandCapacity);
        }

        penality = (violationTw * data.betaTw) + (violationDemand * data.betaDemand) + (violationBattery * data.betaBattery);
        
        objective = justDistance + penality + vehicleCost;
    }
}

void Solution::shakeTW(Data &data, int intensity, int &lastShake){

	int iter = 0, maxIntensity = 3;
	
	if(intensity > maxIntensity){
        intensity   = maxIntensity;
    }
    
    while(iter < intensity){
        uniform_int_distribution<int> distribution(0, 99);
        int select = distribution(generator);

        if(select < 33){
            shakeSwapTW(data);
            lastShake = 3;
        }else if(select < 66){
            shakeRelocationTW(data);
            lastShake = 1;
        }else{
            shakeRemoveStationTW(data);
        }
        iter++;
    }
}

void Solution::shakeSwapTW(Data &data){
    int v1, v2, r1, r2, k = 0;
    Vehicle car1, car2;

    if((int)vehicles.size() > 1){
    	uniform_int_distribution<int> distribution(0, vehicles.size()-1);
    	do{
    		v1  = distribution(generator);
    		v2  = distribution(generator);
    		if(k > 10){
    			shakeRelocationTW(data);
    			return;
    		}
    		k++;
    	}while(v1 == v2 || (int)vehicles[v1].route.size() < 3 || (int)vehicles[v2].route.size() < 3);

    	uniform_int_distribution<int> distribution2(1,(int)vehicles[v1].route.size()-2);
	    r1      = distribution2(generator);
	    uniform_int_distribution<int> distribution3(1,(int)vehicles[v2].route.size()-2);
	    r2      = distribution3(generator);
		
		car1    = vehicles[v1];
	    car2    = vehicles[v2];
        
	    swap(car1.route[r1], car2.route[r2]);
        
	    car1.evaluateVehicleTW(data);
	    car2.evaluateVehicleTW(data);

	    ride                    = ride - 					vehicles[v1].ride - 					vehicles[v2].ride 					+ car1.ride 					+ car2.ride;
	    demand                  = demand - 					vehicles[v1].demand - 					vehicles[v2].demand 				+ car1.demand 					+ car2.demand;
	    penality                = penality - 				vehicles[v1].penality - 				vehicles[v2].penality 				+ car1.penality 				+ car2.penality;
	    violationTw             = violationTw - 			vehicles[v1].violationTw - 				vehicles[v2].violationTw 			+ car1.violationTw 				+ car2.violationTw;
	    justDistance            = justDistance - 			vehicles[v1].justDistance - 			vehicles[v2].justDistance 			+ car1.justDistance 			+ car2.justDistance;
	    violationDemand         = violationDemand - 		vehicles[v1].violationDemand - 			vehicles[v2].violationDemand 		+ car1.violationDemand 			+ car2.violationDemand;
	    violationBattery        = violationBattery - 		vehicles[v1].violationBattery - 		vehicles[v2].violationBattery 		+ car1.violationBattery 		+ car2.violationBattery;
	    numberStationsVisited   = numberStationsVisited - 	vehicles[v1].numberStationsVisited - 	vehicles[v2].numberStationsVisited 	+ car1.numberStationsVisited 	+ car2.numberStationsVisited;

	    vehicles[v1]            = car1;
	    vehicles[v2]            = car2;

	    objective = justDistance + vehicleCost + penality;  
    }else{
    	v1 = 0;
    	uniform_int_distribution<int> distribution2(1,(int)vehicles[v1].route.size()-2);
	    r1      = distribution2(generator);
	    uniform_int_distribution<int> distribution3(1,(int)vehicles[v1].route.size()-2);
	    r2      = distribution3(generator);
		
		car1    = vehicles[v1];

	    swap(car1.route[r1], car1.route[r2]);
		    
	    car1.evaluateVehicleTW(data);

	    ride                    = ride - 					vehicles[v1].ride  					+ car1.ride;
	    demand                  = demand - 					vehicles[v1].demand  				+ car1.demand;
	    penality                = penality - 				vehicles[v1].penality  				+ car1.penality;
	    violationTw             = violationTw - 			vehicles[v1].violationTw  			+ car1.violationTw;
	    justDistance            = justDistance - 			vehicles[v1].justDistance  			+ car1.justDistance;
	    violationDemand         = violationDemand - 		vehicles[v1].violationDemand  		+ car1.violationDemand;
	    violationBattery        = violationBattery - 		vehicles[v1].violationBattery  		+ car1.violationBattery;
	    numberStationsVisited   = numberStationsVisited - 	vehicles[v1].numberStationsVisited 	+ car1.numberStationsVisited;

	    vehicles[v1]            = car1;
	    objective = justDistance + vehicleCost + penality;
    }
}

void Solution::shakeRelocationTW(Data &data){
    int v1, v2, r1, r2;
    Vehicle car1, car2;
    if((int)vehicles.size() > 1){
    	uniform_int_distribution<int> distribution(0, vehicles.size()-1);
    	
    	do{
	        v1  = distribution(generator);
	        v2  = distribution(generator);
	    }while(v1 == v2 || (int)vehicles[v1].route.size() < 3);

	    uniform_int_distribution<int> distribution2(1,(int)vehicles[v1].route.size()-2);
	    r1      = distribution2(generator);

	    if((int)vehicles[v2].route.size() < 3){
	        r2 = 1;
	    }else{
	        uniform_int_distribution<int> distribution3(1,(int)vehicles[v2].route.size()-2);
	        r2      = distribution3(generator);
	    }

	    car1    = vehicles[v1];
	    car2    = vehicles[v2];
        
	    car2.route.insert(car2.route.begin() + r2, car1.route[r1]);
	    car1.route.erase(car1.route.begin() + r1);
	    
	    car1.evaluateVehicleTW(data);
	    car2.evaluateVehicleTW(data);

	    ride                    = ride -                vehicles[v1].ride - vehicles[v2].ride + car1.ride + car2.ride;
	    demand                  = demand -              vehicles[v1].demand - vehicles[v2].demand + car1.demand + car2.demand;
	    penality                = penality -            vehicles[v1].penality - vehicles[v2].penality + car1.penality + car2.penality;
	    violationTw             = violationTw -         vehicles[v1].violationTw - vehicles[v2].violationTw + car1.violationTw + car2.violationTw;
	    justDistance            = justDistance -        vehicles[v1].justDistance - vehicles[v2].justDistance + car1.justDistance + car2.justDistance;
	    violationDemand         = violationDemand -     vehicles[v1].violationDemand - vehicles[v2].violationDemand + car1.violationDemand + car2.violationDemand;
	    violationBattery        = violationBattery -    vehicles[v1].violationBattery - vehicles[v2].violationBattery + car1.violationBattery + car2.violationBattery;
	    numberStationsVisited   = numberStationsVisited - vehicles[v1].numberStationsVisited - vehicles[v2].numberStationsVisited + car1.numberStationsVisited + car2.numberStationsVisited;
	    
	    if((int)vehicles[v1].route.size() > 2 && (int)car1.route.size() < 3){
	        vehicleCost = vehicleCost - car1.vehicleCost;
	        amountVehicles--;
	    }
	    if((int)vehicles[v2].route.size() < 3 && (int)car2.route.size() > 2){
	        vehicleCost = vehicleCost + car2.vehicleCost;
	        amountVehicles++;
	    }
	    
	    vehicles[v1]            = car1;
	    vehicles[v2]            = car2;

	    objective = justDistance + vehicleCost + penality;
    }else{
    	v1  = 0;

	    uniform_int_distribution<int> distribution2(1,(int)vehicles[v1].route.size()-2);
	    do{
	    	r1      = distribution2(generator);
	        r2      = distribution2(generator);
	    }while(r1 >= r2);

	    car1    = vehicles[v1];

	    car1.route.insert(car1.route.begin() + r2+1, car1.route[r1]);
	    car1.route.erase(car1.route.begin() + r1);

	    car1.evaluateVehicleTW(data);

	    ride                    = ride -                  vehicles[v1].ride 					+ car1.ride;
	    demand                  = demand -                vehicles[v1].demand 					+ car1.demand;
	    penality                = penality -              vehicles[v1].penality 				+ car1.penality;
	    violationTw             = violationTw -           vehicles[v1].violationTw 				+ car1.violationTw;
	    justDistance            = justDistance -          vehicles[v1].justDistance 			+ car1.justDistance;
	    violationDemand         = violationDemand -       vehicles[v1].violationDemand 			+ car1.violationDemand;
	    violationBattery        = violationBattery -      vehicles[v1].violationBattery 		+ car1.violationBattery;
	    numberStationsVisited   = numberStationsVisited - vehicles[v1].numberStationsVisited 	+ car1.numberStationsVisited;
	    	    
	    vehicles[v1]            = car1;

	    objective = justDistance + vehicleCost + penality;
    }
}

void Solution::shakeRemoveStationTW(Data &data){
    int v1;
    Vehicle car1;
    Request st1;
    vector< pair < pair < int, int >, int > > stations;
    for(int i = 0; i < (int)vehicles.size(); i++){
        for(int j = 0; j < (int)vehicles[i].route.size(); j++){
            if(vehicles[i].route[j].batteryStation){
                stations.push_back(make_pair(make_pair(i, j), vehicles[i].route[j].id));
            }
        }

    }
    if((int)stations.size() > 0){
        uniform_int_distribution<int> distribution(0, stations.size()-1);
        v1 = distribution(generator);
        st1 = data.requests[stations[v1].second];
        for(int i = (int)stations.size() -1; i >= 0; i--){
            if(stations[i].second == st1.id){
                vehicles[stations[i].first.first].route.erase(vehicles[stations[i].first.first].route.begin() + stations[i].first.second);
            }
        }
        evaluateSolutionTW(data);
    }
}

void Solution::interRVNDTW(Data &data, int lastShake){
    double bestObjective = objective;
    int k = 0;
    vector < pair < int, int > > idRemove;
    vector<int> neighbors;
    neighbors.push_back(1);
    neighbors.push_back(2);
    neighbors.push_back(3);
    neighbors.push_back(4);
    neighbors.push_back(5);
    neighbors.push_back(6);
    neighbors.push_back(7);
    if(lastShake != 0){
        neighbors.erase(neighbors.begin() + lastShake-1);
    }
    shuffle(neighbors.begin(), neighbors.end(), generator);
    
    for(int i = 0; i < (int)vehicles.size(); i++){
        if(vehicles[i].route.size() > 2){
            if(strcmp(vehicles[i].route[1].sigla,"S0") == 0){
                idRemove.push_back(make_pair(i, 1));
            }
            if(strcmp(vehicles[i].route[vehicles[i].route.size()-2].sigla, "S0") == 0 && vehicles[i].route.size() > 3){
                idRemove.push_back(make_pair(i, vehicles[i].route.size()-2));
            }
        }
    }
    for(int i = (int)idRemove.size() -1; i >= 0; i--){
        vehicles[idRemove[i].first].route.erase(vehicles[idRemove[i].first].route.begin() + idRemove[i].second);
        vehicles[idRemove[i].first].evaluateVehicleTW(data);
    }

    while(true){
        if (k >= (int)neighbors.size()){break;}

        if(neighbors[k] == 1){
            interRelocationTW(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                k = 0;
                shuffle(neighbors.begin(), neighbors.end(), generator);
            }else{
                k++;
            }
        }else if(neighbors[k] == 2){
            interRelocation2TW(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if(neighbors[k] == 3){
            interSwapTW(data);
            
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 4){
            interSwap2TW(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 5){
            addStationTW(data);
            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 6){
            removeStationTW(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else if (neighbors[k] == 7){
            interSwap2x1TW(data);

            if(bestObjective - objective > EPS){
                bestObjective = objective;
                shuffle(neighbors.begin(), neighbors.end(), generator);
                k = 0;
            }else{
                k++;
            }
        }else{
            k++;
        }
    }
}

void Solution::addStationTW(Data &data){
    Solution solution;
    Vehicle car, bestCar;
    bool improve = false;
    double bestObjective = objective, newObjective, newJustDistance, newPenality;
    int idCar;

    for(int i = 1; i <= data.numBatteryStations; i++){
        for(int k = 0; k < (int)vehicles.size(); k++){
            if((int)vehicles[k].route.size() > 2){
                car = vehicles[k];
                for(int j = 1; j < (int)vehicles[k].route.size(); j++){
                    if(!(j == 1 && strcmp(data.requests[i].sigla, "S0") == 0)){
                        if(!(j == (int)vehicles[k].route.size() -1 && strcmp(data.requests[i].sigla, "S0") == 0)){

                            car.route.insert(car.route.begin() + j, data.requests[i]);
                            car.evaluateVehicleTW(data);

                            newJustDistance = justDistance + car.justDistance - vehicles[k].justDistance;
                            newPenality     = penality     + car.penality     - vehicles[k].penality;

                            newObjective = newJustDistance + vehicleCost + newPenality;
                            if(bestObjective - newObjective > EPS){
                                improve = true;
                                bestCar = car;
                                idCar = k;
                                bestObjective = newObjective;
                            }
                            car = vehicles[k];
                        }
                    }
                }
            }
        }
    }

    if(improve){
        ride                    = ride -                    vehicles[idCar].ride                   + bestCar.ride;
        penality                = penality -                vehicles[idCar].penality               + bestCar.penality;
        violationTw             = violationTw -             vehicles[idCar].violationTw            + bestCar.violationTw;
        justDistance            = justDistance -            vehicles[idCar].justDistance           + bestCar.justDistance;
        violationDemand         = violationDemand -         vehicles[idCar].violationDemand        + bestCar.violationDemand;
        violationBattery        = violationBattery -        vehicles[idCar].violationBattery       + bestCar.violationBattery;
        numberStationsVisited   = numberStationsVisited -   vehicles[idCar].numberStationsVisited  + bestCar.numberStationsVisited;
        
        swap(vehicles[idCar], bestCar);
        
        objective = justDistance + penality + vehicleCost;
    }
}

void Solution::removeStationTW(Data &data){
    Vehicle car, bestCar;
    int idCar;
    bool improve = false;
    double newObjective, bestObjective = objective, newPenality, newJustDistance, newVehiclesCost;
    vector < pair < int, int > > idRemove;

    for(int i = 0; i < (int)vehicles.size(); i++){
        if((int)vehicles[i].route.size() > 2){
            for(int j = 1; j < (int)vehicles[i].route.size() - 1; j++){
                if(vehicles[i].route[j].batteryStation){
                    if(vehicles[i].route[j-1].id == vehicles[i].route[j].id){
                        idRemove.push_back(make_pair(i, j-1));
                    }
                }
            }
        }
    }

    for(int i = (int)idRemove.size() -1; i >= 0; i--){
        vehicles[idRemove[i].first].route.erase(vehicles[idRemove[i].first].route.begin() + idRemove[i].second);
        vehicles[idRemove[i].first].evaluateVehicleTW(data);
    }

    for(int i = 0; i < (int)vehicles.size(); i++){
        if((int)vehicles[i].route.size() > 2){
            for(int j = 1; j < (int)vehicles[i].route.size() - 1; j++){
                newVehiclesCost = vehicleCost;
                if(vehicles[i].route[j].batteryStation){
                    car = vehicles[i];
                    
                    car.route.erase(car.route.begin() + j);
                    car.evaluateVehicleTW(data);

                    newJustDistance = justDistance + car.justDistance - vehicles[i].justDistance;
                    newPenality     = penality     + car.penality     - vehicles[i].penality;

                    newObjective = newJustDistance + newVehiclesCost + newPenality;

                    if(bestObjective - newObjective > EPS){
                        improve = true;
                        bestCar = car;
                        idCar = i;
                        bestObjective = newObjective;
                    }
                }
            }
        }
    }

    if(improve){
        ride                    = ride -                    vehicles[idCar].ride                   + bestCar.ride;
        penality                = penality -                vehicles[idCar].penality               + bestCar.penality;
        violationTw             = violationTw -             vehicles[idCar].violationTw            + bestCar.violationTw;
        justDistance            = justDistance -            vehicles[idCar].justDistance           + bestCar.justDistance;
        violationDemand         = violationDemand -         vehicles[idCar].violationDemand        + bestCar.violationDemand;
        violationBattery        = violationBattery -        vehicles[idCar].violationBattery       + bestCar.violationBattery;
        numberStationsVisited   = numberStationsVisited -   vehicles[idCar].numberStationsVisited  + bestCar.numberStationsVisited;
        
        swap(vehicles[idCar], bestCar);
        
        objective = justDistance + penality + vehicleCost;
    }
}

void Solution::interRelocationTW(Data &data){
    int bestV1, bestV2, contEmptyVehicle = 0;
    double newTravelledDistance, newPenality, bestObjective = objective, newObjective, newVehiclesCost = vehicleCost;
    bool improve = true;
    Vehicle car1, car2, bestCar1, bestCar2;

    vector < pair < pair < int, int >, pair < int, int > > > vt;
    improve = false;
    for(int i = 0; i < (int)vehicles.size();i++) {
        if((int)vehicles[i].route.size() > 2){
            for(int j = 0; j < (int)vehicles.size(); j++) {
                if(i != j){
                    if((int)vehicles[j].route.size() > 3 || contEmptyVehicle == 0){
                        for(int r1 = 1; r1 < (int)vehicles[i].route.size()-1; r1++) {
                            if((int)vehicles[j].route.size() < 3){
                                vt.push_back(make_pair(make_pair(i, r1), make_pair(j, 1)));
                            }else{
                                for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                                    vt.push_back(make_pair(make_pair(i, r1), make_pair(j, r2)));
                                }                        
                            }
                        }
                        contEmptyVehicle++;
                    }
                }
            }
        }
    }

    for(int i = 0; i < (int)vt.size(); i++){
        newVehiclesCost = vehicleCost;
            
        car1                    = vehicles[vt[i].first.first];
        car2                    = vehicles[vt[i].second.first];

        car2.route.insert(car2.route.begin() + vt[i].second.second, car1.route[vt[i].first.second]);
        car1.route.erase(car1.route.begin() + vt[i].first.second);
        
        car1.evaluateVehicleTW(data);
        car2.evaluateVehicleTW(data);
    
        newTravelledDistance    = justDistance      + car1.justDistance      + car2.justDistance -      vehicles[vt[i].first.first].justDistance -      vehicles[vt[i].second.first].justDistance;
        newPenality             = penality          + car1.penality          + car2.penality -          vehicles[vt[i].first.first].penality -          vehicles[vt[i].second.first].penality;
        
        if(vehicles[vt[i].first.first].route.size() > 2 && car1.route.size() < 3){
            newVehiclesCost = vehicleCost - car1.vehicleCost;
        }

        if (vehicles[vt[i].second.first].route.size() < 3 && car2.route.size() > 2){
            newVehiclesCost = vehicleCost + car2.vehicleCost;
        }

        newObjective = newTravelledDistance + newVehiclesCost + newPenality;

        if(bestObjective - newObjective > EPS){
            bestV1      = vt[i].first.first;
            bestV2      = vt[i].second.first;
            bestCar1    = car1;
            bestCar2    = car2;
            improve     = true;
            bestObjective = newObjective;
        }
    }

    if(improve){
        if(bestCar1.route.size() > 2){
            bestCar1.intraRVNDTW(data);
        }

        bestCar2.intraRVNDTW(data);

        ride                    = ride -                    vehicles[bestV1].ride -                  vehicles[bestV2].ride                  + bestCar1.ride                  + bestCar2.ride;
        demand                  = demand -                  vehicles[bestV1].demand -                vehicles[bestV2].demand                + bestCar1.demand                + bestCar2.demand;
        penality                = penality -                vehicles[bestV1].penality -              vehicles[bestV2].penality              + bestCar1.penality              + bestCar2.penality;
        violationTw             = violationTw -             vehicles[bestV1].violationTw -           vehicles[bestV2].violationTw           + bestCar1.violationTw           + bestCar2.violationTw;
        justDistance            = justDistance -            vehicles[bestV1].justDistance -          vehicles[bestV2].justDistance          + bestCar1.justDistance          + bestCar2.justDistance;
        violationDemand         = violationDemand -         vehicles[bestV1].violationDemand -       vehicles[bestV2].violationDemand       + bestCar1.violationDemand       + bestCar2.violationDemand;
        violationBattery        = violationBattery -        vehicles[bestV1].violationBattery -      vehicles[bestV2].violationBattery      + bestCar1.violationBattery      + bestCar2.violationBattery;
        numberStationsVisited   = numberStationsVisited -   vehicles[bestV1].numberStationsVisited - vehicles[bestV2].numberStationsVisited + bestCar1.numberStationsVisited + bestCar2.numberStationsVisited;

        if(vehicles[bestV1].route.size() > 2 && bestCar1.route.size() < 3){
            vehicleCost = vehicleCost - bestCar1.vehicleCost;
            amountVehicles--;
        }

        if(vehicles[bestV2].route.size() < 3 && bestCar2.route.size() > 2){
            vehicleCost = vehicleCost + bestCar2.vehicleCost;
            amountVehicles++;
        }
        
        objective               = justDistance + vehicleCost + penality;

        swap(vehicles[bestV1], bestCar1);
        swap(vehicles[bestV2], bestCar2);
    }
}

void Solution::interRelocation2TW(Data &data){
    int bestV1 = -1, bestV2 = -1, contEmptyVehicle = 0, v1, v2, r1, r1_2, r2;
    double newTravelledDistance, newPenality, bestObjective = objective, newObjective, newVehiclesCost = vehicleCost;
    bool improve = true;
    Vehicle car1, car2, bestCar1, bestCar2;

    vector < pair < pair < pair < int, int >, int >, pair < int, int > > > vt;
    improve = false;
    for(int i = 0; i < (int)vehicles.size();i++) {
        if((int)vehicles[i].route.size() > 3){
            for(int j = 0; j < (int)vehicles.size(); j++) {
                if(i != j){
                    if((int)vehicles[j].route.size() > 2 || contEmptyVehicle == 0){
                        for(int r1 = 1; r1 < (int)vehicles[i].route.size()-2; r1++) {
                            if((int)vehicles[j].route.size() < 3){
                                vt.push_back(make_pair(make_pair(make_pair(i, r1), r1+1), make_pair(j, 1)));
                            }else{
                                for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                                    vt.push_back(make_pair(make_pair(make_pair(i, r1), r1+1), make_pair(j, r2)));
                                }                        
                            }
                        }
                        contEmptyVehicle++;
                    }
                }
            }
        }
    }
    for(int i = 0; i < (int)vt.size(); i++){
        newVehiclesCost = vehicleCost;
        v1 = vt[i].first.first.first;
        v2 = vt[i].second.first;
        r1 = vt[i].first.first.second;
        r1_2 = vt[i].first.second;
        r2 = vt[i].second.second;

        car1                    = vehicles[vt[i].first.first.first];
        car2                    = vehicles[vt[i].second.first];
        
        car2.route.insert(car2.route.begin() + r2, car1.route[r1_2]);
        car2.route.insert(car2.route.begin() + r2, car1.route[r1]);
        car1.route.erase(car1.route.begin() + r1_2);
        car1.route.erase(car1.route.begin() + r1);
        
        car1.evaluateVehicleTW(data);
        car2.evaluateVehicleTW(data);
    
        newTravelledDistance    = justDistance + car1.justDistance + car2.justDistance - vehicles[vt[i].first.first.first].justDistance - vehicles[vt[i].second.first].justDistance;
        newPenality             = penality          + car1.penality          + car2.penality -          vehicles[vt[i].first.first.first].penality -          vehicles[vt[i].second.first].penality;
        
        if(vehicles[v1].route.size() > 2 && car1.route.size() < 3){
            newVehiclesCost = vehicleCost - car1.vehicleCost;
        }

        if (vehicles[v2].route.size() < 3 && car2.route.size() > 2){
            newVehiclesCost = vehicleCost + car2.vehicleCost;
        }

        newObjective = newTravelledDistance + newVehiclesCost + newPenality;

        if(bestObjective - newObjective > EPS){
            bestV1      = vt[i].first.first.first;
            bestV2      = vt[i].second.first;
            bestCar1    = car1;
            bestCar2    = car2;
            improve     = true;
            bestObjective = newObjective;
        }

        // Inverte
        swap(car2.route[r2], car2.route[r2+1]);

        car2.evaluateVehicleTW(data);
    
        newTravelledDistance    = justDistance + car1.justDistance + car2.justDistance - vehicles[vt[i].first.first.first].justDistance - vehicles[vt[i].second.first].justDistance;
        newPenality             = penality          + car1.penality          + car2.penality -          vehicles[vt[i].first.first.first].penality -          vehicles[vt[i].second.first].penality;
        
        if(vehicles[vt[i].first.first.first].route.size() > 2 && car1.route.size() < 3){
            newVehiclesCost = vehicleCost - car1.vehicleCost;
        }

        if (vehicles[vt[i].second.first].route.size() < 3 && car2.route.size() > 2){
            newVehiclesCost = vehicleCost + car2.vehicleCost;
        }

        newObjective = newTravelledDistance + newVehiclesCost + newPenality;

        if(bestObjective - newObjective > EPS){
            bestV1      = vt[i].first.first.first;
            bestV2      = vt[i].second.first;
            bestCar1    = car1;
            bestCar2    = car2;
            improve     = true;
            bestObjective = newObjective;
        }
    }

    if(improve){
        if(bestCar1.route.size() > 2){
            bestCar1.intraRVNDTW(data);
        }

        bestCar2.intraRVNDTW(data);

        ride                    = ride -                    vehicles[bestV1].ride -                  vehicles[bestV2].ride                  + bestCar1.ride                  + bestCar2.ride;
        demand                  = demand -                  vehicles[bestV1].demand -                vehicles[bestV2].demand                + bestCar1.demand                + bestCar2.demand;
        penality                = penality -                vehicles[bestV1].penality -              vehicles[bestV2].penality              + bestCar1.penality              + bestCar2.penality;
        violationTw             = violationTw -             vehicles[bestV1].violationTw -           vehicles[bestV2].violationTw           + bestCar1.violationTw           + bestCar2.violationTw;
        justDistance            = justDistance -            vehicles[bestV1].justDistance -          vehicles[bestV2].justDistance          + bestCar1.justDistance          + bestCar2.justDistance;
        violationDemand         = violationDemand -         vehicles[bestV1].violationDemand -       vehicles[bestV2].violationDemand       + bestCar1.violationDemand       + bestCar2.violationDemand;
        violationBattery        = violationBattery -        vehicles[bestV1].violationBattery -      vehicles[bestV2].violationBattery      + bestCar1.violationBattery      + bestCar2.violationBattery;
        numberStationsVisited   = numberStationsVisited -   vehicles[bestV1].numberStationsVisited - vehicles[bestV2].numberStationsVisited + bestCar1.numberStationsVisited + bestCar2.numberStationsVisited;
        
        if(vehicles[bestV1].route.size() > 2 && bestCar1.route.size() < 3){
            vehicleCost = vehicleCost - bestCar1.vehicleCost;
            amountVehicles--;
        }
        
        if(vehicles[bestV2].route.size() < 3 && bestCar2.route.size() > 2){
            vehicleCost = vehicleCost + bestCar2.vehicleCost;
            amountVehicles++;
        }

        objective               = justDistance + vehicleCost + penality;
        
        swap(vehicles[bestV1], bestCar1);
        swap(vehicles[bestV2], bestCar2);
    }
}

void Solution::interSwapTW(Data &data){
    int bestV1, bestV2;//, bestR1, bestR2, k = 0;
    double newTravelledDistance, newPenality, bestObjective = objective, newObjective, newVehiclesCost = vehicleCost;
    bool improve = false;
    Vehicle car1, car2, bestCar1, bestCar2;

    vector < pair < pair < int, int >, pair < int, int > > > vt;
    for(int i = 0; i < (int)vehicles.size();i++) {
        if((int)vehicles[i].route.size() > 2){
            for(int j = 0; j < (int)vehicles.size(); j++) {
                if(i != j){
                    for(int r1 = 1; r1 < (int)vehicles[i].route.size()-1; r1++) {
                        if((int)vehicles[j].route.size() > 2){
                            for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                                vt.push_back(make_pair(make_pair(i, r1), make_pair(j, r2)));
                            }                        
                        }
                    }
                }
            }
        }
    }

    for(int i = 0; i < (int)vt.size(); i++){
        car1                    = vehicles[vt[i].first.first];
        car2                    = vehicles[vt[i].second.first];

        swap(car1.route[vt[i].first.second], car2.route[vt[i].second.second]);

        car1.evaluateVehicleTW(data);
        car2.evaluateVehicleTW(data);
    
        newTravelledDistance    = justDistance + car1.justDistance + car2.justDistance - vehicles[vt[i].first.first].justDistance - vehicles[vt[i].second.first].justDistance;
        newPenality             = penality          + car1.penality          + car2.penality -          vehicles[vt[i].first.first].penality -          vehicles[vt[i].second.first].penality;
        
        newObjective = newTravelledDistance + newVehiclesCost + newPenality;
        
        if(bestObjective - newObjective > EPS){
            bestV1      = vt[i].first.first;
            bestV2      = vt[i].second.first;
            bestCar1    = car1;
            bestCar2    = car2;
            improve     = true;
            bestObjective = newObjective;
        }
    }

    if(improve){

        bestCar1.intraRVNDTW(data);
        bestCar2.intraRVNDTW(data);

        ride                    = ride -                    vehicles[bestV1].ride -                  vehicles[bestV2].ride                  + bestCar1.ride                  + bestCar2.ride;
        demand                  = demand -                  vehicles[bestV1].demand -                vehicles[bestV2].demand                + bestCar1.demand                + bestCar2.demand;
        penality                = penality -                vehicles[bestV1].penality -              vehicles[bestV2].penality              + bestCar1.penality              + bestCar2.penality;
        violationTw             = violationTw -             vehicles[bestV1].violationTw -           vehicles[bestV2].violationTw           + bestCar1.violationTw           + bestCar2.violationTw;
        justDistance            = justDistance -            vehicles[bestV1].justDistance -          vehicles[bestV2].justDistance          + bestCar1.justDistance          + bestCar2.justDistance;
        violationDemand         = violationDemand -         vehicles[bestV1].violationDemand -       vehicles[bestV2].violationDemand       + bestCar1.violationDemand       + bestCar2.violationDemand;
        violationBattery        = violationBattery -        vehicles[bestV1].violationBattery -      vehicles[bestV2].violationBattery      + bestCar1.violationBattery      + bestCar2.violationBattery;
        numberStationsVisited   = numberStationsVisited -   vehicles[bestV1].numberStationsVisited - vehicles[bestV2].numberStationsVisited + bestCar1.numberStationsVisited + bestCar2.numberStationsVisited;

        objective               = justDistance + vehicleCost + penality;

        swap(vehicles[bestV1], bestCar1);
        swap(vehicles[bestV2], bestCar2);
    }
}

void Solution::interSwap2TW(Data &data){
    int bestV1, bestV2;//, bestR1, bestR2, k = 0;
    double newTravelledDistance, newPenality, bestObjective = objective, newObjective, newVehiclesCost = vehicleCost;
    bool improve = false;
    Vehicle car1, car2, bestCar1, bestCar2;

    vector < pair < pair < pair < int, int >, int >, pair < pair < int, int >, int > > > vt;
    for(int i = 0; i < (int)vehicles.size(); i++) {
        for(int j = 0; j < (int)vehicles.size(); j++) {
            if(i != j){
                if((int)vehicles[i].route.size() > 3 && (int)vehicles[j].route.size() > 3){                
                    for(int r1 = 1; r1 < (int)vehicles[i].route.size()-2; r1++) {                    
                        for(int r2 = 1; r2 < (int)vehicles[j].route.size()-2; r2++) {
                            vt.push_back( make_pair( make_pair( make_pair(i, r1), r1+1), make_pair( make_pair(j, r2), r2+1)));
                        }
                    }
                }
            }
        }
    }


    for(int i = 0; i < (int)vt.size(); i++){

        car1                    = vehicles[vt[i].first.first.first];
        car2                    = vehicles[vt[i].second.first.first];

        swap(car1.route[vt[i].first.first.second], car2.route[vt[i].second.first.second]);
        swap(car1.route[vt[i].first.second], car2.route[vt[i].second.second]);
        
        car1.evaluateVehicleTW(data);
        car2.evaluateVehicleTW(data);

        newTravelledDistance    = justDistance + car1.justDistance + car2.justDistance - vehicles[vt[i].first.first.first].justDistance - vehicles[vt[i].second.first.first].justDistance;
        newPenality             = penality     + car1.penality     + car2.penality -     vehicles[vt[i].first.first.first].penality -     vehicles[vt[i].second.first.first].penality;
        
        newObjective = newTravelledDistance + newVehiclesCost + newPenality;

        if(bestObjective - newObjective > EPS){
            bestV1      = vt[i].first.first.first;
            bestV2      = vt[i].second.first.first;
            bestCar1    = car1;
            bestCar2    = car2;
            improve     = true;
            bestObjective = newObjective;
        }
        
        // Inverte car 1
        
        swap(car1.route[vt[i].first.second], car1.route[vt[i].first.first.second]);
        
        car1.evaluateVehicleTW(data);
        car2.evaluateVehicleTW(data);

        newTravelledDistance    = justDistance + car1.justDistance + car2.justDistance - vehicles[vt[i].first.first.first].justDistance - vehicles[vt[i].second.first.first].justDistance;
        newPenality             = penality     + car1.penality     + car2.penality -     vehicles[vt[i].first.first.first].penality -     vehicles[vt[i].second.first.first].penality;
        
        newObjective = newTravelledDistance + newVehiclesCost + newPenality;

        if(bestObjective - newObjective > EPS){
            bestV1      = vt[i].first.first.first;
            bestV2      = vt[i].second.first.first;
            bestCar1    = car1;
            bestCar2    = car2;
            improve     = true;
            bestObjective = newObjective;
        }

        // Inverte Car 2
        
        swap(car1.route[vt[i].first.second], car1.route[vt[i].first.first.second]);
        swap(car2.route[vt[i].second.first.second], car2.route[vt[i].second.second]);

        car1.evaluateVehicleTW(data);
        car2.evaluateVehicleTW(data);

        newTravelledDistance    = justDistance + car1.justDistance + car2.justDistance - vehicles[vt[i].first.first.first].justDistance - vehicles[vt[i].second.first.first].justDistance;
        newPenality             = penality     + car1.penality     + car2.penality -     vehicles[vt[i].first.first.first].penality -     vehicles[vt[i].second.first.first].penality;
        
        newObjective = newTravelledDistance + newVehiclesCost + newPenality;

        if(bestObjective - newObjective > EPS){
            bestV1      = vt[i].first.first.first;
            bestV2      = vt[i].second.first.first;
            bestCar1    = car1;
            bestCar2    = car2;
            improve     = true;
            bestObjective = newObjective;
        }
    }

    if(improve){
        bestCar1.intraRVNDTW(data);
        bestCar2.intraRVNDTW(data);
        
        ride                    = ride -                    vehicles[bestV1].ride -                  vehicles[bestV2].ride                  + bestCar1.ride                  + bestCar2.ride;
        demand                  = demand -                  vehicles[bestV1].demand -                vehicles[bestV2].demand                + bestCar1.demand                + bestCar2.demand;
        penality                = penality -                vehicles[bestV1].penality -              vehicles[bestV2].penality              + bestCar1.penality              + bestCar2.penality;
        violationTw             = violationTw -             vehicles[bestV1].violationTw -           vehicles[bestV2].violationTw           + bestCar1.violationTw           + bestCar2.violationTw;
        justDistance            = justDistance -            vehicles[bestV1].justDistance -          vehicles[bestV2].justDistance          + bestCar1.justDistance          + bestCar2.justDistance;
        violationDemand         = violationDemand -         vehicles[bestV1].violationDemand -       vehicles[bestV2].violationDemand       + bestCar1.violationDemand       + bestCar2.violationDemand;
        violationBattery        = violationBattery -        vehicles[bestV1].violationBattery -      vehicles[bestV2].violationBattery      + bestCar1.violationBattery      + bestCar2.violationBattery;
        numberStationsVisited   = numberStationsVisited -   vehicles[bestV1].numberStationsVisited - vehicles[bestV2].numberStationsVisited + bestCar1.numberStationsVisited + bestCar2.numberStationsVisited;

        objective               = justDistance + vehicleCost + penality;

        swap(vehicles[bestV1], bestCar1);
        swap(vehicles[bestV2], bestCar2);
    }
}

void Solution::interSwap2x1TW(Data &data){
    int bestV1, bestV2;
    double newTravelledDistance, newPenality, bestObjective = objective, newObjective, newVehiclesCost = vehicleCost;
    bool improve = false;
    Vehicle car1, car2, bestCar1, bestCar2;

    vector < pair < pair < pair < int, int >, int >, pair < pair < int, int >, int > > > vt;
    for(int i = 0; i < (int)vehicles.size(); i++) {
        for(int j = 0; j < (int)vehicles.size(); j++) {
            if(i != j){
                if((int)vehicles[i].route.size() > 3 && (int)vehicles[j].route.size() > 2){                
                    for(int r1 = 1; r1 < (int)vehicles[i].route.size()-2; r1++) {                    
                        for(int r2 = 1; r2 < (int)vehicles[j].route.size()-1; r2++) {
                            vt.push_back( make_pair( make_pair( make_pair(i, r1), r1+1), make_pair( make_pair(j, r2), r2+1)));
                        }
                    }
                }
            }
        }
    }


    for(int i = 0; i < (int)vt.size(); i++){

        car1                    = vehicles[vt[i].first.first.first];
        car2                    = vehicles[vt[i].second.first.first];

        swap(car1.route[vt[i].first.first.second], car2.route[vt[i].second.first.second]);
        car2.route.insert(car2.route.begin() + vt[i].second.second, car1.route[vt[i].first.second]);
        car1.route.erase(car1.route.begin() + vt[i].first.second);
        
        car1.evaluateVehicleTW(data);
        car2.evaluateVehicleTW(data);

        newTravelledDistance    = justDistance + car1.justDistance + car2.justDistance - vehicles[vt[i].first.first.first].justDistance - vehicles[vt[i].second.first.first].justDistance;
        newPenality             = penality     + car1.penality     + car2.penality -     vehicles[vt[i].first.first.first].penality -     vehicles[vt[i].second.first.first].penality;
        
        newObjective = newTravelledDistance + newVehiclesCost + newPenality;

        if(bestObjective - newObjective > EPS){
            bestV1      = vt[i].first.first.first;
            bestV2      = vt[i].second.first.first;
            bestCar1    = car1;
            bestCar2    = car2;
            improve     = true;
            bestObjective = newObjective;
        }
        
        // Inverte car 1

        swap(car2.route[vt[i].second.first.second], car2.route[vt[i].second.second]);
 
        car1.evaluateVehicleTW(data);
        car2.evaluateVehicleTW(data);

        newTravelledDistance    = justDistance + car1.justDistance + car2.justDistance - vehicles[vt[i].first.first.first].justDistance - vehicles[vt[i].second.first.first].justDistance;
        newPenality             = penality     + car1.penality     + car2.penality -     vehicles[vt[i].first.first.first].penality -     vehicles[vt[i].second.first.first].penality;
        
        newObjective = newTravelledDistance + newVehiclesCost + newPenality;

        if(bestObjective - newObjective > EPS){
            bestV1      = vt[i].first.first.first;
            bestV2      = vt[i].second.first.first;
            bestCar1    = car1;
            bestCar2    = car2;
            improve     = true;
            bestObjective = newObjective;
        }
    }

    if(improve){
        bestCar1.intraRVNDTW(data);
        bestCar2.intraRVNDTW(data);
        
        ride                    = ride -                    vehicles[bestV1].ride -                  vehicles[bestV2].ride                  + bestCar1.ride                  + bestCar2.ride;
        demand                  = demand -                  vehicles[bestV1].demand -                vehicles[bestV2].demand                + bestCar1.demand                + bestCar2.demand;
        penality                = penality -                vehicles[bestV1].penality -              vehicles[bestV2].penality              + bestCar1.penality              + bestCar2.penality;
        violationTw             = violationTw -             vehicles[bestV1].violationTw -           vehicles[bestV2].violationTw           + bestCar1.violationTw           + bestCar2.violationTw;
        justDistance            = justDistance -            vehicles[bestV1].justDistance -          vehicles[bestV2].justDistance          + bestCar1.justDistance          + bestCar2.justDistance;
        violationDemand         = violationDemand -         vehicles[bestV1].violationDemand -       vehicles[bestV2].violationDemand       + bestCar1.violationDemand       + bestCar2.violationDemand;
        violationBattery        = violationBattery -        vehicles[bestV1].violationBattery -      vehicles[bestV2].violationBattery      + bestCar1.violationBattery      + bestCar2.violationBattery;
        numberStationsVisited   = numberStationsVisited -   vehicles[bestV1].numberStationsVisited - vehicles[bestV2].numberStationsVisited + bestCar1.numberStationsVisited + bestCar2.numberStationsVisited;

        objective               = justDistance + vehicleCost + penality;

        swap(vehicles[bestV1], bestCar1);
        swap(vehicles[bestV2], bestCar2);
    }
}

void Vehicle::intraRVNDTW(Data &data){
    double bestObjective = justDistance + penality + vehicleCost, newObjective;
    int count = 2, i;
    vector<int> vR;

    for(i = 0; i <= count; i++){
        vR.push_back(i);
    }
    
    shuffle(vR.begin(), vR.end(), generator);
    count = 0;
    
    while(1){
        if (count >= (int)vR.size()){break;}
        
        if(vR[count] == 0){
            intraRealocationTW(data);

            newObjective = justDistance + penality + vehicleCost;

            if(bestObjective - newObjective > EPS){
                bestObjective = newObjective;
                count = 0;
                shuffle(vR.begin(), vR.end(), generator);
            }else{
                count++;
            }
         }else if (vR[count] == 1){
            intraShift2TW(data);
            
            newObjective = justDistance + penality + vehicleCost;

            if(bestObjective - newObjective > EPS){
                bestObjective = newObjective;
                count = 0;
                shuffle(vR.begin(), vR.end(), generator);
            }else{
                count++;
            }
        }else if (vR[count] == 2){
            intraSwapTW(data);
            
            newObjective = justDistance + penality + vehicleCost;

            if(bestObjective - newObjective > EPS){
                bestObjective = newObjective;
                count = 0;
                shuffle(vR.begin(), vR.end(), generator);
            }else{
                count++;
            }
        }
    }
}

void Vehicle::intraSwapTW(Data &data) {
    Vehicle inicialVehicle = *this;
    int i, j, bestI = -1, bestJ = -1;
    double bestObjective = justDistance + penality + vehicleCost, newObjective;
    vector < pair < int, int > > vt; 
    
    inicialVehicle = *this;
    bestI = -1, bestJ = -1;

    for (i = 1; i < (int)route.size()-1; i++) {
        for (j = i+1; j < (int)route.size()-1; j++) {
            vt.push_back(make_pair(i, j));
        }
    }

    for (i = 0; i < (int)vt.size(); i++) {
        if(vt[i].first != vt[i].second){
            
            swap(route[vt[i].first], route[vt[i].second]);
            
            evaluateVehicleTW(data);
            
            newObjective       = justDistance + penality + vehicleCost;
            
            if(bestObjective - newObjective > EPS){
                bestObjective = newObjective;
                bestI = vt[i].first;
                bestJ = vt[i].second;
            }

            //====================Volta os valores iniciais=============================
            *this = inicialVehicle;
        }
    }

    if(bestI != -1){
        swap(route[bestI], route[bestJ]);
        evaluateVehicleTW(data);
    }

}

void Vehicle::intraShift2TW(Data &data) {
    Vehicle inicialVehicle = *this;
    int i, j, bestI = -1, bestJ = -1, i2;
    double bestObjective = justDistance + penality + vehicleCost, newObjective;
    vector < pair < int, int > > vt; 
    
    inicialVehicle = *this;
    bestI = -1, bestJ = -1;

    for (i = 1; i < (int)route.size()-1; i++) {
        for (j = i+2; j < (int)route.size()-1; j++) {
            vt.push_back(make_pair(i, j));
        }
    }
    
    for (i = 0; i < (int)vt.size(); i++) {
        i2 = vt[i].first+1;
           
        //==============================Troca de lugares para recalculo TW ==================================
        route.insert(route.begin()+vt[i].second+1, route[vt[i].first]);
        route.insert(route.begin()+vt[i].second+2, route[i2]);
        route.erase(route.begin()+i2);
        route.erase(route.begin()+vt[i].first);
        
        evaluateVehicleTW(data);
        
        newObjective       = justDistance + penality + vehicleCost;
        
        if(bestObjective - newObjective > EPS){
            bestObjective = newObjective;
            bestI = vt[i].first;
            bestJ = vt[i].second;
        }

        //====================Volta os valores iniciais==================================================
        *this = inicialVehicle;
    }

    if(bestI != -1){
        route.insert(route.begin()+bestJ+1, route[bestI]);
        route.insert(route.begin()+bestJ+2, route[bestI+1]);
        route.erase(route.begin()+bestI+1);
        route.erase(route.begin()+bestI);
        evaluateVehicleTW(data);
    }
    
}

void Vehicle::intraRealocationTW(Data &data) {
    Vehicle inicialVehicle = *this;
    int i, j, bestI = -1, bestJ = -1;
    double bestObjective = justDistance + penality + vehicleCost, newObjective;
    vector < pair < int, int > > vt; 

    bestI = -1, bestJ = -1;
    for (i = 1; i < (int)route.size()-1; i++) {
        for (j = i+1; j < (int)route.size()-1; j++) {
            vt.push_back(make_pair(i, j));
        }
    }

    for (i = 0; i < (int)vt.size(); i++) {
        //==============================Troca de lugares para recalculo TW ==================================
        route.insert(route.begin()+vt[i].second+1, route[vt[i].first]);
        route.erase(route.begin()+vt[i].first);

        evaluateVehicleTW(data);

        newObjective       = justDistance + penality + vehicleCost;

        if(bestObjective - newObjective > EPS){
            bestObjective = newObjective;
            bestI = vt[i].first;
            bestJ = vt[i].second;
        }

        //====================Volta os valores iniciais==================================================
        *this = inicialVehicle;
    }

    if(bestI != -1){
        route.insert(route.begin()+bestJ+1, route[bestI]);
        route.erase(route.begin()+bestI);
        evaluateVehicleTW(data);
    }
}

void Solution::showSolutionTW() {
    int numVehicles = 0;
    for (int i = 0; i < (int)vehicles.size(); i++) {
        if((int)vehicles[i].route.size() < 3){
            continue;
        }
        numVehicles++;
        for (int j = 0; j < (int)vehicles[i].route.size(); j++) {
            if(vehicles[i].route[j].batteryStation){
                cout << vehicles[i].route[j].sigla << "(S) - ";
            }else{
                cout << vehicles[i].route[j].sigla << " - ";
            }
        }
        cout << endl;
        cout << "Demand: " << vehicles[i].demand << " | "; 
        cout << "   Capacity: " << vehicles[i].demandCapacity << " | ";// << endl;
        cout << "   VTW: " << vehicles[i].violationTw << " | " << "     VD: " << vehicles[i].violationDemand << " | " << "     VB: " << vehicles[i].violationBattery << " | ";
        cout << "   Ride: " << vehicles[i].ride << endl;
    }
    cout << "Just Distance: " << justDistance << endl;
    cout << "Objective Solution: " << objective << endl;
    cout << "Amount Vehicles: " << numVehicles << endl;
    amountVehicles = numVehicles;
}