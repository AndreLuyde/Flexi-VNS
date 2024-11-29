#ifndef SOLUTION_H_
#define SOLUTION_H_

#include "../util/util.h"
#include <cstdio>
#include <cstring>

using namespace std;

class Request {
    public:
        int     id;
        double  twA;
        double  twB;
        double  coordX;
        double  coordY;
        double  demand;
        double  serviceTime = 0;
        double  waitTimeStation;
        bool    batteryStation = false;
        double  stationCost;
        double  violationDemand;
        double  rechargeRate;
        char    sigla[10];
        double  distance;

        Request(){}
        Request(int _id, double _twA, double _twB, double _coordX, double _coordY, double _demand, double _serviceTime, bool _batteryStation, double _stationCost) 
        : id(_id), twA(_twA), twB(_twB), coordX(_coordX), coordY(_coordY), demand(_demand), serviceTime(_serviceTime), batteryStation(_batteryStation), stationCost(_stationCost) {}

        Request(int _id, double _twA, double _twB, double _coordX, double _coordY, double _demand, double _serviceTime, double _waitTimeStation, bool _batteryStation) 
        : id(_id), twA(_twA), twB(_twB), coordX(_coordX), coordY(_coordY), demand(_demand), serviceTime(_serviceTime), waitTimeStation(_waitTimeStation), batteryStation(_batteryStation) {}
};

class Evaluation{
    public:
        bool    batteryStation = false;
        double  violation = 0;
        double  beginToStation = 0;
        double  stationToEnd = 0;
        int     idBeginToStation = -1;
        int     idStationToEnd = -1;

        double  distance = 0;
};

class Data {

    public:
        int                             numTotalPoints;
        int                             numRequests;
        int                             numBatteryStations;
        int                             numVehicles;
        int                             qtdVehicle;
        int                             copyDepot = 0;
        double                          betaTw = 100;
        double                          betaDemand = 100;
        double                          betaBattery = 1598;
        double                          betaTour = 2600;
        double                          demandCapacity;
        double                          batteryCapacity;
        double                          rateConsumption = 1;
        double                          stationCost = 0;
        double                          maxTour;
        double                          averageVelocity;
        double                          milhasMinuto = 0.66666667;
        double                          customerService = 30;
        double                          stationService = 15;

        vector < int >                  checked;
        vector < int >                  usedStations;
        vector < Request >              requests;
        vector < Request >              orderRequests;
        vector < Request >              batteryStations;
        vector < vector < double > >    distances;
        vector < vector < Request > >    orderNodes;
        vector < vector < double > >    realDistances;
        vector < pair < pair < int, int >, double > >           closerStation;  // qual a estação mais próxima a cada cidade
        vector < pair < pair < double, double >, pair < pair < double, double >, int > > >  dataVehicles;
        Evaluation ***eval;
        
        Data() {}

        Data(string file, string problem){
            ifstream fin(file.c_str());
            if(!fin){
                clog << "Deu ruim!" << endl;
                exit(0);
            }

            string  twA;
            string  twB;
            string  coordX;
            string  coordY;
            string  serviceTime;
            double  demand;
            double  somaDemand = 0;
            double  biggest = 0;
            double  rechargeRate;
            int     contClient = 0;
            int     contStation = 0;
            vector < Request >              requests2;
            
            string word1, word2, clientes, stations, type, stringId;

            if(problem == "BSS"){
                //Head
                fin >> word1 >> numRequests;
                fin >> word1 >> demandCapacity;

                // //Data
                fin >> word1 >> word1 >> word1 >> word1;
                fin >> stringId >> coordX >> coordY >> demand;

                int startI;
                startI = 2;
                requests.push_back(Request(0, 0, 10000, stod(coordX), stod(coordY), 0.0, 0.0, false, 0));
                stringId.insert(0, "D");
                strcpy(requests.back().sigla, stringId.c_str());
                requests.push_back(Request(1, 0, 10000, stod(coordX), stod(coordY), 0.0, 0.0, true, stationCost));
                stringId = "S0";
                strcpy(requests.back().sigla, stringId.c_str());
                
                for(int i = startI; i <= numRequests; i++){
                    fin >> stringId >> coordX >> coordY >> demand;
                    somaDemand += demand;
                    requests.push_back(Request(i, 0, 10000, stod(coordX), stod(coordY), demand, 0.0, true, stationCost));
                    stringId.insert(0, "S");
                    strcpy(requests.back().sigla, stringId.c_str());
                }

                for(int i = startI; i <= numRequests; i++){
                    requests.push_back(Request(i+numRequests-1, 0, 10000, requests[i].coordX, requests[i].coordY, requests[i].demand, 0.0, false, 0.0));
                    stringId = "C"+to_string(i);
                    strcpy(requests.back().sigla, stringId.c_str());
                    requests[i].demand = 0;
                }
                requests.push_back(requests[0]);

                numVehicles = ceil(somaDemand/demandCapacity) + 1;
                
                copyDepot = requests.size() - 1;            
                requests[copyDepot].id = copyDepot;
                strcpy(requests[copyDepot].sigla, "D2");
                
                requests[0].coordX = 1;
                requests[0].coordY = -1;
                requests[1].coordX = 1;
                requests[1].coordY = -1;
                requests[copyDepot].coordX = 1;
                requests[copyDepot].coordY = -1;
                numTotalPoints      = numRequests + numRequests + 1;
                numBatteryStations  = numRequests +1;

                usedStations.assign(numBatteryStations, 0);
                
                distances.resize(numTotalPoints);
                for(int i = 0; i < (int)distances.size(); i++){
                    distances[i].resize(numTotalPoints);
                }
                
                for(int i = 0; i < (int)distances.size(); i++){
                    for(int j = i + 1; j < (int)distances[i].size(); j++){
                        distances[i][j] = distances[j][i] = sqrt(pow((requests[i].coordX - requests[j].coordX), 2.0) + pow((requests[i].coordY - requests[j].coordY), 2.0));
                        if(distances[i][j] > biggest){
                            biggest = distances[i][j];
                        }
                    }
                    distances[i][i] = 0;
                }
                batteryCapacity = round(biggest * 1.2);
                for(int i = 1; i < numBatteryStations; i++){
                    requests[i].stationCost = round(batteryCapacity/2);
                }

                closerStation.resize(numTotalPoints);
                for(int i = 1; i < numTotalPoints; i++){
                    closerStation[i].first.first =  requests[i].id;
                    closerStation[i].first.second =  1;
                    closerStation[i].second =  distances[i][1];
                }

                for(int i = 0; i < (int)closerStation.size(); i++){
                    for(int j = 1; j < numTotalPoints; j++){
                        if(requests[j].batteryStation){
                            if(distances[closerStation[i].first.first][requests[j].id] < closerStation[i].second){
                                closerStation[i].second         = distances[closerStation[i].first.first][requests[j].id];
                                closerStation[i].first.second   = requests[j].id;
                            }
                        }
                    }
                }
            }else if(problem == "EVRP"){
                //Head
                fin >> word1 >> numRequests;
                fin >> word1 >> numBatteryStations;
                
                // //Data
                fin >> word1 >> word1 >> word1 >> word1;
        
                int cont = 0;
                fin >> stringId >> type >> coordX >> coordY;
                requests.push_back(Request(0, 0, 10000, stod(coordX), stod(coordY), 0.0, 0.0, false, stationCost));
                strcpy(requests.back().sigla, stringId.c_str());
                for(int i = 1; i < numRequests; i++){
                    fin >> stringId >> type >> coordX >> coordY;
                    if(type == "f" || type == "F"){
                        requests.push_back(Request(i, 0, 10000, stod(coordX), stod(coordY), 0.0, 15, true, stationCost));
                    }else{
                        requests.push_back(Request(i, 0, 10000, stod(coordX), stod(coordY), 0.0, 30, false, stationCost));
                        cont++;
                    }
                    strcpy(requests.back().sigla, stringId.c_str());
                }
                requests.push_back(requests[0]);
                copyDepot = requests.size() - 1;            
                requests[copyDepot].id = copyDepot;
                strcpy(requests[copyDepot].sigla, "D2");

                fin >> word1 >> word1 >> word1 >> word1 >> word1 >> word1;
                batteryCapacity = stod(word1.substr(1,5));
                fin >> word1 >> word1 >> word1 >> word1 >> word1;
                rateConsumption = stod(word1.substr(1,3));
                fin >> word1 >> word1 >> word1;
                maxTour = stod(word1.substr(1,4)) * 60;
                fin >> word1 >> word1 >> word1 >> word1;
                averageVelocity = stod(word1.substr(1,4));
                fin >> word1 >> word1 >> word1;
                numVehicles = stod(word1.substr(1,4));
                
                batteryCapacity = batteryCapacity / rateConsumption;
                maxTour = maxTour - 15;

                numTotalPoints = requests.size();
                usedStations.assign(numBatteryStations, 0);

                orderNodes.resize(numTotalPoints);
                distances.resize(numTotalPoints);
                for(int i = 0; i < (int)distances.size(); i++){
                    distances[i].resize(numTotalPoints);
                    orderNodes[i].resize(numTotalPoints);
                }
                
                for(int i = 0; i < (int)distances.size(); i++){
                    for(int j =0; j < (int)distances[i].size(); j++){
                        
                        double lat1 = requests[i].coordY;
                        double lat2 = requests[j].coordY;
                        double lon1 = requests[i].coordX;
                        double lon2 = requests[j].coordX;
                        double radiusOfEarth = 4182.44949; // miles, 6371km; 
                        double dLat = degreesToRadians(lat2-lat1); 
                        double dLon = degreesToRadians(lon2-lon1); 
                        double a = sin(dLat/2) * sin(dLat/2) + cos(degreesToRadians(lat1)) * cos(degreesToRadians(lat2)) * sin(dLon/2) * sin(dLon/2); 
                        double c = 2 * atan2(sqrt(a), sqrt(1-a)); 
                        double distance = radiusOfEarth * c;
                        distances[i][j] = distance;
                        orderNodes[i][j] = requests[j];
                        orderNodes[i][j].distance = distance;

                        if(distances[i][j] > biggest){
                            biggest = distances[i][j];
                        }
                    }
                    distances[i][i] = 0;
                    orderNodes[i][i].distance = 0;
                }

                closerStation.resize((int)requests.size());
                for(int i = 0; i < (int)closerStation.size(); i++){
                    closerStation[i].first.first =  requests[i].id;
                    closerStation[i].first.second =  1;
                    closerStation[i].second =  distances[i][1];
                }

                for(int i = 0; i < (int)closerStation.size(); i++){
                    for(int j = 1; j <= numBatteryStations; j++){
                        if(requests[j].batteryStation){
                            if(distances[closerStation[i].first.first][requests[j].id] < closerStation[i].second){
                                closerStation[i].second         = distances[closerStation[i].first.first][requests[j].id];
                                closerStation[i].first.second   = requests[j].id;
                            }
                        }
                    }
                }
            }else{
                //Head
                fin >> word1 >> word1 >> word1 >> word1 >> word1 >> word1 >> word1 >> word1;
                int k = 0;
                word1 = "recharging";

                //Data
                fin >> stringId >> type >> coordX >> coordY >> word2 >> twA >> twB >> serviceTime;
                while(stringId != "Q"){
                    word1 = stringId;
                    if(stringId.front() == 'S'){
                        requests.push_back(Request(k, stod(twA), stod(twB), stod(coordX), stod(coordY), stod(word2), stod(serviceTime), 0.0, true));
                        strcpy(requests.back().sigla, stringId.c_str());
                        contStation++;
                    }else{
                        requests.push_back(Request(k, stod(twA), stod(twB), stod(coordX), stod(coordY), stod(word2), stod(serviceTime), 0.0, false));
                        strcpy(requests.back().sigla, stringId.c_str());
                        contClient++;
                    }
                    k++;
                    fin >> stringId >> type >> coordX >> coordY >> word2 >> twA >> twB >> serviceTime;
                }

                batteryCapacity = stod(twA.substr(1,5));
                fin >> word1 >> word1 >> word1;
                demandCapacity = stod(word1.substr(1,4));
                fin >> word1 >> word1 >> word1 >> word1 >> word1;
                rateConsumption = stod(word1.substr(1,3));
                fin >> word1 >> word1 >> word1 >> word1 >> word1;
                rechargeRate = stod(word1.substr(1,4));

                numTotalPoints      = contStation + contClient;
                numRequests         = contClient;
                numBatteryStations  = contStation;

                for(int i=0; i < (int)requests.size(); i++){
                    if(requests[i].batteryStation){
                        requests[i].rechargeRate = rechargeRate;
                    }else{
                        requests[i].rechargeRate = 0;
                    }
                }

                //distância euclidiana
                distances.resize(numTotalPoints);
                realDistances.resize(numTotalPoints);
                for(int i = 0; i < (int)distances.size(); i++){
                    distances[i].resize(numTotalPoints);
                    realDistances[i].resize(numTotalPoints);
                }

                for(int i = 0; i < (int)distances.size(); i++){
                    for(int j = i + 1; j < (int)distances[i].size(); j++){
                        distances[i][j] = distances[j][i] = sqrt(pow((requests[i].coordX - requests[j].coordX), 2.0) + pow((requests[i].coordY - requests[j].coordY), 2.0));
                    }
                    distances[i][i] = 0;
                }
                
                for(int i = 0; i < (int)realDistances.size(); i++){
                    for(int j = i + 1; j < (int)realDistances[i].size(); j++){
                        normal_distribution<double> distribution (distances[i][j],2.0);
                        realDistances[i][j] = distribution(generator);
                    }
                    realDistances[i][i] = 0;
                }
                
                closerStation.resize(numTotalPoints);
                for(int i = 1; i < numTotalPoints; i++){
                    closerStation[i].first.first =  requests[i].id;
                    closerStation[i].first.second =  1;
                    closerStation[i].second =  distances[i][1];
                }

                for(int i = 0; i < (int)closerStation.size(); i++){
                    for(int j = 1; j < numTotalPoints; j++){
                        if(requests[j].batteryStation){
                            if(distances[closerStation[i].first.first][requests[j].id] < closerStation[i].second){
                                closerStation[i].second         = distances[closerStation[i].first.first][requests[j].id];
                                closerStation[i].first.second   = requests[j].id;
                            }
                        }
                    }
                }
            }
        }

        double degreesToRadians(double degrees){
            return degrees * M_PI / 180;
        }

        long double toRadians(const long double degree){
            long double one_deg = (M_PI) / 180;
            return (one_deg * degree);
        }

        long double calculateDistance(long double lat1, long double long1, long double lat2, long double long2){
            lat1 = toRadians(lat1);
            long1 = toRadians(long1);
            lat2 = toRadians(lat2);
            long2 = toRadians(long2);
            
            // Haversine Formula
            long double dlong = long2 - long1;
            long double dlat = lat2 - lat1;
        
            long double ans = pow(sin(dlat / 2), 2) +
                                cos(lat1) * cos(lat2) *
                                pow(sin(dlong / 2), 2);
        
            ans = 2 * atan2(sqrt(ans), sqrt(1-ans)); 
        
            long double R = 4182.44949;
            
            ans = ans * R;
        
            return ans;
        }
};

class Vehicle{

    public:

        void intraRVND(                         Data &data, int v);
        void intraSwap(                         Data &data, int v);
        void showRoute();
        void intraShift2(                       Data &data, int v);
        void intraRealocation(                  Data &data, int v);

        void intraRVNDBSS(                         Data &data, int v);
        void intraSwapBSS(                         Data &data, int v);
        void intraShift2BSS(                       Data &data, int v);
        void intraRealocationBSS(                  Data &data, int v);

        void intraRVNDTW(                         Data &data);
        void intraSwapTW(                         Data &data);
        void showRouteTW();
        void intraShift2TW(                       Data &data);
        void evaluateVehicleTW(                   Data &data);
        void intraRealocationTW(                  Data &data);
        

        int                 numberStationsVisited   = 0;
        int                 numCustomers = 0;
        int                 numStations = 0;

        double              ride                    = 0;
        double              demand                  = 0;
        double              penality                = 0;
        double              objective               = 0;
        double              timeCharging            = 0;
        double              batteryKm;
        double              batteryUsed             = 0;
        double              violationTw             = 0;
        double              stationCost             = 0;
        double              vehicleCost;
        double              distance                = 0;
        double              justDistance            = 0;
        double              demandCapacity;
        double              violationDemand         = 0;
        double              violationBattery        = 0;
        double              violationTimeTour       = 0;
        double              batteryCapacity;
        double              rateConsumption;
        
        vector < Request >  route;
        vector < pair < int, double > > chargingTimeStation;
};

class Solution{
    public:
        void VNS(                           Data &data, int vnsMax);
        void VNS(                           Data &data, int vnsMax, double timeMax);
        void shake(                         Data &data, int intensity, int &lastShake);
        void shakeSwap(                     Data &data);
        void buildData(                     Data &data);
        void interRVND(                     Data &data, int lastShake);
        void interSwap(                     Data &data);
        void interSwap2(                    Data &data);
        void addStation(                    Data &data);
        void updateData(                    Data &data, int k);
        void interSwap2x1(                  Data &data);
        void removeStation(                 Data &data);
        void shakeRelocation(               Data &data);
        void interRelocation(               Data &data);
        void interRelocation2(              Data &data);
        void shakeRemoveStations(           Data &data);
        void generateGreedySolution(        Data &data);
        void inicialSolutionAvailable(      Data &data);
        void showSolution();

        void VNSBSS(                           Data &data, int vnsMax, double timeMax);
        void shakeBSS(                         Data &data, int intensity, int &lastShake);
        void shakeSwapBSS(                     Data &data);
        void buildDataBSS(                     Data &data);
        void interRVNDBSS(                     Data &data, int lastShake);
        void interSwapBSS(                     Data &data);
        void interSwap2BSS(                    Data &data);
        void addStationBSS(                    Data &data);
        void updateDataBSS(                    Data &data, int k);
        void interSwap2x1BSS(                  Data &data);
        void removeStationBSS(                 Data &data);
        void shakeRelocationBSS(               Data &data);
        void interRelocationBSS(               Data &data);
        void interRelocation2BSS(              Data &data);
        void shakeRemoveStationsBSS(           Data &data);
        void generateGreedySolutionBSS(        Data &data);
        void inicialSolutionAvailableBSS(      Data &data);
        void showSolutionBSS();
        
        void VNSTW(                           Data &data, int vnsMax, double timeMax);
        void shakeTW(                         Data &data, int intensity, int &lastShake);
        void shakeSwapTW(                     Data &data);
        void interRVNDTW(                     Data &data, int lastShake);
        void interSwapTW(                     Data &data);
        void interSwap2TW(                    Data &data);
        void addStationTW(                    Data &data);
        void interSwap2x1TW(                  Data &data);
        void shakeStationTW(                  Data &data);
        void removeStationTW(                 Data &data);
        void UpdateSolutionTW(                Data &data);
        void shakeRelocationTW(               Data &data);
        void interRelocationTW(               Data &data);
        void evaluateSolutionTW(              Data &data);
        void interRelocation2TW(              Data &data);
        void shakeRemoveStationTW(            Data &data);
        void shakeReduceVehicleTW(            Data &data);
        void updateSolutionDataTW(            Data &data, int vnsMax);
        void generateGreedySolutionTW(        Data &data);
        void showSolutionTW();

        int                 amountVehicles          = 0;
        int                 numberStationsVisited   = 0;
        double              ride                    = 0;
        double              demand                  = 0;
        double              penality                = 0;
        double              objective               = 0;
        double              violationTw             = 0;
        double              vehicleCost             = 0;
        double              stationCost             = 0;
        double              distance                = 0;
        double              justDistance            = 0;
        double              violationDemand         = 0;
        double              violationBattery        = 0;
        double              violationTimeTour       = 0;
        double              travelledDistance       = 0;
        
        vector < Vehicle >  vehicles;
};

#endif /* SOLUTION_H_ */
