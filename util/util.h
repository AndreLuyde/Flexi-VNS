#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <climits>
#include <vector>
#include <cstring>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <random>

using namespace std;

#define INF 987654321
#define EPS 1.0e-5 
// #define PI 3.14159265358979323846

static default_random_engine generator((int) chrono::system_clock::now().time_since_epoch().count());
//static default_random_engine generator(11235813);
const int SHIFT_CHAR = -SCHAR_MIN;
#endif
