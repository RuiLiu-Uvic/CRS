#ifndef RUIVEHICLEBETAH
#define RUIVEHICLEBETAH
#include <map>
#include "ns3/ipv4-address.h"
#include <random>

static int group_size = 20; //The total number of vehicles in a group.
static double vehicle_beta[20] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 0.98};
//used for network encoding
static double vehicle_obser[20] = {51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70};

static int node_list[20] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
static int head_node = 10; //index of the vehicle you set as the cluster head. 


extern std::map<int, int> node_ID_to_index;
//used as observation values/machine learning results

extern std::vector<double> mask_obser;
extern std::vector<std::vector<double>> mask_obser_instance;

static int num_entries = 2000;

extern std::map<ns3::Ipv4Address, int> address_to_id; //to map IP address to vehicle IDs 


extern std::map<int, int> stat_network_coding_time; //the time used for network_coding part for each router/node in the whole process 


static std::random_device rd;
static std::mt19937 gen(rd());
static std::uniform_int_distribution<> dis(0, 100); //random numbers
static double error_rate = 0.01;//0.03; //the constant drop rate
// In the emulated scenario, the constant drop rate is set to 1% and 2%. In the realistic scenario, we further set the constant drop rate to 0%, 1%, 2% and 3%.


#endif
