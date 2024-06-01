# CRS
*This is the codes used for simulations in the paper: R. Liu and J. Pan, "CRS: A Privacy-Preserving Two-Layered Distributed Machine Learning Framework for IoV," in IEEE Internet of Things Journal, vol. 11, no. 1, pp. 1080-1095, 1 Jan.1, 2024, doi: 10.1109/JIOT.2023.3287799.* 

Only for academic research.

**Please properly cite the paper if you use the code.**

For the details, we refer to the papers.

The simulations are conducted in two scenarios: 1) an emulated scenario, where we can have full control over the vehicles and observe the impact of vehicle speed clearly. 2) A realistic scenario in Bologna, Italy, which is more complex and can evaluate the practicability of CRS in the real world. 

## A. Requirements

1. Requirements: ns-3.34. sumo-gui and NetAnim. 

2. Three help files: rui-vehicle-beta.h, rui-equation-cal.h and rui-equation-cal.cpp
   
rui-vehicle-beta.h contains some necessary settings and records some necessary variables. 

rui-equation-cal.h and rui-equation-cal.cpp are used for packet handling.

The above three files rui-vehicle-beta.h, rui-equation-cal.h and rui-equation-cal.cc should be declared in the corresponding wscript files in ns3 (to learn more, please read instructions provided by ns3).

3. aodv-routing-protocol.cc and ipv4-l3-protocol.cc in ns-3.34 should be replaced with the ones we provided.
   
In aodv-routing-protocol.cc, we modify the AODV routing protocol. We ask each router, i.e., each member vehicle, to perform the message encoding algorithm when receives a packet.

In ipv4-l3-protocol.cc, to control the actual packet loss rate, we drop additional packets with a constant drop rate (set by rui-vehicle-beta.h) in the IP layer. udp-header.h and udp-header.cc should also be replaced. 

## B. Emulated highway scenario

vanet-routing-Rui.cc is the main simulation code. It is built from vanet-routing-compare.cc provided by ns-3. 
This is the code that simulates the communications and tasks (i.e., encoding, masking, decoding, calculating and so on) in clusters.

Statistical results (packet loss rate, recovery rate, end-to-end delays, average end-to-end delays, masking time, encoding time, and handling time) will be printed and saved to a file named as “rui_statistic_x.csv”. 

## C. Realistic scenario in Bologna, Italy

Dataset used in the realistic scenario: https://github.com/DLR-TS/sumo-scenarios/tree/main/bologna/
L. Bieker, D. Krajzewicz, A. Morra, C. Michelacci, and F. Cartolano, “Traffic simulation for all: A real world traffic scenario from the city of Bologna,” in Modeling Mobility with Open Data. Springer, 2015, pp. 47–60.

To be specific, we use the Acosta data.
vanet-sumo-Rui-real_scenario_acosta.cc is the main simulation code. It is built from vanet-routing-compare.cc provided by ns-3.

You should manually set the node_list[20] and head_node in rui-vehicle-beta.h to locate a cluster you would like to observe. 

Statistical results (packet loss rate, recovery rate, end-to-end delays, average end-to-end delays, masking time, encoding time, and handling time) will be printed and saved to a file named as “rui_statistic_x.csv”. 

## C. Set up protocol

The main simulation code is Rui_setup.cc. It is built from wave-simple-80211p.cc provided by ns-3.

## D. Communication between a vehicle and an RSU

The communication overhead between a cluster head and an RSU in the emulated scenario is simulated with Rui_RSU_vehicle.cc. The program is modified from wave-simple-80211p.cc provided by ns-3.

The communication overhead between a cluster head and an RSU in the realistic scenario is simulated with Rui_RSU_vehicle_real.cc. The process can be observed with NetAnim.

## E. Communication between an RSU and the server

Rui_RSU_S.cc. 

## F. Evaluation with large w_i in the emulated scenario

The evaluation within a cluster is achieved with vanet-routing-Rui_instance_new.cc, which is the same with vanet-routing-Rui.cc but with different settings.

The communication overhead between a cluster head and an RSU is simulated with Rui_RSU_vehicle_large_R2.cc. 

The communication overhead between a server and an RSU is simulated with Rui_RSU_S_large_R2.cc. 
