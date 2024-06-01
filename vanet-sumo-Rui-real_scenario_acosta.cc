//This code is build from vanet-routing-compare.cc provided by ns-3. 
// We keep some original notes for easy-understanding
// The m_nNodes variable is set according to the data used. For example, 
// when there are 1000 vehicles appear in the traffic flow data you choose, then set m_nNodes=1000 
// You should manually set the node_list[20] and head_node in rui-vehicle-beta.h to locate a cluster you would like to observe. 


#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/itu-r-1411-los-propagation-loss-model.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/config-store-module.h"
#include "ns3/integer.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/netanim-module.h" 
#include "ns3/point-to-point-module.h"

#include <list>
#include <map>
#include "ns3/rui-vehicle-beta.h"
#include "ns3/rui-equation-cal.h"


using namespace ns3;
using namespace dsr;
using namespace std;

extern map<Ipv4Address, int> address_to_id; //Rui: to map IP adress with user ID
extern vector<double> mask_obser; //Rui: To simulate the observation value/machine learning result for all users
extern map<int, int> node_ID_to_index; //Rui: to map the index of nodes in the mobility node container and routing node container



NS_LOG_COMPONENT_DEFINE ("vanet-sumo-rui-real-scenario");

/**
 * \brief The RoutingHelper class generates routing data between
 * nodes (vehicles) 
 * A routing protocol is configured, and all nodes attempt to send
 * (i.eadd. route) small packets to another node, which acts as
 * data sinks.  Rui: We only have one sink in our work. 
 */

class RoutingHelper : public Object
{
public:
  /**
   * \brief Get class TypeId
   * \return the TypeId for the class
   */
  static TypeId GetTypeId (void);

  /**
   * \brief Constructor
   * \return none
   */
  RoutingHelper ();

  /**
   * \brief Destructor
   * \return none
   */
  virtual ~RoutingHelper ();

  /**
   * \brief Installs routing functionality on nodes and their
   * devices and interfaces.
   * \param c node container
   * \param d net device container
   * \param i IPv4 interface container
   * \param totalTime the total time that nodes should attempt to
   * route data
   * \param protocol the routing protocol (1=OLSR;2=AODV;3=DSDV;4=DSR)
   * \param nSinks the number of nodes which will act as data sinks
   * \param routingTables dump routing tables at t=5 seconds (0=no;1=yes)
   * \return none
   */
  void Install (NodeContainer & c,
                NetDeviceContainer & d,
                Ipv4InterfaceContainer & i,
                double totalTime,
                int protocol,
                uint32_t nSinks,
                int routingTables);

  
  DataManagementHelper m_data_mangement_helper; //Rui: to mange the received data
  DataRecoveryHelper m_data_recovery_helper; //Rui: to recover the lost packets

  /**
   * \brief Enable/disable logging
   * \param log non-zero to enable logging
   * \return none
   */
  void SetLogging (int log);

  list<Ipv4Address> GetReceiveList (); //Rui: received from which members

  list<string> GetReceiveContent (); //Rui: the received values, to calculate the avearage 

  map<int, Time> GetEndDelay (); //Rui: for statistic: end-to-end delay

  map<int, int> GetMaskingTime (); //Rui: for statistic: masking time

  void AddRecoveryTime(int time_duration); //Rui: for statistic: recovery time

  int GetRecoveryTime();//Rui: for statistic: recovery time



private:
  /**
   * \brief Sets up the protocol protocol on the nodes
   * \param c node container
   * \return none
   */
  void SetupRoutingProtocol (NodeContainer & c);

  /**
   * \brief Assigns IPv4 addresses to net devices and their interfaces
   * \param d net device container
   * \param adhocTxInterfaces IPv4 interface container
   * \return none
   */
  void AssignIpAddresses (NodeContainer & c, NetDeviceContainer & d,
                          Ipv4InterfaceContainer & adhocTxInterfaces);

  



  /**
   * \brief Sets up routing messages on the nodes and their interfaces
   * \param c node container
   * \param adhocTxInterfaces IPv4 interface container
   * \return none
   */
  void SetupRoutingMessages (NodeContainer & c,
                             Ipv4InterfaceContainer & adhocTxInterfaces);

  /**
   * \brief Sets up a routing packet for tranmission
   * \param addr destination address
   * \param node source node
   * \return Socket to be used for sending/receiving a routed data packet
   */
  Ptr<Socket> SetupRoutingPacketReceive (Ipv4Address addr, Ptr<Node> node);

  /**
   * \brief Process a received routing packet
   * \param socket the receiving socket
   * \return none
   */
  void ReceiveRoutingPacket (Ptr<Socket> socket);

  void SendOnePacket (Ptr<Socket> socket);//Rui: Send out a packet

  double m_TotalSimTime;        ///< seconds
  uint32_t m_protocol;       ///< routing protocol; 0=NONE, 1=OLSR, 2=AODV, 3=DSDV, 4=DSR
  uint32_t m_port;           ///< port
  uint32_t m_nSinks;              ///< number of sink nodes (< all nodes)
  int m_routingTables;      ///< dump routing table (at t=5 sec).  0=No, 1=Yes
  std::string m_protocolName; ///< protocol name
  int m_log; ///< log
  list<Ipv4Address> receive_list;
  list<string> receive_content; 
  map<int, Time> end_to_end_delay; 
  map<int, int> stat_masking_time; 
  int stat_recovery_and_unmasking_time; 
  
};

NS_OBJECT_ENSURE_REGISTERED (RoutingHelper);

TypeId
RoutingHelper::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RoutingHelper")
    .SetParent<Object> ()
    .AddConstructor<RoutingHelper> ();
  return tid;
}

RoutingHelper::RoutingHelper ()
  : m_data_mangement_helper (group_size-1),
    m_data_recovery_helper (),
    m_TotalSimTime (300.01),
    m_protocol (0),
    m_port (9),
    m_nSinks (0),
    m_routingTables (1),//Rui: We need this for debugging. can be set as 0 if we don't need to debug
    m_log (0),
    receive_list (),
    receive_content (),
    end_to_end_delay (),
    stat_masking_time (),
    stat_recovery_and_unmasking_time (0)
    
{
}

RoutingHelper::~RoutingHelper ()
{
}

void
RoutingHelper::Install (NodeContainer & c,
                        NetDeviceContainer & d,
                        Ipv4InterfaceContainer & i,
                        double totalTime,
                        int protocol,
                        uint32_t nSinks,
                        int routingTables)
{
  m_TotalSimTime = totalTime;
  m_protocol = protocol;
  m_nSinks = nSinks;
  m_routingTables = routingTables;
  //NodeContainer sub_node_contaniner;

  SetupRoutingProtocol (c);
  NS_LOG_INFO("Install IP now: ");
  AssignIpAddresses (c, d, i); 
  NS_LOG_INFO("Install RoutingMessages now: ");
  SetupRoutingMessages (c, i);
}

//Sets up a routing packet for tranmission
Ptr<Socket>
RoutingHelper::SetupRoutingPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, m_port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&RoutingHelper::ReceiveRoutingPacket, this));

  return sink;
}

void
RoutingHelper::SetupRoutingProtocol (NodeContainer & c)
{
  AodvHelper aodv;
  OlsrHelper olsr;
  DsdvHelper dsdv;
  DsrHelper dsr;
  DsrMainHelper dsrMain;
  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;

  Time rtt = Time (38.0);
  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> rtw = ascii.CreateFileStream ("routing_table_sumo_Rui");

  switch (m_protocol)
    {
    case 0:
      m_protocolName = "NONE";
      break;
    case 1:
      if (m_routingTables != 0)
        {
          olsr.PrintRoutingTableAllAt (rtt, rtw);
        }
      list.Add (olsr, 100);
      m_protocolName = "OLSR";
      break;
    case 2: //Rui: this is what we used for simulation
      if (m_routingTables != 0)
        {
          aodv.PrintRoutingTableAllAt (rtt, rtw);
        }
      list.Add (aodv, 100);
      m_protocolName = "AODV";
      break;
    case 3:
      if (m_routingTables != 0)
        {
          dsdv.PrintRoutingTableAllAt (rtt, rtw);
        }
      list.Add (dsdv, 100);
      m_protocolName = "DSDV";
      break;
    case 4:
      // setup is later
      m_protocolName = "DSR";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << m_protocol);
      break;
    }

  if (m_protocol < 4)
    {
      internet.SetRoutingHelper (list);//Rui: setup routing Ipv4ListRoutingHelper list
      internet.Install (c);
    }
  else if (m_protocol == 4)
    {
      internet.Install (c);
      dsrMain.Install (dsr, c);
    }

  if (m_log != 0)
    {
      NS_LOG_UNCOND ("Routing Setup for " << m_protocolName);
    }






}

void
RoutingHelper::AssignIpAddresses (NodeContainer & c, NetDeviceContainer & d,
                                  Ipv4InterfaceContainer & adhocTxInterfaces)
//we made little adjustment here becasue of difference in versions
{
  NS_LOG_UNCOND ("Assigning IP addresses");
  Ipv4AddressHelper addressAdhoc;
  NS_LOG_UNCOND (c.GetN());
  NS_LOG_UNCOND (d.GetN());
  // we may have a lot of nodes, and want them all
  // in same subnet, to support broadcast
  addressAdhoc.SetBase ("10.1.0.0", "255.255.0.0");
  adhocTxInterfaces = addressAdhoc.Assign (d);

  NS_LOG_UNCOND ("Finish IP add assigning.");
  for (int i = 0; i < c.GetN(); ++i)
  {
    //Ipv4Address address_ipv4;
    address_to_id.insert(make_pair(adhocTxInterfaces.GetAddress(i),c.Get(i)->GetId()));//Ipv4Address(d.Get(i)->GetAddress())
  }
  NS_LOG_UNCOND ("Finish IP add mapping.");
}

//Rui: to mask a data. The masks can be chosen with a more complex method.
int
Masking (int raw_obser, int nodeID)
{

  NS_LOG_INFO("raw_obser: "<<raw_obser);//cout << raw_obser ;
  double mask = 0;
  for (int j=nodeID+1; j<group_size; j++)
  {
    mask = ( j + nodeID ) % 10; 
    raw_obser = raw_obser + mask;
    //NS_LOG_DEBUG ("+" << mask);
  }
  for (int j=0; j<nodeID; j++)
  {
    mask = ( j + nodeID ) % 10; 
    raw_obser = raw_obser - mask;
    //NS_LOG_DEBUG ( "-" << mask);
  }
  // node_observ = node_observ % 100;
  //cout << endl;
  return raw_obser; //masked value
}


void
RoutingHelper::SendOnePacket (Ptr<Socket> socket)
{
  int nodeID = socket->GetNode ()->GetId ();
  int node_index = node_ID_to_index.at(nodeID); 
  int node_observ = vehicle_obser[node_index]; //observation value


  std::ostringstream data; data << node_observ << '\0';
  uint32_t dataSize = data.str().length()+1;

  Ptr<Packet> packet = Create<Packet> ((uint8_t*) data.str().c_str(), dataSize);

  TimestampTag timestamp;
  timestamp.SetTimestamp (Simulator::Now ());
  packet->AddByteTag (timestamp);
  
  NS_LOG_INFO("t = " << Simulator::Now() << " " << nodeID << " Send a message: " << node_observ);

  socket->Send (packet);

}

//Rui: for warm up. To send out a regular packet RP|Hello
void
SendRegularPacket (Ptr<Socket> socket)
{
  std::ostringstream data; data << "RP|Hello" << '\0';
  uint32_t dataSize = data.str().length()+1;

  Ptr<Packet> packet = Create<Packet> ((uint8_t*) data.str().c_str(), dataSize);
  socket->Send (packet);
  //NS_LOG_ISend a messageNFO(socket << "Send a regular message");
}

void
SendRegularPacket2 (Ptr<Socket> socket, int head, uint32_t member) //from center to members
{
  std::ostringstream data; data << "RP|Hello" << '\0';
  uint32_t dataSize = data.str().length()+1;

  Ptr<Packet> packet = Create<Packet> ((uint8_t*) data.str().c_str(), dataSize);
  socket->Send (packet);
  cout<< socket << "Send a regular message:";
  packet ->Print (cout);
  cout<<" from "<<head<<" to "<<member<<endl;
}

void
RoutingHelper::SetupRoutingMessages (NodeContainer & c,
                                     Ipv4InterfaceContainer & adhocTxInterfaces)
{
  // Setup routing transmissions

  m_nSinks = 1; //Rui: only 0 is the sink
  
  for (int j =0; j< group_size; j++)
  {
    node_ID_to_index.insert(make_pair(node_list[j],j));
  }
  int i = head_node; 
   
  
  int index_head = node_ID_to_index.at(i);
  if (m_protocol != 0)
  {
    Ptr<Socket> sink = SetupRoutingPacketReceive (adhocTxInterfaces.GetAddress (index_head), c.Get (index_head));
  }

  cout << "Maksing process Start" << endl;
  for (int node_j = 0 ;node_j < group_size; node_j++)
  {
     auto begin = chrono::high_resolution_clock::now();
     mask_obser.push_back (Masking (vehicle_obser[node_j], node_j));//head
     auto end = chrono::high_resolution_clock::now();
     stat_masking_time[node_j] = stat_masking_time[node_j]+chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count();
  }
 

  cout<< "LOG: send out the messages from all nodes:" <<endl;
  
  double schedule_clock = 15; 
  double schedule_clock_text_arp = 10; 

  for (uint32_t senderNode = 0; senderNode < group_size; senderNode ++) //RuiTest
  {
    //senderNode = node_list[node_index];
    if (senderNode != index_head) //not the receiver
    {

      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      Ptr<Socket> source = Socket::CreateSocket (c.Get (senderNode), tid);
      source->Bind(InetSocketAddress (adhocTxInterfaces.GetAddress (senderNode), m_port));
      source->Connect(InetSocketAddress (adhocTxInterfaces.GetAddress (index_head), m_port));
      Simulator::Schedule(Seconds(schedule_clock_text_arp), &SendRegularPacket, source); 
      void (RoutingHelper::*fp)(Ptr<Socket> socket) = &RoutingHelper::SendOnePacket;
      Simulator::Schedule(Seconds(schedule_clock), fp, this, source);
      schedule_clock=schedule_clock+0.1;
      schedule_clock_text_arp=schedule_clock_text_arp+0.15;
    }
  }  
}

static std::string PrintReceivedRoutingPacket (Ptr<Socket> socket, Address srcAddress, list<Ipv4Address>& receive_list)
{
  std::ostringstream oss;

  oss << "t = " << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (InetSocketAddress::IsMatchingType (srcAddress))
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (srcAddress);
      oss << " received one packet from " << addr.GetIpv4 ();
      receive_list.push_back (addr.GetIpv4 ());
    }  
  else
    {
      oss << " received one packet!";
    }
  return oss.str ();
}


void
RoutingHelper::ReceiveRoutingPacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address srcAddress;
  while ((packet = socket->RecvFrom (srcAddress))) // Rui: receive packet, this is the one already removed all the header by other layers
    {
      Time receive_time = Simulator::Now ();//Rui: Receive time
      Ptr<Packet> pkt_nc = packet->Copy();
      uint8_t *buffer = new uint8_t[pkt_nc->GetSize ()];
      uint16_t size = pkt_nc->CopyData(buffer, pkt_nc->GetSize ());
      string s = string(buffer, buffer+pkt_nc->GetSize());

      //Rui: for regular messages
      vector<string> ss = split(s, "|");
      if (ss[0]=="RP")
      {
        cout << "REGULAR" << PrintReceivedRoutingPacket (socket, srcAddress, receive_list)<< endl;
        cout << s << endl;
      }
      else
      {
        if (m_log != 0)
          {
            NS_LOG_INFO (m_protocolName + " " + PrintReceivedRoutingPacket (socket, srcAddress, receive_list));
          }

        NS_LOG_INFO("CenterReceive: the containt of the received packet: " << size << "(size) " << s );
        receive_content.push_back (s);//Rui: add the content

        InetSocketAddress source_addr = InetSocketAddress::ConvertFrom (srcAddress);
        int source_id = address_to_id.at(source_addr.GetIpv4 ());

        TimestampTag timestamp;
        if (packet->FindFirstMatchingByteTag (timestamp)) {
          Time tx = timestamp.GetTimestamp (); //Send time
          Time e2e_delay = receive_time - tx;
          NS_LOG_INFO("Send time: " << tx);
          NS_LOG_INFO("Receive time: " << receive_time);
          NS_LOG_INFO("End to End Delay: " << e2e_delay);
          end_to_end_delay.insert ({source_id, e2e_delay});
        }
        
        auto begin = chrono::high_resolution_clock::now();

        int source_index = node_ID_to_index.at(source_id);
        m_data_mangement_helper.MessageHandle(s, source_index);

        auto end = chrono::high_resolution_clock::now();
        AddRecoveryTime(chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count());
      }
  
    }
}


list<Ipv4Address> 
RoutingHelper::GetReceiveList ()
{
  return receive_list;
}

list<string>
RoutingHelper::GetReceiveContent ()
{
  return receive_content;
}

map<int, Time> 
RoutingHelper::GetEndDelay ()
{
  return end_to_end_delay;
}

map<int, int>
RoutingHelper::GetMaskingTime ()
{
  return stat_masking_time; 
}

void
RoutingHelper::AddRecoveryTime (int time_duration)
{
  stat_recovery_and_unmasking_time += time_duration;
}

int 
RoutingHelper::GetRecoveryTime()
{
  return stat_recovery_and_unmasking_time;
}

void
RoutingHelper::SetLogging (int log)
{
  m_log = log;
}

//WifiAPP

class WifiApp
{
public:
  /**
   * \brief Constructor
   * \return none
   */
  WifiApp ();

  /**
   * \brief Destructor
   * \return none
   */
  virtual ~WifiApp ();

  /**
   * \brief Enacts simulation of an ns-3 wifi application
   * \param argc program arguments count
   * \param argv program arguments
   * \return none
   */
  void Simulate (int argc, char **argv);

protected:
  /**
   * \brief Sets default attribute values
   * \return none
   */
  virtual void SetDefaultAttributeValues ();

  /**
   * \brief Process command line arguments
   * \param argc program arguments count
   * \param argv program arguments
   * \return none
   */
  virtual void ParseCommandLineArguments (int argc, char **argv);

  /**
   * \brief Configure nodes
   * \return none
   */
  virtual void ConfigureNodes ();

  /**
   * \brief Configure channels
   * \return none
   */
  virtual void ConfigureRoutingNodes ();
  //rui


  virtual void ConfigureChannels ();

  /**
   * \brief Configure devices
   * \return none
   */
  virtual void ConfigureDevices ();

  /**
   * \brief Configure mobility
   * \return none
   */
  virtual void ConfigureMobility ();

  /**
   * \brief Configure applications
   * \return none
   */
  virtual void ConfigureApplications ();

  /**
   * \brief Configure tracing
   * \return none
   */
  virtual void ConfigureTracing ();

  /**
   * \brief Run the simulation
   * \return none
   */
  virtual void RunSimulation ();

};

WifiApp::WifiApp ()
{
}

WifiApp::~WifiApp ()
{
}

void
WifiApp::Simulate (int argc, char **argv)
{
  // Simulator Program Flow:
  // (source:  NS-3 Annual Meeting, May, 2014, session 2 slides 6, 28)
  //   (HandleProgramInputs:)
  //   SetDefaultAttributeValues
  //   (ConfigureTopology:)
  //   ConfigureNodes
  //   ConfigureChannels
  //   ConfigureDevices
  //   ConfigureMobility
  //   ConfigureApplications
  //     e.g AddInternetStackToNodes
  //         ConfigureIpAddressingAndRouting
  //         configureSendMessages
  //   ConfigureTracing
  //   RunSimulation

  SetDefaultAttributeValues ();
  ParseCommandLineArguments (argc, argv);
  ConfigureNodes (); //condigure the mobility nodes

  ConfigureMobility ();//configue mobility first
  cout<<"Condigure mobility nodes successd"<<endl;;
   //int head_index = 314; //Rui: the sink is node 25. We only have one sink in a cluster, i.e., the head.

  ConfigureRoutingNodes();
  cout<<"Condigure routing nodes successd"<<endl;

  ConfigureChannels ();
  ConfigureDevices ();
  //m_adhocTxNodes.Create (group_size);
  ConfigureApplications ();
  ConfigureTracing ();
  RunSimulation ();
}

void
WifiApp::SetDefaultAttributeValues ()
{
}

void
WifiApp::ParseCommandLineArguments (int argc, char **argv)
{
}

void
WifiApp::ConfigureNodes ()
{
}

void 
WifiApp::ConfigureRoutingNodes ()
{

}

void
WifiApp::ConfigureChannels ()
{
}

void
WifiApp::ConfigureDevices ()
{
}

void
WifiApp::ConfigureMobility ()
{
}

void
WifiApp::ConfigureApplications ()
{
}

void
WifiApp::ConfigureTracing ()
{
}

void
WifiApp::RunSimulation ()
{
}


class VanetRoutingExperiment : public WifiApp 
{  //Rui: WifiApp::Simulate
public:
  /**
   * \brief Constructor
   * \return none
   */
  VanetRoutingExperiment ();

  void PrintReceiveList ();

protected:
  /**
   * \brief Sets default attribute values
   * \return none
   */
  virtual void SetDefaultAttributeValues ();

    /**
   * \brief Process command line arguments
   * \param argc program arguments count
   * \param argv program arguments
   * \return none
   */
  virtual void ParseCommandLineArguments (int argc, char **argv);

  /**
   * \brief Configure nodes
   * \return none
   */
  virtual void ConfigureNodes ();

  /**
   * \brief Configure channels
   * \return none
   */
  virtual void ConfigureRoutingNodes ();
  //Rui

  virtual void ConfigureChannels ();

  /**
   * \brief Configure devices
   * \return none
   */
  virtual void ConfigureDevices ();

  /**
   * \brief Configure mobility
   * \return none
   */
  virtual void ConfigureMobility ();

  /**
   * \brief Configure applications
   * \return none
   */
  virtual void ConfigureApplications ();

  /**
   * \brief Configure tracing
   * \return none
   */
  virtual void ConfigureTracing ();

  /**
   * \brief Run the simulation
   * \return none
   */
  virtual void RunSimulation ();

  /**
   * \brief Process outputs
   * \return none
   */

   private:
  /**
   * \brief Run the simulation
   * \return none
   */
  void Run ();

  /**
   * \brief Run the simulation
   * \param argc command line argument count
   * \param argv command line parameters
   * \return none
   */
  void CommandSetup (int argc, char **argv);


  /**
   * \brief Set up log file
   * \return none
   */
  void SetupLogFile ();

  /**
   * \brief Set up logging
   * \return none
   */
  void SetupLogging ();

  /**
   * \brief Configure default attributes
   * \return none
   */
  void ConfigureDefaults ();

  /**
   * \brief Set up the adhoc mobility nodes
   * \return none
   */
  void SetupAdhocMobilityNodes ();

  /**
   * \brief Set up the adhoc devices
   * \return none
   */

  void SetupRoutingNodesFromMobNodes();
//RUI

  void SetupAdhocDevices ();

  /**
   * \brief Set up generation of IEEE 1609 WAVE messages,
   * as a Basic Safety Message (BSM).  The BSM is typically
   * a ~200-byte packets broadcast by all vehicles at a nominal
   * rate of 10 Hz
   * \return none
   */
  void SetupWaveMessages ();

  /**
   * \brief Set up generation of packets to be routed
   * through the vehicular network
   * \return none
   */
  void SetupRoutingMessages ();

  /**
   * \brief Set up a prescribed scenario
   * \return none
   */
  void SetupScenario ();


 /**
   * Course change function
   * \param os the output stream
   * \param context trace source context (unused)
   * \param mobility the mobility model
   */

   static void
  CourseChange (std::ostream *os, std::string context, Ptr<const MobilityModel> mobility);

  uint32_t m_port; ///< port
  uint32_t m_nSinks; ///< number of sinks
  std::string m_protocolName; ///< protocol name
  double m_txp; ///< distance //
  bool m_traceMobility; ///< trace mobility
  uint32_t m_protocol; ///< protocol

  uint32_t m_lossModel; ///< loss model
  uint32_t m_fading; ///< fading
  std::string m_lossModelName; ///< loss model name

  std::string m_phyMode; ///< phy mode
  uint32_t m_80211mode; ///< 80211 mode

  std::string m_traceFile; ///< trace file 
  std::string m_logFile; ///< log file
  uint32_t m_mobility; ///< mobility
  uint32_t m_nNodes; ///< number of nodes
  double m_TotalSimTime; ///< total sim time
  std::string m_rate; ///< rate
  std::string m_phyModeB; ///< phy mode
  std::string m_trName; ///< trace file name
  int m_nodeSpeed; ///< in m/s
  int m_nodePause; ///< in s
  int m_verbose; ///< verbose
  std::ofstream m_os; ///< output stream
  NetDeviceContainer m_adhocTxDevices; ///< adhoc transmit devices
  Ipv4InterfaceContainer m_adhocTxInterfaces; ///< adhoc transmit interfaces
  uint32_t m_scenario; ///< scenario
  double m_gpsAccuracyNs; ///< GPS accuracy
  double m_txMaxDelayMs; ///< transmit maximum delay
  int m_routingTables; ///< routing tables
  int m_asciiTrace; ///< ascii trace
  int m_pcap; ///< PCAP

  Ptr<RoutingHelper> m_routingHelper; ///< routing helper
  int m_log; ///< log
  /// used to get consistent random numbers across scenarios
  int64_t m_streamIndex;
  NodeContainer m_adhocTxNodes; ///< adhoc transmit nodes
  NodeContainer m_mobilityNodes; ///< adhoc transmit nodes
};

VanetRoutingExperiment::VanetRoutingExperiment ()
  : m_port (9),
    m_nSinks (1),
    m_protocolName ("protocol"),
    m_txp (20),
    m_traceMobility (false),
    // AODV
    m_protocol (2),
    m_lossModel (3),
    m_fading (0),
    m_lossModelName (""),
    m_phyMode ("OfdmRate6MbpsBW10MHz"),
    // 1=802.11p
    m_80211mode (1),
    m_traceFile (""),
    m_logFile ("low99-ct-unterstrass-1day.filt.7.adj.log"),
    m_mobility (), 
    m_nNodes (156),
    m_TotalSimTime (300.01),
    m_rate ("2048bps"),
    m_phyModeB ("DsssRate11Mbps"),
    m_trName ("vanet-sumo-Rui"),
    m_nodeSpeed (20), //Rui: Node speed (m/s) for RWP model
    m_nodePause (0),
    m_verbose (0),
    m_scenario (1),
    m_gpsAccuracyNs (40), //GPS sync accuracy (ns)
    m_txMaxDelayMs (10),
    m_routingTables (0), //Dump routing tables at t=5 seconds 0=no;1=yes//RuiTest
    m_asciiTrace (0),
    m_pcap (0),
    m_log (0),
    m_streamIndex (0),
    m_adhocTxNodes ()
{
  m_routingHelper = CreateObject<RoutingHelper> ();

  // set to non-zero value to enable
  // simply uncond logging during simulation run
  m_log = 1;
}

///Rui: 1. to print all the packets the center/head received. 
// 2. To handle all these packets and finally get the average.
// 3. To get the statistic values
void
VanetRoutingExperiment::PrintReceiveList ()
{

  NS_LOG_INFO("Finish: experiment. Start: statistic + summary.");
  list<Ipv4Address> mlist = m_routingHelper->GetReceiveList();
  list<string> mlist_content = m_routingHelper->GetReceiveContent();
  
  NS_LOG_INFO("mlist:");
  for (auto const &v : mlist)
    NS_LOG_INFO(v);
    NS_LOG_INFO("mlist_content:");
  for (auto const &v : mlist_content)
    NS_LOG_INFO(v);
  cout << "The center receive " << mlist.size() << "packets in total" << "\n";
  

  vector<vector<double> > coef = m_routingHelper->m_data_mangement_helper.GetCoef();
  int i,j;
  

  NS_LOG_LOGIC("Functions_Raw");
  for (i = 0; i < coef.size(); i++)
  {
      for(j = 0; j < coef[0].size(); j++)
          NS_LOG_LOGIC(coef[i][j]);
  }

  NS_LOG_LOGIC("The center received observation values (before recovery): ");
  vector<double> obser_received = m_routingHelper->m_data_mangement_helper.GetObserList();
  for (i=0; i<obser_received.size(); i++)
  {
      NS_LOG_LOGIC(obser_received[i]);
  }

  int num_of_received = count_if(obser_received.begin(), obser_received.end(), [](int c){return c != -100;});


  auto begin = chrono::high_resolution_clock::now();
  m_routingHelper->m_data_mangement_helper.FunctionsClean();
  coef = m_routingHelper->m_data_mangement_helper.GetCoef();
  if (coef.empty())
  {
    NS_LOG_LOGIC("no recoverd");
    auto end = chrono::high_resolution_clock::now();
    m_routingHelper->AddRecoveryTime(chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count());

  }
  else
  {
    NS_LOG_LOGIC("Functions_Raw after clean: ");
    for (i = 0; i < coef.size(); i++)
    {
        for(j = 0; j < coef[0].size(); j++)
            NS_LOG_LOGIC(coef[i][j]);
    }
    vector<int> map_id = m_routingHelper->m_data_mangement_helper.GetID_Map();

    //m_routingHelper->m_data_recovery_helper();
    m_routingHelper->m_data_recovery_helper.SetParameters(map_id.size(),coef.size(),coef);
    m_routingHelper->m_data_recovery_helper.pc();

    auto end = chrono::high_resolution_clock::now();
    m_routingHelper->AddRecoveryTime(chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count());

    vector<double> cal_result = m_routingHelper->m_data_recovery_helper.GetResults();

    cout << "The center received observation values (before recovery): " << endl;
    for (i=0; i<obser_received.size(); i++)
    {
        cout << obser_received[i] << " ";
    }
    cout << endl;

    for (i = 0; i < cal_result.size(); i++)
    {
        obser_received[map_id[i]] = cal_result[i]; 
    }


    cout << "The center received observation values (after recovery): " << endl;
    for (i=0; i<obser_received.size(); i++)
    {
        cout << obser_received[i] << " ";
    }
    cout<<endl;
  }
  //unmasking

  double sum_of_all = 0.0;
  for (i=0; i<obser_received.size(); i++)
  {
      if (obser_received[i] != -100)
        sum_of_all = sum_of_all + obser_received[i];
      
  }

  NS_LOG_LOGIC("The masked value: " );
  for (i=0; i<mask_obser.size(); i++)
  {
      NS_LOG_LOGIC(mask_obser[i]); 
  }

  
  double head_mask = mask_obser[0];
  NS_LOG_LOGIC("Masked value of head: " << head_mask);
  int num_of_received_2 = count_if(obser_received.begin(), obser_received.end(), [](int c){return c != -100;});
  double aveage_value = (sum_of_all+head_mask)/(num_of_received_2+1);
  cout << "[Statistic] Received number: "<< num_of_received_2 << "  [Statistic] Average local gradient:" << aveage_value << endl << endl;

  std::ofstream myfile;
  

  myfile.open ("rui_statistic_x.csv",std::ios::app);
  /*myfile << "Received number after recovery, Average local gradient, packet_loss_rate, packet_loss_rate_after_recovery,
  packet_recovery_rate, end_to_end_delay(average), Masking time(average), network coding time(average), Recovery time (decoding and unmasking)for head,
  \n";*/


  double packet_loss_rate = 1.0 - (double)num_of_received/(group_size-1);
  cout << "[Statistic] packet_loss_rate: "<< packet_loss_rate << endl;
  double packet_loss_rate_after_recovery = 1.0 - (double)num_of_received_2/(group_size-1);
  cout << "[Statistic] packet_loss_rate_after_recovery: "<< packet_loss_rate_after_recovery  << endl << endl;
  double packet_recovery_rate = 0.0;
  if (num_of_received != group_size-1)
  {
    packet_recovery_rate = (double)(num_of_received_2-num_of_received)/(group_size-1-num_of_received);
    cout << "[Statistic] packet_recovery_rate: "<< packet_recovery_rate << endl << endl;
  }


  map<int, Time> end_delay = m_routingHelper->GetEndDelay();
  cout << "[Statistic] end_to_end_delay:" << endl << endl;
  double average_end_to_end_delay = 0.0;
  double average_end_to_end_delay_raw = 0.0;
  int legal_dealy = 0;
  
  for (auto const &v : end_delay)
  {
    cout << "Node "<< v.first << "Delay(s) "<< v.second.GetSeconds() << "   ";
    if(v.second.GetSeconds()<0.10) //remove the ones that too long because of no ARP
    {
       average_end_to_end_delay = average_end_to_end_delay + v.second.GetSeconds();
       legal_dealy++;
    }
    average_end_to_end_delay_raw  = average_end_to_end_delay_raw + v.second.GetSeconds();
  }
  cout << endl;
  cout << "Average(ms): "<< 1000.0*average_end_to_end_delay/legal_dealy << endl << endl;
  cout << "Average(ms)_raw: "<< 1000.0*average_end_to_end_delay_raw/end_delay.size() << endl << endl;



  cout << "[Statistic] Handle time (network coding) for each router:" << endl << endl;
  double average_handle = 0.0;
  for (auto const &v : stat_network_coding_time)
  {
    cout << "Node "<< v.first << "Time(ns) "<< v.second<< "   ";
    average_handle = average_handle + v.second;
  }
  cout << endl;
  cout << "Average(ms): "<< (average_handle/stat_network_coding_time.size())/1000000.0 << endl << endl;

  map<int, int> masking_time = m_routingHelper->GetMaskingTime();
  double average_masking = 0.0;
  cout << "[Statistic] Masking time for each sender:" << endl;
  for (auto const &v : masking_time)
  {
    cout << "Node "<< v.first << "Time(ns) "<< v.second<< "   ";
    average_masking = average_masking + v.second;
  }
  cout << endl;
  cout << "Average(ms): "<< (average_masking/masking_time.size())/1000000.0 << endl << endl;

  int recovery_time = m_routingHelper->GetRecoveryTime();
  cout << "[Statistic] Recovery time (decoding and unmasking)for head: " << recovery_time/1000000.0 << "ms" << endl;

  
  // Rui: print out to files for charts, tables and figures.
  myfile << num_of_received_2 << "," << aveage_value << "," << packet_loss_rate << "," << packet_loss_rate_after_recovery  
  << "," << packet_recovery_rate << "," << 1000.0*average_end_to_end_delay/legal_dealy<< ","<< 1000.0*average_end_to_end_delay_raw/end_delay.size() << "," <<  (average_masking/masking_time.size())/1000000.0 
  << "," << (average_handle/stat_network_coding_time.size())/1000000.0 << "," << recovery_time/1000000.0 << "\n";
  myfile.close();



}

void
VanetRoutingExperiment::SetDefaultAttributeValues ()
{
  // handled in constructor
}

// important configuration items stored in global values
static ns3::GlobalValue g_port ("VRCport",
                                "Port",
                                ns3::UintegerValue (9),
                                ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_nSinks ("VRCnSinks",
                                  "Number of sink nodes for routing non-BSM traffic",
                                  ns3::UintegerValue (10),
                                  ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_traceMobility ("VRCtraceMobility",
                                         "Trace mobility 1=yes;0=no",
                                         ns3::UintegerValue (0),
                                         ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_protocol ("VRCprotocol",
                                    "Routing protocol",
                                    ns3::UintegerValue (2),
                                    ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_lossModel ("VRClossModel",
                                     "Propagation Loss Model",
                                     ns3::UintegerValue (5),
                                     ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_fading ("VRCfading",
                                  "Fast Fading Model",
                                  ns3::UintegerValue (0),
                                  ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_80211mode ("VRC80211mode",
                                     "802.11 mode (0=802.11a;1=802.11p)",
                                     ns3::UintegerValue (1),
                                     ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_mobility ("VRCmobility",
                                    "Mobility mode 0=random waypoint;1=mobility trace file",
                                    ns3::UintegerValue (2),
                                    ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_nNodes ("VRCnNodes",
                                  "Number of nodes (vehicles)",
                                  ns3::UintegerValue (156),
                                  ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_nodeSpeed ("VRCnodeSpeed",
                                     "Node speed (m/s) for RWP model",
                                     ns3::UintegerValue (20),
                                     ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_nodePause ("VRCnodePause",
                                     "Node pause time (s) for RWP model",
                                     ns3::UintegerValue (0),
                                     ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_verbose ("VRCverbose",
                                   "Verbose 0=no;1=yes",
                                   ns3::UintegerValue (0),
                                   ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_scenario ("VRCscenario",
                                    "Scenario",
                                    ns3::UintegerValue (1),
                                    ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_routingTables ("VRCroutingTables",
                                         "Dump routing tables at t=5 seconds 0=no;1=yes",
                                         ns3::UintegerValue (1),//Ruitest
                                         ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_asciiTrace ("VRCasciiTrace",
                                      "Dump ASCII trace 0=no;1=yes",
                                      ns3::UintegerValue (0),
                                      ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_pcap ("VRCpcap",
                                "Generate PCAP files 0=no;1=yes",
                                ns3::UintegerValue (0),
                                ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_txp ("VRCtxp",
                               "Transmission power dBm",
                               ns3::DoubleValue (7.5),
                               ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_gpsAccuracyNs ("VRCgpsAccuracyNs",
                                         "GPS sync accuracy (ns)",
                                         ns3::DoubleValue (40),
                                         ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_txMaxDelayMs ("VRCtxMaxDelayMs",
                                        "Tx May Delay (ms)",
                                        ns3::DoubleValue (10),
                                        ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_phyMode ("VRCphyMode",
                                   "PHY mode (802.11p)",
                                   ns3::StringValue ("OfdmRate6MbpsBW10MHz"),
                                   ns3::MakeStringChecker ());
static ns3::GlobalValue g_traceFile ("VRCtraceFile",
                                     "Mobility trace filename",
                                     ns3::StringValue ("./src/wave/examples/low99-ct-unterstrass-1day.filt.7.adj.mob"),
                                     ns3::MakeStringChecker ());
static ns3::GlobalValue g_logFile ("VRClogFile",
                                   "Log filename",
                                   ns3::StringValue ("low99-ct-unterstrass-1day.filt.7.adj.log"),
                                   ns3::MakeStringChecker ());
static ns3::GlobalValue g_rate ("VRCrate",
                                "Data rate",
                                ns3::StringValue ("2048bps"),
                                ns3::MakeStringChecker ());
static ns3::GlobalValue g_phyModeB ("VRCphyModeB",
                                    "PHY mode (802.11a)",
                                    ns3::StringValue ("DsssRate11Mbps"),
                                    ns3::MakeStringChecker ());
static ns3::GlobalValue g_trName ("VRCtrName",
                                  "Trace name",
                                  ns3::StringValue ("vanet-routing-compare"),
                                  ns3::MakeStringChecker ());

void
VanetRoutingExperiment::ParseCommandLineArguments (int argc, char **argv)
{
  //CommandSetup (argc, argv); //Rui: I don't use the command line setup method.
  SetupScenario ();
  ConfigureDefaults ();
  m_routingHelper->SetLogging (m_log);
}

void
VanetRoutingExperiment::ConfigureNodes ()
{
  //m_adhocTxNodes.Create (group_size); // Rui: generate m-nNodes)(40) nodes

  m_mobilityNodes.Create (m_nNodes);

}
void
VanetRoutingExperiment::ConfigureRoutingNodes ()
{
  
  SetupRoutingNodesFromMobNodes();

}

  

void
VanetRoutingExperiment::ConfigureChannels ()
{
  //Rui: set up channel and devices
  SetupAdhocDevices ();
}

void
VanetRoutingExperiment::ConfigureDevices ()
{
  // We don't need this
}

void
VanetRoutingExperiment::ConfigureMobility ()
{
  SetupAdhocMobilityNodes ();
}

void
VanetRoutingExperiment::ConfigureApplications ()
{

  SetupRoutingMessages ();
}

void
VanetRoutingExperiment::ConfigureTracing ()
{
  SetupLogFile ();
  SetupLogging ();

  AsciiTraceHelper ascii;
  MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (m_trName + ".mob"));
}

void
VanetRoutingExperiment::RunSimulation ()
{
  Run ();
}


void
VanetRoutingExperiment::Run ()
{
  NS_LOG_INFO ("Run Simulation.");

  Simulator::Stop (Seconds (m_TotalSimTime));
  //AnimationInterface anim("vanet_sumo_Rui.xml"); //Rui: to run under NetAnim
 // anim.SetMaxPktsPerTraceFile(200000);
  //anim.EnableIpv4RouteTracking ("routingTable_Rui.xml", Seconds(289), Seconds(296), Seconds(1));//RuiTest
  
  //anim.AddSourceDestination (7, "10.1.0.11");
  //anim.AddSourceDestination (48, "10.1.0.26");
  //anim.AddSourceDestination (8, "10.1.0.11");//Rui: This is for observation when debugging

  //anim.EnablePacketMetadata (true);;//RuiTest
  
  Simulator::Run ();
  Simulator::Destroy ();
}


// Prints actual position and velocity when a course change event occurs
void
VanetRoutingExperiment::
CourseChange (std::ostream *os, std::string context, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition (); // Get position
  Vector vel = mobility->GetVelocity (); // Get velocity

  pos.z = 1.5;

  // Prints position and velocities
  *os << Simulator::Now () << " POS: x=" << pos.x << ", y=" << pos.y
      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
      << ", z=" << vel.z << std::endl;
}

void
VanetRoutingExperiment::SetupLogFile ()
{
  // open log file for output
  m_os.open (m_logFile.c_str ());
}

void VanetRoutingExperiment::SetupLogging ()
{

  // Enable logging from the ns2 helper
  //LogComponentEnable ("vanet-sumo-rui-real-scenario",LOG_LEVEL_ALL);
  
  //LogComponentEnable ("Ipv4L3Protocol",LOG_LEVEL_INFO);
  //LogComponentEnable ("AodvRoutingProtocol",LOG_LEVEL_DEBUG);
  //LogComponentEnable ("WifiPhy",LOG_LEVEL_DEBUG);
  //LogComponentEnable ("InterferenceHelper",LOG_LEVEL_DEBUG);
  //LogComponentEnable ("ThresholdPreambleDetectionModel",LOG_LEVEL_DEBUG);
  //LogComponentEnable ("PhyEntity",LOG_LEVEL_DEBUG);
  //LogComponentEnable ("WifiPhy",LOG_LEVEL_DEBUG);
  //AsciiTraceHelper ascii;
  //ns3::GlobalRoutingLinkRecord::pointToPoint.EnableAsciiAll (ascii.CreateFileStream ("myfirst.tr"));
  //LogComponentEnable ("ArpL3Protocol",LOG_LEVEL_ALL);
  //LogComponentEnable ("PropagationLossModel",LOG_LEVEL_ALL);

  Packet::EnablePrinting ();
}

void
VanetRoutingExperiment::ConfigureDefaults ()
{
 
  //Set Non-unicastMode rate to unicast mode
  if (m_80211mode == 2)
    {
      Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (m_phyModeB));
    }
  else
    {
      Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (m_phyMode));
    }
}


void
VanetRoutingExperiment::SetupAdhocMobilityNodes ()
{
  if (m_mobility == 1)
    {
      Ns2MobilityHelper ns2 = Ns2MobilityHelper (m_traceFile);
      ns2.Install (); // configure movements for each node, while reading trace file
    }
  else if (m_mobility == 2)
    {

    }
  else if (m_mobility == 3)
    {
  

    }
  else if (m_mobility == 4) 
  {     


  }
  else if (m_mobility == 5) 
  {     


  }
  cout<<"MobilityModel:"<<m_mobility<<endl;

  // Configure callback for logging
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeBoundCallback (&VanetRoutingExperiment::CourseChange, &m_os));
}


void
VanetRoutingExperiment::SetupRoutingNodesFromMobNodes()
{
  
  
  for (uint32_t node_index = 0; node_index < group_size; node_index ++) //RuiTest
  {  
     Ptr<Node> p = m_mobilityNodes.Get(node_list[node_index]);
     m_adhocTxNodes.Add(p);
  }
  NS_LOG_UNCOND("sub_node_contaniner generated.");

}

void
VanetRoutingExperiment::SetupAdhocDevices ()
{
  //Rui: Packet loss 
  if (m_lossModel == 1)
    {
      m_lossModelName = "ns3::FriisPropagationLossModel";
    }
  else if (m_lossModel == 2)
    {
      m_lossModelName = "ns3::ItuR1411LosPropagationLossModel";
    }
  else if (m_lossModel == 3)
    {
      m_lossModelName = "ns3::TwoRayGroundPropagationLossModel";
    }
  else if (m_lossModel == 4)
    {
      m_lossModelName = "ns3::LogDistancePropagationLossModel";
    }
  else if (m_lossModel == 5) 
    {
      m_lossModelName = "ns3::RateErrorModel";
      cout<<"ERROR MODEL:"<<m_lossModelName<<endl;
    }
  else if (m_lossModel == 6) 
    {
      m_lossModelName = "noLoss";
      YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    }
  else if (m_lossModel == 7) // 
   {
      m_lossModelName = "ns3::RangePropagationLossModel";
   }
  else if (m_lossModel == 8) 
   {
      m_lossModelName = "ns3::RangePropagationLossModel";
   }
  else
    {
      // Unsupported propagation loss model.
      // Treating as ERROR
      NS_LOG_ERROR ("Invalid propagation loss model specified.  Values must be [1-4], where 1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance");
    }

  // frequency
  double freq = 0.0;
  if ((m_80211mode == 1)
      || (m_80211mode == 3))
    {
      // 802.11p 5.9 GHz
      freq = 5.9e9;
    }
  else
    {
      // 802.11b 2.4 GHz
      freq = 2.4e9;
    }

  // Setup propagation models
  
  YansWifiChannelHelper wifiChannel;


  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  
  if (m_lossModel == 3)
    {
      // two-ray requires antenna height (else defaults to Friss)
      cout<<"wifiChannel add propagation loss:"<< m_lossModelName <<endl;
      wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq), "HeightAboveZ", DoubleValue (1.5));//1.5 RuiTestLoss
    }
  else if (m_lossModel == 4)
    {
      cout<<"wifiChannel add propagation loss:"<< m_lossModelName <<endl;
      //wifiChannel.AddPropagationLoss (m_lossModelName, "Exponent", DoubleValue (3.0));
      wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq));
    }
  else if (m_lossModel == 7) //What we used
    {
      //Config::SetDefault( "ns3::RangePropagationLossModel::MaxRange", DoubleValue( *100.0* ) ); 
      cout<<"wifiChannel add propagation loss:"<< m_lossModelName <<endl;
      wifiChannel.AddPropagationLoss (m_lossModelName, "MaxRange", DoubleValue (100.0));
    }
  // Propagation loss models are additive.

  else if (m_lossModel == 8) //What we used
    {
      //Config::SetDefault( "ns3::RangePropagationLossModel::MaxRange", DoubleValue( *100.0* ) ); 
      cout<<"wifiChannel add propagation loss: range plus two-ray" <<endl;
      wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel", "Frequency", DoubleValue (freq), "HeightAboveZ", DoubleValue (1.5));//1.5 RuiTestLoss
      wifiChannel.AddPropagationLoss (m_lossModelName, "MaxRange", DoubleValue (200.0));
    }

  if (m_fading != 0)
    {
      // if no obstacle model, then use Nakagami fading if requested
      cout<<"wifiChannel add propagation loss:"<< "ns3::NakagamiPropagationLossModel" <<endl;
      wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
    }

 //Rui: LogDistancePropagationLossModel

  cout<< "m_lossModel:"<<m_lossModel<<endl;
    
  // the channel
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy;// =  YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);


  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.SetChannel (channel);
  wavePhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  // Setup WAVE PHY and MAC
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (m_verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
      // likewise, turn on WAVE PHY logging
      waveHelper.EnableLogComponents ();
    }

  WifiHelper wifi;

  // Setup 802.11b stuff
  //wifi.SetStandard (WIFI_PHY_STANDARD_80211b); //'SetStandard' has been explicitly marked deprecated in ns3.34 so we made a little bit adjustment here
  wifi.SetStandard (WIFI_STANDARD_80211b);

  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (m_phyModeB),
                                "ControlMode",StringValue (m_phyModeB));

  // Setup 802.11p stuff
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (m_phyMode),
                                      "ControlMode",StringValue (m_phyMode));

  // Setup WAVE-PHY stuff
  waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (m_phyMode),
                                      "ControlMode",StringValue (m_phyMode));

  // Set Tx Power
  wifiPhy.Set ("TxPowerStart",DoubleValue (m_txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txp));
  wavePhy.Set ("TxPowerStart",DoubleValue (m_txp));
  wavePhy.Set ("TxPowerEnd", DoubleValue (m_txp));

  // Add an upper mac and disable rate control
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();

  // Setup net devices

  if (m_80211mode == 3)
    {
      m_adhocTxDevices = waveHelper.Install (wavePhy, waveMac, m_adhocTxNodes);
    }
  else if (m_80211mode == 1)
    {
      m_adhocTxDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, m_adhocTxNodes);
    }
  else
    {
      m_adhocTxDevices = wifi.Install (wifiPhy, wifiMac, m_adhocTxNodes);
    }

  if (m_asciiTrace != 0)
    {
      AsciiTraceHelper ascii;
      Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (m_trName + ".tr").c_str ());
      wifiPhy.EnableAsciiAll (osw);
      wavePhy.EnableAsciiAll (osw);
    }
  if (m_pcap != 0)
    {
      wifiPhy.EnablePcapAll ("vanet-rui-pcap");
      wavePhy.EnablePcapAll ("vanet-rui-pcap");
    }
}

void
VanetRoutingExperiment::SetupRoutingMessages ()
{
  m_routingHelper->Install (m_adhocTxNodes,
                            m_adhocTxDevices,
                            m_adhocTxInterfaces,
                            m_TotalSimTime,
                            m_protocol,
                            m_nSinks,
                            m_routingTables);
}

void
VanetRoutingExperiment::SetupScenario ()
{

  if (m_scenario == 1)
    {
      // 40 nodes in RWP 300 m x 1500 m synthetic highway, 10s
      m_traceFile = "tracefile/trace_acosta_rui.tcl";
      m_logFile = "";
      m_mobility = 1;
      m_lossModel = 7;
      if (m_nNodes == 156)
        {
          m_nNodes = 1004; 
        }
      if (m_TotalSimTime == 300.01)
        {
          m_TotalSimTime = 20; //Rui:similuation time
        }
    }
  else if (m_scenario == 2)
    {
      //We don't need this. Didn't delete the entrance for furthur test
    }
}

int
main (int argc, char *argv[])
{
  std::cout << "LOG: start the experiment"<< std::endl;

  uint32_t m_seed = 22;
/*  uint32_t m_seed = 2;
  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
  m_seed = x->GetInteger (0,100);
  

  CommandLine cmd;
  cmd.AddValue ("seed", "randomize seed", m_seed);
  cmd.Parse (argc, argv);

  SeedManager::SetSeed (m_seed);
  cout<<"Seed:"<<m_seed<<endl;*/
  CommandLine cmd;
  cmd.AddValue ("seed", "randomize seed", m_seed);
  cmd.Parse (argc, argv);

  SeedManager::SetSeed (m_seed);

  VanetRoutingExperiment experiment;
  experiment.Simulate (argc, argv);
  experiment.PrintReceiveList();
}