
#include <iostream>
#include <string>
 
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/csma-module.h"
#include "ns3/csma-helper.h"

#include "ns3/rui-equation-cal.h"

#include "ns3/simulator.h"

// Don’t forget to change the drop rate to 0: static double error_rate = 0; in rui-vehicle-beta.h.

using namespace std;
using namespace ns3;

vector<double> sendtime;
vector<double> recvtime;
double total_receive_size;
 
NS_LOG_COMPONENT_DEFINE("Rui_RSU_S_large");
 
static void recvCallback(Ptr<Socket> sock)
{
  Ptr<Packet> packet = 0;
  Address from;
  while ((packet = sock->RecvFrom (from)))
    {
      Time receive_time = Simulator::Now ();//Receive time

      TimestampTag timestamp;
      if (packet->FindFirstMatchingByteTag (timestamp)) {
        Time tx = timestamp.GetTimestamp (); //Send time
        Time e2e_delay = receive_time - tx;
        recvtime.push_back(receive_time.GetSeconds()*1000);
        total_receive_size=total_receive_size+packet->GetSize();
      }

      //NS_LOG_UNCOND ("Received one packet!");
    }
}
void send(Ptr<Socket> sock, uint32_t pkt_size)
{
  uint32_t packetSize = pkt_size; //1536; // bytes  12288 bits = 1536 bytes; consider 531 entries, then 544256 bytes
  Ptr<Packet> packet = Create<Packet> (packetSize);
  TimestampTag timestamp;
  timestamp.SetTimestamp (Simulator::Now ());
  packet->AddByteTag (timestamp);
  sock->Send(packet);
  sendtime.push_back(Simulator::Now().GetSeconds()*1000);
  NS_LOG_INFO(sock->GetErrno());
}
 
static bool
ConnectionRequest(Ptr<Socket> Socket,const Address &clientaddress)
{
  NS_LOG_INFO("ConnectionRequest");
  NS_LOG_INFO(Socket->GetSocketType());
  NS_LOG_INFO(Socket);
  NS_LOG_INFO(clientaddress);
  return true;
}
 

static void
NewConnectionCreated(Ptr<Socket> Socket,const Address &clientaddress)
{
  NS_LOG_INFO("newConnectionCreated");
  NS_LOG_INFO(Socket->GetSocketType());
  NS_LOG_INFO(Socket);
  NS_LOG_INFO(clientaddress);
  Socket->SetRecvCallback(MakeCallback(&recvCallback));
}
 
 
//make a connection and send&receive packets between two nodes in ns3
//there are many examples and codes online, for example: https://blog.csdn.net/weixin_42314534/article/details/88322853
//you can use any of them to achieve the same, simple, function.
 
int main(int argc, char *argv[])
{

  total_receive_size = 0; 

  uint32_t m_seed = 1;

  CommandLine cmd;
  cmd.AddValue ("seed", "randomize seed", m_seed);
  cmd.Parse (argc, argv);

  SeedManager::SetSeed (m_seed);

  //LogComponentEnable("Rui_RSU_S_large",LOG_LEVEL_ALL);
  NodeContainer nodes;
  nodes.Create(2);
 
  InternetStackHelper stack;
  stack.Install(nodes);
 
  CsmaHelper csmaHelper;
  csmaHelper.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (35324)));//.0149 ms 14944 ns for victoria; 9.19636ms 9196362 ns for canada
    //Bologna: 10.59 km, 0.03532ms  35324ns  Italy: 592.5km 1.976367ms 1976367ns
  
  NetDeviceContainer csmaNetDevices = csmaHelper.Install(nodes);
 
  csmaHelper.EnablePcap("haha",csmaNetDevices.Get(0),true);
  Ipv4AddressHelper addressHelper;
  addressHelper.SetBase("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = addressHelper.Assign(csmaNetDevices);
 
  TypeId tid = TypeId::LookupByName("ns3::TcpSocketFactory");
  Ptr<Socket> server = Socket::CreateSocket(nodes.Get(0), tid);
  InetSocketAddress addr = InetSocketAddress(Ipv4Address::GetAny(),10086);
    NS_LOG_INFO(addr);
  server->Bind(addr);
  server->Listen();
  NS_LOG_INFO(server);
  server->SetAcceptCallback(MakeCallback(&ConnectionRequest),MakeCallback(&NewConnectionCreated));
 
 
  //client sockets
  Ptr<Socket> client = Socket::CreateSocket(nodes.Get(1), tid);
  InetSocketAddress serverAddr = InetSocketAddress(interfaces.GetAddress(0), 10086);
  client->Connect(serverAddr);
  NS_LOG_INFO(client);

  for(int i =0; i<10; i++)
  {
    Simulator::Schedule(Seconds(1+0.35*i),&send,client, 100000); //0.0075 for victoria, 2.74 for canada
    //0.015 or victoria, 2.74 for canada, 0.017 for bonolga, 1.10 for Italy //1.74  for Italy 0.9 for bonolga
  }//3.1 for canada 
  Simulator::Run();
  cout<<"total receive size: "<<total_receive_size<<" bytes."<<endl;
  cout<<"end-to-end delay: "<< recvtime[recvtime.size()-1]-sendtime[0]<<"ms"<<endl;

  std::ofstream myfile;
  myfile.open ("rui_statistic_RSU_S_country_large_R2.csv",std::ios::app);
  myfile << recvtime[recvtime.size()-1]-sendtime[0]<< "," ;
  myfile.close();


  Simulator::Destroy();
 
 
  return 0;
}
