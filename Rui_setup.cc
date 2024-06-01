/*Rui: This code is build from wave-simple-80211p.cc provided by ns-3
We keep some original notes for easy-understanding.
Instead of two node communication, we modify it to cluster communication.
*/

/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Author: Junling Bu <linlinjavaer@gmail.com>
 *
 */
/**
 * This example shows basic construction of an 802.11p node.  Two nodes
 * are constructed with 802.11p devices, and by default, one node sends a single
 * packet to another node (the number of packets and interval between
 * them can be configured by command-line arguments).  The example shows
 * typical usage of the helper classes for this mode of WiFi (where "OCB" refers
 * to "Outside the Context of a BSS")."
 */

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include <iostream>

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

#include "ns3/rui-equation-cal.h"
#include "ns3/netanim-module.h" 

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Ruisetup");

std::vector<double> end_to_end_delay;

/*
 * In WAVE module, there is no net device class named like "Wifi80211pNetDevice",
 * instead, we need to use Wifi80211pHelper to create an object of
 * WifiNetDevice class.
 *
 * usage:
 *  NodeContainer nodes;
 *  NetDeviceContainer devices;
 *  nodes.Create (2);
 *  YansWifiPhyHelper wifiPhy;
 *  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
 *  wifiPhy.SetChannel (wifiChannel.Create ());
 *  NqosWaveMacHelper wifi80211pMac = NqosWave80211pMacHelper::Default();
 *  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
 *  devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);
 *
 * The reason of not providing a 802.11p class is that most of modeling
 * 802.11p standard has been done in wifi module, so we only need a high
 * MAC class that enables OCB mode.
 */

/**
 * Receive a packet
 * \param socket Rx socket
 */

map<Ipv4Address, int> address_to_id;


static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
        Ptr<Packet> packet_1 = Create<Packet> (pktSize);
        
        TimestampTag timestamp;
        timestamp.SetTimestamp (Simulator::Now ());
        packet_1->AddByteTag (timestamp);
        socket->Send (packet_1);
           
    }
  else
    {
      //socket->Close ();
    }
}

void Sendout(Ptr<Socket> socket, Ptr<Packet> packet)
{
    NS_LOG_INFO("Start to send out by Id:"<<socket->GetNode ()->GetId()<<"current time"<< Simulator::Now () );
    TimestampTag timestamp;
    timestamp.SetTimestamp (Simulator::Now ());
    packet->AddByteTag (timestamp);
    socket->Send (packet);
    
}

void ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet = 0;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
        Time receive_time = Simulator::Now ();//Receive time

        InetSocketAddress addr = InetSocketAddress::ConvertFrom (from);
        
        TimestampTag timestamp;
        if (packet->FindFirstMatchingByteTag (timestamp)) {
          Time tx = timestamp.GetTimestamp (); //Send time
          Time e2e_delay = receive_time - tx;
          int source_id = address_to_id.at(addr.GetIpv4 ());
          int receive_id = socket->GetNode ()->GetId();
          std::cout << "t= " << Simulator::Now() << receive_id<< " Received one packet! from "<< source_id<<" The send time is:" << tx.GetSeconds() <<"The size is "<< packet->GetSize() << std::endl;
          NS_LOG_INFO("source_id:"<< source_id<<"receive_id"<<receive_id);
          if (source_id<receive_id)//receive it and add my list
          {
            NS_LOG_INFO("source_id<receive_id");
            if (receive_id!=19)
            {
 
              int size=packet->GetSize();//+256;//+256; 
              Ptr<Packet> packet_1 = Create<Packet> (size);
             
              InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
              socket->SetAllowBroadcast (true);
              socket->Connect (remote);
              double scheduletime=0.01*(receive_id%3);
              NS_LOG_INFO("Sechedule time: "<<scheduletime);
              Simulator::Schedule( Seconds(scheduletime), &Sendout,
                        socket, packet_1);

            }
            
          }
          end_to_end_delay.push_back(e2e_delay.GetSeconds()*1000);
          }
          
    }
}

/**
 * Geerate traffic
 * \param socket Tx socket
 * \param pktSize packet size
 * \param pktCount number of packets
 * \param pktInterval interval between packet generation
 */






int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize = 2;
  uint32_t numPackets = 1;
  double interval = 0.0; 
  bool verbose = false;

  CommandLine cmd (__FILE__);

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds (interval);


  NodeContainer c;
  c.Create (20);

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (100.0));

  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
    }

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, c);

  // Tracing
  wifiPhy.EnablePcap ("wave-simple-80211p", devices);


  AsciiTraceHelper ascii;
  std::string m_trName = "rui-setup-trace"; 
  Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (m_trName + ".tr").c_str ());
  wifiPhy.EnableAsciiAll (osw);


  wifiPhy.EnablePcapAll ("rui-setup-pcap");
  


  MobilityHelper mobility;


  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                     "MinX", DoubleValue (1.0),
                                     "MinY", DoubleValue (1.0),
                                     "DeltaX", DoubleValue (44),//44
                                     "DeltaY", DoubleValue (3.7),
                                     "GridWidth", UintegerValue (2),
                                     "LayoutType", StringValue ("ColumnFirst"));


  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                                 "Mode", StringValue ("Time"),
                                 "Time", StringValue ("120s"),
                                 "Speed", StringValue ("ns3::NormalRandomVariable[Mean=22.22|Variance=0.07716|Bound=5.55]"),
                                 "Direction", StringValue ("ns3::ConstantRandomVariable[Constant=0]"),
                                 "Bounds", RectangleValue (Rectangle (0.0, 4000.0, 0.0, 20.0)));
  mobility.Install (c);
  mobility.AssignStreams (c, 0);

  InternetStackHelper internet;
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  for (int j = 0; j < c.GetN(); ++j)
  {
    //Ipv4Address address_ipv4;
    address_to_id.insert(make_pair(i.GetAddress(j),c.Get(j)->GetId()));//Ipv4Address(d.Get(i)->GetAddress())
  }

  
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  for (int j=0; j<20; j++)
  {
    Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (j), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);//i.GetAddress (j)
    recvSink->Bind (local);
    recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  }
  for (int j=0; j<20; j++)
  {
    Ptr<Socket> source = Socket::CreateSocket (c.Get (j), tid);
    InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
    source->SetAllowBroadcast (true);
    source->Connect (remote);

    if(j==0)
    {
      Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (1.0), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);
    }

  }

  
  //LogComponentEnable ("Ruisetup",LOG_LEVEL_ALL);

  Simulator::Stop (Seconds (120));
  AnimationInterface anim("Rui_anim_setup.xml"); 
  anim.EnablePacketMetadata (true);;
    
  Simulator::Run ();
    
    
    cout << "The end_to_end_delay(ms): " << endl;
    double average = 0.0;

    for (int i = 0; i<end_to_end_delay.size(); i++)
    {
        cout << end_to_end_delay[i] << " ";
        average += end_to_end_delay[i];
    }
    cout << endl;
    average = average / end_to_end_delay.size();
    cout << "The average end_to_end_delay(ms): " << average << endl;
    cout << "Number of end_to_end_delay: " << end_to_end_delay.size() << endl;
    
    
  Simulator::Destroy ();

  return 0;
}

