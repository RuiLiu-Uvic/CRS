//The program is is modified from wave-simple-80211p.cc provided by ns-3.

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

#include "ns3/netanim-module.h" 
#include "ns3/simulator.h"
#include "ns3/rui-equation-cal.h"
//#include "ns3/stats-module.h"
//#include "ns3/network-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");

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
 *  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
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

void ReceivePacket (Ptr<Socket> socket)
{
  //NS_LOG_UNCOND ("Start to handle: Received one packet!");
  Ptr<Packet> packet = 0;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      Time receive_time = Simulator::Now ();//Receive time

      TimestampTag timestamp;
      if (packet->FindFirstMatchingByteTag (timestamp)) {
        Time tx = timestamp.GetTimestamp (); //Send time
        Time e2e_delay = receive_time - tx;
        std::cout << "t= " << Simulator::Now() << " Received one packet! The send time is:" << tx.GetSeconds() << std::endl;

        end_to_end_delay.push_back(e2e_delay.GetSeconds()*1000);
      }

      //NS_LOG_UNCOND ("Received one packet!");
    }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {

        Ptr<Packet> packet = Create<Packet> (pktSize);

        TimestampTag timestamp;
        timestamp.SetTimestamp (Simulator::Now ());
        packet->AddByteTag (timestamp);
        socket->Send (packet);
        std::cout << "t = " << Simulator::Now () << " Send one packet with size !" << pktSize << endl;

        Simulator::Schedule (pktInterval, &GenerateTraffic,
                             socket, pktSize,pktCount - 1, pktInterval);
      


    }
  else
    {
      socket->Close ();
    }
}

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize =  2560; // bytes  12288 bits = 1536 bytes; consider 531 entries, then 544256 bytes
  //2560 bytes for 20480bits
  uint32_t numPackets = 30;
  double interval = 1.0; 
  bool verbose = false;
  double m_txp = 55;

  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.Parse (argc, argv);
  Time interPacketInterval = Seconds (interval);


  NodeContainer c;
  c.Create (2);

  // The below set of helpers will help us to put together the wifi NICs we want
 
  //You may need to adjust the codes according to the version/updates of ns3
  //The below two lines are replaced becasue in ns3.35, no member named 'Default' in 'ns3::YansWifiPhyHelper'
  //YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  //YansWifiChannelHelper wifiChannel ;//= YansWifiChannelHelper::Default ();
  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();

  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (400.0));


  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  wifiPhy.Set ("TxPowerStart",DoubleValue (m_txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txp));

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

  
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  //positionAlloc->Add (Vector (10.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                                 "Mode", StringValue ("Time"),
                                 "Time", StringValue ("30s"),
                                 "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=20.0]"),
                                 "Direction", StringValue ("ns3::ConstantRandomVariable[Constant=0]"),
                                 "Bounds", RectangleValue (Rectangle (0.0, 600.0, 0.0, 20.0)));
  mobility.Install (c.Get (1));

  AnimationInterface::SetConstantPosition (c.Get (0), 300, 0, 10); 

  InternetStackHelper internet;
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (c.Get (1), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (1.0), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);


  Simulator::Stop (Seconds (30));
  AnimationInterface anim("Rui_anim_V2I.xml"); 
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
  Simulator::Destroy ();

  return 0;
}
