#include "ns3/attribute-container.h"
#include "ns3/applications-module.h"
#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/eht-phy.h"
#include "ns3/enum.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"

#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/udp-server.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-module.h"

#include "ns3/netanim-module.h"
#include <algorithm>
#include <array>
#include <functional>
#include <numeric>
#include <iostream>
#include "ns3/core-module.h"
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiOnlyExample"); 
/**
 * @param udp true if UDP is used, false if TCP is used
 * @param serverApp a container of server applications
 * @param payloadSize the size in bytes of the packets
 * @return the bytes received by each server application
 */
std::vector<uint64_t>
GetRxBytes(bool udp, const ApplicationContainer& serverApp, uint32_t payloadSize)
{
    std::vector<uint64_t> rxBytes(serverApp.GetN(), 0);
    if (udp)
    {
        for (uint32_t i = 0; i < serverApp.GetN(); i++)
        {
            rxBytes[i] = payloadSize * DynamicCast<UdpServer>(serverApp.Get(i))->GetReceived();
        }
    }
    else
    {
        for (uint32_t i = 0; i < serverApp.GetN(); i++)
        {
            rxBytes[i] = DynamicCast<PacketSink>(serverApp.Get(i))->GetTotalRx();
        }
    }
    return rxBytes;
}

/**
 * Print average throughput over an intermediate time interval.
 * @param rxBytes a vector of the amount of bytes received by each server application
 * @param udp true if UDP is used, false if TCP is used
 * @param serverApp a container of server applications
 * @param payloadSize the size in bytes of the packets
 * @param tputInterval the duration of an intermediate time interval
 * @param simulationTime the simulation time in seconds
 */
void
PrintIntermediateTput(std::vector<uint64_t>& rxBytes,
                      bool udp,
                      const ApplicationContainer& serverApp,
                      uint32_t payloadSize,
                      Time tputInterval,
                      Time simulationTime)
{
    auto newRxBytes = GetRxBytes(udp, serverApp, payloadSize);
    Time now = Simulator::Now();

    std::cout << "[" << (now - tputInterval).As(Time::S) << " - " << now.As(Time::S)
              << "] Per-STA Throughput (Mbit/s):";

    for (std::size_t i = 0; i < newRxBytes.size(); i++)
    {
        std::cout << "\t\t(" << i << ") "
                  << (newRxBytes[i] - rxBytes[i]) * 8. / tputInterval.GetMicroSeconds(); // Mbit/s
    }
    std::cout << std::endl;

    rxBytes.swap(newRxBytes);

    if (now < (simulationTime - NanoSeconds(1)))
    {
        Simulator::Schedule(Min(tputInterval, simulationTime - now - NanoSeconds(1)),
                            &PrintIntermediateTput,
                            rxBytes,
                            udp,
                            serverApp,
                            payloadSize,
                            tputInterval,
                            simulationTime);
    }
}

//------function----
                       
MobilityHelper InstallStaMove(double maxRadius, uint32_t nwifiSTA, NodeContainer wifiStaNodes)
{
  // Create Uniform Distribution STA Position
  
    Ptr<ListPositionAllocator> staPosAlloc = CreateObject<ListPositionAllocator>();
    Ptr<UniformRandomVariable> rhoGen = CreateObject<UniformRandomVariable>(); //scaling ratio
    rhoGen->SetAttribute("Min", DoubleValue(0.0));
    rhoGen->SetAttribute("Max", DoubleValue(1.0)); 
    
    Ptr<UniformRandomVariable> angleGen = CreateObject<UniformRandomVariable>();//Random angle
    angleGen->SetAttribute("Min", DoubleValue(0.0));
    angleGen->SetAttribute("Max", DoubleValue(2 * M_PI));

    MobilityHelper mobilitysta;
    for (uint32_t i = 0; i < nwifiSTA;++i)
    {
      double radius = maxRadius * std::sqrt(rhoGen->GetValue()); //uniform radius maxRadius * scaling ratio
      double angle = angleGen->GetValue();
      
      double x = radius * std::cos(angle);
      double y = radius * std::sin(angle);
      staPosAlloc->Add(Vector(x,y,0.0));
        
    }
    mobilitysta.SetPositionAllocator(staPosAlloc);
    mobilitysta.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Bounds",
                              RectangleValue(Rectangle(-maxRadius, maxRadius, -maxRadius, maxRadius)));
    mobilitysta.Install(wifiStaNodes);
    return mobilitysta;
}

MobilityHelper InstallApMove(NodeContainer wifiApNode)
{
    MobilityHelper mobilityAp;
    Ptr<ListPositionAllocator> apAlloc = CreateObject<ListPositionAllocator>();
    apAlloc->Add(Vector(0.0,0.0,0.0)); //AP position
    mobilityAp.SetPositionAllocator(apAlloc);
    mobilityAp.SetMobilityModel("ns3::ConstantPositionMobilityModel"); 
    mobilityAp.Install(wifiApNode);
    return mobilityAp;
}

std::vector<uint64_t> mcsValueSet(uint32_t min, uint max)
{
  std::vector<uint64_t> mcsValues;
  for(uint8_t mcs = min; mcs <= max; ++mcs) mcsValues.push_back(mcs);
  return mcsValues;
}
 

int main(int argc, char* argv[])
{
  //command variable name  !!
  bool verbose = true;
  CommandLine cmd(__FILE__);

  cmd.Parse (argc,argv);

  if (verbose)
  {
    LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
    LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
  }
//simulationTime 
  Time simulationTime{"10s"};
  Time tputInterval{0}; // interval for detailed throughput measurement
  bool udp{false};
  //number of STA & AP
  uint32_t nWifiAP = 1;
  uint32_t nwifiSTA = 10;
//802.11be  Variable Name (ex:gi, mcs, channelwidth ........)
  //open OFDMA
    bool enableUlOfdma{false};
    std::string dlAckSeqType{"NO-OFDMA"};
    bool enableBsrp{false};
    Time accessReqInterval{0};

// 802.11be set variabele  
//mcs ->  range 0~13 -> 4
    uint8_t minMcs = 4;
    uint8_t maxMcs = 4;
    std::vector<uint64_t> mcsValues;
    mcsValues = mcsValueSet(minMcs, maxMcs);
//channelWidth 20~160 -> 40MHz
    int channelWidth = 40;
    int minChannelWidth = channelWidth;
    int maxChannelWidth = channelWidth;
//Gi 800~3200 -> 800
    int guardInterval{800}; // in nanoseconds, -1 indicates an unset value
    int minGi = enableUlOfdma ? 1600 : 800;
    int maxGi = 3200;
    if (guardInterval >= minGi && guardInterval <= maxGi)
    {
        minGi = guardInterval;
        maxGi = guardInterval;
    }
//payloadSize 
    uint32_t payloadSize = 700;


//--------------run--------------------
for(const auto mcs : mcsValues)
{
    uint8_t index = 0;
    double previous = 0;
    const std::string widthStr = std::to_string(channelWidth);
    const auto segmentWidthStr = widthStr;

  //payloadSize set
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(payloadSize));
                      
  //wifi node create 
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(nwifiSTA);
    NetDeviceContainer staDevices;

    NodeContainer wifiApNode;
    wifiApNode.Create(nWifiAP);
    NetDeviceContainer apDevice;
    
//MAC layer  
    WifiMacHelper mac;
    Ssid ssid = Ssid("ns3-80211be");
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211be);
  //channel link
    std::array<std::string, 3> channelStr; //channel No.
    uint8_t nLinks = 0; //number of link
  //Freq
    std::array<FrequencyRange, 3> freqRanges;
    double frequency{2.4};
    double frequency2{5};
    double frequency3{6};

  //mcs -> Mbp
    std::string dataModeStr = "EhtMcs" + std::to_string(mcs);
    std::string ctrlRateStr;
    uint64_t nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs) / 1e6;

    for (auto freq : {frequency, frequency2, frequency3})
    {
      channelStr[nLinks] = "{0, " + segmentWidthStr + ", ";
      if (freq == 6)
      {
        channelStr[nLinks] += "BAND_6GHZ, 0}";
        freqRanges[nLinks] = WIFI_SPECTRUM_6_GHZ;
        Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
        DoubleValue(48));
        wifi.SetRemoteStationManager(nLinks,
                                    "ns3::ConstantRateWifiManager",
                                    "DataMode",
                                    StringValue(dataModeStr),
                                    "ControlMode",
                                    StringValue(dataModeStr));
      }
      else if (freq == 5)
      {
        channelStr[nLinks] += "BAND_5GHZ, 0}";
        freqRanges[nLinks] = WIFI_SPECTRUM_5_GHZ;
        //mcs-> Mbps
        ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
        wifi.SetRemoteStationManager(nLinks,
                                     "ns3::ConstantRateWifiManager",
                                     "DataMode",
                                     StringValue(dataModeStr),
                                     "ControlMode",
                                     StringValue(ctrlRateStr));
      }
      else if (freq == 2.4)
      {
        channelStr[nLinks] += "BAND_2_4GHZ, 0}";
        freqRanges[nLinks] = WIFI_SPECTRUM_2_4_GHZ;
        Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                           DoubleValue(40));
        ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
        wifi.SetRemoteStationManager(nLinks,
                                     "ns3::ConstantRateWifiManager",
                                     "DataMode",
                                     StringValue(dataModeStr),
                                     "ControlMode",
                                     StringValue(ctrlRateStr));
      }
      nLinks++;
    }

//PHYchannel
    Time channelSwitchDelay{"100us"};
    //EMLSR(I Don't know)
    std::string emlsrMgrTypeId{"ns3::DefaultEmlsrManager"};
    std::string emlsrLinks;
    uint16_t paddingDelayUsec{32};
    uint16_t transitionDelayUsec{128};
    bool switchAuxPhy{true};
    uint16_t auxPhyChWidth{20};
    bool auxPhyTxCapable{true};
  //PHY layer setting 
    SpectrumWifiPhyHelper phy(nLinks);
    phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phy.Set("ChannelSwitchDelay", TimeValue(channelSwitchDelay));    
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    //SetEmlsrManager (I Don't know)
    mac.SetEmlsrManager(emlsrMgrTypeId,
                        "EmlsrLinkSet",
                        StringValue(emlsrLinks),
                        "EmlsrPaddingDelay",
                        TimeValue(MicroSeconds(paddingDelayUsec)),
                        "EmlsrTransitionDelay",
                        TimeValue(MicroSeconds(transitionDelayUsec)),
                        "SwitchAuxPhy",
                        BooleanValue(switchAuxPhy),
                        "AuxPhyTxCapable",
                        BooleanValue(auxPhyTxCapable),
                        "AuxPhyChannelWidth",
                        UintegerValue(auxPhyChWidth));
    for(uint8_t linkId = 0; linkId < nLinks; linkId++)
    {
      phy.Set(linkId, "ChannelSettings", StringValue(channelStr[linkId]));
      //spectrum
      auto spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
      //lossModel 
      auto lossModel = CreateObject<LogDistancePropagationLossModel>();
      //channel install lossModel
      spectrumChannel->AddPropagationLossModel(lossModel);
      phy.AddChannel(spectrumChannel, freqRanges[linkId]);
    }
    //wifi.Install(wifiStaNodes);
    staDevices = wifi.Install(phy, mac, wifiStaNodes);

    //OFDMA Setting
    if (dlAckSeqType != "NO-OFDMA")
    {
        mac.SetMultiUserScheduler("ns3::RrMultiUserScheduler",
                                  "EnableUlOfdma",
                                  BooleanValue(enableUlOfdma),
                                  "EnableBsrp",
                                  BooleanValue(enableBsrp),
                                  "AccessReqInterval",
                                   TimeValue(accessReqInterval));
    }
    // wifi.Install(wifiApNode);
    mac.SetType("ns3::ApWifiMac",
                "EnableBeaconJitter",
                BooleanValue(false),
                "Ssid",
                SsidValue(ssid));
    apDevice = wifi.Install(phy, mac, wifiApNode);
    int64_t streamNumber = 100;
    streamNumber += WifiHelper::AssignStreams(apDevice, streamNumber);
    streamNumber += WifiHelper::AssignStreams(staDevices, streamNumber);

    // move model--------------------
    MobilityHelper mobilitysta;
    double maxRadius = 45.0 ; //max radius 
    mobilitysta = InstallStaMove(maxRadius,nwifiSTA, wifiStaNodes);
    
    MobilityHelper mobilityAp;
    mobilityAp = InstallApMove(wifiApNode);

    //guard interval
    Config::Set(
    "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
    TimeValue(NanoSeconds(guardInterval)));
    //MPDU Buffersize
    uint16_t mpduBufferSize{512};
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MpduBufferSize",
                UintegerValue(mpduBufferSize));

    //------------------NEtwork layer -----------------
    //Network layer : (must install)
    InternetStackHelper stack;
    stack.Install(wifiApNode);
    stack.Install(wifiStaNodes);
    streamNumber += stack.AssignStreams(wifiApNode, streamNumber);
    streamNumber += stack.AssignStreams(wifiStaNodes, streamNumber);
      
    // IP address
    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0","255.255.255.0");
    // assign STA
    Ipv4InterfaceContainer staNodeInterfaces;
    staNodeInterfaces = address.Assign(staDevices);
    Ipv4InterfaceContainer apNodeInterface;
    apNodeInterface = address.Assign(apDevice);
//---------------------------------------

    // STA flow down & up
    bool downlink{false};
    ApplicationContainer serverApp;
    auto serverNodes = downlink ? std::ref(wifiStaNodes) : std::ref(wifiApNode);
    Ipv4InterfaceContainer serverInterfaces;
    NodeContainer clientNodes;

    // uplink&downlink setting 
    for (std::size_t i = 0; i < nwifiSTA; i++)
    {
      serverInterfaces.Add(downlink ? staNodeInterfaces.Get(i)
                                    : apNodeInterface.Get(0));
      clientNodes.Add(downlink ? wifiApNode.Get(0) 
                               : wifiStaNodes.Get(i));
    }
    const auto maxLoad = nLinks *EhtPhy::GetDataRate(mcs,
                                                     MHz_u{static_cast<double>(channelWidth)},
                                                     NanoSeconds(guardInterval),
                                                     1) / 
                                                     nwifiSTA;
    // TCP flow
    uint16_t port = 50000;
    Address localAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", localAddress);
    serverApp = packetSinkHelper.Install(serverNodes.get());
    streamNumber += packetSinkHelper.AssignStreams(serverNodes.get(), streamNumber);

    serverApp.Start(Seconds(0));
    serverApp.Stop(simulationTime + Seconds(1));
      
    for (std::size_t i = 0; i < nwifiSTA; i++)
    {
      OnOffHelper onoff("ns3::TcpSocketFactory", Ipv4Address::GetAny());
      onoff.SetAttribute("OnTime",
                         StringValue("ns3::ConstantRandomVariable[Constant=1]"));
      onoff.SetAttribute("OffTime",
                         StringValue("ns3::ConstantRandomVariable[Constant=0]"));
      onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
      onoff.SetAttribute("DataRate", DataRateValue(maxLoad));
      AddressValue remoteAddress(
          InetSocketAddress(serverInterfaces.GetAddress(i), port));
      onoff.SetAttribute("Remote", remoteAddress);
      ApplicationContainer clientApp = onoff.Install(clientNodes.Get(i));
      streamNumber += onoff.AssignStreams(clientNodes.Get(i), streamNumber);

      clientApp.Start(Seconds(1));
      clientApp.Stop(simulationTime + Seconds(1));
    }

    // cumulative number of bytes received by each server application
    std::vector<uint64_t> cumulRxBytes(nwifiSTA, 0);
    if(tputInterval.IsStrictlyPositive())
    {
        Simulator::Schedule(Seconds(1) + tputInterval,
                            &PrintIntermediateTput,
                            cumulRxBytes,
                            udp,
                            serverApp,
                            payloadSize,
                            tputInterval,
                            simulationTime + Seconds(1));
    }
    Simulator::Stop(simulationTime + Seconds(1));
    Simulator::Run();
  //-----------------------------------------
    auto tolerance = 0.10;
    cumulRxBytes = GetRxBytes(udp, serverApp, payloadSize);
    auto rxBytes = std::accumulate(cumulRxBytes.cbegin(), cumulRxBytes.cend(), 0.0);
    auto throughput = (rxBytes * 8) / simulationTime.GetMicroSeconds(); // Mbit/s
    
    Simulator::Destroy();

    std::cout << +mcs << "\t\t\t" << widthStr << " MHz\t\t"
              << (widthStr.size() > 3 ? "" : "\t") << guardInterval << " ns\t\t\t" << throughput
              << " Mbit/s" << std::endl;

  }

  return 0;
}

