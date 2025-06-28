/*
 * Copyright (c) 2022
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Sebastien Deronne <sebastien.deronne@gmail.com>
 */

#include "ns3/attribute-container.h"
#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/eht-phy.h"
#include "ns3/enum.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/udp-server.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"

#include "ns3/spectrum-model.h"
#include "ns3/spectrum-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"

#include <algorithm>
#include <array>
#include <functional>
#include <numeric>



using namespace ns3;

NS_LOG_COMPONENT_DEFINE("eht-wifi-network");


//func
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
// packet receive
void RxTrace(std::string context, Ptr<const Packet> packet)
{
    std::cout << Simulator::Now().GetSeconds() << "s: Packet received" << std::endl;
}
// sta SNR
void MyRxCallback(Ptr<const WifiPsdu> psdu, RxSignalInfo info, const WifiTxVector &txvector,const std::vector<bool> &mpduStatus)
{
    std::cout << "packet get SNR: " << info.snr <<"dB"<< std::endl;
}


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

int
main(int argc, char* argv[])
{
    bool udp{true};
    bool downlink{false};
    bool useRts{false};
    uint16_t mpduBufferSize{512};
    std::string emlsrMgrTypeId{"ns3::DefaultEmlsrManager"};
    std::string emlsrLinks;
    uint16_t paddingDelayUsec{32};
    uint16_t transitionDelayUsec{128};
    Time channelSwitchDelay{"100us"};
    bool switchAuxPhy{true};
    uint16_t auxPhyChWidth{20};
    bool auxPhyTxCapable{true};
    Time simulationTime{"10s"};
    
    double frequency{5};  // whether the first link operates in the 2.4, 5 or 6 GHz
    double frequency2{0}; // whether the second link operates in the 2.4, 5 or 6 GHz (0 means no
                          // second link exists)
    double frequency3{0}; // whether the third link operates in the 2.4, 5 or 6 GHz (0 means no third link exists)
    
    std::size_t nStations{1};
    std::string dlAckSeqType{"NO-OFDMA"};
    bool enableUlOfdma{false};
    bool enableBsrp{false};
    std::string mcsStr;
    std::vector<uint64_t> mcsValues;
    uint32_t payloadSize = 700; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
    Time tputInterval{0}; // interval for detailed throughput measurement
    Time accessReqInterval{0};

    //move 
    double maxRadius = 45.0;

    CommandLine cmd(__FILE__);
    
    cmd.Parse(argc, argv);

    if (useRts)
    {
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("0"));
        Config::SetDefault("ns3::WifiDefaultProtectionManager::EnableMuRts", BooleanValue(true));
    }

    if (dlAckSeqType == "ACK-SU-FORMAT")
    {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_BAR_BA_SEQUENCE));
    }
    else if (dlAckSeqType == "MU-BAR")
    {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_TF_MU_BAR));
    }
    else if (dlAckSeqType == "AGGR-MU-BAR")
    {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_AGGREGATE_TF));
    }
    else if (dlAckSeqType != "NO-OFDMA")
    {
        NS_ABORT_MSG("Invalid DL ack sequence type (must be NO-OFDMA, ACK-SU-FORMAT, MU-BAR or "
                     "AGGR-MU-BAR)");
    }
    
    std::cout << "MCS value"
              << "\t\t"
              << "Channel width"
              << "\t\t"
              << "GI"
              << "\t\t\t"
              << "Throughput" << '\n';
    uint8_t minMcs = 4;
    uint8_t maxMcs = 4;

    for (uint8_t mcs = minMcs; mcs <= maxMcs; ++mcs)
    {
        mcsValues.push_back(mcs);
    }
    
    int minChannelWidth = 20;
    int maxChannelWidth = 40;
    
    int minGi = 800;
    int maxGi = 800;
   


    for (const auto mcs : mcsValues)
    {
        for (int width = minChannelWidth; width <= maxChannelWidth; width *= 2) // MHz
        {
            
            const std::string widthStr = std::to_string(width);
            const auto segmentWidthStr = widthStr;
            for (int gi = maxGi; gi >= minGi; gi /= 2) // Nanoseconds
            {
                if (!udp)
                {
                    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(payloadSize));
                }
            // WiFi Node Create
                NodeContainer wifiStaNodes;
                wifiStaNodes.Create(nStations);
                NodeContainer wifiApNode;
                wifiApNode.Create(1);
                NetDeviceContainer apDevice;
                NetDeviceContainer staDevices;
                WifiMacHelper mac;
                WifiHelper wifi;
           
                
                
                wifi.SetStandard(WIFI_STANDARD_80211be);
                std::array<std::string, 3> channelStr;
                std::array<FrequencyRange, 3> freqRanges;
                uint8_t nLinks = 0;
                std::string dataModeStr = "EhtMcs" + std::to_string(mcs);
                std::string ctrlRateStr;
                uint64_t nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs) / 1e6;

                if (frequency2 == frequency || frequency3 == frequency ||
                    (frequency3 != 0 && frequency3 == frequency2))
                {
                    NS_FATAL_ERROR("Frequency values must be unique!");
                }

                for (auto freq : {frequency, frequency2, frequency3})
                {
                    if (nLinks > 0 && freq == 0)
                    {
                        break;
                    }
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
                    else
                    {
                        NS_FATAL_ERROR("Wrong frequency value!");
                    }
                    nLinks++;
                }

                
                Ssid ssid = Ssid("ns3-80211be");

            //PHY
                SpectrumWifiPhyHelper phy(nLinks);
                phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
                phy.Set("ChannelSwitchDelay", TimeValue(channelSwitchDelay));
            //error model
                phy.SetErrorRateModel("ns3::YansErrorRateModel");


                mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
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
            //channel model   
                
                for (uint8_t linkId = 0; linkId < nLinks; linkId++)
                {
                    phy.Set(linkId, "ChannelSettings", StringValue(channelStr[linkId]));


                    auto spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
                    
                    
                    //signal loss model - have value ->get ptr:lossmodel
                    auto lossModel = CreateObject<LogDistancePropagationLossModel>();
                    
                    //add error  channel install lossmodel
                    spectrumChannel->AddPropagationLossModel(lossModel);
                    
                    phy.AddChannel(spectrumChannel, freqRanges[linkId]);
                }

            //install STA
                staDevices = wifi.Install(phy, mac, wifiStaNodes);

            //OFDMA(wait)
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
                mac.SetType("ns3::ApWifiMac",
                            "EnableBeaconJitter",
                            BooleanValue(false),
                            "Ssid",
                            SsidValue(ssid));
            //install AP
                apDevice = wifi.Install(phy, mac, wifiApNode);

                int64_t streamNumber = 100;
                streamNumber += WifiHelper::AssignStreams(apDevice, streamNumber);
                streamNumber += WifiHelper::AssignStreams(staDevices, streamNumber);

            // Set guard interval and MPDU buffer size
                Config::Set(
                    "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                    TimeValue(NanoSeconds(gi)));
                Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MpduBufferSize",
                            UintegerValue(mpduBufferSize));

            // mobility
                MobilityHelper mobilitySTA;
                MobilityHelper mobilityAP;
                
                mobilitySTA =InstallStaMove(maxRadius,nStations,wifiStaNodes);
                mobilityAP = InstallApMove(wifiApNode);

            /* Internet stack*/
                InternetStackHelper stack;
                stack.Install(wifiApNode);
                stack.Install(wifiStaNodes);
                streamNumber += stack.AssignStreams(wifiApNode, streamNumber);
                streamNumber += stack.AssignStreams(wifiStaNodes, streamNumber);

                Ipv4AddressHelper address;
                address.SetBase("192.168.1.0", "255.255.255.0");
                Ipv4InterfaceContainer staNodeInterfaces;
                Ipv4InterfaceContainer apNodeInterface;

                staNodeInterfaces = address.Assign(staDevices);
                apNodeInterface = address.Assign(apDevice);

             /* Setting applications */
                ApplicationContainer serverApp;
                auto serverNodes = downlink ? std::ref(wifiStaNodes) : std::ref(wifiApNode);
                Ipv4InterfaceContainer serverInterfaces;
                NodeContainer clientNodes;
                for (std::size_t i = 0; i < nStations; i++)
                {
                    serverInterfaces.Add(downlink ? staNodeInterfaces.Get(i)
                                                  : apNodeInterface.Get(0));
                    clientNodes.Add(downlink ? wifiApNode.Get(0) : wifiStaNodes.Get(i));
                }

                const auto maxLoad = nLinks *
                                     EhtPhy::GetDataRate(mcs,
                                                         MHz_u{static_cast<double>(width)},
                                                         NanoSeconds(gi),
                                                         1) /
                                     nStations;
                if (udp)
                {
                    // UDP flow
                    uint16_t port = 9;
                    UdpServerHelper server(port);
                    serverApp = server.Install(serverNodes.get());
                    streamNumber += server.AssignStreams(serverNodes.get(), streamNumber);

                    serverApp.Start(Seconds(0));
                    serverApp.Stop(simulationTime + Seconds(1));
                    const auto packetInterval = payloadSize * 8.0 / maxLoad;

                    for (std::size_t i = 0; i < nStations; i++)
                    {
                        UdpClientHelper client(serverInterfaces.GetAddress(i), port);
                        client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
                        client.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
                        client.SetAttribute("PacketSize", UintegerValue(payloadSize));
                        ApplicationContainer clientApp = client.Install(clientNodes.Get(i));
                        streamNumber += client.AssignStreams(clientNodes.Get(i), streamNumber);

                        clientApp.Start(Seconds(1));
                        clientApp.Stop(simulationTime + Seconds(1));
                        
                    //test
                        //serverApp.Get(0)->TraceConnect("Rx", "", MakeCallback(&RxTrace));
                        //Ptr<WifiNetDevice> dev = staDevices.Get(0)->GetObject<WifiNetDevice>();
                        //Ptr<WifiPhy> pp = dev->GetPhy();
                        //Ptr<YansErrorRateModel> yans =DynamicCast<YansErrorRateModel>(err);
                        //pp->SetReceiveOkCallback(MyRxCallback);


                    }
                }
                else
                {
                    // TCP flow
                    uint16_t port = 50000;
                    Address localAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
                    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", localAddress);
                    serverApp = packetSinkHelper.Install(serverNodes.get());
                    streamNumber += packetSinkHelper.AssignStreams(serverNodes.get(), streamNumber);

                    serverApp.Start(Seconds(0));
                    serverApp.Stop(simulationTime + Seconds(1));

                    for (std::size_t i = 0; i < nStations; i++)
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
                }
    
            // cumulative number of bytes received by each server application
                std::vector<uint64_t> cumulRxBytes(nStations, 0);

                if (tputInterval.IsStrictlyPositive())
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

            //throughput calulate
                cumulRxBytes = GetRxBytes(udp, serverApp, payloadSize);
                auto rxBytes = std::accumulate(cumulRxBytes.cbegin(), cumulRxBytes.cend(), 0.0);
                auto throughput = (rxBytes * 8) / simulationTime.GetMicroSeconds(); // Mbit/s

                Simulator::Destroy();
            
                std::cout << +mcs << "\t\t\t" << widthStr << " MHz\t\t"
                          << (widthStr.size() > 3 ? "" : "\t") << gi << " ns\t\t\t" << throughput
                          << " Mbit/s" << std::endl;
            }
        }
    }
    return 0;
}
