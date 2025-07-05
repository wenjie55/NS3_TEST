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
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/waveform-generator-helper.h"
#include "ns3/waveform-generator.h"
#include "ns3/non-communicating-net-device.h"
#include "ns3/wifi-phy-common.h"
#include "ns3/wifi-mode.h"

#include <algorithm>
#include <array>
#include <functional>
#include <numeric>



using namespace ns3;


NS_LOG_COMPONENT_DEFINE("eht-wifi-network");

int64_t streamNumber = 100;

//struct
struct TxRecord{
    Ptr<Node> txNode;
    uint32_t nodeId;
    uint16_t channel;
    uint32_t linkId;
    Time startTime;
    Time endTime;
    Mac48Address srcMac; 
    std::string Type ;
    double BER;
    double Signal;
    double Noise;
};
struct DropTable{
    Mac48Address addr;
    Time startTime;
    Time endTime;
};
struct LinkTable
{
    uint32_t c_count = 0;
    uint32_t n_count = 0;
    uint32_t h_count = 0;
};
//global map
std::map<std::pair<uint32_t, Mac48Address>,TxRecord> g_activeTx;
std::vector<DropTable> StaDrop;
std::vector<LinkTable> apLinkTable(3);
std::vector<Time> lastStart(3);
std::vector<Time> lastEnd(3);
std::vector<Mac48Address> lastMacAddr(3);

uint32_t count = 0;
// static void cleanupTxRecord(uint16_t channel, Time now)
// {
//     if(g_activeTx.find(channel) == g_activeTx.end()) return;
//     auto &vec = g_activeTx[channel];

//     vec.erase(std::remove_if(vec.begin(),vec.end(),
//                             [now](const TxRecord& rec){ return rec.endTime < now;}),
//                             vec.end());
// }

//get Tx record
WifiPhyBand GetBandFromFreq(uint16_t freq)
{
        if (freq >= 2400 && freq < 2500)
        {
            return WifiPhyBand::WIFI_PHY_BAND_2_4GHZ;
        }
        else if (freq >= 5000 && freq < 6000)
        {
            return WifiPhyBand::WIFI_PHY_BAND_5GHZ;
        }
        else if (freq >= 5925 && freq <= 7125)
        {
            return  WifiPhyBand::WIFI_PHY_BAND_6GHZ;
        }
        else
        {
            NS_FATAL_ERROR("Unknown frequency: " << freq);
        }
}

static void PhyTxBeginTrace(Ptr<WifiNetDevice> dev,uint32_t linkId, Ptr<const Packet> packet, Watt_u txDbm)
{
    
    Ptr<Node> node = dev->GetNode();
    uint32_t nodeId = node->GetId();
    Ptr<WifiPhy> phy = dev->GetPhy(linkId);
    MHz_u allowedWidth = phy->GetChannelWidth();
    Ptr<WifiRemoteStationManager> StaManger = dev->GetRemoteStationManager();
    //channel
    uint16_t channelNum = phy->GetChannelNumber();
    // get header
    WifiMacHeader hdr;
    packet->PeekHeader(hdr);
    WifiTxVector txVector = StaManger->GetDataTxVector(hdr,allowedWidth);
    uint32_t pktSize = packet-> GetSize();
    WifiMode Mode = txVector.GetMode();
    Time duration = phy->CalculateTxDuration(pktSize,txVector,GetBandFromFreq(phy->GetFrequency()));
    
    // 記錄傳輸事件
    TxRecord rec;
    rec.txNode    = node;
    rec.nodeId    = nodeId;
    rec.channel   = channelNum;
    rec.linkId    = linkId;
    rec.startTime = Simulator::Now();
    rec.endTime   = Simulator::Now() + duration;
    rec.srcMac    = Mac48Address::ConvertFrom(dev->GetAddress());
    Mac48Address srcPHY = hdr.GetAddr2();
    rec.Type      = hdr.GetTypeString() ; 
    std::pair<uint32_t, Mac48Address> key = std::make_pair(linkId,srcPHY); //LinkId +PHYaddr -> Macaddr

    g_activeTx[key] = rec;
    
     
    // std::cout << "Type : " << hdr.GetTypeString() 
    //           << "  pktSize : " << pktSize <<" Byte " 
    //           << " \nStartTime : " << rec.startTime.GetSeconds()<< "s"
    //           << "  Duration : " << duration.GetMicroSeconds() << "us"
    //           << "  EndTime : " << rec.endTime.GetSeconds() << "s"
    //           << "\nchannelNum : " << channelNum
    //           << "  linkId : " << linkId
    //           << "  Freq : " << phy->GetFrequency()
    //           << "\nsrcMac : " << rec.srcMac
    //           << "  SrcPHY : " << srcPHY
    //           << "  dstMac : " << hdr.GetAddr1()
    //           <<"\n-------------------------------------------------------------"
    //           << std::endl;
}

void
// + linkId,
MyDropCallback(uint32_t linkId, Ptr<WifiNetDevice> dev, Ptr<const Packet> p, WifiPhyRxfailureReason reason)
{
    Time now = Simulator::Now();

    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    Mac48Address srcMac = hdr.GetAddr2(); //物理層address!!!! 非mac address!!!!
    std::pair<uint32_t, Mac48Address> key = std::make_pair(linkId, srcMac);
    


    auto it = g_activeTx.find(key);
    
    if( it != g_activeTx.end())
    {
        const TxRecord& cur = it->second;
        StaDrop.push_back({srcMac, cur.startTime, cur.endTime });
        if(std::abs((lastStart[linkId] - cur.startTime).GetMicroSeconds()) < 1)
        {
            if(count == 0 && (lastMacAddr[linkId]!= cur.srcMac))
            {
                apLinkTable[linkId].c_count += 1;
                // std::cout << " link :" << linkId << " collision ! " << " LastStart : " << lastStart[linkId].GetSeconds()
                //           << " StartTime : "<< cur.startTime.GetSeconds()  << "\n";
                count++;
            }
            else
            {
                count++;
            }

        }
        else
        {
            count = 0 ;
            if(lastEnd[linkId] > cur.startTime && std::abs((lastStart[linkId] - cur.startTime).GetMicroSeconds()) > 5)
            {
                if(lastMacAddr[linkId]!= cur.srcMac)
                {
                    apLinkTable[linkId].h_count += 1;
                    // std::cout  << " link :" << linkId << " hidden_node ! "
                    //            << " lastEnd : " << lastEnd[linkId].GetSeconds()
                    //            << " StartTime : "<< cur.startTime.GetSeconds()
                    //            << " hidenTime :" << apLinkTable[linkId].h_count << "\n";
                }
            }
            else if(lastMacAddr[linkId]!= cur.srcMac)
            {

            }
        }
        
        // std::cout << "srcMac :  "<< cur.srcMac 
        //           <<"\nnow : " << now.GetSeconds() <<"    startTime : " << cur.startTime.GetSeconds() << "    endTime : " <<cur.endTime.GetSeconds() 
        //           << "\nlink : " << linkId << " Type : "<< cur.Type << "  reason :  "  << reason  
        //           << " Beacon : "<< now.GetSeconds() / double(0.1) << "\n"
        //           << "---------------------------------------------------------------------------------------------------\n";
        lastStart[linkId]   = cur.startTime;
        lastEnd[linkId]     = cur.endTime;
        lastMacAddr[linkId] = cur.srcMac;
    }
                
}

void 
SnifferRxCallback(uint32_t linkId, uint32_t mcs, Ptr<const Packet> p, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise,uint16_t stdId)
{
    
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    Mac48Address srcMac = hdr.GetAddr2(); //物理層address!!!! 非mac address!!!!
    std::pair<uint32_t, Mac48Address> key = std::make_pair(linkId, srcMac);
    
    WifiMode mode = txVector.GetMode();
    DataRate dr = EhtPhy::GetDataRate(mcs,MHz_u{static_cast<double>(20)},NanoSeconds(800), 1);
    
    //BER Rate
    
    double signal = signalNoise.signal; // dBm
    double noise  = signalNoise.noise;  // dBm
    double snr_dB = signal - noise;
    double snr_linear = std::pow(10.0, snr_dB / 10.0);
    double EbNo = snr_linear * 20 * 1e6 / dr.GetBitRate();
    double z = std::sqrt((1.5 * log2(16) * EbNo) / (16 - 1.0));
    double z1 = ((1.0 - 1.0 / std::sqrt(16)) * erfc(z));
    double z2 = 1 - std::pow((1 - z1), 2);
    double BER = z2 / log2(16);
    double SUC = std::pow(700, (1.0 - BER));
    std::cout << "S : " << signal 
              << " N : " << noise 
              << " SNR_dB : " << snr_dB 
              << " SNR_L : " << snr_linear
              << " BER : " << BER 
              << " SUC : " << SUC
              << " DateRate : " << dr << "\n";


    auto it = g_activeTx.find(key);
    if(it != g_activeTx.end())
    {
        TxRecord& cur = it->second;
        cur.Noise = noise;
        cur.Signal =signal;
        cur.BER = BER ;
    }
    
}

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

NodeContainer CreateMultipleInterferers(std::string bandName, uint32_t numInterferers, double radius,
                                        double txPowerDbm, double startTime, double stopTime)
{
    NodeContainer interfererNodes;
    interfererNodes.Create(numInterferers);

    // 位置配置：分布在圓周上
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numInterferers; ++i)
    {
        double angle = 2 * M_PI * i / numInterferers;
        double x = radius * std::cos(angle);
        double y = radius * std::sin(angle);
        positionAlloc->Add(Vector(x, y, 0.0));
    }

    MobilityHelper mobility;
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(interfererNodes);
    FrequencyRange freqRange;
    // 設定 PHY / MAC
    if(bandName == "2.4GHz")
    {
        freqRange = WIFI_SPECTRUM_2_4_GHZ;
    }
    else if(bandName == "5GHz")
    {
        freqRange = WIFI_SPECTRUM_5_GHZ;
    }
    else if(bandName == "6GHz")
    {
        freqRange = WIFI_SPECTRUM_6_GHZ;
    }
    else
    {
        std::cout<<"error freq\n";
    }

    SpectrumWifiPhyHelper phyHelper;
    Ptr<MultiModelSpectrumChannel> InspectrumChannel = CreateObject<MultiModelSpectrumChannel>();
    phyHelper.SetErrorRateModel("ns3::YansErrorRateModel");
    phyHelper.Set("TxPowerStart", DoubleValue(txPowerDbm));
    phyHelper.Set("TxPowerEnd", DoubleValue(txPowerDbm));
    if(bandName == "2.4GHz")
    {
        phyHelper.Set("ChannelSettings", StringValue("{0, 20, BAND_2_4GHZ, 0}"));
    }
    else if(bandName == "5GHz")
    {
        phyHelper.Set("ChannelSettings", StringValue("{0, 20, BAND_5GHZ, 0}"));
    }
    else if(bandName == "6GHz")
    {
        phyHelper.Set("ChannelSettings", StringValue("{0, 20, BAND_6GHZ, 0}"));
    }
    
    auto lossModel = CreateObject<LogDistancePropagationLossModel>();
    InspectrumChannel->AddPropagationLossModel(lossModel);
    phyHelper.AddChannel(InspectrumChannel, freqRange);

    WifiHelper wifiHelper;
    wifiHelper.SetStandard(WIFI_STANDARD_80211be);
    WifiMacHelper macHelper;
    macHelper.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifiHelper.Install(phyHelper, macHelper, interfererNodes);

    // 網路協定 & IP 分配（實際上不需要真實通訊）
    InternetStackHelper stack;
    stack.Install(interfererNodes);
    Ipv4AddressHelper ipv4;
    if(bandName == "2.4GHz")
    {
        ipv4.SetBase("10.3.0.0", "255.255.255.0");    
    }
    else if(bandName == "5GHz")
    {
        ipv4.SetBase("10.4.0.0", "255.255.255.0");
    }
    else if(bandName == "6GHz")
    {
        ipv4.SetBase("10.5.0.0", "255.255.255.0");
    }
    ipv4.Assign(devices);

// 安裝 OnOff 應用產生干擾（傳給一個不存在的節點或廣播位址）
    for (uint32_t i = 0; i < interfererNodes.GetN(); ++i)
    {
        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress("255.255.255.255", 9999));
        std::ostringstream rate;
        Ptr<UniformRandomVariable> rateVar = CreateObject<UniformRandomVariable>();
        rateVar -> SetAttribute("Min",DoubleValue(10));
        rateVar ->SetAttribute("Max", DoubleValue(100));
        rate << static_cast<uint32_t>(rateVar->GetValue()) <<"Mbps";
        onoff.SetConstantRate(DataRate(rate.str())); 
       
        
        Ptr<UniformRandomVariable> starVar = CreateObject<UniformRandomVariable>();
        starVar -> SetAttribute("Min",DoubleValue(0.5));
        starVar -> SetAttribute("Max", DoubleValue(2.0));
        double start = starVar->GetValue();
        
        onoff.SetAttribute("StartTime", TimeValue(Seconds(start))); 
        
        onoff.SetAttribute("OnTime", StringValue("ns3::UniformRandomVariable[Mu=0.0|Sigma=1.0][Min=0.002|Max=2.0])")); 
        onoff.SetAttribute("OffTime", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=2.0])"));
    
        onoff.Install(interfererNodes.Get(i));
    }

    return interfererNodes;
}

NetDeviceContainer CreateInterferers(std::string bandName, uint32_t numInterferers, double radius, 
                                Watt_u waveformPower,std::string freqSet)
{
    uint16_t freqRange = 5180;
    BandInfo bandInfo;
    Bands bands;
    Ptr<SpectrumModel>SpectrumBand ;
    if (bandName == "2.4GHz")
    {
        
        freqRange = 2412;
        bandInfo.fc = 2412e6;
        bandInfo.fl = 2412e6 - 10e6;
        bandInfo.fh = 2412e6 + 10e6;
        bands.push_back(bandInfo);
        SpectrumBand = Create<SpectrumModel>(bands);
    }
    else if (bandName == "5GHz")
    {
        freqRange = 5180;
        bandInfo.fc = 5180e6;
        bandInfo.fl = 5180e6 - 10e6;
        bandInfo.fh = 5180e6 + 10e6;
        bands.push_back(bandInfo);
        SpectrumBand = Create<SpectrumModel>(bands);

    }
    else if (bandName == "6GHz")
    {
        freqRange = 5965;
        bandInfo.fc = 5965e6;
        bandInfo.fl = 5965e6 - 10e6;
        bandInfo.fh = 5965e6 + 10e6;
        bands.push_back(bandInfo);
        SpectrumBand = Create<SpectrumModel>(bands);
    }
        
    
    NodeContainer interfererNodes;
    interfererNodes.Create(numInterferers);
    
    SpectrumWifiPhyHelper spectrumPhy;
    Ptr<MultiModelSpectrumChannel> SpectrumChannel = CreateObject<MultiModelSpectrumChannel>();
    Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
    lossModel->SetFrequency(freqRange * 1e6);
    SpectrumChannel->AddPropagationLossModel(lossModel);
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =CreateObject<ConstantSpeedPropagationDelayModel>();
    SpectrumChannel->SetPropagationDelayModel(delayModel);

    spectrumPhy.SetChannel(SpectrumChannel);
    spectrumPhy.SetErrorRateModel("ns3::NistErrorRateModel");
    spectrumPhy.Set("ChannelSettings",StringValue(freqSet));
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numInterferers; ++i)
    {
        double angle = 2 * M_PI * i / numInterferers;
        double x = radius * std::cos(angle);
        double y = radius * std::sin(angle);
        positionAlloc->Add(Vector(x, y, 0.0));
    }
    MobilityHelper mobility;
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(interfererNodes);
    // Ptr<LogNormalRandomVariable> InRv = CreateObject<LogNormalRandomVariable>();
    // InRv->SetAttribute("Mu",DoubleValue(std::log(waveformPower)));
    // InRv->SetAttribute("sigma",DoubleValue(1.5));
    // double noistPower = InRv->GetValue();
    Ptr<SpectrumValue> wgPsd = Create<SpectrumValue>(SpectrumBand);
    *wgPsd = waveformPower/20e6;
    
    WaveformGeneratorHelper waveformGeneratorHelper;
    waveformGeneratorHelper.SetChannel(SpectrumChannel);
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd);

    waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(0.0007)));
    waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(1));
    NetDeviceContainer waveformGeneratorDevices = waveformGeneratorHelper.Install(interfererNodes);

    return waveformGeneratorDevices;
}

ApplicationContainer CreateClientFlow(UintegerValue ACLevel, UintegerValue payloadSize, uint64_t DataRate, InetSocketAddress dest, Ptr<Node> STA,
                                      DoubleValue ontimeStart, DoubleValue onTimeEnd, DoubleValue offTimeStart, DoubleValue offTimeEnd)
{
  //{0x70, 0x28, 0xb8, 0xc0}; // AC_BE, AC_BK, AC_VI, AC_VO
  OnOffHelper onoff("ns3::UdpSocketFactory", dest);
  onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
  
  onoff.SetAttribute("DataRate", DataRateValue(DataRate * 1e6));
  onoff.SetAttribute("Tos",UintegerValue(ACLevel));
  
  Ptr<UniformRandomVariable> onTime = CreateObject<UniformRandomVariable>();
  onTime -> SetAttribute("Min",DoubleValue(ontimeStart));
  onTime -> SetAttribute("Max",DoubleValue(onTimeEnd));
  onoff.SetAttribute("OnTime", PointerValue(onTime));

  Ptr<UniformRandomVariable> offTime = CreateObject<UniformRandomVariable>();
  offTime -> SetAttribute("Min",DoubleValue(offTimeStart));
  offTime -> SetAttribute("Max",DoubleValue(offTimeEnd));

  onoff.SetAttribute("OffTime", PointerValue(offTime));

  onoff.SetAttribute("Remote",AddressValue(dest));
  ApplicationContainer clientApp = onoff.Install(STA);
  streamNumber += onoff.AssignStreams(STA,streamNumber);
  //std::cout << "Install App on STA: " << STA->GetDevice(0)->GetAddress() << std::endl;
  return clientApp;
}

void 
MyRxCallback(Ptr<const WifiPsdu> psdu, RxSignalInfo info, const WifiTxVector &txvector,const std::vector<bool> &mpduStatus)
{
    std::cout << "packet get SNR: " << info.snr <<"dB"<< std::endl;
}
// packet receive

void
RxDropCallback(Ptr<const Packet> p, WifiPhyRxfailureReason reason)
{
    std::cout << "----------------------------" << "\n";
    //Happen time
    std::cout << "[PHY DROP time]: "
              << Simulator::Now().GetSeconds()<<"s";
    std::cout <<"\n[RxDrop] Packet UID = "<< p->GetUid() <<", Reason = " << reason <<"\n"<< std::endl;
    
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

int main(int argc, char* argv[])
{
    bool udp{true};
    bool intterf(false);
    bool useRts{true};
    bool flowinfo{false};
    bool STAaddr_linkid(false);
    uint16_t mpduBufferSize{512};
    //TXOP
    //bool TXOPsystem{true};
    std::string txopLimit{"3200us,3200us,3200us"};
    //OFDMA
    //bool OFDMAsystem(false);
    //EMLSR
    std::string emlsrMgrTypeId{"ns3::DefaultEmlsrManager"};
    std::string emlsrLinks{"0,1,2"}; //3link is EMLSR
    uint16_t paddingDelayUsec{32};
    uint16_t transitionDelayUsec{128};
    Time channelSwitchDelay{"150us"};
    bool switchAuxPhy{false};//Aux Phy wait primary phy finish
    uint16_t auxPhyChWidth{20};
    bool auxPhyTxCapable{true}; // Can UPlink ?

    Time simulationTime{"5s"};
    //simulation time (interference time)
    // double simStartTime = 1.0;

    double frequency{2.4};  // whether the first link operates in the 2.4, 5 or 6 GHz
    double frequency2{5}; // whether the second link operates in the 2.4, 5 or 6 GHz (0 means no
                          // second link exists)
    double frequency3{6}; // whether the third link operates in the 2.4, 5 or 6 GHz (0 means no third link exists)
    
    std::size_t nStations{5};
    std::string dlAckSeqType{"NO-OFDMA"};
    bool enableUlOfdma{false};
    bool enableBsrp{false};
    std::string mcsStr;
    std::vector<uint64_t> mcsValues;
    uint32_t payloadSize = 700; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
    Time tputInterval{0}; // interval for detailed throughput measurement
    Time accessReqInterval{0};

    // BSS size
    double maxRadius = 50.0;

    CommandLine cmd(__FILE__);
    //wait->connection Drop
    
    cmd.Parse(argc, argv);

    //Open RTS/CTS
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
    //----------------------------------------
    uint8_t mcs = 4;
    mcsValues.push_back(mcs); 
    
    int minChannelWidth = 20;
    int maxChannelWidth = 20;
    
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
                NodeContainer wifiAPNode;
                NetDeviceContainer apDevice;
                wifiAPNode.Create(1);

                NodeContainer wifiStaNodes;
                NetDeviceContainer staDevices;
                wifiStaNodes.Create(nStations);

                
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
                                                     "ns3::ConstantRateWifiManager",//Can't change Rate  for SNR
                                                     "DataMode",StringValue(dataModeStr),
                                                     "ControlMode",StringValue("EhtMcs0"));
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
                phy.Set("TxPowerStart", DoubleValue(20.0));
                phy.Set("TxPowerEnd", DoubleValue(20.0));
                phy.Set("TxPowerLevels", UintegerValue(1));
            //error model
                phy.SetErrorRateModel("ns3::YansErrorRateModel");
                
            //EMlsr     
                mac.SetType("ns3::StaWifiMac", 
                            "Ssid", SsidValue(ssid),
                            "QosSupported", BooleanValue(true),
                            "ActiveProbing",BooleanValue(false));

                // wifi.ConfigEhtOptions("EmlsrActivated",BooleanValue(false));

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

            //channel model install to link i  
                
                for (uint8_t linkId = 0; linkId < nLinks; linkId++)
                {
                    phy.Set(linkId, "ChannelSettings", StringValue(channelStr[linkId]));
                    auto spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
                    
                    //signal loss model - have value ->hidden node
                    auto lossModel = CreateObject<LogDistancePropagationLossModel>();
                    // give channel my lossModel
                    // auto delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
                    // spectrumChannel->SetPropagationDelayModel(delayModel);

                    spectrumChannel->AddPropagationLossModel(lossModel);
                    phy.AddChannel(spectrumChannel, freqRanges[linkId]);
                    // std::cout << channelStr[linkId] << std::endl;
                    // std::cout << freqRanges[linkId]<< std::endl;
                }
            //install STA
                staDevices = wifi.Install(phy, mac, wifiStaNodes);
            //Open TXOP System
            
                
                Ptr<NetDevice> dev;
                Ptr<WifiNetDevice>wifi_dev;
                PointerValue ptr;
                Ptr<QosTxop> edca;

                for(uint32_t i = 0; i < nStations;i++)
                {
                    //open A-MPDU
                    dev = wifiStaNodes.Get(i)->GetDevice(0);
                    wifi_dev = DynamicCast<WifiNetDevice>(dev);
                    wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(32768));
                    wifi_dev->GetMac()->SetAttribute("BE_MaxAmsduSize", UintegerValue(3839));
                }
                
                
                
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
                
            //install AP
                mac.SetType("ns3::ApWifiMac",
                            "EnableBeaconJitter",BooleanValue(false),
                            "QosSupported", BooleanValue(true),
                            "Ssid",SsidValue(ssid));

            // Modify EDCA
                mac.SetEdca(AC_BE, "TxopLimits", StringValue(txopLimit));
                mac.SetEdca(AC_BK, "TxopLimits", StringValue(txopLimit));
                mac.SetEdca(AC_VO, "TxopLimits", StringValue(txopLimit));
                mac.SetEdca(AC_VI,"TxopLimits", StringValue(txopLimit));
                phy.Set("TxPowerStart", DoubleValue(70.0));
                phy.Set("TxPowerEnd", DoubleValue(70.0));
                phy.Set("TxPowerLevels", UintegerValue(1));
                phy.Set("TxGain", DoubleValue(0.0));
                phy.Set("RxGain", DoubleValue(0.0));
                apDevice = wifi.Install(phy, mac, wifiAPNode);

                dev = wifiAPNode.Get(0)->GetDevice(0);
                wifi_dev = DynamicCast<WifiNetDevice>(dev);
                wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(32768));
                wifi_dev->GetMac()->SetAttribute("BE_MaxAmsduSize", UintegerValue(3839));
                //can use to get bit,
                wifi_dev->GetMac()->GetAttribute("BE_Txop", ptr);
                edca = ptr.Get<QosTxop>();
                
            //random number
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
                mobilityAP = InstallApMove(wifiAPNode);
            //interferer
            if(intterf == true)
            {
                Ptr<SpectrumWifiPhy> interPhy;
                double interfererPower = 1e-12;
                u_int32_t interferNum = 3;
                //time
                double simStopTIme = 5.0;

                NetDeviceContainer interferers2_4 = CreateInterferers("2.4GHz",interferNum, maxRadius,interfererPower,channelStr[0]);
                NetDeviceContainer interferers5 = CreateInterferers("5GHz",interferNum, maxRadius,interfererPower,channelStr[1]);
                NetDeviceContainer interferers6 = CreateInterferers("6GHz",interferNum, maxRadius,interfererPower,channelStr[2]);
                
                Ptr<UniformRandomVariable> onTimeVar = CreateObject<UniformRandomVariable>();
                onTimeVar -> SetAttribute("Min",DoubleValue(0.02));
                onTimeVar -> SetAttribute("Max",DoubleValue(1.0));

                Ptr<UniformRandomVariable> offTimeVar = CreateObject<UniformRandomVariable>();
                offTimeVar -> SetAttribute("Min",DoubleValue(0.5));
                offTimeVar -> SetAttribute("Max",DoubleValue(1.0));

                for(u_int32_t i = 0; i < interferNum; i++)
                {
                double currentTime2_4 = 0.0;
                while(currentTime2_4 < simStopTIme)
                {
                    double onTime = onTimeVar-> GetValue();
                    double offTime = offTimeVar -> GetValue();
                     Simulator::Schedule(Seconds(onTime),
                                        &WaveformGenerator::Start,
                                        interferers2_4.Get(i)
                                            ->GetObject<NonCommunicatingNetDevice>()
                                            ->GetPhy()
                                            ->GetObject<WaveformGenerator>());

                    Simulator::Schedule(Seconds(currentTime2_4 + onTime),
                                        &WaveformGenerator::Stop,
                                        interferers2_4.Get(i)
                                            ->GetObject<NonCommunicatingNetDevice>()
                                            ->GetPhy()
                                            ->GetObject<WaveformGenerator>());
                    currentTime2_4 += (onTime + offTime);
                }    
                double currentTime5 = 0;
                while(currentTime5 < simStopTIme)
                {
                    double onTime = onTimeVar-> GetValue();
                    double offTime = offTimeVar -> GetValue();
                    Simulator::Schedule(Seconds(currentTime5),&WaveformGenerator::Start,
                                        interferers5.Get(i)
                                            ->GetObject<NonCommunicatingNetDevice>()
                                            ->GetPhy()
                                            ->GetObject<WaveformGenerator>());

                    Simulator::Schedule(Seconds(currentTime5 + onTime),&WaveformGenerator::Stop,
                                        interferers5.Get(i)
                                            ->GetObject<NonCommunicatingNetDevice>()
                                            ->GetPhy()
                                            ->GetObject<WaveformGenerator>());
                    currentTime5 += (onTime + offTime);
                }
                double currentTime6 = 0;
                while(currentTime6 < simStopTIme)
                {
                    double onTime = onTimeVar-> GetValue();
                    double offTime = offTimeVar -> GetValue();
                    Simulator::Schedule(Seconds(currentTime6),&WaveformGenerator::Start,
                                        interferers6.Get(i)
                                            ->GetObject<NonCommunicatingNetDevice>()
                                            ->GetPhy()
                                            ->GetObject<WaveformGenerator>());
                    Simulator::Schedule(Seconds(currentTime6 + onTime),&WaveformGenerator::Stop,
                                        interferers6.Get(i)
                                            ->GetObject<NonCommunicatingNetDevice>()
                                            ->GetPhy()
                                            ->GetObject<WaveformGenerator>());
                    currentTime6 += (onTime + offTime);
                }
                }
            } 
            /* Internet stack*/
                InternetStackHelper stack;
                stack.Install(wifiAPNode);
                stack.Install(wifiStaNodes);
                streamNumber += stack.AssignStreams(wifiAPNode, streamNumber);
                streamNumber += stack.AssignStreams(wifiStaNodes, streamNumber);
         
            //Assign address
                Ipv4AddressHelper address;
                address.SetBase("192.168.1.0", "255.255.255.0");
                Ipv4InterfaceContainer apNodeInterface = address.Assign(apDevice);
                Ipv4InterfaceContainer staNodeInterfaces = address.Assign(staDevices);
         

             /* Setting applications */
                 
                ApplicationContainer serverApp;
                auto serverNodes = std::ref(wifiAPNode);
                Ipv4InterfaceContainer serverInterfaces;

                NodeContainer clientNodes;
                
                for (std::size_t i = 0; i < nStations; ++i)
                {
                    serverInterfaces.Add(apNodeInterface.Get(0));
                    clientNodes.Add(wifiStaNodes.Get(i));
                }
                
                const auto maxLoad = nLinks *EhtPhy::GetDataRate(mcs,MHz_u{static_cast<double>(width)},NanoSeconds(gi), 1) / nStations;

                if (udp)
                {
                    //server & Client connect 
                    uint16_t port = 9;
                    UdpServerHelper server(port);
                    serverApp = server.Install(serverNodes.get());
                    streamNumber += server.AssignStreams(serverNodes.get(), streamNumber);
                    //server open time
                    serverApp.Start(Seconds(0));
                    serverApp.Stop(simulationTime + Seconds(1));
                    //const auto packetInterval = payloadSize * 8.0 / maxLoad;
                    InetSocketAddress dest(serverInterfaces.GetAddress(0),port);

                    for (std::size_t i = 0; i < nStations; i++)
                    {   
                        AddressValue remoteAddress(dest);
                        
                        //{0x70, 0x28, 0xb8, 0xc0}; // AC_BE, AC_BK, AC_VI, AC_VO
                        //BE
                        ApplicationContainer clientAppBE = CreateClientFlow(0x70, payloadSize,nonHtRefRateMbps,dest,clientNodes.Get(i),
                                                                            0.015,0.025,0,0.005);
                        clientAppBE.Start(Seconds(1.0 ) + MicroSeconds(i * 100));
                        clientAppBE.Stop(simulationTime + Seconds(1));
                        //BK
                        ApplicationContainer clientAppBK = CreateClientFlow(0x28, payloadSize,nonHtRefRateMbps,dest,clientNodes.Get(i),
                                                                            0.03,0.05,0,0.01);
                        clientAppBE.Start(Seconds(1.0 ) + MicroSeconds(i * 100));
                        clientAppBE.Stop(simulationTime + Seconds(1));
                        // //VI
                        // ApplicationContainer clientAppVI = CreateClientFlow(0xb8, payloadSize,nonHtRefRateMbps,dest,clientNodes.Get(i),
                        //                                                     0.2,1,0.1,2); 
                        // clientAppVI.Start(Seconds(1.0) + MicroSeconds(i * 100));
                        // clientAppVI.Stop(simulationTime + Seconds(1));
                        // // //VO
                        // ApplicationContainer clientAppVO = CreateClientFlow(0xc0, payloadSize-100,nonHtRefRateMbps,dest,clientNodes.Get(i),
                        //                                                     0.5,1,1,2);
                        // clientAppVO.Start(Seconds(1.0 ) + MicroSeconds(i * 100));
                        // clientAppVO.Stop(simulationTime + Seconds(1));
                    //test
                        // serverApp.Get(0)->TraceConnect("Rx", "", MakeCallback(&RxTrace));
                        // Ptr<WifiNetDevice> dev = staDevices.Get(0)->GetObject<WifiNetDevice>();
                        // Ptr<WifiPhy> pp = dev->GetPhy();
                        // Ptr<YansErrorRateModel> yans =DynamicCast<YansErrorRateModel>(err);
                        // pp->SetReceiveOkCallback(MyRxCallback);


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
        //--------------------------system info and function(my)--------------------------------------
        
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
            // flow information monitor (need to install before simulator::Run() )
                FlowMonitorHelper flowmon;
                Ptr<FlowMonitor> monitor = flowmon.InstallAll();
                

            // STA掛勾三條link是否成功
            if(STAaddr_linkid == true)
            {
                for(u_int32_t i = 0; i < nStations ;++i)
                {
                    const Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(staDevices.Get(i));
                    Ptr<WifiMac> mac = mloDevice->GetMac();
                    for(uint8_t Linkid = 0; Linkid <std::max<uint8_t>(mloDevice->GetNPhys(),1);++Linkid)
                    {
                        auto fem = mac->GetFrameExchangeManager(Linkid); //mac of limk i
                        std::cout<<" staDevice " << i << " LinkId " << std::to_string(Linkid) 
                                 << " mac address: " << fem->GetAddress() <<"\n";
                    }

                }
            }
            //trace

            //物理層傳送的封包資訊
            for(uint32_t i = 0; i < nStations; ++i)
            {
               const Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(staDevices.Get(i));
               for(uint32_t j = 0; j < nLinks; j++)
               {
                   Ptr<WifiPhy> phy = mloDevice->GetPhy(j);
                   phy->TraceConnectWithoutContext("PhyTxBegin",MakeBoundCallback(&PhyTxBeginTrace,mloDevice,j));
                   phy->TraceConnectWithoutContext("PhyRxDrop", MakeBoundCallback(&MyDropCallback,j,mloDevice));

               }   
            }

            //物理層錯誤查詢
            for(uint32_t i = 0; i < nLinks; ++i)
            {
                const Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(apDevice.Get(0));
                Ptr<WifiPhy> phy = mloDevice->GetPhy(i);
                phy->TraceConnectWithoutContext("PhyRxDrop", MakeBoundCallback(&MyDropCallback,i,mloDevice));
                phy->TraceConnectWithoutContext("MonitorSnifferRx",MakeBoundCallback(&SnifferRxCallback,i,mcs));
            }

               
            //start--------------------------------------
                Simulator::Stop(simulationTime + Seconds(1)); //end time (need define before simulator::Run())
                Simulator::Run();

            //throughput calulate
            
                cumulRxBytes = GetRxBytes(udp, serverApp, payloadSize);
                auto rxBytes = std::accumulate(cumulRxBytes.cbegin(), cumulRxBytes.cend(), 0.0);
                auto throughput = (rxBytes * 8) / simulationTime.GetMicroSeconds(); // Mbit/s

                Simulator::Destroy();
                std::cout << +mcs << "\t\t\t" << widthStr << " MHz\t\t"
                          << (widthStr.size() > 3 ? "" : "\t") << gi << " ns\t\t\t" << throughput
                          << " Mbit/s" << std::endl;
                          

            //monitor get loss packets info
                monitor->CheckForLostPackets(); 

                Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier()); 
                FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();//Get Flow info TX/RX packets /delaySUm /lose packet /throughput
                if(flowinfo==true)
                {
                    for (auto i = stats.begin(); i != stats.end(); ++i)
                    {
                        //i->first : FlowId , i -> second || i->second : packet information
                            Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first); //Five Tuple (address information)
                            std::cout << "Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
                            std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
                            std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
                            std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / 9.0 / 1000 / 1000  //throughput caculate
                                    << " Mbps\n";
                            // std::cout << "  Avg Delay:  " << i->second.delaySum / i->second.rxPackets << "\n";
                            // std::cout << "  Packet loss " << i->second.lostPackets << " / " << i->second.txPackets << " = "
                            //         << (double)i->second.lostPackets/(double)i->second.txPackets *100 <<"\n";
                            std::cout << "----------------------------------------------------------------"<<"\n";
                            std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
                            std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
                            std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / 9.0 / 1000 / 1000
                                    << " Mbps\n";
                    }  
                } 
    
            }
        }
    }
    return 0;
}
