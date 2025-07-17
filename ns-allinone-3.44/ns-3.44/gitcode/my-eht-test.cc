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
    double signaldBm =-65.2;
    double noisedBm =-85.5;
};


//my function use table & count &
struct LinkTable
{
    uint32_t B_cnt = 0 ; //HCCA open time
    uint32_t c_count = 0;
    uint32_t n_count = 0;
    uint32_t h_count = 0;
    uint32_t s_count = 0;
    uint32_t unknown = 0;
    uint32_t total_count = 0;
    //hidden node :
    std::map<Mac48Address,uint32_t> hideTable;
    std::vector<std::pair<uint32_t,uint32_t>> HN_list; //STAID ,QueueSize
    std::queue<uint32_t> HN_Queue;

    // noise :
    std::map<Mac48Address,uint32_t> NoiseTable;
    std::vector<Mac48Address> Noise_list;
    std::unordered_set<int32_t> NoiseStop;

    bool NoiseFlag = false;
    //collision :
    bool HCCA = false;
    double Pab = 1;
};

//global
std::map<std::pair<uint32_t, Mac48Address>,TxRecord> g_activeTx;
std::vector<LinkTable> apLinkTable(3); //count 
std::vector<Time> lastStart(3);
std::vector<Time> lastEnd(3);
std::vector<Mac48Address> lastMacAddr(3);
std::map<Mac48Address,uint32_t> STAlinkTable; // phylink -> STA Id


double GlobalBER = 0;
double GlobalSUC = 0;
Time beaconInterval = MilliSeconds (500);
double HN_threshold = 0.05;

//my function 
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

//Get  STA : TX startTime & endTime   STA info : link ID  + srcMAc + packet type
static void 
PhyTxBeginTrace(Ptr<WifiNetDevice> dev,uint32_t linkId, Ptr<const Packet> packet, Watt_u txDbm)
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
    // WifiMode Mode = txVector.GetMode();
    Time duration = phy->CalculateTxDuration(pktSize,txVector,GetBandFromFreq(phy->GetFrequency()));
    
    // 記錄傳輸事件
    Mac48Address srcPHY = hdr.GetAddr2(); 
    std::pair<uint32_t, Mac48Address> key = std::make_pair(linkId,srcPHY); //LinkId +PHYaddr -> Macaddr
    g_activeTx[key].txNode = node;
    g_activeTx[key].nodeId = nodeId;
    g_activeTx[key].channel = channelNum;
    g_activeTx[key].linkId = linkId;
    g_activeTx[key].startTime = Simulator::Now();
    g_activeTx[key].endTime = Simulator::Now() + duration;
    // g_activeTx[key].srcMac = Mac48Address::ConvertFrom(dev->GetAddress());
    g_activeTx[key].srcMac = srcPHY;
    g_activeTx[key].Type = hdr.GetTypeString() ;
    
    //test:
    // std::cout << "Type : " << hdr.GetTypeString() 
    //           << "  pktSize : " << pktSize <<" Byte " 
    //           << " \nStartTime : " << g_activeTx[key].startTime.GetSeconds()<< "s"
    //           << "  Duration : " << duration.GetMicroSeconds() << "us"
    //           << "  EndTime : " << g_activeTx[key].endTime.GetSeconds() << "s"
    //           << "\nchannelNum : " << channelNum
    //           << "  linkId : " << linkId
    //           << "  Freq : " << phy->GetFrequency()
    //           << "\nSrcPHY : " <<  g_activeTx[key].srcMac
    //           << "  dstMac : " << hdr.GetAddr1()
    //           <<"\n-------------------------------------------------------------"
    //           << std::endl;
    
    // std ::cout <<"RX key : " << key << "  Tx srcMac : " << srcPHY <<"\n";
}


//noise stop interval 
void
NoiseStopInterval(NodeContainer STA, uint32_t link)
{
    for(const auto& addr : apLinkTable[link].Noise_list)
    {
        uint32_t  STAId = STAlinkTable[addr];

        Ptr<WifiNetDevice> Dev = DynamicCast<WifiNetDevice>(STA.Get(STAId)->GetDevice(0));
        Ptr<StaWifiMac> DevUMac = DynamicCast<StaWifiMac>(Dev->GetMac());
        auto fem = DevUMac->GetFrameExchangeManager(link);
            
        if(std::abs(1 - apLinkTable[link].Pab) >  0.001 )
        {
            Ptr<WifiMacQueue>  Queue = DevUMac->GetQosTxop(AC_VO)->GetWifiMacQueue(); //VO : 3 , VI : 2 , BE : 1 , BK : 0
            uint32_t nBytes   = Queue->GetNBytes();
            Queue = DevUMac->GetQosTxop(AC_VI)->GetWifiMacQueue();
            nBytes += Queue->GetNBytes();
            // std::cout << "link : " << link << " address : " << addr << " nBytes : " << nBytes << "\n";

            if(nBytes == 0) apLinkTable[link].NoiseStop.insert(STAId); //暫停一個觀察時間
        }   
        DevUMac->BlockTxOnLink(link,WifiQueueBlockedReason::POWER_SAVE_MODE);
    }
    
} 
// count of every failure in the every beacon   and check Collision probability & Hidden node probability
void
ClearElement(NodeContainer STA)
{
    for(uint32_t i = 0; i < 3; ++i)
    {
        Time now = Simulator::Now();
        std::cout << "Beacon : " << now.GetSeconds() * 10 <<"\n" 
                  << "link : "         <<  i
                  << "  collision : "     << apLinkTable[i].c_count 
                  << "  hidden node : " << apLinkTable[i].h_count 
                  << "  noise : "        << apLinkTable[i].n_count 
                  << " success : " << apLinkTable[i].s_count
                  << " unknown : " << apLinkTable[i].unknown
                  << " total_real : " << apLinkTable[i].c_count + apLinkTable[i].h_count + apLinkTable[i].n_count + 
                                         apLinkTable[i].s_count + apLinkTable[i].unknown;

        double total = apLinkTable[i].c_count + apLinkTable[i].h_count  + apLinkTable[i].n_count + apLinkTable[i].s_count;

        double Pc = apLinkTable[i].c_count / total;
        double Pt = 0;
        double Pab = 1;
        double Ph = apLinkTable[i].h_count / total;
        double Pn = apLinkTable[i].n_count / total;

        //Access Barring - Collision
        std::cout<< "\nPc : " << Pc << "\n";
        std::cout<< "\nPh : " << Ph << "\n";
        std::cout<< "\nPn : " << Pn << "\n";

        if(Pc > 0.2)
        {
            Pt = 1.0 - std::pow(1.0 - Pc, 1.0 / (STA.GetN() - 1));
            double denom = (STA.GetN() - 1) * std::log(1.0 - Pt);
            double numer = std::log(1.0 - 0.15);
            Pab   = numer / denom;
        }else
        {
            Pab = 1; 
        }

        
        //HCCA - hidden node

        apLinkTable[i].HN_list.clear();
        std::queue<uint32_t> empty;
        std::swap(apLinkTable[i].HN_Queue,empty);

        if(Ph > HN_threshold) //0.05
        {   //startHCCA
            uint32_t NHN = apLinkTable[i].hideTable.size(); // num of hidden node

            double AVG_HN =  apLinkTable[i].h_count / NHN;  //  happen count / num of hidden node 

            for (auto const& kv : apLinkTable[i].hideTable) 
            {
                const Mac48Address& addr = kv.first;
                uint32_t            cnt  = kv.second;
                uint32_t            STAId =  STAlinkTable[addr];

                if( cnt > AVG_HN )  
                {   
                    Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(STAId)->GetDevice(0));
                    Ptr<StaWifiMac> DevUMac = DynamicCast<StaWifiMac>(wifiDev->GetMac());
                    //test:
                        
                        uint32_t STA_Byte = 0;
                        Ptr<WifiMacQueue>  Queue = DevUMac->GetQosTxop(AC_VO)->GetWifiMacQueue(); //VO : 3 , VI : 2 , BE : 1 , BK : 0
                        
                        uint32_t nBytes   = Queue->GetNBytes   ();
                        // std::cout << "STA : " << STAId << " AC_VO " <<" nByte : " << nBytes <<"\n"; 
                        STA_Byte += nBytes;
                        Queue = DevUMac->GetQosTxop(AC_VI)->GetWifiMacQueue();                       
                        nBytes   = Queue->GetNBytes   ();
                        // std::cout << "STA : " << STAId << " AC_VI " <<" nByte : " << nBytes <<"\n"; 
                        STA_Byte += nBytes;

                        Queue = DevUMac->GetQosTxop(AC_BE)->GetWifiMacQueue();                        
                        nBytes   = Queue->GetNBytes   ();
                        // std::cout << "STA : " << STAId << " AC_BE " <<" nByte : " << nBytes <<"\n"; 
                        STA_Byte += nBytes;

                        Queue = DevUMac->GetQosTxop(AC_BK)->GetWifiMacQueue();
                        nBytes   = Queue->GetNBytes   ();
                        // std::cout << "STA : " << STAId << " AC_BK " <<" nByte : " << nBytes <<"\n"; 
                        STA_Byte += nBytes;
                        

                        //test :
                        
                        // std::cout << "STA : " << STAId << " nByte : " << STA_Byte <<"\n";        
                        apLinkTable[i].HN_list.push_back(std::make_pair(STAId, STA_Byte)); //編號 + bit
                }
            }

    
            //open HCCA
            if(!(apLinkTable[i].HN_list.empty()))
            {
                std::cout << "link : " << i << " have hidden block" << "\n";
                apLinkTable[i].HCCA = true;
                apLinkTable[i].B_cnt = 4;
                
                std::sort(apLinkTable[i].HN_list.begin(), apLinkTable[i].HN_list.end(),
                          [](auto const& a, auto const& b){ return a.second > b.second; });
                          
                //Q_VO, Q_VI, Q_BE, Q_BK
                for(auto const& HN : apLinkTable[i].HN_list)
                {
                    for(uint32_t ac = 0; ac < 4; ac++)
                    {
                        uint32_t STAId = HN.first;
    
                        Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(STAId)->GetDevice(0));
                        Ptr<StaWifiMac> DevUMac = DynamicCast<StaWifiMac>(wifiDev->GetMac());
                        Ptr<WifiMacQueue>  Q = DevUMac->GetQosTxop(3 - ac)->GetWifiMacQueue();
                        uint32_t nBytes   = Q->GetNBytes ();
                        if(nBytes != 0) 
                        {
                            apLinkTable[i].HN_Queue.push(STAId);
                            break;
                        }
                    }
                }
            }
            else
            {
                apLinkTable[i].HCCA = false;
                apLinkTable[i].B_cnt = 0;
            } 
            

            
            
            
            // test: hidden node
            // std::cout << "hiden node :" << "\n";
            // for(auto it = apLinkTable[i].HN_list.begin(); it != apLinkTable[i].HN_list.end(); ++it)
            // std::cout <<it->first << " : " << it->second << "\n";
            // std::cout << "queue : " << apLinkTable[i].HN_Queue.size() << "\n";

            // std::queue q = apLinkTable[i].HN_Queue;
            // while(!(q.empty()))
            // {
            //     std::cout<< q.front()<<" ";
            //     q.pop();
            // }
            // std::cout << "\n";
        }
        else
        {
            apLinkTable[i].B_cnt = 0;
            apLinkTable[i].HCCA = false;
        }
        
        
        //noise :
        apLinkTable[i].Noise_list.clear();
        uint32_t NN = apLinkTable[i].NoiseTable.size(); // num of Noise
        double StdDev = 0;
        double sum = 0;  
        if(NN != 0)
        {
            uint32_t AVG_N =  apLinkTable[i].n_count / NN;
            for(auto const& Nv : apLinkTable[i].NoiseTable)
            {
                uint32_t cnt  = Nv.second;

                sum += (cnt - AVG_N) * (cnt - AVG_N);
            }

            StdDev = sqrt(sum/NN);
            // std::cout << "sum : " << sum << " AVG : " << AVG_N <<  "  NN : " << NN << "  Std : " << StdDev << "\n";

            for(auto const& Nv : apLinkTable[i].NoiseTable)
            {
                const Mac48Address& addr = Nv.first;
                uint32_t            cnt  = Nv.second;
                if((cnt / StdDev) > 2)
                {
                    apLinkTable[i].Noise_list.push_back(addr);
                }
            }
            
        }
        
        if(apLinkTable[i].Noise_list.size() > 0)
        {
            apLinkTable[i].NoiseFlag = true;
            apLinkTable[i].NoiseStop.clear();
            Simulator::Schedule(Seconds(0.0001), &NoiseStopInterval, STA, i);
        }
        else
        {
            apLinkTable[i].NoiseFlag = false;
        }

        //test : noise 
        // for(auto itt = apLinkTable[i].NoiseTable.begin(); itt!= apLinkTable[i].NoiseTable.end(); ++itt)
        // std::cout << itt->first << " : " << itt->second << "\n";

        for(const auto& addr : apLinkTable[i].Noise_list)
        std::cout << "Noise STA: " << addr << "\n";
        


        //clean link i
        apLinkTable[i].n_count = 0;
        apLinkTable[i].h_count = 0;
        apLinkTable[i].c_count = 0;
        apLinkTable[i].s_count = 0;
        apLinkTable[i].unknown = 0;
        apLinkTable[i].Pab = Pab;
        apLinkTable[i].hideTable.clear();
        apLinkTable[i].NoiseTable.clear();
        std::cout << "\n---------------------------------------------------------------"<<"\n";
    }
    Simulator::Schedule(beaconInterval, &ClearElement,STA);
}

void
HCCAshchule(NodeContainer STA, uint32_t link, Time StopTime, uint32_t lastSTAId)
{   
    auto &q = apLinkTable[link].HN_Queue;

    // uint32_t M = q.size(); 
    // std::cout << "link : " <<link << " M :" << M <<"\n";

    Time now = Simulator::Now();

    Ptr<WifiNetDevice> lastwifiDev = DynamicCast<WifiNetDevice>(STA.Get(lastSTAId)->GetDevice(0));
    Ptr<StaWifiMac> lastDevMac = DynamicCast<StaWifiMac>(lastwifiDev->GetMac());

    if(apLinkTable[link].HCCA) apLinkTable[link].HCCA = false;
    else lastDevMac->BlockTxOnLink(link, WifiQueueBlockedReason::POWER_SAVE_MODE);
 
    if( now < StopTime && !(q.empty()))
    {
        uint32_t STAId = q.front();
        q.pop();
        // std::cout <<"STA Id : " << STAId <<" link :" << link << '\n';
        Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(STAId)->GetDevice(0));
        Ptr<StaWifiMac> DevMac = DynamicCast<StaWifiMac>(wifiDev->GetMac());
        Ptr<WifiPhy> phy = wifiDev->GetPhy(link);
       
        
        std::set<uint8_t> No;
        No.insert(link);
        DevMac->UnblockTxOnLink(No,WifiQueueBlockedReason::POWER_SAVE_MODE);
        Simulator::Schedule(Seconds(0.0010),&HCCAshchule,STA, link, StopTime, STAId); 
    }
    else
    {
        // std::cout <<" is ok : " << " link :" << link << '\n';
        Ptr<UniformRandomVariable> ran = CreateObject<UniformRandomVariable>();
        for(uint32_t i = 0; i < STA.GetN(); i++) 
        {
            double r = ran->GetValue();
            Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(i)->GetDevice(0));
            Ptr<StaWifiMac> DevMac = DynamicCast<StaWifiMac>(wifiDev->GetMac());
            auto fem = DevMac->GetFrameExchangeManager(link);
            Mac48Address linkAddress = fem->GetAddress();

            if((apLinkTable[link].Pab + 0.0001) < r)
            {
                DevMac->BlockTxOnLink(link,WifiQueueBlockedReason::POWER_SAVE_MODE);                
            }
            else
            {
                std::set<uint8_t> No;
                No.insert(link);
                if(apLinkTable[link].NoiseFlag)
                {
                    bool flag = true;
                    for(const auto & addr : apLinkTable[link].Noise_list) if(addr == linkAddress) flag = false; //有在noise表單內
                    if(flag)DevMac->UnblockTxOnLink(No,WifiQueueBlockedReason::POWER_SAVE_MODE);
                }
                else
                {
                    if(apLinkTable[link].NoiseStop.count(i) == 0) DevMac->UnblockTxOnLink(No,WifiQueueBlockedReason::POWER_SAVE_MODE);
                    //確認是否因為壅塞導致一部分STA需要鎖住一個觀察週期
                }
            }
            
        }
        apLinkTable[link].NoiseFlag = false;

        apLinkTable[link].B_cnt -= 1;
        if(apLinkTable[link].B_cnt > 0)
        {

            for(const auto &p : apLinkTable[link].HN_list)
            {
                Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(p.first)->GetDevice(0));
                Ptr<StaWifiMac> DevMac = DynamicCast<StaWifiMac>(wifiDev->GetMac());
                DevMac->BlockTxOnLink(link,WifiQueueBlockedReason::POWER_SAVE_MODE);
                for(uint32_t ac = 0; ac < 4 ; ac++)
                {
                    Ptr<WifiMacQueue> AC_Q = DevMac->GetQosTxop(3-ac)->GetWifiMacQueue();
                    uint32_t nByte = AC_Q->GetNBytes();
                    if(nByte)
                    {
                        apLinkTable[link].HN_Queue.push(p.first);
                        break;
                    }
                }
            }
            
            apLinkTable[link].HCCA = true;

            // std::cout <<  "link : " << link <<" apLinkTable[link].B_cnt : " << apLinkTable[link].B_cnt << "\n";
            // std::queue q2 = apLinkTable[link].HN_Queue;
            // while(!(q2.empty()))
            // {
            //     std::cout<< q2.front() << " ";
            //     q2.pop();
            // }
            // std::cout << "\n";
        }
    }
}

void //HCCA & AB
function2(NodeContainer STA)
{
    Time now = Simulator::Now(); 
    for( uint32_t link = 0; link < 3; link ++)
    { 
        // std::cout <<  "this 1" << "\n";
        //HCCA       
        if(apLinkTable[link].HCCA && (apLinkTable[link].HN_Queue.size()!= 0)) //雖然要HCCA但這次為空
        {   //all STA stop
            for(uint32_t stopSTA = 0; stopSTA < STA.GetN(); stopSTA++)
            {
                
                Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(stopSTA)->GetDevice(0));
                Ptr<StaWifiMac> DevMac = DynamicCast<StaWifiMac>(wifiDev->GetMac());
                DevMac->BlockTxOnLink(link,WifiQueueBlockedReason::POWER_SAVE_MODE);
            }
            //Open HCCA
            Time StartTime = now + MicroSeconds(1);
            Time StopTime = now + MilliSeconds(10); //1 beacon = 100ms -> 1 HCCA = 10 ms (1/10 beacon)
            uint32_t STAId = apLinkTable[link].HN_Queue.front();
            Simulator::Schedule(Seconds(0.000001),&HCCAshchule, STA ,link, StopTime, STAId);
    
        }
        else
        {
            //STA Access Baring 
            Ptr<UniformRandomVariable> ran = CreateObject<UniformRandomVariable>();
            for(uint32_t i = 0; i < STA.GetN(); i++)
            {
                double r = ran->GetValue();
                Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(i)->GetDevice(0));
                Ptr<StaWifiMac> DevMac = DynamicCast<StaWifiMac>(wifiDev->GetMac());
                auto fem = DevMac->GetFrameExchangeManager(link);
                Mac48Address linkAddress = fem->GetAddress();
                if((apLinkTable[link].Pab + 0.0001) < r)
                {
                    DevMac->BlockTxOnLink(link,WifiQueueBlockedReason::POWER_SAVE_MODE); 
                    // std::cout << " STA" << i << " Link :" << link  << " is block \n";                 
                }
                else
                {
                    std::set<uint8_t> No;
                    No.insert(link);
                    if(apLinkTable[link].NoiseFlag)
                    {
                        bool flag = true;
                        for(const auto & addr : apLinkTable[link].Noise_list) if(addr == linkAddress) flag = false; //有在noise表單內
                        if(flag)DevMac->UnblockTxOnLink(No,WifiQueueBlockedReason::POWER_SAVE_MODE);
                    }
                    else
                    {
                        if(apLinkTable[link].NoiseStop.count(i) == 0) DevMac->UnblockTxOnLink(No,WifiQueueBlockedReason::POWER_SAVE_MODE);
                    }
                }
                
            }
            
            apLinkTable[link].NoiseFlag = false;
        }
    
            

        
    }

    Simulator::Schedule(Seconds(0.100001), &function2,STA);
}

// collect  signal & noise 
static void 
SnifferRxCallback( uint32_t mcs, Ptr<const Packet> p, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise,uint16_t stdId)
{
    //BER Rate
    DataRate dr = EhtPhy::GetDataRate(mcs,MHz_u{static_cast<double>(20)},NanoSeconds(800), 1);
    double signal = signalNoise.signal; // dBm
    double noise  = signalNoise.noise;  // dBm
    double snr_dB = signal - noise;
    double snr_linear = std::pow(10.0, snr_dB / 10.0);
    double EbNo = snr_linear * 20 * 1e6 / dr.GetBitRate();
    double z = std::sqrt((1.5 * log2(16) * EbNo) / (16 - 1.0));
    double z1 = ((1.0 - 1.0 / std::sqrt(16)) * erfc(z));
    double z2 = 1 - std::pow((1 - z1), 2);
    double BER = z2 / log2(16); //bit error rate

    int bits = p->GetSize()*8;

    GlobalSUC = std::pow(1.0 - BER, bits); //Success rate
    GlobalBER = BER;
    
    // test: system noise & signal 
    // std::cout << "S : " << signal 
    //           << " N : " << noise 
    //           << " SNR_dB : " << snr_dB 
    //           << " SNR_L : " << snr_linear
    //           << " BER : " << GlobalBER 
    //           << " SUC : " << GlobalSUC
    //           << " DateRate : " << dr << "\n";
}

void
// collect failure count++ 
MyDropCallback(uint32_t linkId, Ptr<const Packet> p, WifiPhyRxfailureReason reason)
{
    Time now = Simulator::Now();
    
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    Mac48Address srcMac = hdr.GetAddr2(); //物理層address!!!! 非mac address!!!!
    std::pair<uint32_t, Mac48Address> key = std::make_pair(linkId, srcMac);
    // std::cout << "Drop key :" << key << "\n";


    auto it = g_activeTx.find(key);
    if( it != g_activeTx.end())
    {
        const TxRecord& cur = it->second;
        if(std::abs((lastStart[linkId] - cur.startTime).GetMicroSeconds()) < 7)
        {
            if(lastMacAddr[linkId]!= cur.srcMac)
            {
                apLinkTable[linkId].c_count += 1;
                // test:
                // std::cout << " link :" << linkId << " collision ! " << " LastStart : " << lastStart[linkId].GetSeconds()
                //           << " StartTime : "<< cur.startTime.GetSeconds()  << "\n"
                //           << " Address : "  << cur.srcMac << " lastAddress : " << lastMacAddr[linkId] <<"\n";
            }
            else
            {
                // apLinkTable[linkId].unknown += 1;
                // std::cout << " link :" << linkId << " same address ! " << " LastStart : " << lastStart[linkId].GetSeconds()
                //           << " StartTime : "<< cur.startTime.GetSeconds()  << "\n"
                //           << " Address : "  << cur.srcMac << " lastAddress : " << lastMacAddr[linkId] <<"\n"
                //           << " reason : "   << reason <<"\n";

            }
        }
        else
        {
            if(lastEnd[linkId] > cur.startTime && std::abs((lastStart[linkId] - cur.startTime).GetMicroSeconds()) > 7)
            {
                if(lastMacAddr[linkId]!= cur.srcMac)
                {
                    apLinkTable[linkId].h_count += 1;
                    if(apLinkTable[linkId].hideTable.count(lastMacAddr[linkId]) > 0)
                    {
                        apLinkTable[linkId].hideTable[lastMacAddr[linkId]] += 1;
                        // std::cout<< "Address : " << lastMacAddr[linkId] << " count  : " <<  apLinkTable[linkId].hideTable[lastMacAddr[linkId]]<<"\n";
                    }
                    else
                    {
                        apLinkTable[linkId].hideTable.emplace(lastMacAddr[linkId], 1);
                        // std::cout<< "Address : " << lastMacAddr[linkId] << " count  : " <<  apLinkTable[linkId].hideTable[lastMacAddr[linkId]]<<"\n";
                    }

                    // test:
                    // std::cout  << " link :" << linkId << " hidden_node ! "
                    //            << " lastEnd : " << lastEnd[linkId].GetSeconds()
                    //            << " StartTime : "<< cur.startTime.GetSeconds()
                    //            << " hidenTime :" << apLinkTable[linkId].h_count << "\n";
                }
            }
            else
            {
                apLinkTable[linkId].unknown += 1;
            }
            
        }
        // test:

        // std::cout << "srcMac :  "<< cur.srcMac << " print :" << hdr.GetDuration().GetSeconds()
        //           <<"\nnow : " << now.GetSeconds() <<"    startTime : " << cur.startTime.GetSeconds() << "    endTime : " <<cur.endTime.GetSeconds() 
        //           << "\nlink : " << linkId << " Type : "<< cur.Type << "  reason :  "  << reason  
        //           << " Beacon : "<< now.GetSeconds() / double(0.1) << "\n"
        //           << "---------------------------------------------------------------------------------------------------\n";
        // apLinkTable[linkId].total_count += 1;

        lastStart[linkId]   = cur.startTime;
        lastEnd[linkId]     = cur.endTime;
        lastMacAddr[linkId] = cur.srcMac;
    }
    else
    {
        std::cout << "error!" << "\n";
    }
                
}

void
RandomNoise(NodeContainer STA)
{
    //lognormal distribute
    Ptr<LogNormalRandomVariable> logRan = CreateObject<LogNormalRandomVariable>();
    logRan->SetAttribute("Mu", DoubleValue(-20.05));
    logRan->SetAttribute("Sigma", DoubleValue(0.15));

    Ptr<UniformRandomVariable> durationRan = CreateObject<UniformRandomVariable>();
    durationRan->SetAttribute("Min", DoubleValue(0.1)); //10us
    durationRan->SetAttribute("Max", DoubleValue(0.75)); // 0.5s
    
    for(uint32_t i = 0; i < STA.GetN(); i++)
    {
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(STA.Get(i)->GetDevice(0));
        for(uint32_t linkId = 0; linkId < 3; linkId++)
        {
            Ptr<SpectrumWifiPhy> phy =  DynamicCast<SpectrumWifiPhy>(dev->GetPhy(linkId));   if (phy == nullptr) continue;
            Ptr<WifiMac>  mac = DynamicCast<WifiMac>(dev->GetMac());
            auto fem = mac->GetFrameExchangeManager(linkId);
            Mac48Address src = fem->GetAddress();
            double noise = logRan->GetValue();
            double noiseDb = 10 * std::log10(noise);
            Time holdTime = Seconds(durationRan->GetValue());
            std::pair key = std::make_pair(linkId,src);
            g_activeTx[key].noisedBm = noiseDb;
            // std::cout << "noise : " << noise << " noiseDb : " << noiseDb <<'\n';
            Simulator::Schedule(holdTime,[key](){g_activeTx[key].noisedBm = -87.3;}); //noise end -> base noise 
        }
    }
    Simulator::Schedule(Seconds(0.5),&RandomNoise,STA);
}
// success count++

void
MySucCallback(uint32_t linkId, Ptr<const Packet> p)
{
    // Time now = Simulator::Now();
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    Mac48Address src = hdr.GetAddr2();
    auto type = hdr.GetTypeString();
    // std::cout << " packet type : " << type  << " time :" << now.GetSeconds() << "\n";
    // std::cout << "RTS is  : " << hdr.IsRts() << "\n";

    std::pair key = std::make_pair(linkId,src);
    DataRate dr = EhtPhy::GetDataRate(11, MHz_u{static_cast<double>(20)}, NanoSeconds(800), 1); // mcs , bandwidth , GI
    double signal = g_activeTx[key].signaldBm; // dBm
    double noise  = g_activeTx[key].noisedBm;  // dBm
    double snr_dB = signal - noise;
    double snr_linear = std::pow(10.0, snr_dB / 10.0);
    double EbNo = snr_linear * 20 * 1e6 / dr.GetBitRate();
    double Q_arg = std::sqrt((3 * log2(16) * EbNo) / (16 - 1.0));
    double Q_val = 0.5 * std::erfc(Q_arg / std::sqrt(2.0));
    double BER = (4.0 / log2(16)) * (1 - 1.0 / std::sqrt(16)) * Q_val;
    int bits = p->GetSize()*8;
    double SUC = std::pow(1.0 - BER, bits);

    // test
    // std::cout << "key : " << key 
    //           << " noise : " << g_activeTx[key].noisedBm 
    //           << " signal : " <<  g_activeTx[key].signaldBm 
    //           << " snr_dB : " << snr_dB
    //           << " BER : "    <<  BER
    //           << "\nbits : "  << bits
    //           << " SUC : "    <<  SUC
    //           <<"\n";

    Ptr<UniformRandomVariable> RV = CreateObject<UniformRandomVariable>();
    RV->SetAttribute("Min", DoubleValue(0.0));
    RV->SetAttribute("Max", DoubleValue(1.0));

    double r =  RV->GetValue();
    if(r < SUC)
    {
        if(hdr.IsRts())
        {
            apLinkTable[linkId].s_count += 1;
        }
    }
    else
    {
        if(hdr.IsRts())
        {   
            apLinkTable[linkId].n_count += 1;

            if(apLinkTable[linkId].NoiseTable.count(src) > 0)
            {
                apLinkTable[linkId].NoiseTable[src] += 1;
            }
            else
            {
                apLinkTable[linkId].NoiseTable.emplace(src, 1);
            }
        }
    }
}


// move model
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


// interferers (remove)
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

//flow
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
            std ::cout <<DynamicCast<UdpServer>(serverApp.Get(i))->GetReceived() <<"\n";
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
    bool TXOPopen{false};
    bool AMopen{false};

    // uint16_t mpduBufferSize{512};
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
    
    std::size_t nStations{60};
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

    //mcs & ChannelWidth gi
    uint8_t mcs = 11;
    mcsValues.push_back(mcs); 
    
    int minChannelWidth = 20;
    int maxChannelWidth = 20;
    
    int minGi = 800;
    int maxGi = 800;
   
  

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
 
//----------------------------------------------------------------------------------------------------------
    std::cout << "MCS value"
              << "\t\t"
              << "Channel width"
              << "\t\t"
              << "GI"
              << "\t\t\t"
              << "Throughput" << '\n';


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
            // setting frequency
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
                
                SpectrumWifiPhyHelper phy(nLinks);
            //STA PHY
                phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
                phy.Set("ChannelSwitchDelay", TimeValue(channelSwitchDelay));

                //手動設定雜訊->關閉原系統雜訊(避免自動降速 (mcs11))
                phy.Set("TxPowerStart", DoubleValue(25.0));
                phy.Set("TxPowerEnd", DoubleValue(25.0));
                phy.Set("TxPowerLevels", UintegerValue(1));
                phy.Set("RxNoiseFigure", DoubleValue(0.0));
            //error model
                phy.SetErrorRateModel("ns3::YansErrorRateModel");
                
            //STA Mac
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
                    phy.Set(linkId, "ChannelSettings", StringValue(channelStr[linkId])); // binding link and frequency  : link 0 -> 2.4GHz
                    auto spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
                    
                    //signal loss model - have value ->hidden node
                    auto lossModel = CreateObject<LogDistancePropagationLossModel>();
                    spectrumChannel->AddPropagationLossModel(lossModel);
                    phy.AddChannel(spectrumChannel, freqRanges[linkId]);
                  
                }
                
            //install STA
                staDevices = wifi.Install(phy, mac, wifiStaNodes);

            //Open TXOP System
                Ptr<NetDevice> dev;
                Ptr<WifiNetDevice>wifi_dev;
                PointerValue ptr;
                Ptr<QosTxop> edca;

            if(AMopen == true)
                for(uint32_t i = 0; i < nStations;i++)
                {
                    //open A-MPDU
                    dev = wifiStaNodes.Get(i)->GetDevice(0);
                    wifi_dev = DynamicCast<WifiNetDevice>(dev);
                    // wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(32768));
                    // wifi_dev->GetMac()->SetAttribute("BK_MaxAmpduSize", UintegerValue(32768));
                    // wifi_dev->GetMac()->SetAttribute("VO_MaxAmpduSize", UintegerValue(32768));
                    // wifi_dev->GetMac()->SetAttribute("VI_MaxAmpduSize", UintegerValue(32768));

                    wifi_dev->GetMac()->SetAttribute("BE_MaxAmsduSize", UintegerValue(3839)); //3839
                    wifi_dev->GetMac()->SetAttribute("BK_MaxAmsduSize", UintegerValue(3839));
                    wifi_dev->GetMac()->SetAttribute("VO_MaxAmsduSize", UintegerValue(3839));
                    wifi_dev->GetMac()->SetAttribute("VI_MaxAmsduSize", UintegerValue(3839));
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
                //開啟會有多封包損毀(以排除)， BE/BK在原始設定為 txopLimit = 0;
            if(TXOPopen ==true) //TXOPopen
            {
                mac.SetEdca(AC_VO, "TxopLimits", StringValue(txopLimit));
                mac.SetEdca(AC_VI,"TxopLimits", StringValue(txopLimit));
            }
            
                phy.Set("TxPowerStart", DoubleValue(70.0));
                phy.Set("TxPowerEnd", DoubleValue(70.0));
                phy.Set("TxPowerLevels", UintegerValue(1));
                phy.Set("TxGain", DoubleValue(0.0));
                phy.Set("RxGain", DoubleValue(0.0));

                apDevice = wifi.Install(phy, mac, wifiAPNode);

                //avoid disconnect
                Config::Set ("/NodeList/*/DeviceList/*/$ns3::StaWifiMac/MaxMissedBeacons",UintegerValue (10000));
                Config::Set ("/NodeList/*/DeviceList/*/$ns3::StaWifiMac/ListenInterval",UintegerValue (1));
                Config::Set ("/NodeList/*/DeviceList/*/$ns3::ApWifiMac/AssocFailTimeout", TimeValue (Seconds (0))); 
                Config::Set ("/NodeList/*/DeviceList/*/$ns3::ApWifiMac/DisassocTimeout", TimeValue (Seconds (0)));

                
            //random number
                streamNumber += WifiHelper::AssignStreams(apDevice, streamNumber);
                streamNumber += WifiHelper::AssignStreams(staDevices, streamNumber);

            // Set guard interval and MPDU buffer size
                Config::Set(
                    "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                    TimeValue(NanoSeconds(gi)));
                // Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MpduBufferSize",
                //             UintegerValue(mpduBufferSize));

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

                    std::vector<ApplicationContainer> clientApp(nStations);

                    for (std::size_t i = 0; i < nStations; i++)
                    {   
                        AddressValue remoteAddress(dest);
                        
                        //{0x70, 0x28, 0xb8, 0xc0}; // AC_BE, AC_BK, AC_VI, AC_VO
                        //BE
                        ApplicationContainer clientAppBE = CreateClientFlow(0x70, payloadSize,nonHtRefRateMbps,dest,clientNodes.Get(i),
                                                                            1,1,0,0);//0.0015,0.025,0,0.005
                        clientAppBE.Start(Seconds(1.0 ) + MicroSeconds(i * 100));
                        clientAppBE.Stop(simulationTime + Seconds(1));
                        //BK
                        ApplicationContainer clientAppBK = CreateClientFlow(0x28, payloadSize,nonHtRefRateMbps,dest,clientNodes.Get(i),
                                                                             1,1,0,0);//0.03,0.05,0,0.01
                        clientAppBE.Start(Seconds(1.0 ) + MicroSeconds(i * 100));
                        clientAppBE.Stop(simulationTime + Seconds(1));
                        // // //VI
                        ApplicationContainer clientAppVI = CreateClientFlow(0xb8, payloadSize,nonHtRefRateMbps,dest,clientNodes.Get(i),
                                                                             1,1,0,0); //0.2,1,0.1,2
                        clientAppVI.Start(Seconds(1.0) + MicroSeconds(i * 100));
                        clientAppVI.Stop(simulationTime + Seconds(1));
                        // // //VO
                        ApplicationContainer clientAppVO = CreateClientFlow(0xc0, payloadSize-100,nonHtRefRateMbps,dest,clientNodes.Get(i),
                                                                             1,1,0,0);//0.5,1,1,2
                        clientAppVO.Start(Seconds(1.0 ) + MicroSeconds(i * 100));
                        clientAppVO.Stop(simulationTime + Seconds(1));
                    //test
                        // Ptr<WifiNetDevice> dev = staDevices.Get(0)->GetObject<WifiNetDevice>();
                        // Ptr<WifiPhy> pp = dev->GetPhy();
                        // pp->SetReceiveOkCallback(MyRxCallback);
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
                
            
            // STA掛勾三條link是否成功 +設定slot同步 (9us)
            //STAaddr_linkid
            for(u_int32_t i = 0; i < nStations ;++i)
            {
                const Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(staDevices.Get(i));
                Ptr<WifiMac> mac = mloDevice->GetMac();
                
                for(uint8_t Linkid = 0; Linkid <std::max<uint8_t>(mloDevice->GetNPhys(),1); ++Linkid)
                {   
                    Ptr<WifiPhy> devphy = mloDevice->GetPhy(Linkid);
                    devphy->SetSlot(MicroSeconds(9));
                    auto fem = mac->GetFrameExchangeManager(Linkid); //mac of limk i
                    Mac48Address LowerMacAddress =  fem->GetAddress();
                    STAlinkTable[LowerMacAddress] = i;
                    
                    // std::cout << " staDevice : " << LowerMacAddress << " is : " << STAlinkTable.count(LowerMacAddress)
                    //           << " linkId : " << STAlinkTable[LowerMacAddress];
                    // std::cout << " mac : "<<DynamicCast<WifiNetDevice>(staDevices.Get(STAlinkTable[LowerMacAddress]))->GetMac()->GetAddress()
                    //           <<"\n";
                }

            }
          
            
            //trace
            //物理層錯誤查詢 (ap)
           

            for(uint32_t i = 0; i < nLinks; ++i)
            {
                const Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(apDevice.Get(0));
                Ptr<WifiPhy> wifiphy = mloDevice->GetPhy(i);
                wifiphy->TraceConnectWithoutContext("MonitorSnifferRx",MakeBoundCallback(&SnifferRxCallback, mcs));
                wifiphy->TraceConnectWithoutContext("PhyRxDrop", MakeBoundCallback(&MyDropCallback,i));
                wifiphy->TraceConnectWithoutContext("PhyRxEnd", MakeBoundCallback(&MySucCallback, i));
            }


            //物理層傳送的封包資訊
            for(uint32_t i = 0; i < nStations; ++i)
            {
               const Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(staDevices.Get(i));
               for(uint32_t j = 0; j < nLinks; j++)
               {
                   Ptr<WifiPhy> phy = mloDevice->GetPhy(j);
                   phy->TraceConnectWithoutContext("PhyTxBegin",MakeBoundCallback(&PhyTxBeginTrace,mloDevice,j));

               }   
            }
            
           
            //start--------------------------------------
    
                Simulator::Schedule(Seconds(0.0),&ClearElement,wifiStaNodes);
                Simulator::Schedule(Seconds(0.0),&RandomNoise,wifiStaNodes);
                Simulator::Schedule(Seconds(0.0),&function2,wifiStaNodes);
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
