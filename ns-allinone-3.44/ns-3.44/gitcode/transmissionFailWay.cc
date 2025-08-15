/*
 * Copyright (c) 2022
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Sebastien Deronne <sebastien.deronne@gmail.com>
 */

 /*
 1. hidden node 數量 ->採用外圍移動解決數量不足問題
 3. 流量設定 
 4. 計算吞吐量 -> bit/total time
 5. 確保吞吐量在2000年bichi的論文是可以實現的
 6. 確定beacon K要設定多少
 7. 為何2.4GHz在開始前不會被使用
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
    uint16_t channel;
    uint32_t linkId;
    Time startTime;
    Time endTime;
    Mac48Address srcMac; 
    std::string Type ;
    double signaldBm =-64.5;
    double noisedBm =-87.0;
};


//my function use table & count &
struct LinkTable
{
    uint32_t B_cnt = 0 ; //HCCA open time
    uint32_t c_count = 0;
    uint32_t n_count = 0;
    uint32_t nr_count = 0;
    uint32_t h_count = 0;
    uint32_t s_count = 0;
    uint32_t unknown = 0;
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

struct linkBackofftime
{
    std::vector<uint32_t> link;
    std::vector<uint32_t> Pcnt; // packetcount
    std::vector<uint32_t> nowDelay;
};

//global
double simStart = 10.0;
//每次發生傳送的封包個別資訊
std::map<std::pair<uint32_t, Mac48Address>,TxRecord> g_activeTx;

//AP紀錄各種傳輸異常與參數資訊
std::vector<LinkTable> apLinkTable(3); //count 

//每次傳輸的accessDelay時間
std::map<uint32_t, linkBackofftime> staLinkBackoff;

//吞吐量
uint32_t totalbit = 0;

// 因比對collision & hidden node 所以需要記住link i 上的 上一個STA起始與位址
std::vector<Time> lastStart(3);
std::vector<Time> lastEnd(3);
std::vector<Mac48Address> lastMacAddr(3);
//提供物理層快速判別STD Id的方法
std::map<Mac48Address,uint32_t> STAlinkTable; // phylink -> STA Id


double GlobalBER = 0;
double GlobalSUC = 0;
Time beaconInterval = MilliSeconds (300);
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

//Get  STA : TX startTime & endTime   STA info : link ID  + srcMAc + packet type (OK)
static void 
PhyTxBeginTrace(Ptr<WifiNetDevice> dev,uint32_t linkId, Ptr<const Packet> packet, Watt_u txDbm)
{
    Ptr<WifiPhy> phy = dev->GetPhy(linkId);
    MHz_u allowedWidth = phy->GetChannelWidth();
    Ptr<WifiRemoteStationManager> StaManger = dev->GetRemoteStationManager(linkId); 
    //channel
    uint16_t channelNum = phy->GetChannelNumber(); //GHZ 所對應預設頻道"編號"
    // get header
    WifiMacHeader hdr;
    packet->PeekHeader(hdr); //備份packet 的hdr ->避免解封包動作
    WifiTxVector txVector = StaManger->GetDataTxVector(hdr,allowedWidth);
    uint32_t pktSize = packet-> GetSize();
    // WifiMode Mode = txVector.GetMode();
    Time duration = phy->CalculateTxDuration(pktSize,txVector,GetBandFromFreq(phy->GetFrequency()));
    
    // 記錄傳輸事件
    Mac48Address srcPHY = hdr.GetAddr2(); 
    std::pair<uint32_t, Mac48Address> key = std::make_pair(linkId,srcPHY); //LinkId +PHYaddr -> Macaddr
    g_activeTx[key].channel = channelNum;
    g_activeTx[key].linkId = linkId;
    g_activeTx[key].startTime = Simulator::Now();
    g_activeTx[key].endTime = Simulator::Now() + duration;
    // g_activeTx[key].srcMac = Mac48Address::ConvertFrom(dev->GetAddress());
    g_activeTx[key].srcMac = srcPHY;
    g_activeTx[key].Type = hdr.GetTypeString() ;
    
    //test:
    // uint32_t STAId = dev->GetNode()->GetId();
    // std::cout << "STA ID : " << STAId << "\n"
    //           << "Type : " << hdr.GetTypeString() 
    //           << "  pktSize : " << pktSize <<" Byte " 
    //           << " \nStartTime : " << g_activeTx[key].startTime.GetSeconds()<< "s"
    //           << "  Duration : " << duration.GetMicroSeconds() << "us"
    //           << "  EndTime : " << g_activeTx[key].endTime.GetSeconds() << "s"
    //           << "\nchannelNum : " << channelNum
    //           << "  linkId : " << linkId
    //           << "  Freq : " << phy->GetFrequency()
    //           << "\nSrcPHY : " <<  g_activeTx[key].srcMac
    //           << "  dstMac : " << hdr.GetAddr1()
    //           << " UID: " << packet->GetUid()
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

        Ptr<WifiNetDevice> Dev = DynamicCast<WifiNetDevice>(STA.Get(STAId)->GetDevice(link));
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

        double total = apLinkTable[i].c_count + apLinkTable[i].h_count  + apLinkTable[i].nr_count + apLinkTable[i].s_count;

        double Pc = apLinkTable[i].c_count / total;
        double Pt = 0;
        double Pab = 1;
        double Ph = apLinkTable[i].h_count / total;
        double Pn = apLinkTable[i].n_count / total;

        //Access Barring - Collision
        std::cout<< "\nPc : " << Pc << "\n";
        std::cout<< "\nPh : " << Ph << "\n";
        std::cout<< "\nPn : " << Pn << "\n";

        if(Pc >= 0.2)
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
                    Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(STAId)->GetDevice(i));
                    Ptr<StaWifiMac> DevUMac = DynamicCast<StaWifiMac>(wifiDev->GetMac());
                    //test:
                        
                        uint32_t STA_Byte = 0;
                        Ptr<WifiMacQueue>  Queue = DevUMac->GetQosTxop(AC_VO)->GetWifiMacQueue(); //VO : 3 , VI : 2 , BE : 1 , BK : 0
                        
                        uint32_t nBytes   = Queue->GetNBytes();
                        // std::cout << "STA : " << STAId << " AC_VO " <<" nByte : " << nBytes <<"\n"; 
                        STA_Byte += nBytes;
                        Queue = DevUMac->GetQosTxop(AC_VI)->GetWifiMacQueue();                       
                        nBytes   = Queue->GetNBytes();
                        // std::cout << "STA : " << STAId << " AC_VI " <<" nByte : " << nBytes <<"\n"; 
                        STA_Byte += nBytes;

                        Queue = DevUMac->GetQosTxop(AC_BE)->GetWifiMacQueue();                        
                        nBytes   = Queue->GetNBytes();
                        // std::cout << "STA : " << STAId << " AC_BE " <<" nByte : " << nBytes <<"\n"; 
                        STA_Byte += nBytes;

                        Queue = DevUMac->GetQosTxop(AC_BK)->GetWifiMacQueue();
                        nBytes   = Queue->GetNBytes();
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
    
                        Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(STAId)->GetDevice(i));
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
        apLinkTable[i].nr_count = 0;
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

    Time now = Simulator::Now();

    Ptr<WifiNetDevice> lastwifiDev = DynamicCast<WifiNetDevice>(STA.Get(lastSTAId)->GetDevice(link));
    Ptr<StaWifiMac> lastDevMac = DynamicCast<StaWifiMac>(lastwifiDev->GetMac());

    if(apLinkTable[link].HCCA) apLinkTable[link].HCCA = false;
    else lastDevMac->BlockTxOnLink(link, WifiQueueBlockedReason::POWER_SAVE_MODE);
 
    if( now < StopTime && !(q.empty()))
    {
        uint32_t STAId = q.front();
        q.pop();
        Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(STAId)->GetDevice(link));
        Ptr<StaWifiMac> DevMac = DynamicCast<StaWifiMac>(wifiDev->GetMac());
        Ptr<WifiPhy> phy = wifiDev->GetPhy(link);
       
        
        std::set<uint8_t> No;
        No.insert(link);
        DevMac->UnblockTxOnLink(No,WifiQueueBlockedReason::POWER_SAVE_MODE);
        Simulator::Schedule(Seconds(0.0010),&HCCAshchule,STA, link, StopTime, STAId); 
    }
    else
    {
        Ptr<UniformRandomVariable> ran = CreateObject<UniformRandomVariable>();
        for(uint32_t i = 0; i < STA.GetN(); i++) 
        {
            double r = ran->GetValue();
            Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(i)->GetDevice(link));
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
                Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(p.first)->GetDevice(link));
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
                Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(stopSTA)->GetDevice(link));
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
                Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(STA.Get(i)->GetDevice(link));
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
        if(std::abs((lastStart[linkId] - cur.startTime).GetMicroSeconds()) < 9)
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
            if(lastEnd[linkId] > cur.startTime /*表示上一個封包還沒結束但下一個封包已經開始了*/ && std::abs((lastStart[linkId] - cur.startTime).GetMicroSeconds()) > 7 /*上一次與這一次開始已經大於一個slot(非同位碰撞)*/)
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
                    //            << " src Mac : " << cur.srcMac
                    //            << " last Mac :" << lastMacAddr[linkId]
                    //            << " lastEnd : " << lastEnd[linkId].GetSeconds()
                    //            << " StartTime : "<< cur.startTime.GetSeconds()
                    //            << " hidenTime :" << apLinkTable[linkId].h_count << "\n";
                }
            }
            else
            {
                apLinkTable[linkId].unknown += 1;
                // std::cout  << " link :" << linkId << " unknown ! "
                //                << "reason : "   << reason
                //                << "\nsrc Mac : " << cur.srcMac
                //                << " last Mac :" << lastMacAddr[linkId]
                //                << "\nlast Start : " << lastStart[linkId].GetSeconds()
                //                << " lastEnd : " << lastEnd[linkId].GetSeconds()
                //                << " StartTime : "<< cur.startTime.GetSeconds()
                //                << " EndTime : " << cur.endTime.GetSeconds()
                //                << " now : " << now.GetSeconds()
                //                << " type : " << cur.Type
                //                << "\n";
            }
            
        }
        // test:
        // std::cout << "srcMac :  "<< cur.srcMac << " print :" << hdr.GetDuration().GetSeconds()
        //           <<"\nnow : " << now.GetSeconds() <<"    startTime : " << cur.startTime.GetSeconds() << "    endTime : " <<cur.endTime.GetSeconds() 
        //           << "\nlink : " << linkId << " Type : "<< cur.Type << "  reason :  "  << reason  
        //           << " Beacon : "<< now.GetSeconds() / double(0.1) << "\n"
        //           << "---------------------------------------------------------------------------------------------------\n";

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
        for(uint32_t linkId = 0; linkId < 3; linkId++)
        {
            Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(STA.Get(i)->GetDevice(linkId));
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
            // Simulator::Schedule(Seconds(1),[key](){g_activeTx[key].noisedBm = -87.0;}); //noise end -> base noise 
        }
    }
    Simulator::Schedule(Seconds(1),&RandomNoise,STA);
}

// success count++
void
MySucCallback(uint32_t linkId,uint8_t mcs, Ptr<const Packet> p)
{
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    Mac48Address src = hdr.GetAddr2();
    Time now = Simulator::Now();
    // auto type = hdr.GetTypeString();
    // std::cout << "src : " << src << " des : " << hdr.GetAddr1() <<'\n';
    // std::cout << "Packet type : " << type  << " | time : " << now.GetSeconds();
    // std::cout << " | RTS is  : " << hdr.IsRts() << " | linkId : " << linkId << "\n";

    std::pair key = std::make_pair(linkId,src);

    //訊雜比轉化至BER與PLR的數值
    DataRate dr = EhtPhy::GetDataRate(mcs, MHz_u{static_cast<double>(20)}, NanoSeconds(800), 1); // mcs , bandwidth , GI
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
        if(now.GetSeconds() > simStart) totalbit += bits;
        if(hdr.IsRts())
        {
            apLinkTable[linkId].s_count += 1;
        }
    }
    else
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

        if(hdr.IsRts())
        {   
            apLinkTable[linkId].nr_count += 1;
        }
    }
}


//紀錄給一個sta的backoff調整
void
MyBackoffTraceCallback(uint32_t STAid,AcIndex ac, uint32_t backoff, uint8_t linkid)
{
    staLinkBackoff[STAid].link[linkid] += backoff;
    staLinkBackoff[STAid].Pcnt[linkid] += 1;
    // uint32_t avg =  staLinkBackoff[STAid].link[linkid] / staLinkBackoff[STAid].Pcnt[linkid];
    // std::cout << "STA :" << STAid << " linkid : " << unsigned(linkid) 
    //           << " backoff : " << backoff  <<" total : " << staLinkBackoff[STAid].link[linkid]  
    //           << " cnt : " << staLinkBackoff[STAid].Pcnt[linkid]  << " AVG : "<< avg <<'\n';
}

void
updateLinkDelay(NodeContainer wifiStaNodes)
{
    uint32_t avg = 0;
    for(uint32_t dev = 0; dev < wifiStaNodes.GetN(); dev ++)
    {
        for(uint32_t linkid = 0; linkid < 3; linkid++)
        {
            if(staLinkBackoff[dev].Pcnt[linkid] == 0) 
            {
                // std::cout << "STA : " << dev << " link : " << linkid <<" is not use !"<<'\n';
            }
            else
            {
                avg = staLinkBackoff[dev].link[linkid]/ staLinkBackoff[dev].Pcnt[linkid];
                staLinkBackoff[dev].nowDelay[linkid] =  avg;
                // std:: cout << "STA : " << dev << " link : " << linkid <<" TotalBackoff : " << staLinkBackoff[dev].link[linkid]
                //            << " Pcnt : " << staLinkBackoff[dev].Pcnt[linkid]
                //            << " nowDelay : " <<  staLinkBackoff[dev].nowDelay[linkid] <<'\n';
            }

            staLinkBackoff[dev].link[linkid] = 0;
            staLinkBackoff[dev].Pcnt[linkid] = 0;
        }
        // std::cout << "--------------------------------------------------------------" <<'\n';
    }
    Simulator::Schedule(Seconds(0.1),&updateLinkDelay,wifiStaNodes);
}

// 定期產生流量 payloadSize(封包大小),nSTA(幾個STA), acList(優先權類型)
void
SendPacketRandomly(std::vector<std::vector<Ptr<Socket>>> staSockets,uint32_t payloadSize,uint32_t nSTA, AcIndex ac)
{  
    //備註 :尚未安裝 優先權封包 -> 使用addTag上優先權(wait)
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    
        for(uint32_t staId = 0; staId < nSTA; staId++)
        {
            double sum = 0.0;
            double w = 0.0;
            uint32_t b;
            std::vector<double> weight;
    
            for(uint32_t i = 0; i < 3 ; i++)
            {
                b = std::max(staLinkBackoff[staId].nowDelay[i],1u);
                w =(1.0 / b);
                weight.push_back(w);
                sum += w;
            }
    
            double r = rand->GetValue(0, sum);
            double l1 = weight[0];
            double l2 = weight[0]+weight[1];
            uint32_t link = 0 ;
            if(r <= l1)
            {
                link = 0 ;
            }
            else if(l1 < r && r <= l2)
            {
                link = 1 ;
            }
            else if ( l2 < r)
            {
                link = 2 ;
            }
            else
            {
                std::cout << "error flow !!!!!" <<'\n';
            }

            for(uint32_t num = 0; num < 10; num++)
            {
                Ptr<Packet> pkt = Create<Packet>(payloadSize);
                SocketPriorityTag prio;
                prio.SetPriority(ac);
                pkt->AddPacketTag(prio);
                staSockets[staId][link]->Send(pkt);
            }
        }
    
    // Simulator::Schedule(Seconds(0.01), &SendPacketRandomly, staSockets, payloadSize, nSTA, ac);  
    Simulator::Schedule(Seconds(0.0001), &SendPacketRandomly, staSockets, payloadSize, nSTA, ac);  
}

// move model
MobilityHelper InstallApMove(NodeContainer wifiApNode)
{
    MobilityHelper mobilityAp;
    Ptr<ListPositionAllocator> apAlloc = CreateObject<ListPositionAllocator>();
    apAlloc->Add(Vector(100.0,100.0,0.0)); //AP position
    mobilityAp.SetPositionAllocator(apAlloc);
    mobilityAp.SetMobilityModel("ns3::ConstantPositionMobilityModel"); 
    mobilityAp.Install(wifiApNode);
    return mobilityAp;
}

MobilityHelper InstallStaMove(double maxRadius, uint32_t nwifiSTA, NodeContainer wifiStaNodes)
{
  // Create Uniform Distribution STA Position

    MobilityHelper mobilitysta;

    mobilitysta.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                  "X",
                                  StringValue("100.0"),
                                  "Y",
                                  StringValue("100.0"),
                                  "Rho",
                                  StringValue("ns3::UniformRandomVariable[Min=25|Max=35]"));

    mobilitysta.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Mode",
                              StringValue("Time"),
                              "Time",
                              StringValue("2s"),
                              "Speed",
                              StringValue("ns3::ConstantRandomVariable[Constant=2.0]"),
                              "Bounds",
                              StringValue("65|135|65|135"));                          
    
    mobilitysta.Install(wifiStaNodes);
    return mobilitysta;
}

//吞吐量計算
void
calucationThroughput()
{
    Time totalTime = Simulator::Now();
    double duration = totalTime.GetSeconds();
    double throghput = (totalbit/ duration)/1e6;
    std::cout << "Time : " << totalTime.GetSeconds() <<  " throghput : " << throghput << " Mbps" <<"\n";

    Simulator::Schedule(Seconds(0.5),&calucationThroughput);  
}

int main(int argc, char* argv[])
{
    bool udp{true};
    // bool intterf(false);
    bool useRts{true};
    // bool TXOPopen{false};
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
    // uint16_t paddingDelayUsec{32};
    // uint16_t transitionDelayUsec{128};
    Time channelSwitchDelay{"150us"};
    // bool switchAuxPhy{false};//Aux Phy wait primary phy finish
    // uint16_t auxPhyChWidth{20};
    // bool auxPhyTxCapable{true}; // Can UPlink ?

    Time simulationTime{"20s"};
    //simulation time (interference time)
    // double simStartTime = 1.0;

    double frequency{2.4};  // whether the first link operates in the 2.4, 5 or 6 GHz
    double frequency2{5}; // whether the second link operates in the 2.4, 5 or 6 GHz (0 means no
                          // second link exists)
    double frequency3{6}; // whether the third link operates in the 2.4, 5 or 6 GHz (0 means no third link exists)
    
    std::size_t nStations{20};
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
                                                     "DataMode", StringValue(dataModeStr),
                                                     "ControlMode", StringValue(ctrlRateStr));
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
                //STA PHY
                SpectrumWifiPhyHelper phy(nLinks);
                phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
                phy.Set("ChannelSwitchDelay", TimeValue(channelSwitchDelay));

                //手動設定雜訊->關閉原系統雜訊(避免自動降速 (mcs11))
                phy.Set("TxPowerStart", DoubleValue(25.0));
                phy.Set("TxPowerEnd", DoubleValue(25.0));
                phy.Set("TxPowerLevels", UintegerValue(1));
                phy.Set("RxNoiseFigure", DoubleValue(0.0));
                phy.SetErrorRateModel("ns3::YansErrorRateModel");

                mac.SetType("ns3::StaWifiMac", 
                            "Ssid", SsidValue(ssid),
                            "QosSupported", BooleanValue(true),
                            "ActiveProbing",BooleanValue(false));
                
                
                for (uint8_t linkId = 0; linkId < nLinks; linkId++)
                {
                    //error model
                    phy.Set(linkId,"ChannelSettings", StringValue(channelStr[linkId])); // binding link and frequency  : link 0 -> 2.4GHz
                    
                    // //signal loss model - have value ->hidden node
                    Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
                    // Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
                    Ptr<RangePropagationLossModel> lossModel = CreateObject<RangePropagationLossModel>();
                    lossModel->SetAttribute("MaxRange", DoubleValue((maxRadius)));
                    spectrumChannel->AddPropagationLossModel(lossModel);
                    phy.AddChannel(spectrumChannel, freqRanges[linkId]);
                    
                }

                NetDeviceContainer newDevs;
                newDevs = wifi.Install(phy, mac, wifiStaNodes);
                staDevices.Add(newDevs);
                newDevs = wifi.Install(phy, mac, wifiStaNodes);
                staDevices.Add(newDevs);
                newDevs = wifi.Install(phy, mac, wifiStaNodes);
                staDevices.Add(newDevs);

                //device number -> 1 STA have 3 device
                // std::cout<<"total device : " << staDevices.GetN()<<"\n";
                // for(uint32_t i = 0; i < staDevices.GetN(); i++) 
                // {
                //     std::cout << "STA : " <<staDevices.Get(i)->GetNode()->GetId();
                //     std::cout << " Device addr : " << staDevices.Get(i)->GetAddress() <<'\n';
                // }
                
                //STA install check &Block link
                for(uint32_t STA = 0; STA < wifiStaNodes.GetN(); STA++)
                {
                    for(uint32_t linkId = 0; linkId < nLinks; linkId++)
                    {
                        Ptr<WifiNetDevice> wifidev =  DynamicCast<WifiNetDevice>(wifiStaNodes.Get(STA)->GetDevice(linkId));
                        Ptr<WifiMac> devmac = wifidev->GetMac();
                        Mac48Address devaddr = devmac->GetAddress();
                        MHz_u devfre =  wifidev->GetPhy(linkId)->GetFrequency();
                        std::cout<< "STA : " << STA 
                                 << " Device : " << linkId 
                                 << " Address : " << devaddr  
                                 << " Freq : " << devfre 
                                 <<"\n";
                        for(uint32_t i = 1; i <nLinks; i++)
                        {
                            Ptr<StaWifiMac> StaMac = DynamicCast<StaWifiMac>(devmac);
                            StaMac->BlockTxOnLink((linkId + i)%3,WifiQueueBlockedReason::POWER_SAVE_MODE);
                        }
                    }
                } 

                if(AMopen == true)
                {
                    for(uint32_t i = 0; i < nStations;i++)
                    {
                        Ptr<NetDevice> dev;
                        Ptr<WifiNetDevice>wifi_dev;

                        for(uint32_t linkId = 0; linkId < nLinks; linkId++)
                        {
                            dev = wifiStaNodes.Get(i)->GetDevice(linkId);
                            wifi_dev = DynamicCast<WifiNetDevice>(dev);
                            //open A-MPDU
                            // wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(32768));
                            // wifi_dev->GetMac()->SetAttribute("BK_MaxAmpduSize", UintegerValue(32768));
                            // wifi_dev->GetMac()->SetAttribute("VO_MaxAmpduSize", UintegerValue(32768));
                            // wifi_dev->GetMac()->SetAttribute("VI_MaxAmpduSize", UintegerValue(32768));
    
                            wifi_dev->GetMac()->SetAttribute("BE_MaxAmsduSize", UintegerValue(3839)); //3839
                            wifi_dev->GetMac()->SetAttribute("BK_MaxAmsduSize", UintegerValue(3839));
                            wifi_dev->GetMac()->SetAttribute("VO_MaxAmsduSize", UintegerValue(3839));
                            wifi_dev->GetMac()->SetAttribute("VI_MaxAmsduSize", UintegerValue(3839));
                        }
                    }
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

                if(true) //TXOPopen
                {
                    // mac.SetEdca(AC_VO, "TxopLimits", StringValue(txopLimit));
                    // mac.SetEdca(AC_VI,"TxopLimits", StringValue(txopLimit));
                    // mac.SetEdca(AC_BK, "TxopLimits", StringValue(txopLimit));
                    // mac.SetEdca(AC_BE,"TxopLimits", StringValue(txopLimit));
                    mac.SetEdca(AC_BE, "TxopLimits", StringValue("0,0,0"));
                    mac.SetEdca(AC_BK, "TxopLimits", StringValue("0,0,0"));
                    mac.SetEdca(AC_VI, "TxopLimits", StringValue("0,0,0"));
                    mac.SetEdca(AC_VO, "TxopLimits", StringValue("0,0,0"));
                }
            
                //AP install
               
                
                phy.Set("TxPowerStart", DoubleValue(70.0));
                phy.Set("TxPowerEnd", DoubleValue(70.0));
                phy.Set("TxPowerLevels", UintegerValue(1));
                phy.Set("TxGain", DoubleValue(0.0));
                phy.Set("RxGain", DoubleValue(0.0));
                
                apDevice = wifi.Install(phy, mac, wifiAPNode);
                for(uint32_t linkId = 0; linkId < nLinks; linkId++)
                {
                    Ptr<WifiNetDevice> wifiapdev =  DynamicCast<WifiNetDevice>(wifiAPNode.Get(0)->GetDevice(0));
                    Ptr<WifiMac> devmac = wifiapdev->GetMac();
                    Mac48Address devaddr = devmac->GetAddress();
                    MHz_u devfre = wifiapdev->GetPhy(linkId)->GetFrequency();
                    std::cout<< "AP : " << 0
                             << " device : " << 0
                             << " Address : " << devaddr
                             << " Freq : " <<devfre
                             << '\n';
                }
                //avoid disconnect
                Config::Set ("/NodeList/*/DeviceList/*/$ns3::StaWifiMac/MaxMissedBeacons",UintegerValue (std::numeric_limits<uint32_t>::max()));
                Config::Set ("/NodeList/*/DeviceList/*/$ns3::StaWifiMac/ListenInterval",UintegerValue (1));
                Config::Set ("/NodeList/*/DeviceList/*/$ns3::StaWifiMac/AssocRequestTimeout",TimeValue(Seconds(0.1)));
                Config::SetDefault("ns3::StaWifiMac::WaitBeaconTimeout", TimeValue(Seconds(10)));


                // test :check apdevice
                // for(uint32_t linkId = 0; linkId < nLinks; linkId++)
                // {
                //     Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(wifiAPNode.Get(0)->GetDevice(0));
                //     Ptr<WifiMac> devMac = dev->GetMac();
                //     Ptr<WifiPhy> devPhy = dev->GetPhy(linkId);
                //     auto fem = devMac->GetFrameExchangeManager(linkId);
                //     Mac48Address addr = fem->GetAddress();
                //     // std::cout<< "linkId : " << linkId << " addr : " << addr <<"\n";
                // }

                streamNumber += WifiHelper::AssignStreams(apDevice, streamNumber);
                streamNumber += WifiHelper::AssignStreams(staDevices, streamNumber);

                // Set guard interval and MPDU buffer size
                Config::Set(
                    "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                    TimeValue(NanoSeconds(gi)));

                //關閉聚合 :確定
                Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VO_MaxAmpduSize", UintegerValue(0));
                Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_MaxAmpduSize", UintegerValue(0));
                Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize", UintegerValue(0));
                Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BK_MaxAmpduSize", UintegerValue(0));

                // Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MpduBufferSize",
                //             UintegerValue(mpduBufferSize));

                // mobility
                MobilityHelper mobilitySTA;
                MobilityHelper mobilityAP;

                mobilitySTA =InstallStaMove(maxRadius,nStations,wifiStaNodes);
                mobilityAP = InstallApMove(wifiAPNode);
                
    
                
                if (udp)
                {
                    std::vector<std::vector<Ptr<Socket>>> staSockets;
                    PacketSocketHelper socketHelper;
                    socketHelper.Install(wifiStaNodes);
                    std::vector<AcIndex> acList ={AC_VO,AC_VI,AC_BE,AC_BK};  
                    for (std::size_t staId = 0; staId < nStations; staId++)
                    {
                        std::vector<Ptr<Socket>> socketsPerSta;
                        for(uint32_t devIdx = 0; devIdx < 3; devIdx++)
                        {
                            Ptr<WifiNetDevice> dev =  DynamicCast<WifiNetDevice>(wifiStaNodes.Get(staId)->GetDevice(devIdx));
                            PacketSocketAddress addr;
                            addr.SetSingleDevice(dev->GetIfIndex()); 
                            addr.SetPhysicalAddress(Mac48Address::ConvertFrom(wifiAPNode.Get(0)->GetDevice(0)->GetAddress())); 
                            addr.SetProtocol(17);
                            Ptr<Socket> sock = Socket::CreateSocket(wifiStaNodes.Get(staId), PacketSocketFactory::GetTypeId());
                            sock->Bind();     
                            sock->Connect(addr);
                            socketsPerSta.push_back(sock);
                            
                        }
                        staSockets.push_back(socketsPerSta);
                    }
                    for(AcIndex ac : acList)
                    {
                        Simulator::Schedule(Seconds(simStart), &SendPacketRandomly, staSockets, payloadSize, nStations,ac);
                    }
                    
                }
            
                //--------------------------system info and function(my)--------------------------------------
            
        
            
            // flow information monitor (need to install before simulator::Run() )
                FlowMonitorHelper flowmon;
                Ptr<FlowMonitor> monitor = flowmon.InstallAll();

            // STA掛勾三條link是否成功 
            //STAaddr_linkid
            for(u_int32_t i = 0; i < nStations ;++i)
            {
                Ptr<Node> sta = wifiStaNodes.Get(i);
                for(uint8_t Linkid = 0; Linkid <nLinks; ++Linkid)
                {
                    Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(sta->GetDevice(Linkid)); //sta的第 i個設備
                    Ptr<WifiMac> devmac = mloDevice->GetMac();
                    Ptr<WifiPhy> devphy = mloDevice->GetPhy(Linkid); //第i個設備的 第i個物理層
                    devphy->SetSlot(MicroSeconds(9));
                    auto fem = devmac->GetFrameExchangeManager(Linkid); // 第i個設備的 第i個link 管理
                    Mac48Address LowerMacAddress = fem->GetAddress(); 
                    STAlinkTable[LowerMacAddress] = i; //紀錄實際有在使用的link  位址-> STA i
                    std::vector<AcIndex> acList =  {AC_BE,AC_BK,AC_VI,AC_VO};
                    staLinkBackoff[i].link.push_back(0);
                    staLinkBackoff[i].Pcnt.push_back(0);
                    staLinkBackoff[i].nowDelay.push_back(0);
                    for(AcIndex ac : acList)
                    {
                        Ptr<QosTxop> devtxop = devmac->GetQosTxop(ac); //競爭dcf/edca模組
                        devtxop->TraceConnectWithoutContext("BackoffTrace", MakeBoundCallback(&MyBackoffTraceCallback,i,ac));
                    }  

                    // std::cout << " staDevice : " << LowerMacAddress << " is : " << STAlinkTable.count(LowerMacAddress)
                    //           << " Staid : " << STAlinkTable[LowerMacAddress] << "\n";
                    // std::cout << " mac : "<<DynamicCast<WifiNetDevice>(wifiStaNodes.Get(STAlinkTable[LowerMacAddress])->GetDevice(Linkid))->GetMac()->GetAddress()
                    //           <<"\n";
                }
            }

            //trace
            //物理層查詢 (ap) 
            for(uint32_t i = 0; i < nLinks; ++i)
            {
                const Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(apDevice.Get(0));
                Ptr<WifiPhy> wifiphy = mloDevice->GetPhy(i);
                wifiphy->TraceConnectWithoutContext("MonitorSnifferRx",MakeBoundCallback(&SnifferRxCallback, mcs));
                wifiphy->TraceConnectWithoutContext("PhyRxDrop", MakeBoundCallback(&MyDropCallback,i));
                wifiphy->TraceConnectWithoutContext("PhyRxEnd", MakeBoundCallback(&MySucCallback, i,mcs));
                
            }
            
            //物理層傳送的封包資訊(STA)
            for(uint32_t i = 0; i < nStations; ++i)
            {
                for(uint32_t linkId = 0; linkId < nLinks; linkId++)
                {
                   const Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(wifiStaNodes.Get(i)->GetDevice(linkId)); //STA i 的設備 linkId
                   Ptr<WifiPhy> phy = mloDevice->GetPhy(linkId);
                   Ptr<WifiMac> mac = mloDevice->GetMac();
                //    std::cout <<  mloDevice->GetMac()->GetFrameExchangeManager(linkId)->GetAddress() <<'\n';
                //    std::cout << "Binding Trace: STA " << i
                //              << " linkId " << linkId
                //              << " device: " << mloDevice
                //              << " phy: " << phy << std::endl;
                   phy->TraceConnectWithoutContext("PhyTxBegin",MakeBoundCallback(&PhyTxBeginTrace,mloDevice,linkId));

               }   
            }

            //start--------------------------------------
                Simulator::Schedule(Seconds(0.0),&ClearElement,wifiStaNodes);
                Simulator::Schedule(Seconds(simStart),&RandomNoise,wifiStaNodes);
                Simulator::Schedule(Seconds(simStart),&function2,wifiStaNodes);
                Simulator::Schedule(Seconds(simStart),&updateLinkDelay,wifiStaNodes);
                Simulator::Schedule(Seconds(simStart),&calucationThroughput);                
                Simulator::Stop(simulationTime + Seconds(1)); //end time (need define before simulator::Run())
                Simulator::Run();
                Simulator::Destroy();
               
                          
            }
        }
    }
    return 0;
}
