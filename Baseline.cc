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
#include "ns3/traffic-control-module.h"
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
    uint32_t h_count = 0;
    uint32_t s_count = 0;
    uint32_t ds_count = 0;
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

//global
double simStart = 5.0; //實際的開始時間
//每次發生傳送的封包個別資訊
std::map<std::pair<uint32_t, Mac48Address>,TxRecord> g_activeTx;

//AP紀錄各種傳輸異常與參數資訊
std::vector<LinkTable> apLinkTable(3); //count 


//吞吐量
uint32_t totalbit = 0; 
std::map<AcIndex,uint32_t>  totalActhroughput;
std::map<AcIndex,uint32_t>  totalAcpkt;

//RTA
uint32_t RTAthroughput;
uint32_t RTAAcpkt;
double RTAAvgAcDelay;
uint32_t RTADelayAcpkt;

//延遲相關:
uint32_t totalPacket = 0;
double totaldelay = 0;
double perAvgDelay  = 0; // 總封包/總延遲 ->平均延遲 
uint32_t DelayCount = 0;
std::map<AcIndex,double>  perAvgAcDelay;
std::map<AcIndex,uint32_t>  totalDelayAcpkt;
// 因比對collision & hidden node 所以需要記住link i 上的 上一個STA起始與位址
std::vector<Time> lastStart(3);
std::vector<Time> lastEnd(3);
std::vector<Mac48Address> lastMacAddr(3);
std::vector<std::map<Time,bool>> timeFlag(3);
//提供物理層快速判別STD Id的方法
std::map<Mac48Address,uint32_t> STAlinkTable; // phylink -> STA Id

// Breach 更新
struct SlaSample {
    ns3::Time t;     // 取樣時間（用 ACK 到達時刻）
    uint8_t    b;    // breach_bit ∈ {0,1}，STA_Delay > D_TH ? 1 : 0
};
// ---- 參數 ----
static const double TSLA_SEC = 1.0;     // 時間窗長度（論文精神，建議 1s 起）
static const double EPS      = 1e-9;    // 除零保護
// ---- 每 STA 狀態 ----
std::vector<std::deque<SlaSample>> g_slaWin(5);       // g_slaWin[staId] = 最近 TSLA 秒內樣本
std::vector<uint32_t> g_winCount(5);     // 每 STA 視窗樣本數
std::vector<uint32_t> g_winBreach(5);    // 每 STA 視窗內 breach=1 的個數
//全部優先權的delay設定
std::vector<uint32_t> DelayTh ={5,100,200,1000,1000}; 

Time beaconInterval = MilliSeconds (300);
double HN_threshold = 0.05;

//my function 
WifiPhyBand 
GetBandFromFreq(uint16_t freq)
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
    Ptr<WifiPhy> phy = dev->GetPhy(0);
    MHz_u allowedWidth = phy->GetChannelWidth();
    Ptr<WifiRemoteStationManager> StaManger = dev->GetRemoteStationManager(0); 
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
                  << " data success : " << apLinkTable[i].ds_count 
                  << " unknown : " << apLinkTable[i].unknown
                  << " total_real : " << apLinkTable[i].c_count + apLinkTable[i].h_count + apLinkTable[i].n_count + 
                                         apLinkTable[i].s_count + apLinkTable[i].unknown;

        double total = apLinkTable[i].c_count + apLinkTable[i].h_count + apLinkTable[i].s_count;

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
        apLinkTable[i].ds_count = 0;
        apLinkTable[i].unknown = 0;
        apLinkTable[i].Pab = Pab;
        apLinkTable[i].hideTable.clear();
        apLinkTable[i].NoiseTable.clear();
        std::cout << "\n---------------------------------------------------------------"<<"\n";
    }
    Simulator::Schedule(beaconInterval, &ClearElement,STA);
}

void
// collect failure count++  (check-ok)
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
            if(lastMacAddr[linkId]!= cur.srcMac && !(timeFlag[linkId][lastStart[linkId]]))
            {
                timeFlag[linkId][lastStart[linkId]] = true;
                apLinkTable[linkId].c_count += 1;
                // test:
                // std::cout << " link :" << linkId << " collision ! " << " LastStart : " << lastStart[linkId].GetSeconds()
                //           << " StartTime : "<< cur.startTime.GetSeconds()  << "\n"
                //           << " Address : "  << cur.srcMac << " lastAddress : " << lastMacAddr[linkId] <<"\n";
            }
        }
        else
        {   /*表示上一個封包還沒結束但下一個封包已經開始了*/ /*上一次與這一次開始已經大於一個slot(非同位碰撞)*/
            if((lastEnd[linkId]  - cur.startTime).GetMicroSeconds() > 1 && (cur.startTime - lastStart[linkId]).GetMicroSeconds() > 9  && reason != WifiPhyRxfailureReason::TXING) 
            {
                if(lastMacAddr[linkId]!= cur.srcMac )
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
                    //            << " Type : "<< cur.Type
                    //            << " src Mac : " << cur.srcMac
                    //            << " last Mac :" << lastMacAddr[linkId]
                    //            << " lastEnd : " << lastEnd[linkId].GetSeconds() //a2
                    //            << " StartTime : "<< cur.startTime.GetSeconds() //b1
                    //            << " B1 - A1 : "     << (cur.startTime - lastStart[linkId]).GetMicroSeconds()
                    //            << " A2 - B1 : "     << std::abs((lastEnd[linkId]  - cur.startTime).GetMicroSeconds())
                    //            << " hidenTime :" << apLinkTable[linkId].h_count << "\n";

                    // std::cout << "srcMac :  "<< cur.srcMac << " print :" << hdr.GetDuration().GetSeconds()
                    //           <<"\nnow : " << now.GetSeconds() <<"    startTime : " << cur.startTime.GetSeconds() << "    endTime : " <<cur.endTime.GetSeconds() 
                    //           << "\nlink : " << linkId << " Type : "<< cur.Type << "  reason :  "  << reason <<"\n" 
                    //           << "---------------------------------------------------------------------------------------------------\n";
                }
            } 
        }
        // test:
        // std::cout << "srcMac :  "<< cur.srcMac << " print :" << hdr.GetDuration().GetSeconds()
        //           <<"\nnow : " << now.GetSeconds() <<"    startTime : " << cur.startTime.GetSeconds() << "    endTime : " <<cur.endTime.GetSeconds() 
        //           << "\nlink : " << linkId << " Type : "<< cur.Type << "  reason :  "  << reason  
        //           << " Beacon : "<< now.GetSeconds() / double(0.1) << "\n"
        //           << "---------------------------------------------------------------------------------------------------\n";

        if(lastStart[linkId] != cur.startTime) timeFlag[linkId][cur.startTime] = false; // 新時間尚未被統計 因pc= 1(1-tou)^n-1 ->因此只統計一次
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
// (check-ok)
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
            Ptr<SpectrumWifiPhy> phy =  DynamicCast<SpectrumWifiPhy>(dev->GetPhy(0));   if (phy == nullptr) continue;
            Ptr<WifiMac>  mac = DynamicCast<WifiMac>(dev->GetMac());
            auto fem = mac->GetFrameExchangeManager(0);
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


// success count++ (check-ok)
void
MySucCallback(uint32_t linkId,uint8_t mcs,Ptr<const Packet> p)
{
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    Mac48Address src = hdr.GetAddr2();
    //現在時間
    Time now = Simulator::Now();
    //產生時間標籤
    TimestampTag timestamp;
    p->FindFirstMatchingByteTag (timestamp);
    Time tx = timestamp.GetTimestamp();
    double start = tx.ToDouble(Time::MS);

    if(start > 5000 && (hdr.IsData()||hdr.IsQosData()))
    {
        Time txdelay = Simulator::Now() - tx;
        double delay = txdelay.ToDouble(Time::MS);
        SocketPriorityTag prio;
        p->FindFirstMatchingByteTag (prio);              
        AcIndex ac = QosUtilsMapTidToAc(prio.GetPriority());
        //總延遲
        totaldelay += delay;
        totalPacket ++;
        FlowIdTag flowTag;
        if(p->PeekPacketTag(flowTag))
        {
            uint32_t id = flowTag.GetFlowId();
            if(id != 0)
            {
                perAvgAcDelay[ac] += delay;
                totalDelayAcpkt[ac]++;
            }else
            {
                RTAAvgAcDelay +=delay;
                RTADelayAcpkt++;
            }
        }

        //整塊都是SLA
        if(p->PeekPacketTag(flowTag))
        {
            uint32_t id = flowTag.GetFlowId();
            uint8_t breach = (delay > DelayTh[id]) ? 1 : 0;
            //紀錄時間now下 STA staId 的 第id個優先權的延遲時間用於進行滑動視窗
            g_slaWin[id].push_back({now, breach}); 
            g_winCount[id] += 1;
            g_winBreach[id] += breach;
            ns3::Time cutoff = now - ns3::Seconds(TSLA_SEC);
            while(!g_slaWin[id].empty() && g_slaWin[id].front().t < cutoff){
                g_winCount[id]  -= 1;
                g_winBreach[id] -= g_slaWin[id].front().b;
                g_slaWin[id].pop_front();
            }
        }  
        // double nowTime = now.ToDouble(Time::MS);
        // std::cout<< " now : " << nowTime << " Start : " << start  <<" delay : " << delay << '\n';
    }
    
    // auto type = hdr.GetTypeString();
    // std::cout << "src : " << src << " des : " << hdr.GetAddr1() <<'\n';
    // std::cout << "Packet type : " << type  << " | time : " << now.GetSeconds();
    // std::cout << " bits : "  << p-> GetSize() * 8 <<'\n';
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
    if(now.GetSeconds() > simStart)
    {
        if(hdr.IsData()||hdr.IsQosData())
        {
            FlowIdTag flowTag;
            uint32_t id = 5;
            if(p->PeekPacketTag(flowTag)) id = flowTag.GetFlowId();

            totalbit += 272;  //RTS (160)  + CTS (112)
            SocketPriorityTag prio;
            p->FindFirstMatchingByteTag (prio);              
            AcIndex ac = QosUtilsMapTidToAc(prio.GetPriority());

            if(id!=0)totalActhroughput[ac] += 272;
            else RTAthroughput += 272;
            
            if(r < SUC)
            {
                totalbit += bits;
                totalbit += 112 ; // ACK
                apLinkTable[linkId].ds_count +=1;

                 if(id !=0) //RTA分開算
                {
                    totalActhroughput[ac] += bits;
                    totalActhroughput[ac] += 112;
                    totalAcpkt[ac]++;
                }else
                {
                    RTAthroughput += bits;
                    RTAthroughput +=112;
                    RTAAcpkt++;
                    // std:: cout << " RTA throughput :" << RTAthroughput <<"\n";
                }
                // std::cout << "src : " << src
                //           << " Type : " << hdr.GetTypeString()
                //           << " Time : " <<  now.GetSeconds() <<" us"
                //           << " duration : " << (g_activeTx[key].endTime - g_activeTx[key].startTime).GetMicroSeconds()
                //           << " Packet size : " << p->GetSize()
                //           << " Data : " << bits <<" bit"
                //           <<"\n";
            }else
            {
                //把noise導致的 STA放入N_Table 含任何rts/ack/data
                apLinkTable[linkId].n_count += 1;
                if(apLinkTable[linkId].NoiseTable.count(src) > 0)  apLinkTable[linkId].NoiseTable[src] += 1;
                else apLinkTable[linkId].NoiseTable.emplace(src, 1);
                //扣除噪音導致的損失:
                SocketPriorityTag prio;
                p->FindFirstMatchingByteTag (prio);              
                AcIndex ac = QosUtilsMapTidToAc(prio.GetPriority());
                totalPacket --;
                totalDelayAcpkt[ac]--;
            }

            // std::cout << "key : " << key 
            //           << " noise : " << g_activeTx[key].noisedBm 
            //           << " signal : " <<  g_activeTx[key].signaldBm 
            //           << " snr_dB : " << snr_dB
            //           << " BER : "    <<  BER
            //           << "\nbits : "  << bits
            //           << " SUC : "    <<  SUC
            //           <<"\n";
        }
        else if(hdr.IsRts()) apLinkTable[linkId].s_count += 1; 
    }
}






// 定期產生流量 payloadSize(封包大小),nSTA(幾個STA), acList(優先權類型)
void
SendPacket(std::vector<std::vector<Ptr<Socket>>> staSockets,uint32_t staId, std::string name,std::map<std::string, double>& lambdamap)
{  
    // Link selection
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    double r = rand->GetValue(0, 1);
    double l1 = 0.33;
    double l2 = 0.66;
    uint32_t link = 0 ;
    
    if(r <= l1)link = 0 ;
    else if(l1 < r && r <= l2)link = 1 ;
    else if ( l2 < r)link = 2 ;
    else std::cout << "error flow !!!!!" <<'\n';

    // std::cout << " link is " << link << " ac : " << name << "\n";
    /*
    流量產生
    type     ac lamda size       trafficId
    RTA      VO  140  358   Byte  0
    --------------------- 
    VO       VO  50   70    Byte  1
    ---------------------
    VI       VI  60   2083  Byte  2
    ---------------------
    BE       BE  100  1500  Byte  3
    ---------------------
    BK       BK  996  1500  Byte  5
    */
   
   Ptr<Packet> pkt;
   SocketPriorityTag prio;
   TimestampTag timestamp;
   timestamp.SetTimestamp(Simulator::Now());
   //std::cout << "name : " << name << " ac : " << ac << " lambda : " <<  lambdamap[name]<<"\n";
   if(name == "RTA")
    {
       pkt = Create<Packet>(358); 
       prio.SetPriority(6); //VO
       pkt->AddPacketTag(prio);
       pkt->AddByteTag(timestamp);
       pkt->AddByteTag(prio);

       FlowIdTag flowTag;
       flowTag.SetFlowId(0);
       pkt->AddPacketTag(flowTag);
       staSockets[staId][link]->Send(pkt);
    }
    else if(name == "VO")
    {
       pkt = Create<Packet>(182);  
       prio.SetPriority(6); //VO
       pkt->AddPacketTag(prio);
       pkt->AddByteTag(timestamp);
       pkt->AddByteTag(prio);
       FlowIdTag flowTag;
       flowTag.SetFlowId(1);
       pkt->AddPacketTag(flowTag);
       staSockets[staId][link]->Send(pkt);
    }
    else if (name == "VI")
    {
        pkt = Create<Packet>(2083); 
        prio.SetPriority(4); //VI
        pkt->AddPacketTag(prio);
        pkt->AddByteTag(timestamp);
        pkt->AddByteTag(prio);
        FlowIdTag flowTag;
        flowTag.SetFlowId(2);
        pkt->AddPacketTag(flowTag);
        staSockets[staId][link]->Send(pkt);            
    } 
    else if (name == "BE") 
    {
        pkt = Create<Packet>(1500);
        prio.SetPriority(0); //BE
        pkt->AddPacketTag(prio);
        pkt->AddByteTag(timestamp);
        pkt->AddByteTag(prio);
        FlowIdTag flowTag;
        flowTag.SetFlowId(3);
        pkt->AddPacketTag(flowTag);
        staSockets[staId][link]->Send(pkt);                
    }
    else if( name == "BK") 
    {
        pkt = Create<Packet>(1500);
        prio.SetPriority(1); //BK
        pkt->AddPacketTag(prio);
        pkt->AddByteTag(timestamp);
        pkt->AddByteTag(prio);
        FlowIdTag flowTag;
        flowTag.SetFlowId(4);
        pkt->AddPacketTag(flowTag);
        staSockets[staId][link]->Send(pkt);                
    }
   

    // Packet genaration interval
    Ptr<ExponentialRandomVariable> exp = CreateObject<ExponentialRandomVariable>();
    double lambda =  lambdamap[name];
    exp->SetAttribute("Mean", DoubleValue(1.0 / lambda));
    double nextTime = exp->GetValue();
    Simulator::Schedule(Seconds(nextTime), &SendPacket, staSockets, staId, name,lambdamap); 
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
                                  StringValue("ns3::UniformRandomVariable[Min=5|Max=35]"));

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
calucationThroughput(std::ofstream& outFile)
{
    Time totalTime = Simulator::Now();
    double duration = totalTime.GetSeconds() - simStart;
    double throghput = (totalbit/ duration)/1e6;

    //紀錄 time ,  throughput, totalbit , duration
    std::cout << "\ntotalbit : " << totalbit <<  " duration : "<< duration <<'\n';
    std::cout << "Time : " << totalTime.GetSeconds() <<  " throghput : " << throghput << " Mbps" <<"\n";
    outFile << totalTime.GetSeconds() <<", " << throghput <<",";
    outFile << totalbit << "," << duration;

    double RTATH = (RTAthroughput/duration) / 1e6 ;
    std::cout << "RTA " << "Total Bit : " <<  RTAthroughput <<'\n'
              << "Throughput : "<< RTATH << " Mbps"
              << " Packet number : " << RTAAcpkt << '\n';
    outFile << "," <<  RTATH; 

    std::vector<AcIndex> acList ={AC_VO,AC_VI,AC_BE,AC_BK}; 
    for(auto ac: acList)
    {
        double ACthroghput = (totalActhroughput[ac] / duration)/1e6;
        std::cout << "ac : " << ac <<  " Total Bit : "<< totalActhroughput[ac]<<"\n";
        std::cout << "Throghput : " << ACthroghput << " Mbps";
        std::cout << " Packet number :" <<totalAcpkt[ac] << "\n";
        outFile <<"," <<ACthroghput;
    }
    outFile<<'\n';
    Simulator::Schedule(Seconds(0.5),&calucationThroughput,std::ref(outFile));  
}

//延遲計算
void
calucationDelay(std::ofstream& outFile)
{
    double nowdelay;
    Time totalTime = Simulator::Now();
    if(totalPacket !=0) nowdelay = totaldelay / totalPacket;
    DelayCount++;
    perAvgDelay += nowdelay;

    std::cout<< "totaldelay : " << totaldelay << " totalPacket : " << totalPacket; //只記住一輪內全部的封包與延遲
    std::cout<< " perAvgDelay : " << perAvgDelay << " DelayCount : " << DelayCount <<'\n'; //紀錄每一輪的平均時間 並且除以n輪
    outFile << totalTime.GetSeconds() << "," <<  perAvgDelay << ", " << DelayCount;
    //RTA
    double RTAdelay;
    if( RTAAvgAcDelay!=0) RTAdelay = RTAAvgAcDelay / RTADelayAcpkt;
    std::cout << " RTA " << "Total  Time : " << RTAAvgAcDelay;
    std::cout << " Delay : " << RTAdelay << " ms";
    std::cout << " Packet number : " << RTADelayAcpkt <<'\n';
    outFile <<", " <<RTAdelay;
    //RTA
    std::vector<AcIndex> acList ={AC_VO,AC_VI,AC_BE,AC_BK}; 
    for(auto ac: acList)
    {
        double ACdelay;
        if( totalDelayAcpkt[ac] !=0) ACdelay = perAvgAcDelay[ac] / totalDelayAcpkt[ac];
        std::cout << "ac : " << ac <<  " Total Time : "<< perAvgAcDelay[ac];
        std::cout << " Delay : " << ACdelay << " ms";
        std::cout << " Packet number :" <<totalDelayAcpkt[ac] << "\n";
        outFile <<","<< ACdelay;
     
    }
    //RTA
    double BreachWin = 0.0;
    for(uint32_t i = 0;  i < 5; i++)
    {
        if (g_winCount[i] > 0) {
            BreachWin = static_cast<double>(g_winBreach[i]) /
            static_cast<double>(g_winCount[i]);
        }
        std::cout << "Priority :" << i << " Breach : " << BreachWin << '\n';
        outFile <<"," << BreachWin ;
    }
    //RTA
    outFile << '\n';
    totaldelay = 0;
    totalPacket = 0;
    Simulator::Schedule(Seconds(0.5),&calucationDelay,std::ref(outFile));  
}

int main(int argc, char* argv[])
{
    std::ofstream tputFile("./Data/throughput_s_a/STA36/baseline_T_4T.csv");
    std::ofstream tputFile1("./Data/throughput_s_a/STA36/baseline_D_TD.csv");
    tputFile << "Time,Throughput(Mbps),totalbit,duration,RTA,VO,VI,BE,BK" << "\n";
    tputFile1 << "Time,perAvgDelay,DelayCount,RTA,VO,VI,BE,BK,Breach" <<'\n';
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

    Time simulationTime{"14.5s"};
    //simulation time (interference time)
    // double simStartTime = 1.0;

    double frequency{2.4};  // whether the first link operates in the 2.4, 5 or 6 GHz
    double frequency2{5}; // whether the second link operates in the 2.4, 5 or 6 GHz (0 means no
                          // second link exists)
    double frequency3{6}; // whether the third link operates in the 2.4, 5 or 6 GHz (0 means no third link exists)
    
    std::string dlAckSeqType{"NO-OFDMA"};
    bool enableUlOfdma{false};
    bool enableBsrp{false};
    std::string mcsStr;
    std::vector<uint64_t> mcsValues;
    Time tputInterval{0}; // interval for detailed throughput measurement
    Time accessReqInterval{0};
    
    // BSS size
    double maxRadius = 50.0;
    
    //mcs & ChannelWidth gi
    uint8_t mcs = 11;
    mcsValues.push_back(mcs); 
    
    int minChannelWidth = 40;
    int maxGi = 800;
    
    std::size_t nStations{36};
    tputFile << "nStations : " << nStations<<'\n';
    
    std::vector<AcIndex> acList ={AC_VO,AC_VI,AC_BE,AC_BK}; 
    for(auto ac: acList)totalActhroughput[ac] = 0;
    // std::vector<std::string> trafficId={"VI"};
    std::vector<std::string> trafficId={"RTA","VO","VI","BE","BK"};
    std::map<std::string, double> lambda = {
    {"RTA",140}, //pri:1
    {"VO", 50},  //id:2
    {"VI", 60},  //
    {"BE", 10},
    {"BK", 600},
    };


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
            int width = minChannelWidth;
            const std::string widthStr = std::to_string(width);
            const auto segmentWidthStr = widthStr;
            int gi = maxGi; // Nanoseconds
            
            // WiFi Node Create
            NodeContainer wifiAPNode;
            NetDeviceContainer apDevice;
            wifiAPNode.Create(1);

            NodeContainer wifiStaNodes;
            NetDeviceContainer staDevices;
            wifiStaNodes.Create(nStations);
            

            WifiMacHelper mac;
            //STA /AP使用
            WifiHelper wifis1;
            WifiHelper wifis2;
            WifiHelper wifis3;

            wifis1.SetStandard(WIFI_STANDARD_80211be);
            wifis2.SetStandard(WIFI_STANDARD_80211be);
            wifis3.SetStandard(WIFI_STANDARD_80211be);
            std::array<std::string, 3> channelStr;
            std::array<FrequencyRange, 3> freqRanges;
            uint8_t nLinks = 0;
            std::string dataModeStr = "EhtMcs" + std::to_string(mcs);
            std::string ctrlRateStr;
            uint64_t nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs) / 1e6;

            if (frequency2 == frequency || frequency3 == frequency ||
                (frequency3 != 0 && frequency3 == frequency2))
            {
                NS_FATAL_ERROR("Frequency values must be unique!");//避免重複
            }


            // 設定三個link在不同frequency上
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
                    ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                    Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                    DoubleValue(48));
                    
                    wifis3.SetRemoteStationManager(
                                                "ns3::ConstantRateWifiManager",//Can't change Rate  for SNR
                                                "DataMode",StringValue(dataModeStr),
                                                "ControlMode",StringValue(ctrlRateStr));                             
                }
                else if (freq == 5)
                {
                    channelStr[nLinks] += "BAND_5GHZ, 0}";
                    freqRanges[nLinks] = WIFI_SPECTRUM_5_GHZ;
                    ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                    wifis2.SetRemoteStationManager(
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
                    wifis1.SetRemoteStationManager(
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

            std::vector<Ptr<MultiModelSpectrumChannel>> sharedCh;

            sharedCh.resize(nLinks);

            //產生共用spectrumChannel->不可分割STA與AP
            for (uint8_t linkId = 0; linkId < nLinks; linkId++)
            {
                sharedCh[linkId] = CreateObject<MultiModelSpectrumChannel>();
                // Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
                Ptr<RangePropagationLossModel> lossModel = CreateObject<RangePropagationLossModel>();
                lossModel->SetAttribute("MaxRange", DoubleValue((100))); //meter
                sharedCh[linkId]->AddPropagationLossModel(lossModel);
            }

            //STA PHY
            Ssid ssid = Ssid("ns3-80211be");
            mac.SetType("ns3::StaWifiMac", 
                        "Ssid", SsidValue(ssid),
                        "QosSupported", BooleanValue(true),
                        "ActiveProbing",BooleanValue(false));
            
            

            NetDeviceContainer newDevs;
            for (uint8_t linkId = 0; linkId < nLinks; linkId++)
            {
                SpectrumWifiPhyHelper phySta(1);
                phySta.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
                phySta.Set("ChannelSwitchDelay", TimeValue(channelSwitchDelay));

                //手動設定雜訊->關閉原系統雜訊(避免自動降速 (mcs11))
                phySta.Set("TxPowerStart", DoubleValue(25.0));
                phySta.Set("TxPowerEnd", DoubleValue(25.0));
                phySta.Set("TxPowerLevels", UintegerValue(1));
                phySta.Set("RxNoiseFigure", DoubleValue(0.0));
                phySta.SetErrorRateModel("ns3::YansErrorRateModel");

                mac.SetType("ns3::StaWifiMac", 
                            "Ssid", SsidValue(ssid),
                            "QosSupported", BooleanValue(true),
                            "ActiveProbing",BooleanValue(false));

                //error model
                phySta.Set("ChannelSettings", StringValue(channelStr[linkId])); // binding link and frequency  : link 0 -> 2.4GHz
                phySta.AddChannel(sharedCh[linkId], freqRanges[linkId]);
                
                
                if(linkId == 0)
                {
                    newDevs = wifis1.Install(phySta, mac, wifiStaNodes); //2.4
                    staDevices.Add(newDevs);
                }else if (linkId == 1)
                {
                    newDevs = wifis2.Install(phySta, mac, wifiStaNodes); //5
                    staDevices.Add(newDevs);
                }else if(linkId == 2)
                {
                    newDevs = wifis3.Install(phySta, mac, wifiStaNodes); //6
                    staDevices.Add(newDevs);
                }
                
            }

            //check : device number -> 1 STA have 3 device
            std::cout<<"total device : " << staDevices.GetN()<<"\n";
            for(uint32_t i = 0; i < staDevices.GetN(); i++) 
            {
                std::cout << "STA : " <<staDevices.Get(i)->GetNode()->GetId(); //屬於哪個node
                std::cout << " Device addr : " << staDevices.Get(i)->GetAddress() <<'\n'; //他的addr
            }
            

            //check :STA install check &Block link
            // for(uint32_t STA = 0; STA < wifiStaNodes.GetN(); STA++)
            // {
            //     for(uint32_t linkId = 0; linkId < nLinks; linkId++)
            //     {
            //         //第i個STA的第link個設備
            //         Ptr<WifiNetDevice> wifidev =  DynamicCast<WifiNetDevice>(wifiStaNodes.Get(STA)->GetDevice(linkId));
            //         Ptr<WifiMac> devmac = wifidev->GetMac();
            //         Mac48Address devaddr = devmac->GetAddress();
            //         MHz_u devfre =  wifidev->GetPhy(0)->GetFrequency();
            //         std::cout<< "STA : " << STA 
            //                  << " Device : " << linkId 
            //                  << " Address : " << devaddr  
            //                  << " Freq : " << devfre 
            //                  <<"\n";
            //     }
            // } 

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

            mac.SetEdca(AC_BE, "TxopLimits", StringValue("0"));
            mac.SetEdca(AC_BK, "TxopLimits", StringValue("0"));
            mac.SetEdca(AC_VI, "TxopLimits", StringValue("0"));
            mac.SetEdca(AC_VO, "TxopLimits", StringValue("0"));

            //AP install
            for (uint32_t linkId = 0; linkId < nLinks; linkId++)
            {
                SpectrumWifiPhyHelper phy(1);
                phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
                phy.Set("ChannelSwitchDelay", TimeValue(channelSwitchDelay));
                
                //手動設定雜訊->關閉原系統雜訊(避免自動降速 (mcs11))
                phy.SetErrorRateModel("ns3::YansErrorRateModel");
                phy.Set("RxNoiseFigure", DoubleValue(0.0));
                phy.Set("TxPowerStart", DoubleValue(25.0));
                phy.Set("TxPowerEnd", DoubleValue(25.0));
                phy.Set("TxPowerLevels", UintegerValue(1));
                phy.Set("TxGain", DoubleValue(0.0));
                phy.Set("RxGain", DoubleValue(0.0));
                //error model
                phy.Set("ChannelSettings", StringValue(channelStr[linkId])); // binding link and frequency  : link 0 -> 2.4GHz
                phy.AddChannel(sharedCh[linkId], freqRanges[linkId]);

                if(linkId == 0)
                {
                    newDevs = wifis1.Install(phy, mac, wifiAPNode); //2.4
                    apDevice.Add(newDevs);
                }else if (linkId == 1)
                {
                    newDevs = wifis2.Install(phy, mac, wifiAPNode); //5
                    apDevice.Add(newDevs);
                }else if(linkId == 2)
                {
                    newDevs = wifis3.Install(phy, mac, wifiAPNode); //6
                    apDevice.Add(newDevs);
                }

                std::cout<< "linkId : " << linkId <<  '\n';
            }
            
            //check
            for(uint32_t linkId = 0; linkId < nLinks; linkId++)
            {
                Ptr<WifiNetDevice> wifiapdev =  DynamicCast<WifiNetDevice>(wifiAPNode.Get(0)->GetDevice(linkId));
                Ptr<WifiMac> devmac = wifiapdev->GetMac();
                Mac48Address devaddr = devmac->GetAddress();
                MHz_u devfre = wifiapdev->GetPhy(0)->GetFrequency();
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
            //     Ptr<WifiPhy> devPhy = dev->GetPhy(0);
            //     auto fem = devMac->GetFrameExchangeManager(0);
            //     Mac48Address addr = fem->GetAddress();
            //     std::cout<< "linkId : " << linkId << " addr : " << addr <<"\n";
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
                
                //當開啟InternetStackHelper 記得關閉 ->因為內建packetSocketHelper了
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
                        addr.SetPhysicalAddress(Mac48Address::ConvertFrom(wifiAPNode.Get(0)->GetDevice(devIdx)->GetAddress())); 
                        addr.SetProtocol(17);
                        Ptr<Socket> sock = Socket::CreateSocket(wifiStaNodes.Get(staId), PacketSocketFactory::GetTypeId());
                        sock->Bind();     
                        sock->Connect(addr);
                        socketsPerSta.push_back(sock);
                        
                    }
                    staSockets.push_back(socketsPerSta);
                }
                for(uint32_t staId = 0; staId < nStations; staId++)
                {
                    for(auto name : trafficId) Simulator::Schedule(Seconds(simStart), &SendPacket, staSockets, staId,name,lambda);  
                }
            }
            // STA掛勾三條link是否成功 
            //STAaddr_linkid
            for(u_int32_t i = 0; i < nStations ;++i)
            {
                Ptr<Node> sta = wifiStaNodes.Get(i);
                for(uint8_t Linkid = 0; Linkid <nLinks; ++Linkid)
                {
                    Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(sta->GetDevice(Linkid)); //sta的第i個設備
                    Ptr<WifiMac> devmac = mloDevice->GetMac();
                    Ptr<WifiPhy> devphy = mloDevice->GetPhy(0); 
                    devphy->SetSlot(MicroSeconds(9));
                    auto fem = devmac->GetFrameExchangeManager(0);
                    Mac48Address LowerMacAddress = fem->GetAddress(); 
                    STAlinkTable[LowerMacAddress] = i; //紀錄實際有在使用的link  位址-> STA i
 
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
                const Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(apDevice.Get(i));
                Ptr<WifiPhy> wifiphy = mloDevice->GetPhy(0);
                // Ptr<WifiMac> wifimac = mloDevice->GetMac();
                wifiphy->TraceConnectWithoutContext("PhyRxDrop", MakeBoundCallback(&MyDropCallback,i));
                wifiphy->TraceConnectWithoutContext("PhyRxEnd", MakeBoundCallback(&MySucCallback, i,mcs));
            }
            
            //物理層傳送的封包資訊(STA)
            for(uint32_t i = 0; i < nStations; ++i)
            {
                for(uint32_t linkId = 0; linkId < nLinks; linkId++)
                {
                const Ptr<WifiNetDevice> mloDevice = DynamicCast<WifiNetDevice>(wifiStaNodes.Get(i)->GetDevice(linkId)); //STA i 的設備 linkId
                Ptr<WifiPhy> phy = mloDevice->GetPhy(0);
                Ptr<WifiMac> mac = mloDevice->GetMac();
                //    std::cout <<  mloDevice->GetMac()->GetFrameExchangeManager(0)->GetAddress() <<'\n';
                //        std::cout << "Binding Trace: STA " << i
                //                  << " linkId " << linkId
                //                  << " device: " << mloDevice
                //                  << " phy: " << phy << std::endl;
                phy->TraceConnectWithoutContext("PhyTxBegin",MakeBoundCallback(&PhyTxBeginTrace,mloDevice,linkId));

            }   
            }
            std::cout << " this is  OK !" << "\n";
            //start--------------------------------------
                Simulator::Schedule(Seconds(0.0),&ClearElement,wifiStaNodes);
                Simulator::Schedule(Seconds(simStart),&RandomNoise,wifiStaNodes);
                Simulator::Schedule(Seconds(simStart),&calucationThroughput,std::ref(tputFile));
                Simulator::Schedule(Seconds(simStart + 0.5),&calucationDelay,std::ref(tputFile1));                  
                Simulator::Stop(simulationTime + Seconds(1)); //end time (need define before simulator::Run())
                Simulator::Run();
                Simulator::Destroy();
            
                    
        }     
    return 0;
}
