#include <ns3/constant-position-mobility-model.h>
#include <ns3/core-module.h>
#include <ns3/log.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/packet.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <iostream>

using namespace ns3;

double totalEnergy = 0;
double totalDelay;
double unitEnergy = 49.0;
double unitDelay = 10;
int numReceivedPackets= 0;
ns3::Time lastPacketReceivedTime;
ns3::Time firstPacketSentTime ;
ns3::Time latencyAccumulator ;
uint8_t MacMaxFrameRetries = 3;
int numSentPackets =0;
std::map<uint32_t, Time> packetTimestamps;
static std::map<uint8_t, int> retransmissionCounts;

// Constants (tweak these based on your scenario)
const double transmissionPower = 45;  // Power can be adjusted 
Ptr<LrWpanNetDevice> g_device;
Ptr<LrWpanNetDevice> g_coordinatorDevice;

static void SendPacket(Ptr<LrWpanNetDevice> device)
{
  if (numSentPackets < 20) 
  {
    Ptr<Packet> p = Create<Packet> (5);
    McpsDataRequestParams params;
    params.m_dstPanId = 5;
    params.m_srcAddrMode = SHORT_ADDR;
    params.m_dstAddrMode = SHORT_ADDR;
    params.m_dstAddr = Mac16Address ("00:01");
    params.m_msduHandle = 0;
    params.m_txOptions = TX_OPTION_ACK;
    
    // Store the timestamp of the initial packet transmission
    //packetTimestamps[numSentPackets] = Simulator::Now();


    device->GetMac()->McpsDataRequest(params, p);
    numSentPackets++;

    Simulator::Schedule(Seconds(1.0), &SendPacket, device);
  }
}


static void BeaconIndication (MlmeBeaconNotifyIndicationParams params, Ptr<Packet> p)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << " secs | Received BEACON packet of size " << p->GetSize ());
}

static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << " secs | Received DATA packet of size " << p->GetSize ());
}


static void TransEndIndication (McpsDataConfirmParams params)
{
  //retransmissionCounts[params.m_msduHandle]++;
     
    if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS)
    {
        numReceivedPackets++;
        int retries = g_device->GetMac()->GetRetransmissionCount();
        double energyForThisAttempt = unitEnergy * (retries + 1);
        totalEnergy += energyForThisAttempt;
        double Delay= unitDelay * (retries + 1);
        totalDelay += Delay;
        NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << " secs | Transmission successfully sent");
        NS_LOG_UNCOND("Packet " << params.m_msduHandle << " successfully sent after " 
                      << retries<< " retries.");
        //retransmissionCounts.erase(params.m_msduHandle);

    }
    else if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_NO_ACK)
    {
        double energyForThisAttempt = unitEnergy * MacMaxFrameRetries;
        totalEnergy += energyForThisAttempt;
        double Delay= unitDelay * MacMaxFrameRetries;
        totalDelay += Delay;
        //retransmissionCounts.erase(params.m_msduHandle);
    }

}


static void DataIndicationCoordinator (McpsDataIndicationParams params, Ptr<Packet> p)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << "s Coordinator Received DATA packet (size " << p->GetSize () << " bytes)");
}

static void StartConfirm (MlmeStartConfirmParams params)
{
  if (params.m_status == MLMESTART_SUCCESS)
    {
      NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << "Beacon status SUCESSFUL");
    }
}

void ToggleRadio()
{
    Ptr<LrWpanMac> mac = g_coordinatorDevice->GetMac();
    bool currentState = mac->GetRxOnWhenIdle();
    
    mac->SetRxOnWhenIdle(!currentState);  // Toggle the current state
    
    // Schedule the next toggle in 5 seconds
    Simulator::Schedule(ns3::Seconds(4.0), &ToggleRadio);
}



int main (int argc, char *argv[])
{


  LogComponentEnableAll (LOG_PREFIX_TIME);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnable ("LrWpanMac", LOG_LEVEL_INFO);
  LogComponentEnable ("LrWpanCsmaCa", LOG_LEVEL_INFO);


  LrWpanHelper lrWpanHelper;

  // Create 2 nodes, and a NetDevice for each one
  Ptr<Node> n0 = CreateObject <Node> ();
  Ptr<Node> n1 = CreateObject <Node> ();

  Ptr<LrWpanNetDevice> dev0 = CreateObject<LrWpanNetDevice> ();
  Ptr<LrWpanNetDevice> dev1 = CreateObject<LrWpanNetDevice> ();
  g_device = dev1;  // Assigning dev1 to the global variable
  g_coordinatorDevice = dev0;
  dev0->SetAddress (Mac16Address ("00:01"));
  g_device ->SetAddress (Mac16Address ("00:02"));

  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  // Attach the shadowing model to the channel
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  propModel->SetAttribute("Exponent", DoubleValue(2.0));
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);

  g_coordinatorDevice->SetChannel (channel);
  g_device ->SetChannel (channel);
  
  // seting Mac-Max
  g_device ->GetMac()->SetMacMaxFrameRetries(MacMaxFrameRetries);

  n0->AddDevice (g_coordinatorDevice);
  n1->AddDevice (g_device );

  ///////////////// Mobility   ///////////////////////
  Ptr<ConstantPositionMobilityModel> sender0Mobility = CreateObject<ConstantPositionMobilityModel> ();
  sender0Mobility->SetPosition (Vector (0,0,0));
  g_coordinatorDevice->GetPhy ()->SetMobility (sender0Mobility);
  Ptr<ConstantPositionMobilityModel> sender1Mobility = CreateObject<ConstantPositionMobilityModel> ();

  sender1Mobility->SetPosition (Vector (20,0,0)); //10 m distance
  g_device ->GetPhy ()->SetMobility (sender1Mobility);


  /////// MAC layer Callbacks hooks/////////////

  MlmeStartConfirmCallback cb0;
  cb0 = MakeCallback (&StartConfirm);
  g_coordinatorDevice->GetMac ()->SetMlmeStartConfirmCallback (cb0);

  McpsDataConfirmCallback cb1;
  cb1 = MakeCallback(&TransEndIndication);
  g_device ->GetMac ()->SetMcpsDataConfirmCallback (cb1);

  MlmeBeaconNotifyIndicationCallback cb3;
  cb3 = MakeCallback (&BeaconIndication);
  g_device ->GetMac ()->SetMlmeBeaconNotifyIndicationCallback (cb3);

  McpsDataIndicationCallback cb4;
  cb4 = MakeCallback (&DataIndication);
  g_device ->GetMac ()->SetMcpsDataIndicationCallback (cb4);

  McpsDataIndicationCallback cb5;
  cb5 = MakeCallback (&DataIndicationCoordinator);
  g_coordinatorDevice->GetMac ()->SetMcpsDataIndicationCallback (cb5);



  //////////// Manual device association ////////////////////
  // Note: We manually associate the devices to a PAN coordinator
  //       because currently there is no automatic association behavior (bootstrap);
  //       The PAN COORDINATOR does not need to associate or set its
  //       PAN Id or its own coordinator id, these are set
  //       by the MLME-start.request primitive when used.

  g_device ->GetMac ()->SetPanId (5);
  g_device ->GetMac ()->SetAssociatedCoor (Mac16Address ("00:01"));



  ///////////////////// Start transmitting beacons from coordinator ////////////////////////

  MlmeStartRequestParams params;
  params.m_panCoor = true;
  params.m_PanId = 5;
  params.m_bcnOrd = 10; //10
  params.m_sfrmOrd = 6;
  Simulator::ScheduleWithContext (1, Seconds (1.6),
                                  &LrWpanMac::MlmeStartRequest,
                                  g_coordinatorDevice->GetMac (), params);

 
  // 2.93 sec       Enough time, the packet can be transmitted within the CAP of the first superframe


  // MCPS-DATA.request Beacon enabled Direct Transmission (dev1)
  // Frame transmission from End Device to Coordinator (Direct transmission)
  Simulator::ScheduleWithContext (1, Seconds (2.0), &SendPacket, g_device );

  Simulator::Schedule(Seconds(5.0), &ToggleRadio);


  Simulator::Stop (Seconds (40));
  Simulator::Run ();
  
  double throughput = (numReceivedPackets * 20 * 8) / totalDelay;
  double pdr = static_cast<double>(numReceivedPackets) / numSentPackets;
  double Latency = totalDelay;
  
  NS_LOG_UNCOND ("Throughput: " << throughput << " bps");
  NS_LOG_UNCOND ("Packet Delivery Ratio (PDR): " << pdr);
  NS_LOG_UNCOND ("Latency: " << Latency << " ms");
  NS_LOG_UNCOND ("Total Energy Consumed: " << totalEnergy << " mJ");
  
  Simulator::Destroy ();
  return 0;
}
