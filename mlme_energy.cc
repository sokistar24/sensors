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
    packetTimestamps[numSentPackets] = Simulator::Now();

    if (numSentPackets == 0)
    {
      firstPacketSentTime = packetTimestamps[numSentPackets];
    }
    
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
    const double fixedTransmissionTime = 1.0;  // example fixed time in seconds
    double energyForThisAttempt = transmissionPower * fixedTransmissionTime;
    NS_LOG_UNCOND ("Energy for this attempt: " << energyForThisAttempt << " | Total energy so far: " << totalEnergy);
    totalEnergy += energyForThisAttempt;

    if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS)
    {
        numReceivedPackets++;
        lastPacketReceivedTime = Simulator::Now();
        NS_LOG_UNCOND ("Packet " << params.m_msduHandle << " dropped after reaching the max retries.");
        double energyForThisAttempt = transmissionPower * fixedTransmissionTime;
        totalEnergy += energyForThisAttempt;
        latencyAccumulator += Seconds(fixedTransmissionTime);
        NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << " secs | Transmission successfully sent");

        // Remove the timestamp entry
        packetTimestamps.erase(params.m_msduHandle);
        retransmissionCounts.erase(params.m_msduHandle);

        // Schedule next packet transmission
        Simulator::Schedule(Seconds(1.0), &SendPacket, g_device);
    }
    else if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_NO_ACK)
    {
        latencyAccumulator += Seconds(fixedTransmissionTime) * MacMaxFrameRetries;
        NS_LOG_UNCOND ("Packet " << params.m_msduHandle << " dropped after reaching the max retries.");
        double energyForThisAttempt = transmissionPower * fixedTransmissionTime * MacMaxFrameRetries;
        NS_LOG_UNCOND ("Energy for this attempt: " << energyForThisAttempt << " | Total energy so far: " << totalEnergy);
        totalEnergy += energyForThisAttempt;
        // Schedule a new packet transmission as max retransmissions have been reached
        NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << " secs | Max retransmissions reached. Scheduling new packet.");
        retransmissionCounts.erase(params.m_msduHandle);
        Simulator::Schedule(Seconds(1.0), &SendPacket, g_device);
    }
    else 
    {
        // Add the fixed transmission duration (including backoff and CCA wait times) to the latency accumulator
        latencyAccumulator += Seconds(fixedTransmissionTime);
        NS_LOG_UNCOND ("Packet " << params.m_msduHandle << " dropped after reaching the max retries.");
        double energyForThisAttempt = transmissionPower * fixedTransmissionTime;
        NS_LOG_UNCOND ("Energy for this attempt: " << energyForThisAttempt << " | Total energy so far: " << totalEnergy);
        totalEnergy += energyForThisAttempt;
        // Log the time
        NS_LOG_UNCOND("Packet " << params.m_msduHandle << " took " << fixedTransmissionTime << " seconds total (including waiting for CCA and backoffs).");
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


// Assume sink is the PacketSink instance on the receiver side.

double lastTime = 0;  // Store the last time the throughput was calculated
uint64_t lastBytes = 0;  // Store the last bytes received

void CalculateThroughput() 
{
    double curTime = Simulator::Now().GetSeconds();
    double timeDiff = curTime - lastTime;
    uint64_t curBytes = numReceivedPackets;
    uint64_t bytesDiff = curBytes - lastBytes;

    double throughput = bytesDiff / timeDiff; // bytes per second
    NS_LOG_UNCOND("Throughput: " << throughput << " bytes/s");

    lastTime = curTime;
    lastBytes = curBytes;

    Simulator::Schedule(Seconds(1.0), &CalculateThroughput); // Schedule next throughput calculation
}

void ToggleRadio()
{
    Ptr<LrWpanMac> mac = g_coordinatorDevice->GetMac();
    bool currentState = mac->GetRxOnWhenIdle();
    
    mac->SetRxOnWhenIdle(!currentState);  // Toggle the current state
    
    // Schedule the next toggle in 5 seconds
    Simulator::Schedule(ns3::Seconds(5.0), &ToggleRadio);
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
  dev0->SetAddress (Mac16Address ("00:01"));
  g_device ->SetAddress (Mac16Address ("00:02"));

  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  propModel->SetAttribute("Exponent", DoubleValue(2.0));
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);

  dev0->SetChannel (channel);
  g_device ->SetChannel (channel);
  
  // seting Mac-Max
  g_device ->GetMac()->SetMacMaxFrameRetries(MacMaxFrameRetries);

  n0->AddDevice (dev0);
  n1->AddDevice (g_device );

  ///////////////// Mobility   ///////////////////////
  Ptr<ConstantPositionMobilityModel> sender0Mobility = CreateObject<ConstantPositionMobilityModel> ();
  sender0Mobility->SetPosition (Vector (0,0,0));
  dev0->GetPhy ()->SetMobility (sender0Mobility);
  Ptr<ConstantPositionMobilityModel> sender1Mobility = CreateObject<ConstantPositionMobilityModel> ();

  sender1Mobility->SetPosition (Vector (20,0,0)); //10 m distance
  g_device ->GetPhy ()->SetMobility (sender1Mobility);


  /////// MAC layer Callbacks hooks/////////////

  MlmeStartConfirmCallback cb0;
  cb0 = MakeCallback (&StartConfirm);
  dev0->GetMac ()->SetMlmeStartConfirmCallback (cb0);

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
  dev0->GetMac ()->SetMcpsDataIndicationCallback (cb5);



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
  params.m_bcnOrd = 10;
  params.m_sfrmOrd = 6;
  Simulator::ScheduleWithContext (1, Seconds (1.6),
                                  &LrWpanMac::MlmeStartRequest,
                                  dev0->GetMac (), params);

  ///////////////////// Transmission of data Packets from end device //////////////////////
 // Turn off the sink's (dev0's) radio at 3 seconds after the simulation starts
  //Simulator::Schedule(Seconds(3.0), &LrWpanMac::SetRxOnWhenIdle, dev0->GetMac(), false);

  // Turn it back on at 10 seconds
  //Simulator::Schedule(Seconds(35.0), &LrWpanMac::SetRxOnWhenIdle, dev0->GetMac(), true);

  Ptr<Packet> p1 = Create<Packet> (5);
  McpsDataRequestParams params2;
  params2.m_dstPanId = 5;
  params2.m_srcAddrMode = SHORT_ADDR;
  params2.m_dstAddrMode = SHORT_ADDR;
  params2.m_dstAddr = Mac16Address ("00:01");
  params2.m_msduHandle = 0;
  params2.m_txOptions = TX_OPTION_ACK;  // Enable direct transmission with Ack

  /////////////////////////////////////////////////////////////////////////////////////
  // Examples of time parameters for transmissions in the first incoming superframe. //
  /////////////////////////////////////////////////////////////////////////////////////

  // 2.981 sec      No time to finish CCA in CAP, the transmission at this time will cause
  //                the packet to be deferred to the next superframe.

  // 2.982 sec      No time to finish random backoff delay in CAP, the  transmission at this
  //                time will cause the packet to be deferred to the next superframe.

  // 2.93 sec       Enough time, the packet can be transmitted within the CAP of the first superframe


  // MCPS-DATA.request Beacon enabled Direct Transmission (dev1)
  // Frame transmission from End Device to Coordinator (Direct transmission)
  Simulator::ScheduleWithContext (1, Seconds (2.93), &SendPacket, g_device );

  Simulator::Stop (Seconds (40));
  Simulator::Run ();
  double simulationTime = Simulator::Now().GetSeconds();
  double throughput = (numReceivedPackets * 5 * 8) / simulationTime;
  double pdr = static_cast<double>(numReceivedPackets) / numSentPackets;
  double averageLatency = latencyAccumulator.GetSeconds();
  
  NS_LOG_UNCOND ("Throughput: " << throughput << " bps");
  NS_LOG_UNCOND ("Packet Delivery Ratio (PDR): " << pdr);
  NS_LOG_UNCOND ("Average Latency: " << averageLatency << " s");
  NS_LOG_UNCOND ("Total Energy Consumed: " << totalEnergy << " mJ");
  
  Simulator::Destroy ();
  return 0;
}
