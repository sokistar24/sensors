#include "mygym.h"
#include "ns3/opengym-module.h"
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
NS_LOG_COMPONENT_DEFINE ("OpenGym");
int action = 0; // 0: not transmit, 1: transmit
int packetRetries = 0; // Number of packet retries for the current packet
int reward = 0; // Reward based on transmission outcome

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

Ptr<OpenGymSpace> MyGymEnv::GetObservationSpace()
{
  NS_LOG_FUNCTION (this);
  
  // Assuming a maximum number of retries, e.g., 10
  uint32_t maxRetries = 10;

  // Two elements in the observation space:
  // 1. Number of retries (0 to maxRetries)
  // 2. ACK received (0 or 1)
  std::vector<uint32_t> shape = {2,};
  std::vector<double> low = {0.0, 0.0};     // Lower bounds for retries and ACK
  std::vector<double> high = {static_cast<double>(maxRetries), 1.0}; // Upper bounds for retries and ACK

  std::string dtype = TypeNameGet<uint32_t> ();
  Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
  
  NS_LOG_UNCOND ("GetObservationSpace: " << space);
  return space;
}

Ptr<OpenGymSpace> MyGymEnv::GetActionSpace()
{
  NS_LOG_FUNCTION (this);

  // Define a binary action space: 0 = wait, 1 = transmit
  uint32_t actionSpaceSize = 2; // Two actions: wait or transmit
  Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace> (actionSpaceSize);

  NS_LOG_UNCOND ("GetActionSpace: " << space);
  return space;
}

bool MyGymEnv::GetGameOver()
{
  NS_LOG_FUNCTION (this);

  // Game-over condition based on the number of sent packets
  bool isGameOver = (m_numSentPackets >= 200);

  NS_LOG_UNCOND ("GetGameOver: " << isGameOver);
  return isGameOver;
}

bool MyGymEnv::MyExecuteActions(Ptr<OpenGymDataContainer> action)
{
  NS_LOG_FUNCTION (this);

  Ptr<OpenGymDiscreteContainer> discreteAction = DynamicCast<OpenGymDiscreteContainer>(action);
  uint32_t actionValue = discreteAction->GetValue();

  NS_LOG_UNCOND("MyExecuteActions: ActionValue = " << actionValue);

  // Assuming you have a global or accessible device pointer for sending packets
  extern Ptr<LrWpanNetDevice> globalDevice;

  if (actionValue == 1) {
    // Action to transmit a packet
    if (globalDevice != nullptr) {
      SendPacket(globalDevice); // Utilize the existing SendPacket function
    }
  } else {
    // Action to wait (do nothing or implement logic for waiting)
    // Optionally, handle the waiting logic
  }

  return true;
}

void ScheduleNextStateRead(double envStepTime, Ptr<OpenGymInterface> openGymInterface)
{
  // Schedule this function to be called again after envStepTime seconds
  Simulator::Schedule(Seconds(envStepTime), &ScheduleNextStateRead, envStepTime, openGymInterface);

  // Notify the OpenGymInterface that the current state is ready
  openGymInterface->NotifyCurrentState();
}

static void SendPacket(Ptr<LrWpanNetDevice> device)
{

  if (numSentPackets < 200) 
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
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds () This setup ensures that the RL agent in MyGymEnv can learn the optimal policy for packet transmission based on the simulation dynamics defined in sim.cc. The separation of the RL environment (MyGymEnv) and the simulation (sim.cc) adheres to good practices in RL and ns3 integration.
<< " secs | Received BEACON packet of size " << p->GetSize ());
}

static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << " secs | Received DATA packet of size " << p->GetSize ());
}




static void TransEndIndication (McpsDataConfirmParams params) {
    int retries = g_device->GetMac()->GetRetransmissionCount();
    double energyForThisAttempt = unitEnergy * (retries + 1);
    double Delay = unitDelay * (retries + 1);

    if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS) {
        numReceivedPackets++;
        totalEnergy += energyForThisAttempt;
        totalDelay += Delay;
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " secs | Transmission successfully sent");
        NS_LOG_UNCOND("Packet " << params.m_msduHandle << " successfully sent after " << retries << " retries.");
        
        // Update state for MyGymEnv
        packetRetries = retries;

    } else if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_NO_ACK) {
        totalEnergy += unitEnergy * MacMaxFrameRetries;
        totalDelay += unitDelay * MacMaxFrameRetries;

        // Update state for MyGymEnv
        packetRetries = retries;
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

Ptr<OpenGymDataContainer> MyGymEnv::MyGetObservation()
{
  NS_LOG_FUNCTION (this);

  // Define the shape of the observation: 2 elements (retries and ACK status)
  std::vector<uint32_t> shape = {2,};
  Ptr<OpenGymBoxContainer<uint32_t>> box = CreateObject<OpenGymBoxContainer<uint32_t>>(shape);

  // Add the number of packet retries to the observation
  box->AddValue(packetRetries);

  // Determine the ACK status: 1 for ACK received (success), 0 for no ACK
  uint32_t ackStatus = (lastPacketStatus == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS) ? 1 : 0;
  box->AddValue(ackStatus);

  NS_LOG_UNCOND ("MyGetObservation: " << box);
  return box;
}

float MyGymEnv::GetReward()
{
  NS_LOG_FUNCTION (this);
  
  float reward = 0.0;

  // Check the status of the last packet transmission
  if (lastPacketStatus == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS) {
    // Assign a reward of +1 for successful transmission
    reward = 1.0;
  } else {
    // Assign a reward of -1 for failed transmission (no ACK received)
    reward = -1.0;
  }

  NS_LOG_UNCOND ("GetReward: " << reward);
  return reward;
}


double lastTime = 0;  // Store the last time the throughput was calculated
uint64_t lastBytes = 0;  // Store the last bytes received
void CalculateThroughput() 
{
    double curTime = Simulator::Now().GetSeconds();
    double timeDiff = curTime - lastTime;
    uint64_t curBytes = numReceivedPackets; // Assuming this tracks the total received packets
    uint64_t bytesDiff = curBytes - lastBytes;

    double throughput = (bytesDiff * 8) / timeDiff; // Throughput in bits per second

    // Print the current time and the calculated throughput
    NS_LOG_UNCOND("Time: " << curTime << " seconds, Throughput: " << throughput << " bps");

    lastTime = curTime;
    lastBytes = curBytes;
}


void ToggleRadio()
{
    CalculateThroughput();
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
  //LogComponentEnable ("LrWpanMac", LOG_LEVEL_INFO);
  //LogComponentEnable ("LrWpanCsmaCa", LOG_LEVEL_INFO);


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
  params.m_bcnOrd = 6; //10
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
