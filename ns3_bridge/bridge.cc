// Persistent ns-3.46 ad-hoc Wi-Fi transport for SkyFleet (one virtual clock).
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/propagation-loss-model.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
using namespace ns3;

static NodeContainer nodes;
static NetDeviceContainer devices;
static std::map<std::string, unsigned> indexByName;
static std::vector<std::string> names;
static std::vector<std::string> arrivals;
struct Block { double x, y, width, depth, height, base = 0.0; };
static std::vector<Block> buildings;
static double buildingLossDb = 0.0;
static Ptr<PropagationLossModel> telemetryLoss;
static std::map<std::string, unsigned> senderByPacket;

// Liang-Barsky clipping of the horizontal radio segment against a roof footprint.
// A roof blocks only if the ray at its first horizontal intersection is below it.
static bool Blocked(const Vector& a, const Vector& b, const Block& block) {
    double enter = 0.0, leave = 1.0;
    const double dx = b.x - a.x, dy = b.y - a.y;
    auto clip = [&](double start, double delta, double lower, double upper) {
        if (delta == 0.0) return lower <= start && start <= upper;
        double lo = (lower - start) / delta, hi = (upper - start) / delta;
        if (lo > hi) std::swap(lo, hi);
        enter = std::max(enter, lo);
        leave = std::min(leave, hi);
        return enter <= leave;
    };
    if (!clip(a.x, dx, block.x, block.x + block.width) ||
        !clip(a.y, dy, block.y, block.y + block.depth)) return false;
    const double z0 = a.z + (b.z - a.z) * enter;
    const double z1 = a.z + (b.z - a.z) * leave;
    return std::min(z0, z1) <= block.height && std::max(z0, z1) >= block.base;
}

class BuildingShadowLoss : public PropagationLossModel {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("skyfleet::BuildingShadowLoss")
            .SetParent<PropagationLossModel>().SetGroupName("Propagation")
            .AddConstructor<BuildingShadowLoss>();
        return tid;
    }
private:
    double DoCalcRxPower(double tx, Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override {
        for (const auto& building : buildings)
            if (Blocked(a->GetPosition(), b->GetPosition(), building))
                return tx - buildingLossDb;
        return tx;
    }
    int64_t DoAssignStreams(int64_t) override { return 0; }
};
NS_OBJECT_ENSURE_REGISTERED(BuildingShadowLoss);

// Match the MAC queue lifetime; the extra margin covers in-flight transmissions.
static constexpr int64_t packetLifetimeNs = 1000000000;
static constexpr int64_t packetRetentionNs = packetLifetimeNs + 100000000;

static bool Received(unsigned recipient, Ptr<NetDevice>, Ptr<const Packet> packet,
                     uint16_t, const Address&) {
    std::vector<uint8_t> payload(packet->GetSize());
    packet->CopyData(payload.data(), payload.size());
    std::string id(payload.begin(), payload.end());
    id = id.substr(0, id.find('\0'));
    // Read the same deterministic propagation chain as the PHY. This is
    // modeled received power, not a per-packet PHY RSSI measurement.
    double rxDbm = telemetryLoss->CalcRxPower(
        25.0, nodes.Get(senderByPacket.at(id))->GetObject<MobilityModel>(),
        nodes.Get(recipient)->GetObject<MobilityModel>());
    arrivals.push_back("RX " + id + " " + names.at(recipient) + " " +
                       std::to_string(Simulator::Now().GetNanoSeconds()) + " " +
                       std::to_string(rxDbm));
    Simulator::Stop(); // return control to SkyFleet at the receive timestamp
    return true;
}

static void Init(unsigned count, unsigned seed) {
    RngSeedManager::SetSeed(seed ? seed : 1);
    nodes.Create(count);
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                                  StringValue("OfdmRate6Mbps"), "ControlMode",
                                  StringValue("OfdmRate6Mbps"));
    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                               "Exponent", DoubleValue(2.0));
    channel.AddPropagationLoss("skyfleet::BuildingShadowLoss");
    YansWifiPhyHelper phy;
    phy.Set("TxPowerStart", DoubleValue(25.0));
    phy.Set("TxPowerEnd", DoubleValue(25.0));
    auto logLoss = CreateObject<LogDistancePropagationLossModel>();
    logLoss->SetAttribute("Exponent", DoubleValue(2.0));
    logLoss->SetNext(CreateObject<BuildingShadowLoss>());
    telemetryLoss = logLoss;
    auto sharedChannel = channel.Create();
    // The channel's propagation helper constructs an equivalent chain;
    // telemetryLoss is a read-only mirror with identical parameters.
    phy.SetChannel(sharedChannel);
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    devices = wifi.Install(phy, mac, nodes);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
    for (unsigned i = 0; i < count; ++i) {
        auto wifiDevice = DynamicCast<WifiNetDevice>(devices.Get(i));
        wifiDevice->GetMac()->GetTxop()->GetWifiMacQueue()->SetMaxDelay(NanoSeconds(packetLifetimeNs));
        devices.Get(i)->SetReceiveCallback(MakeBoundCallback(&Received, i));
    }
}

static void SendAt(unsigned src, unsigned dst, std::string id, unsigned size) {
    std::vector<uint8_t> data(std::max(size, static_cast<unsigned>(id.size() + 1)), 0);
    std::copy(id.begin(), id.end(), data.begin());
    auto pkt = Create<Packet>(data.data(), data.size());
    senderByPacket[id] = src;
    Address target = dst == names.size() ? Mac48Address::GetBroadcast()
                                         : devices.Get(dst)->GetAddress();
    devices.Get(src)->Send(pkt, target, 0x88B5);
}

int main() {
    std::cout.setf(std::ios::unitbuf);
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string op;
        in >> op;
        if (op == "INIT") {
            unsigned n, seed;
            if (!(in >> n >> seed) || n == 0 || n > 200) return 2;
            Init(n, seed);
            std::cout << "OK\n";
        } else if (op == "BUILDING") {
            Block block;
            if (!(in >> block.x >> block.y >> block.width >> block.depth >> block.height >> block.base) ||
                !std::isfinite(block.x) || !std::isfinite(block.y) ||
                !std::isfinite(block.width) || !std::isfinite(block.depth) ||
                !std::isfinite(block.height) || !std::isfinite(block.base) ||
                block.width <= 0 || block.depth <= 0 || block.height <= 0 ||
                block.base < 0 || block.base >= block.height ||
                buildings.size() >= 10000) return 2;
            buildings.push_back(block);
            std::cout << "OK\n";
        } else if (op == "BUILDING_LOSS") {
            double loss;
            if (!(in >> loss) || !std::isfinite(loss) || loss < 0 || loss > 200) return 2;
            buildingLossDb = loss;
            std::cout << "OK\n";
        } else if (op == "POS") {
            std::string name; double x, y, z;
            if (!(in >> name >> x >> y >> z) || !indexByName.count(name)) return 2;
            nodes.Get(indexByName.at(name))->GetObject<MobilityModel>()->SetPosition(Vector(x, y, z));
            std::cout << "OK\n";
        } else if (op == "NAME") {
            std::string name; unsigned idx;
            if (!(in >> idx >> name) || idx >= nodes.GetN() || indexByName.count(name) ||
                names.size() != idx) return 2;
            indexByName[name] = idx; names.push_back(name);
            std::cout << "OK\n";
        } else if (op == "SEND") {
            std::string id, src, dst; unsigned bytes; long long ns;
            if (!(in >> id >> src >> dst >> bytes >> ns) ||
                !indexByName.count(src) || (dst != "*" && !indexByName.count(dst)) ||
                bytes < id.size() + 1 || ns < Simulator::Now().GetNanoSeconds()) return 2;
            unsigned destination = dst == "*" ? names.size() : indexByName.at(dst);
            Simulator::Schedule(NanoSeconds(ns - Simulator::Now().GetNanoSeconds()),
                                &SendAt, indexByName.at(src), destination, id, bytes);
            Simulator::Schedule(NanoSeconds(ns + packetRetentionNs - Simulator::Now().GetNanoSeconds()),
                                [id]() {
                                    senderByPacket.erase(id);
                                    arrivals.push_back("DONE " + id + " " +
                                                       std::to_string(Simulator::Now().GetNanoSeconds()));
                                    Simulator::Stop();
                                });
            std::cout << "OK\n";
        } else if (op == "RUN") {
            long long ns;
            if (!(in >> ns) || ns < Simulator::Now().GetNanoSeconds()) return 2;
            auto delta = NanoSeconds(ns - Simulator::Now().GetNanoSeconds());
            Simulator::Schedule(delta, []() {});
            EventId deadline = Simulator::Stop(delta);
            Simulator::Run();
            if (!deadline.IsExpired()) Simulator::Cancel(deadline);
            for (auto& arrival : arrivals) std::cout << arrival << "\n";
            arrivals.clear();
            std::cout << "END " << Simulator::Now().GetNanoSeconds() << "\n";
        } else if (op == "QUIT") {
            Simulator::Destroy();
            return 0;
        } else return 2;
    }
    Simulator::Destroy();
    return 0;
}
