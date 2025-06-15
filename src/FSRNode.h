//
// FSRNode.h
// Header file for the C++ implementation of the FSRNode module.
// This should be placed in the `src` directory.
//
#ifndef FSRNODE_H_
#define FSRNODE_H_

#include <omnetpp.h>
#include <map>
#include <vector>
#include <set>
#include <string>
#include <queue> // For Dijkstra's or similar pathfinding
#include <cmath> // For sqrt, pow
#include <limits> // For std::numeric_limits

using namespace omnetpp;

// Define a simple structure for a link state entry (from the paper)
struct LinkStateEntry {
    int sourceId;         // ID of the node originating this link state
    int neighborId;       // ID of the neighbor it has a link with
    int sequenceNumber;   // Sequence number (timestamp) of this entry
    // Add other link state information if needed, e.g., link cost/weight
};

// Define a structure to represent a route (for the routing table/next-hop table)
struct RouteEntry {
    int destinationId;
    int nextHopId;
    int distance; // e.g., hop count
    // Add other metrics like path quality, bandwidth if your FSR implementation uses them
    int sequenceNumber; // Sequence number of the route, for freshness
};

// Define a structure for a packet (control or data)
class FSRPacket : public cPacket { // Inherits from cPacket
  public:
    enum PacketType {
        FSR_UPDATE = 1,
        DATA_PACKET = 2,
        TRAFFIC_GEN_TIMER = 3, // For internal timer messages, not actual packets
        MOBILITY_TIMER = 4     // For internal timer messages, not actual packets
    };

    PacketType type;
    int sourceAddress;
    int destinationAddress;

    std::vector<LinkStateEntry> linkStates; // List of link state entries being sent for FSR_UPDATE

    // Constructor for FSRPacket
    FSRPacket(const char *name = nullptr, PacketType type = FSR_UPDATE, short kind = 0)
        : cPacket(name, kind), type(type) {}

    // Custom copy constructor for duplicating messages
    FSRPacket(const FSRPacket& other) : cPacket(other) {
        type = other.type;
        sourceAddress = other.sourceAddress;
        destinationAddress = other.destinationAddress;
        linkStates = other.linkStates;
    }

    // Override dup() for polymorphic copying
    virtual FSRPacket *dup() const override { return new FSRPacket(*this); }
};


class FSRNode : public cSimpleModule
{
  private:
    int myId;                           // This node's ID
    double transmissionRange;           // Communication range
    double playgroundX, playgroundY;    // Simulation area dimensions

    // Mobility parameters
    double maxSpeed;
    double minSpeed;
    double pauseTime;
    cMessage *mobilityTimer;            // Pointer to the scheduled mobility timer message
    // Current position and velocity
    cXMLElement* displayString;         // OMNeT++ display string XML element for position (though direct set() is used)
    double currentX, currentY;
    double destX, destY;
    double currentSpeed;
    double lastMoveTime;

    // FSR-specific members
    cMessage *fsrUpdateTimer;           // Pointer to the scheduled FSR update timer message
    double fsrUpdateInterval;           // Interval for sending FSR updates
    int numFisheyeScopes;               // Number of fisheye scopes

    // Data structures for FSR
    std::set<int> neighbors;            // Set of direct neighbors
    int currentSequenceNumber;          // Monotonically increasing sequence number for this node's LS updates

    // Topology Table (TT_i): Stores LinkStateEntry from all known nodes
    // Map: sourceNodeId -> map<neighborNodeId, LinkStateEntry>
    std::map<int, std::map<int, LinkStateEntry>> topologyTable;

    // Next Hop Table (NEXT_i): Stores next hop for each destination
    // Map: destinationId -> RouteEntry
    std::map<int, RouteEntry> nextHopTable;

    // Distance Table (D_i): Stores shortest distance to each destination
    // Map: destinationId -> distance
    std::map<int, int> distanceTable;

    // Data for performance metrics
    long packetsSent;
    long packetsReceived;
    long dataBytesSent;
    long dataBytesReceived;
    long controlBytesSent;
    long controlBytesReceived;
    simsignal_t endToEndDelaySignal;
    simsignal_t throughputSignal;
    simsignal_t packetDeliveryRatioSignal;
    simsignal_t dataOverheadSignal;
    simsignal_t controlOverheadSignal;


  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void refreshDisplay() const override; // For updating node position visually

    // FSR specific functions
    void updateFSR();                       // Initiates periodic FSR updates
    void sendLinkStateUpdate();             // Sends link state update messages to neighbors
    void processLinkStateUpdate(FSRPacket *pkt); // Processes received link state updates
    void computeShortestPaths();            // Computes shortest paths using the topology table (e.g., Dijkstra's)
    void updateNeighborList();              // Checks for new/lost neighbors based on proximity
    void sendDataPacket(int destination);   // Example function to send a data packet

    // Mobility functions
    void scheduleMobilityUpdate();
    void updatePosition();
    void setNewDestination();
    double calculateDistance(double x1, double y1, double x2, double y2);

    // Helper function for logging (declared here)
    void log(const std::string& message) const;

  public:
    // Public getter methods for other nodes to query position for neighbor discovery
    double getX() const { return currentX; }
    double getY() const { return currentY; }

    // Virtual destructor for proper cleanup of scheduled events/messages
    virtual ~FSRNode();
};

#endif /* FSRNODE_H_ */
