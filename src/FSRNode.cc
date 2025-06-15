//
// FSRNode.cc
// Source file for the C++ implementation of the FSRNode module.
// This should be placed in the `src` directory.
//
#include "FSRNode.h"
// Standard C++ Libraries for math and containers
#include <vector>
#include <algorithm> // For std::find, std::min
#include <limits>    // For std::numeric_limits
#include <cmath>     // For sqrt, pow
#include <string>    // For std::to_string
#include <queue>     // For std::priority_queue

// Register the module with OMNeT++.
Define_Module(FSRNode);

// Helper function for logging (now properly declared in .h)
void FSRNode::log(const std::string& message) const {
    EV_INFO << "Node " << myId << ": " << message << endl;
}

FSRNode::~FSRNode() {
    // FIXED: Correct cleanup for self-messages (timers).
    // Always cancel if scheduled, then delete. The pointers won't be dangling anymore
    // because we no longer delete them in handleMessage.
    if (fsrUpdateTimer != nullptr) {
        if (fsrUpdateTimer->isScheduled()) {
            cancelEvent(fsrUpdateTimer);
        }
        delete fsrUpdateTimer;
        fsrUpdateTimer = nullptr;
    }

    if (mobilityTimer != nullptr) {
        if (mobilityTimer->isScheduled()) {
            cancelEvent(mobilityTimer);
        }
        delete mobilityTimer;
        mobilityTimer = nullptr;
    }
}

void FSRNode::initialize()
{
    // Get parameters from NED file
    myId = getIndex(); // OMNeT++ assigns index as ID in a module array
    transmissionRange = par("transmissionRange");
    playgroundX = par("playgroundX");
    playgroundY = par("playgroundY");
    maxSpeed = par("maxSpeed");
    minSpeed = par("minSpeed");
    pauseTime = par("pauseTime");
    fsrUpdateInterval = par("fsrUpdateInterval");
    numFisheyeScopes = par("numFisheyeScopes");

    displayString = nullptr; // Not directly used for `set()` method, but for compatibility with previous changes

    // Initialize mobility
    currentX = uniform(0, playgroundX);
    currentY = uniform(0, playgroundY);
    setNewDestination(); // Set initial destination for mobility

    // FIXED: Create timer messages ONCE in initialize.
    // These messages are kept as member variables and re-scheduled, not re-created/deleted.
    mobilityTimer = new cMessage("mobilityTimer");
    fsrUpdateTimer = new cMessage("fsrUpdateTimer");

    scheduleAt(simTime() + uniform(0, pauseTime), mobilityTimer); // Schedule initial mobility update
    lastMoveTime = simTime().dbl(); // Initialize last move time


    // Initialize FSR-specific data structures
    currentSequenceNumber = 0; // Each node starts with sequence 0
    // scheduleAt(simTime() + fsrUpdateInterval, fsrUpdateTimer); // Schedule initial FSR update (this is done later in updateFSR)
    scheduleAt(simTime() + fsrUpdateInterval, fsrUpdateTimer);

    // Initialize performance metrics
    packetsSent = 0;
    packetsReceived = 0;
    dataBytesSent = 0;
    dataBytesReceived = 0;
    controlBytesSent = 0;
    controlBytesReceived = 0;

    // Register signals for recording statistics
    endToEndDelaySignal = registerSignal("endToEndDelay");
    throughputSignal = registerSignal("throughput");
    packetDeliveryRatioSignal = registerSignal("packetDeliveryRatio");
    dataOverheadSignal = registerSignal("dataOverhead");
    controlOverheadSignal = registerSignal("controlOverhead");

    log("Initialized at (" + std::to_string(currentX) + ", " + std::to_string(currentY) + ")");

    // Initial neighbor discovery and link state update
    updateNeighborList();
    sendLinkStateUpdate(); // Send initial link state info
    computeShortestPaths(); // Compute initial routes
}

void FSRNode::handleMessage(cMessage *msg)
{
    // FIXED: DO NOT delete self-messages (timers) here. They are re-scheduled.
    // Other messages (like FSRPacket) are still deleted.
    if (msg == fsrUpdateTimer) {
        updateFSR();
        scheduleAt(simTime() + fsrUpdateInterval, fsrUpdateTimer);
        // Do NOT delete msg here. fsrUpdateTimer is a member variable and will be reused.
    } else if (msg == mobilityTimer) {
        updatePosition();
        scheduleMobilityUpdate();
        // Do NOT delete msg here. mobilityTimer is a member variable and will be reused.
    } else {
        FSRPacket *pkt = dynamic_cast<FSRPacket*>(msg);
        if (!pkt) {
            log("Received unknown or non-FSRPacket message type.");
            delete msg; // Delete unknown or non-FSRPacket messages
            return;
        }

        if (msg->arrivedOn("radio$i")) {
            if (pkt->destinationAddress == myId || pkt->destinationAddress == -1) {
                if (pkt->type == FSRPacket::FSR_UPDATE) {
                    processLinkStateUpdate(pkt);
                } else if (pkt->type == FSRPacket::DATA_PACKET) {
                    if (pkt->destinationAddress == myId) {
                        packetsReceived++;
                        dataBytesReceived += pkt->getByteLength();
                        log("Received DATA_PACKET from " + std::to_string(pkt->sourceAddress) + " for self. Content: " + (pkt->getEncapsulatedPacket() ? pkt->getEncapsulatedPacket()->getName() : "No encapsulated content"));
                        emit(endToEndDelaySignal, simTime() - pkt->getCreationTime());
                    } else {
                        log("Forwarding DATA_PACKET for " + std::to_string(pkt->destinationAddress));
                        auto it = nextHopTable.find(pkt->destinationAddress);
                        if (it != nextHopTable.end()) {
                            int nextHop = it->second.nextHopId;
                            FSRPacket *forwardPkt = pkt->dup(); // Duplicate to send, so original can be deleted
                            forwardPkt->setBitLength(pkt->getBitLength());
                            forwardPkt->setName(pkt->getName());
                            cModule *targetModule = getParentModule()->getSubmodule("node", nextHop);
                            if (targetModule) {
                                cGate *targetGate = targetModule->gate("radio$i");
                                sendDirect(forwardPkt, targetGate);
                                packetsSent++;
                                dataBytesSent += forwardPkt->getByteLength();
                            } else {
                                log("Error: Target module " + std::to_string(nextHop) + " not found for forwarding.");
                                delete forwardPkt;
                            }
                        } else {
                            log("No route found for destination " + std::to_string(pkt->destinationAddress));
                        }
                    }
                }
            } else {
                log("Received packet not for me or broadcast, ignoring.");
            }
        }
        delete pkt; // Delete FSRPacket (control or data) received from the network
    }
}

void FSRNode::refreshDisplay() const {
    std::string pos_str = "p=" + std::to_string((int)currentX) + "," + std::to_string((int)currentY);
    getDisplayString().set(pos_str.c_str());
}

void FSRNode::updateFSR()
{
    log("Initiating FSR update cycle.");
    updateNeighborList();
    sendLinkStateUpdate();
    computeShortestPaths();
}

void FSRNode::sendLinkStateUpdate()
{
    log("Sending Link State Update.");
    FSRPacket *updatePkt = new FSRPacket("FSR_UPDATE", FSRPacket::FSR_UPDATE);
    updatePkt->sourceAddress = myId;
    updatePkt->destinationAddress = -1; // Broadcast for simplicity in this model
    updatePkt->setBitLength(1000);
    currentSequenceNumber++;

    for (int neighborId : neighbors) {
        LinkStateEntry entry;
        entry.sourceId = myId;
        entry.neighborId = neighborId;
        entry.sequenceNumber = currentSequenceNumber;
        updatePkt->linkStates.push_back(entry);
    }

    int totalNodes = getParentModule()->par("numNodes");
    for (int i = 0; i < totalNodes; ++i) {
        if (i == myId) continue;
        cModule *targetModule = getParentModule()->getSubmodule("node", i);
        if (targetModule) {
            cGate *targetGate = targetModule->gate("radio$i");
            FSRPacket *pktCopy = updatePkt->dup(); // Duplicate for each recipient
            sendDirect(pktCopy, targetGate);
        } else {
            log("Error: Target module " + std::to_string(i) + " not found for update broadcast.");
        }
    }
    packetsSent++;
    controlBytesSent += updatePkt->getByteLength() * (totalNodes - 1);
    delete updatePkt; // Delete the original message (and its duplicates will be deleted by OMNeT++ when they arrive or are dropped)
}

void FSRNode::processLinkStateUpdate(FSRPacket *pkt)
{
    log("Processing Link State Update from " + std::to_string(pkt->sourceAddress));
    controlBytesReceived += pkt->getByteLength();

    bool topologyChanged = false;
    for (const auto& receivedEntry : pkt->linkStates) {
        bool foundExisting = false;
        if (topologyTable.count(receivedEntry.sourceId) && topologyTable[receivedEntry.sourceId].count(receivedEntry.neighborId)) {
            const LinkStateEntry& existingEntry = topologyTable[receivedEntry.sourceId][receivedEntry.neighborId];
            if (receivedEntry.sequenceNumber > existingEntry.sequenceNumber) {
                topologyTable[receivedEntry.sourceId][receivedEntry.neighborId] = receivedEntry;
                log("Updated LS for (" + std::to_string(receivedEntry.sourceId) + ", " + std::to_string(receivedEntry.neighborId) + ") with newer sequence " + std::to_string(receivedEntry.sequenceNumber));
                topologyChanged = true;
            }
            foundExisting = true;
        }

        if (!foundExisting) {
            topologyTable[receivedEntry.sourceId][receivedEntry.neighborId] = receivedEntry;
            log("Added new LS for (" + std::to_string(receivedEntry.sourceId) + ", " + std::to_string(receivedEntry.neighborId) + ") sequence " + std::to_string(receivedEntry.sequenceNumber));
            topologyChanged = true;
        }
    }
    if (topologyChanged) {
        computeShortestPaths();
    }
}

void FSRNode::computeShortestPaths()
{
    log("Computing shortest paths (Dijkstra's algorithm).");
    distanceTable.clear();
    nextHopTable.clear();

    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    int totalNodes = getParentModule()->par("numNodes");
    for (int i = 0; i < totalNodes; ++i) {
        distanceTable[i] = std::numeric_limits<int>::max();
    }

    distanceTable[myId] = 0;
    pq.push({0, myId});

    std::map<int, int> predecessor;

    while (!pq.empty()) {
        int d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > distanceTable[u]) {
            continue;
        }

        std::set<int> knownNeighborsOfU;

        if (topologyTable.count(u)) {
            for (auto const& [v_neighbor, linkState] : topologyTable[u]) {
                knownNeighborsOfU.insert(v_neighbor);
            }
        }
        for (auto const& [srcNode, links] : topologyTable) {
            if (links.count(u)) {
                knownNeighborsOfU.insert(srcNode);
            }
        }

        for (int v : knownNeighborsOfU) {
            if (v == u) continue;

            int newDist = distanceTable[u] + 1;

            if (newDist < distanceTable[v]) {
                distanceTable[v] = newDist;
                predecessor[v] = u;
                pq.push({newDist, v});
            }
        }
    }

    for (int destId = 0; destId < totalNodes; ++destId) {
        if (destId == myId) continue;

        if (distanceTable[destId] == std::numeric_limits<int>::max()) {
            continue;
        }

        int current = destId;
        int nextHop = -1;

        while (predecessor.count(current) && predecessor[current] != myId) {
            current = predecessor[current];
        }

        if (predecessor.count(destId) && predecessor[destId] == myId) {
            nextHop = destId;
        } else if (predecessor.count(current) && predecessor[current] == myId) {
            nextHop = current;
        } else {
            // This warning is fine for now; indicates a path issue or a tricky edge case in Dijkstra's reconstruction
            // log("Warning: Could not determine next hop for " + std::to_string(destId) + " from predecessor chain.");
            // continue; // Re-enable if you want to skip such routes
        }


        if (nextHop != -1 && distanceTable[destId] > 0) {
            RouteEntry entry;
            entry.destinationId = destId;
            entry.nextHopId = nextHop;
            entry.distance = distanceTable[destId];
            entry.sequenceNumber = currentSequenceNumber;
            nextHopTable[destId] = entry;
            log("Route to " + std::to_string(destId) + ": next hop " + std::to_string(nextHop) + ", distance " + std::to_string(distanceTable[destId]));
        }
    }
}

void FSRNode::updateNeighborList()
{
    log("Updating neighbor list.");
    std::set<int> newNeighbors;
    int totalNodes = getParentModule()->par("numNodes");

    for (int i = 0; i < totalNodes; ++i) {
        if (i == myId) continue;

        FSRNode *otherNode = check_and_cast<FSRNode*>(getParentModule()->getSubmodule("node", i));
        double otherX = otherNode->getX();
        double otherY = otherNode->getY();

        double dist = calculateDistance(currentX, currentY, otherX, otherY);

        if (dist <= transmissionRange) {
            newNeighbors.insert(i);
        }
    }

    if (newNeighbors != neighbors) {
        log("Neighbor list changed. Old size: " + std::to_string(neighbors.size()) + ", New size: " + std::to_string(newNeighbors.size()));
        neighbors = newNeighbors;
        sendLinkStateUpdate();
    } else {
        log("Neighbor list unchanged. Size: " + std::to_string(neighbors.size()));
    }
}

void FSRNode::sendDataPacket(int destination)
{
    if (destination == myId) {
        log("Cannot send data to self.");
        return;
    }

    auto it = nextHopTable.find(destination);
    if (it != nextHopTable.end()) {
        int nextHop = it->second.nextHopId;
        log("Sending DATA_PACKET to " + std::to_string(destination) + " via next hop " + std::to_string(nextHop));

        FSRPacket *dataPkt = new FSRPacket("DATA_PACKET", FSRPacket::DATA_PACKET);
        dataPkt->sourceAddress = myId;
        dataPkt->destinationAddress = destination;
        dataPkt->setBitLength(512 * 8); // 512 bytes data payload as in the paper
        dataPkt->encapsulate(new cPacket("AppDataPacket"));
        cModule *targetModule = getParentModule()->getSubmodule("node", nextHop);
        if (targetModule) {
            cGate *targetGate = targetModule->gate("radio$i");
            sendDirect(dataPkt, targetGate);
            packetsSent++;
            dataBytesSent += dataPkt->getByteLength();
        } else {
            log("Error: Target module " + std::to_string(nextHop) + " not found for data forwarding.");
            delete dataPkt;
        }
    } else {
        log("No route to destination " + std::to_string(destination) + " found for data packet.");
    }
}

// *** Mobility Functions ***
void FSRNode::scheduleMobilityUpdate() {
    double delay;
    double remainingDistance = calculateDistance(currentX, currentY, destX, destY);
    if (remainingDistance > 0.01) {
        delay = remainingDistance / currentSpeed;
    } else {
        delay = pauseTime;
        setNewDestination();
    }
    if (delay <= 0) delay = 0.001;
    scheduleAt(simTime() + delay, mobilityTimer); // Reschedule the EXISTING timer object
}

void FSRNode::updatePosition() {
    double timeSinceLastMove = simTime().dbl() - lastMoveTime;
    if (timeSinceLastMove < 0) timeSinceLastMove = 0;

    double dx = destX - currentX;
    double dy = destY - currentY;
    double distanceToDest = std::sqrt(dx*dx + dy*dy);

    if (distanceToDest < 0.01) {
        currentX = destX;
        currentY = destY;
        log("Reached destination (" + std::to_string(currentX) + ", " + std::to_string(currentY) + ")");
        updateNeighborList();
        return;
    }

    double ratio = currentSpeed * timeSinceLastMove / distanceToDest;
    if (ratio >= 1.0) {
        currentX = destX;
        currentY = destY;
    } else {
        currentX += dx * ratio;
        currentY += dy * ratio;
    }
    lastMoveTime = simTime().dbl();

    log("Moving to (" + std::to_string(currentX) + ", " + std::to_string(currentY) + ")");
    refreshDisplay();
    updateNeighborList();
}

void FSRNode::setNewDestination() {
    destX = uniform(0, playgroundX);
    destY = uniform(0, playgroundY);
    currentSpeed = uniform(minSpeed, maxSpeed);
    lastMoveTime = simTime().dbl();
    log("Set new destination: (" + std::to_string(destX) + ", " + std::to_string(destY) + ") with speed " + std::to_string(currentSpeed) + " m/s");
}

double FSRNode::calculateDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}
