//
// FSRNode.ned
// Defines the OMNeT++ module for a single Fisheye State Routing node.
// This should be placed in the `src` directory (or a suitable sub-directory)
// within your OMNeT++ project.
//
package fsr.src; // CORRECT: Must be fsr.src because it's in the src/ folder

// Define the FSRNode module.
simple FSRNode
{
    parameters:
        // Node's unique ID, automatically assigned by OMNeT++ (index in node[] array).
        int myId = default(-1);
        // Transmission range of the node. Neighbors are within this range.
        double transmissionRange @unit(m) = default(250m); // CORRECTED @unit PLACEMENT
        // Maximum speed for random waypoint mobility.
        double maxSpeed @unit(mps) = default(20mps); // CORRECTED @unit PLACEMENT
        // Minimum speed for random waypoint mobility.
        double minSpeed @unit(mps) = default(10mps); // CORRECTED @unit PLACEMENT
        // Pause time for random waypoint mobility.
        double pauseTime @unit(s) = default(1s);   // CORRECTED @unit PLACEMENT
        // Area dimensions from the network.
        double playgroundX @unit(m); // CORRECTED @unit PLACEMENT
        double playgroundY @unit(m); // CORRECTED @unit PLACEMENT

        // FSR-specific parameters (you will expand on these based on the paper):
        // Interval for sending link state updates (e.g., every 1 second for closest scope).
        double fsrUpdateInterval @unit(s) = default(1.0s); // CORRECTED @unit PLACEMENT
        // Number of fisheye scopes.
        int numFisheyeScopes = default(3);
        // Scope radii (you might define these as an array or calculate dynamically).
        // For example, scopeRadius[0] for 1-hop, scopeRadius[1] for 2-hops, etc.

    gates:
        // A single gate for sending and receiving messages.
        // In a real wireless simulation, this would be a wireless interface.
        // For this simplified model, it represents the ability to send messages
        // to other nodes within the simulation.
        inout radio;
}
