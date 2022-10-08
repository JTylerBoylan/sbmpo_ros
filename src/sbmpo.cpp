#include <sbmpo_ros/sbmpo.hpp>

//#include <ros/ros.h>

namespace sbmpo {

    void run(Planner &planner) {

        // Reset planner
        resetPlanner(planner);

        // Planner components
        const PlannerOptions &options = planner.options;
        NodeBuffer &buffer = planner.buffer;
        ImplicitGrid &grid = planner.grid;

        // Set comparator function
        const std::function<bool(Index,Index)> comp = [&](Index a, Index b) {
            return buffer[a].heuristic[0] > buffer[b].heuristic[0];
        };

        // Create priority queue
        NodeQueue queue = NodeQueue(comp);

        int &best = planner.results.best;
        int &high = planner.results.high;

        // Begin iterations
        for (int iter = 0; iter < options.max_iterations; iter++) {

            // Get best node from buffer
            Node node = buffer[best];

            // Goal check
            if (isGoal(node.state, options.state_info))
                break;

            // Generation check
            if (node.generation >= options.max_generations)
                break;

            // Run sampling
            for (int n = 0; n < planner.options.sample_size; n++) {
                Index response = sample(planner, node, n);
                if (response != INVALID_INDEX)
                    queue.push(response);
            }

            // Update nodes child index
            node.child_id = high + 1;

            // Update highest node
            high += options.sample_size;

            // Check if queue is empty
            if (queue.empty())
                break;

            // Get next best node
            best = queue.top();
            
            // Remove from queue
            queue.pop();

        }

        // Generate best path (& reverse)
        Path &path = planner.results.path;
        for (int i = best; i != -1; i = buffer[i].parent_id)
            path.push_back(i);
        std::reverse(path.begin(), path.end());

    }

    Index sample(Planner &planner, const Node &node, const int n) {

        // Get child node
        const int index = planner.results.high + n + 1;
        Node &child = planner.buffer[index];
        child.id = index;
        child.parent_id = node.id;
        child.generation = node.generation + 1;
        child.state = node.state;
        child.heuristic = node.heuristic;
        child.control = node.control;

        // Evaluate using external function
        if (!sbmpo_ext::evaluate(child, planner, n))
            return INVALID_INDEX;

        // Get location on implicit grid
        Index& grid_node_index = toNodeIndex(child, planner.grid);

        if (grid_node_index != INVALID_INDEX) {

            // If there is a node on implicit grid, compare with the child
            Node &grid_node = planner.buffer[grid_node_index];

            // If the child node has a lower g score than the existing one, replace the existing node
            const float diff = child.heuristic[1] - grid_node.heuristic[1];
            if (diff < 0) {
                child.child_id = grid_node.child_id;
                grid_node = child;
                // Propogate difference in g to child nodes
                updateSuccessors(grid_node, planner, diff, node.id);
            }

            // No need to add node to priority queue because the existing node was already added and
            // the samples would be identical given they are the same

            return INVALID_INDEX;
        }

        // If there is no node on implicit grid, add child node
        grid_node_index = child.id;

        // Respond with node index
        return child.id;
    }

}