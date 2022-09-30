#include <sbmpo_ros/sbmpo.hpp>

namespace sbmpo {

    void run(Planner &planner) {

        // Reset planner
        reset(planner);

        // Planner components
        const PlannerOptions &options = planner.options;
        NodeBuffer &buffer = planner.buffer;
        ImplicitGrid &grid = planner.grid;
        NodeQueue &queue = planner.queue;

        int &best = planner.results.best;
        int &high = planner.results.high;

        // Begin iterations
        for (int iter = 0; iter < options.max_iterations; iter++) {

            ROS_INFO("Running iteration %d...", iter);

            // Get best node from buffer
            Node node = buffer[best];

            ROS_INFO("Checking goal...");

            // Goal check
            if (isGoal(node.state, options.state_info))
                break;

            ROS_INFO("Checking generation...");

            // Generation check
            if (node.generation >= options.max_generations)
                break;

            ROS_INFO("Sampling...");

            // Run sampling
            sample(planner, node);

            ROS_INFO("Continue.");

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

        ROS_INFO("Planner finished.");

        // Generate best path (& reverse)
        Path &path = planner.results.path;
        for (int i = best; i != -1; i = buffer[i].parent_id)
            path.push_back(i);
        std::reverse(path.begin(), path.end());

    }

    void sample(Planner &planner, const Node &node) {

        // Iterate through samples
        for (int n = 0; n < planner.options.sample_size; n++) {

            // Get child node
            const int index = planner.results.high + n + 1;
            Node &child = planner.buffer[index];
            child.id = index;
            child.parent_id = node.id;
            child.generation = node.generation + 1;
            
            // Generate set of controls
            Control control = generateSamples(planner.options, index, n);
            child.control = control;

            // Evaluate using external function
            if (!evaluate(child, planner))
                continue;

            // Get location on implicit grid
            GridKey grid_key = toGridKey(child.state, planner.grid);
            Index grid_index = toGridIndex(grid_key, planner.grid);
            Index& grid_node_index = planner.grid.buffer[grid_index];

            if (grid_node_index == INVALID_INDEX) {
                
                // If there is no node on implicit grid, add child node
                grid_node_index = child.id;

                // Add to priority queue
                planner.queue.push(child.id);

            } else {

                // If there is a node on implicit grid, compare with the child
                Node &grid_node = planner.buffer[grid_node_index];

                // If the child node has a lower g score than the existing one, replace the existing node
                if (child.heuristic[1] < grid_node.heuristic[1])
                    grid_node = child;

                // No need to add node to priority queue because the existing node was already added and
                // the samples would be identical given they are the same

            }

        }
    }

}