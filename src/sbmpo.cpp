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
            for (int n = 0; n < options.sample_size; n++) {
                // TODO Sampling
                // Update Vertex methods from thesis
            }

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

}