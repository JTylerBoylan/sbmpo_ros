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

            // Get best node from buffer
            Node node = buffer[best];

            // Goal check
            if (isGoal(node.state, options.state_info))
                break;

            // Generation check
            if (node.generation >= options.max_generations)
                break;

            // Run sampling
            sample(planner, node);

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

    void sample(const Planner &planner, const Node &node) {
        for (int n = 0; n < planner.options.sample_size; n++) {

            // Get child node
            const int index = planner.results.high + n + 1;
            Node &child = planner.buffer[index];
            child.id = index;
            child.parent_id = node.id;
            child.generation = node.generation + 1;
            
            Control control = generateSamples(planner.options, index, n);

            evaluate(child, control, planner);

            calculateG(child, node, planner);

            calculateH(child, planner);

            // TODO

            // Check implicit grid

            // Add to queue

        }
    }

}