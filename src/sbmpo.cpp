#include <sbmpo_ros/SBMPO.hpp>

using namespace sbmpo;

// Constructor
SBMPO::SBMPO() {

    configure(states, controls, implicit_grid_size, implicit_grid_resolution);

    buffer = new Node[max_iterations*sample_size + 1];
}

// Destructor
SBMPO::~SBMPO() {
    delete[] buffer;
}

void SBMPO::run() {

    /* 
        Initialize SBMPO
    */

    // Create priority queue
    const std::function<bool(int,int)> comp = [this](int a, int b) { return buffer[a].heuristic[0] > buffer[b].heuristic[0]; };
    std::priority_queue<int, std::vector<int>, const std::function<bool(int,int)>> queue(comp);

    // Create implicit grid
    int implicit_grid[getTotalGridSize(implicit_grid_size)];

    // Reset buffer
    for (int i = 0; i < max_iterations*sample_size + 1; i++)
        buffer[i].id = 0;

    // Initialize implicit grid
    for (int i = 0; i < getTotalGridSize(implicit_grid_size); i++)
        implicit_grid[i] = -1;


    /*
        Create Starting Node
    */
    
    Node starting_node = generateStartingNode(states, controls);

    buffer[0] = starting_node;

    /*
        Start Iterations Loop
    */

    int high = 0;

    int best = 0;

    for (int iter = 0; iter < max_iterations; iter++) {

        // Get best node from buffer
        Node node = buffer[best];

        // Goal check
        if (isGoal(node.state, goal))
            break;

        // Generation check
        if (node.generation >= max_generations)
            break;

        // Run sampling
        for (int n = 0; n < sample_size; n++) {
            // TODO Sampling
            // Update Vertex methods from thesis
        }

        // Update highest node
        high += sample_size;

        // Check if queue is empty
        if (queue.empty())
            break;

        // Get next best node
        best = queue.top();
        
        // Remove from queue
        queue.pop();

    }

    // Generate best path (& reverse)
    path.clear();
    for (int i = best; i != -1; i = buffer[i].parent_id)
        path.push_back(i);
    std::reverse(path.begin(), path.end());

}