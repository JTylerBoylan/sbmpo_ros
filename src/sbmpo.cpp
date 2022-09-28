#include <sbmpo_ros/SBMPO.hpp>

using namespace sbmpo;

// Constructor
SBMPO::SBMPO() {

    configure(states, implicit_grid_size, implicit_grid_resolution);

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
    const std::function<bool(int,int)> comp = [this](int a, int b) { return buffer[a].control[0] > buffer[b].control[0]; };
    std::priority_queue<int, std::vector<int>, const std::function<bool(int,int)>>(comp);

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
    
    Node starting_node = generateStartingNode(states);

    buffer[0] = starting_node;

    /*
        Start Iterations Loop
    */

    int best = 0;

    for (int iter = 0; iter < max_iterations; iter++) {

        // Get best node from buffer
        Node node = buffer[best];

        /*
            ------ TODO ------
        */

        // Goal check

        // Generation check

        // Run sampling

        // Update highest node

        // Check if queue is empty

        // Get best node

        // Pop
        
    }

    // Generate best path (& reverse)

    // Print path

}