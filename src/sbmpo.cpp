#include <sbmpo_ros/SBMPO.hpp>

using namespace sbmpo;

// Constructor
SBMPO::SBMPO() {

    configure(states);

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

    // Reset iters
    int iters = 0;

}