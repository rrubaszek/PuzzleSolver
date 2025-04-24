#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>
#include <cmath>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <bitset>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>

class PuzzleSolver {
private:
    int n; // Grid size
    std::vector<uint8_t> goal;

    // Log mutex to prevent interleaved output
    std::mutex log_mutex;

    struct PuzzleState {
        std::vector<uint8_t> board;
        int g; // cost so far
        int h; // heuristic value
        std::shared_ptr<PuzzleState> parent;

        PuzzleState(std::vector<uint8_t> b, int g_val, int h_val, std::shared_ptr<PuzzleState> p = nullptr)
            : board(std::move(b)), g(g_val), h(h_val), parent(p) {}

        int f() const { return g + h; }
    };

    // Custom comparator for priority queue
    struct Compare {
        bool operator()(const std::shared_ptr<PuzzleState>& a, const std::shared_ptr<PuzzleState>& b) const {
            return a->f() > b->f();
        }
    };

    using StatePtr = std::shared_ptr<PuzzleState>;
    using OpenSet = std::priority_queue<StatePtr, std::vector<StatePtr>, Compare>;
    
    struct BoardHash {
        size_t operator()(const std::vector<uint8_t>& board) const {
            size_t h = 0;
            // Java style hash function
            // for (size_t i = 0; i < board.size(); ++i) {
            //     h = h * 31 + board[i];
            // }

            // More universal hash function
            for (uint8_t val : board) {
                h ^= std::hash<uint8_t>{}(val) + 0x9e3779b9 + (h << 6) + (h >> 2);
            }
            return h;
        }
    };

    using ClosedSet = std::unordered_map<std::vector<uint8_t>, StatePtr, BoardHash>;

    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> found{false};
    StatePtr meet_point_f;
    StatePtr meet_point_b;

    // Pre-computed moves
    std::vector<std::vector<int>> move_table;

    void generateGoal() {
        goal.resize(n * n);
        for (int i = 0; i < n * n - 1; ++i) {
            goal[i] = i + 1;
        }
        goal[n * n - 1] = 0; // empty space
        log("Generated goal permutation");
    }

    // Precompute all possible moves for each position
    void precomputeMoves() {
        move_table.resize(n * n);
        const int dx[4] = {-1, 1, 0, 0};
        const int dy[4] = {0, 0, -1, 1};

        for (int pos = 0; pos < n * n; ++pos) {
            int x = pos / n;
            int y = pos % n;
            
            for (int d = 0; d < 4; ++d) {
                int nx = x + dx[d];
                int ny = y + dy[d];
                if (nx >= 0 && nx < n && ny >= 0 && ny < n) {
                    move_table[pos].push_back(nx * n + ny);
                }
            }
        }
        log("Precomputed moves for each position");
    }

    // Thread-safe logging function
    void log(const std::string& message, bool important = false) {
        std::lock_guard<std::mutex> lock(log_mutex);
        auto now = std::chrono::system_clock::now();
        auto now_time = std::chrono::system_clock::to_time_t(now);
        
        std::stringstream timestamp;
        timestamp << std::put_time(std::localtime(&now_time), "%H:%M:%S");
        
        if (important) {
            std::cout << "\033[1;32m" << timestamp.str() << " " << message << "\033[0m" << std::endl;
        } else {
            std::cout << timestamp.str() << " " << message << std::endl;
        }
    }

/**************************************************************************************************/    

public:
    PuzzleSolver(int size) : n(size) {
        generateGoal();
        precomputeMoves();
    }

    int calculateHeuristic(const std::vector<uint8_t>& board) const {
        int manhattan = 0;
        int linear_conflict = 0;

        for (int i = 0; i < n * n; ++i) {
            uint8_t val = board[i];
            if (val == 0) continue;
            
            int goal_pos = val - 1;  // Goal position for this tile
            int goal_x = goal_pos / n;
            int goal_y = goal_pos % n;
            int cur_x = i / n;
            int cur_y = i % n;
            
            manhattan += std::abs(cur_x - goal_x) + std::abs(cur_y - goal_y);
            
            // tiles in same row/column but in reverse order
            if (cur_x == goal_x) {
                for (int j = 0; j < n * n; ++j) {
                    if (j / n != cur_x) continue;
                    
                    uint8_t val_j = board[j];
                    if (val_j == 0) continue;
                    
                    int goal_j = val_j - 1;
                    if (goal_j / n != cur_x) continue;
                    
                    // going opposite directions
                    if (i < j && val > val_j && (val-1) / n == (val_j-1) / n) {
                        linear_conflict += 2;
                    }
                }
            }
            
            if (cur_y == goal_y) {
                for (int j = 0; j < n * n; ++j) {
                    if (j % n != cur_y) continue;
                    
                    uint8_t val_j = board[j];
                    if (val_j == 0) continue;
                    
                    int goal_j = val_j - 1;
                    if (goal_j % n != cur_y) continue; 
                    
                    // going opposite directions
                    if (i < j && val > val_j && (val-1) % n == (val_j-1) % n) {
                        linear_conflict += 2;
                    }
                }
            }
        }
        
        return manhattan + linear_conflict;
    }

    std::vector<std::vector<uint8_t>> getNeighbors(const std::vector<uint8_t>& board) const {
        std::vector<std::vector<uint8_t>> neighbors;
        
        // Find empty space
        auto it = std::find(board.begin(), board.end(), 0);
        int empty_pos = std::distance(board.begin(), it);
        
        for (int next_pos : move_table[empty_pos]) {
            auto new_board = board;
            std::swap(new_board[empty_pos], new_board[next_pos]);
            neighbors.push_back(std::move(new_board));
        }
        
        return neighbors;
    }

    void searchDirection(OpenSet& open_set, ClosedSet& visited_this, ClosedSet& visited_other, StatePtr& meet_point, bool forward) {
        const std::string direction = forward ? "FORWARD" : "BACKWARD";
        log("[" + direction + "] Search thread started");
        

        while (!open_set.empty() && !found.load(std::memory_order_relaxed)) {
            StatePtr current;

            {
                std::unique_lock<std::mutex> lock(mtx);

                if (open_set.empty()) {
                    log("[" + direction + "] Open set is empty, terminating search", true);
                    break;
                }
                
                current = open_set.top();
                open_set.pop();
                
                // Skip if we've already processed this state with a better path
                auto existing = visited_this.find(current->board);
                if (existing != visited_this.end() && existing->second->g < current->g) {
                    continue;
                }
            }

            auto neighbors = getNeighbors(current->board);
            for (auto& neighbor_board : neighbors) {
                std::unique_lock<std::mutex> lock(mtx);
                
                // Skip if we've already visited this state with a better path
                auto existing = visited_this.find(neighbor_board);
                if (existing != visited_this.end() && existing->second->g <= current->g + 1) {
                    continue;
                }
                
                int g_new = current->g + 1;
                int h_new = calculateHeuristic(neighbor_board);
                auto neighbor = std::make_shared<PuzzleState>(neighbor_board, g_new, h_new, current);
                
                // Update or add to visited set
                visited_this[neighbor_board] = neighbor;
                open_set.push(neighbor);
                
                // Check if we've found a meeting point
                auto match = visited_other.find(neighbor_board);
                if (match != visited_other.end()) {
                    meet_point = neighbor;
                    found.store(true, std::memory_order_relaxed);

                    log("[" + direction + "] SOLUTION FOUND at depth " + 
                        std::to_string(g_new + match->second->g) + "!", true);

                    cv.notify_all();
                    return;
                }
            }
        }
    }

    std::vector<std::vector<uint8_t>> solve(const std::vector<uint8_t>& start) {
        
        std::vector<uint8_t> start_board(start.begin(), start.end());
        
        // Reset state
        found.store(false, std::memory_order_relaxed);
        meet_point_f = nullptr;
        meet_point_b = nullptr;
        
        OpenSet open_f, open_b;
        ClosedSet visited_f, visited_b;
        
        auto start_node = std::make_shared<PuzzleState>(start_board, 0, calculateHeuristic(start_board));
        auto goal_node = std::make_shared<PuzzleState>(goal, 0, calculateHeuristic(goal));
        
        open_f.push(start_node);
        open_b.push(goal_node);
        visited_f[start_board] = start_node;
        visited_b[goal] = goal_node;
        
        std::thread t1(&PuzzleSolver::searchDirection, this, 
                      std::ref(open_f), std::ref(visited_f), 
                      std::ref(visited_b), 
                      std::ref(meet_point_f),
                      true);
        
        std::thread t2(&PuzzleSolver::searchDirection, this, 
                      std::ref(open_b), std::ref(visited_b), 
                      std::ref(visited_f), 
                      std::ref(meet_point_b),
                      false);
        
        // Wait for solution to be found
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [&] { return found.load(std::memory_order_relaxed) || 
                                        (open_f.empty() && open_b.empty()); });
        }
        
        t1.join();
        t2.join();
        
        // Reconstruct path
        return reconstructPath(visited_f, visited_b);
    }

    std::vector<std::vector<uint8_t>> reconstructPath(const ClosedSet& visited_f, const ClosedSet& visited_b) {
        std::vector<std::vector<uint8_t>> path;
        
        if (meet_point_f && visited_b.find(meet_point_f->board) != visited_b.end()) {
            // Collect forward path
            std::vector<std::vector<uint8_t>> forward_path;
            for (auto p = meet_point_f; p; p = p->parent) {
                forward_path.push_back(p->board);
            }
            std::reverse(forward_path.begin(), forward_path.end());
            path = forward_path;
            
            // Collect backward path
            auto match = visited_b.at(meet_point_f->board);
            for (auto p = match->parent; p; p = p->parent) {
                path.push_back(p->board);
            }
        }
        else if (meet_point_b && visited_f.find(meet_point_b->board) != visited_f.end()) {
            // Collect forward path
            std::vector<std::vector<uint8_t>> forward_path;
            auto match = visited_f.at(meet_point_b->board);
            for (auto p = match; p; p = p->parent) {
                forward_path.push_back(p->board);
            }
            std::reverse(forward_path.begin(), forward_path.end());
            path = forward_path;
            
            // Collect backward path
            for (auto p = meet_point_b->parent; p; p = p->parent) {
                path.push_back(p->board);
            }
        }
        
        return path;
    }

    void printBoard(const std::vector<uint8_t>& board) const {
        for (int i = 0; i < n * n; ++i) {
            if (i % n == 0) std::cout << std::endl;
            std::cout << (board[i] ? std::to_string(board[i]) : " ") << "\t";
        }
        std::cout << std::endl;
    }
};

int main() {

    std::vector<uint8_t> puzzle = {
        7, 1, 3, 6, 2, 4, 0, 5, 8
    };

    int n = static_cast<int>(sqrt(puzzle.size()));
    PuzzleSolver solver(n);
    
    auto path = solver.solve(puzzle);
    
    if (path.empty()) {
        std::cout << "No solution found!" << std::endl;
        return 1;
    }
    
    std::cout << "Steps to solve: " << path.size() - 1 << std::endl;

    for (const auto& board : path) {
        solver.printBoard(board);
        std::cout << "-----------------\n";
    }
    
    return 0;
}
