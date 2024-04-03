#include <queue>
#include <set>
#include <vector>
#include <string>
#include "floodfill.hpp"

void printmaze(int* matrix) {
    for (int i = 0; i < 33*33; i++) {
        if ((i + 1) % 33 == 0) {
            printf("%d\n", matrix[i]);
        } else {
            printf("%d ", matrix[i]);
        }
    }
}

void Micromouse::floodfill(unsigned short goal_x, unsigned short goal_y) {
    std::queue<std::pair<unsigned short, unsigned short>> frontier;
    std::set<std::pair<unsigned short, unsigned short>> explored;
    
    mapped_matrix[(goal_y)*33 + goal_x] = 0;
    std::pair p { goal_x, goal_y };
    frontier.push(p);

    while (!frontier.empty()) {
        std::pair front = frontier.front(); frontier.pop();

        if (explored.find(front) != explored.end()) {
            continue;
        }

        std::pair<unsigned short, unsigned short> to_queue;
        unsigned short curr_ind = (front.second)*33 + front.first;

        if (front.second > 0 && mapped_matrix[curr_ind - 33] != -1) {
            to_queue.first = front.first;
            to_queue.second = front.second - 1;

            if (explored.find(to_queue) == explored.end()) {
                mapped_matrix[curr_ind - 33] = 1 + mapped_matrix[curr_ind];
                frontier.push(to_queue);
            }
        }

        if (front.first < 32 && mapped_matrix[curr_ind + 1] != -1) {
            to_queue.first = front.first + 1;
            to_queue.second = front.second;

            if (explored.find(to_queue) == explored.end()) {
                mapped_matrix[curr_ind + 1] = 1 + mapped_matrix[curr_ind];
                frontier.push(to_queue);
            }
        }

        if (front.second < 32 && mapped_matrix[curr_ind + 33] != -1) {
            to_queue.first = front.first;
            to_queue.second = front.second + 1;

            if (explored.find(to_queue) == explored.end()) {
                mapped_matrix[curr_ind + 33] = 1 + mapped_matrix[curr_ind];
                frontier.push(to_queue);
            }
        }

        if (front.first > 0 && mapped_matrix[curr_ind - 1] != -1) {
            to_queue.first = front.first - 1;
            to_queue.second = front.second;

            if (explored.find(to_queue) == explored.end()) {
                mapped_matrix[curr_ind - 1] = 1 + mapped_matrix[curr_ind];
                frontier.push(to_queue);
            }
        }

        explored.insert(front);
        // std::cout << "Had just explored " << front.first << " " << front.second << "\n";
        // Maze::printmaze(mapped_matrix, 16);
        // printset(explored);
        // int dummy;
        // std::cin >> dummy;
    }
}

Micromouse::Micromouse() {
    mapped_matrix = (int*) calloc(33*33, sizeof(int));
    x_dir = 0;
    y_dir = 1;
    floodfill(15, 15);
    curr_x = 1;
    curr_y = 1;
}

bool Micromouse::go_forward(double distanceF, double distanceL, double distanceR, unsigned short goal_x, unsigned short goal_y) {
    // physically set mouse moving

    if (distanceL < 20) {
        if (this->x_dir == 0 && this->y_dir == -1) {
            this->mapped_matrix[(this->curr_y)*(2*16 + 1) + this->curr_x - 1] = -1;
        } else if (this->x_dir == 1 && this->y_dir == 0) {
            this->mapped_matrix[(this->curr_y - 1)*(2*16 + 1) + this->curr_x] = -1;
        } else if (this->x_dir == 0 && this->y_dir == 1) {
            this->mapped_matrix[(this->curr_y)*(2*16 + 1) + this->curr_x + 1] = -1;
        } else {
            this->mapped_matrix[(this->curr_y + 1)*(2*16 + 1) + this->curr_x] = -1;
        }
    }

    if (distanceR < 20) {
        if (this->x_dir == 0 && this->y_dir == -1) {
            this->mapped_matrix[(this->curr_y)*(2*16 + 1) + this->curr_x + 1] = -1;
        } else if (this->x_dir == 1 && this->y_dir == 0) {
            this->mapped_matrix[(this->curr_y + 1)*(2*16 + 1) + this->curr_x] = -1;
        } else if (this->x_dir == 0 && this->y_dir == 1) {
            this->mapped_matrix[(this->curr_y)*(2*16 + 1) + this->curr_x - 1] = -1;
        } else {
            this->mapped_matrix[(this->curr_y - 1)*(2*16 + 1) + this->curr_x] = -1;
        }
    }

    if (distanceF < 20) {
        this->mapped_matrix[(this->curr_y + this->y_dir)*(2*16 + 1) + this->curr_x + this->x_dir] = -1;
        this->floodfill(goal_x, goal_y);
        return false;
    }

    this->curr_x += this->x_dir;
    this->curr_y += this->y_dir;
    return true;
}

void Micromouse::set_dir(char x_dir, char y_dir) {
    this->x_dir = x_dir;
    this->y_dir = y_dir;
}

void Micromouse::turnRight() {
    // physically turn right

    char temp = this->x_dir;
    this->x_dir = -(this->y_dir);
    this->y_dir = temp;
}

void Micromouse::turnLeft() {
    // physically turn left

    char temp = this->x_dir;
    this->x_dir = this->y_dir;
    this->y_dir = -temp;
}

void Micromouse::turnAround() {
    // physically turn around

    this->x_dir *= -1;
    this->y_dir *= -1;
}

std::vector<std::pair<int, int>> Micromouse::navigate_maze(double distanceF, double distanceL, double distanceR, bool to_goal) {
    std::vector<std::pair<int, int>> res;
    unsigned short goal_x = to_goal ? 15 : 1;
    unsigned short goal_y = to_goal ? 15 : 1;

    while (curr_x != goal_x || curr_y != goal_y) {
        bool forward_res = this->go_forward(distanceF, distanceL, distanceR, goal_x, goal_y);

        if (forward_res) {
            res.push_back(std::pair { curr_x, curr_y });
        } else {
            // physically stop mouse
        }

        int up_val, right_val, left_val, down_val;
        up_val = this->mapped_matrix[(this->curr_y - 1)*(2*16 + 1) + this->curr_x];
        right_val = this->mapped_matrix[(this->curr_y)*(2*16 + 1) + this->curr_x + 1];
        left_val = this->mapped_matrix[(this->curr_y)*(2*16 + 1) + this->curr_x - 1];
        down_val = this->mapped_matrix[(this->curr_y + 1)*(2*16 + 1) + this->curr_x];
        
        int min = up_val;

        if (this->x_dir == 0 && this->y_dir == -1) {
        } else if (this->x_dir == 1 && this->y_dir == 0) {
            this->turnLeft();
        } else if (this->x_dir == 0 && this->y_dir == 1) {
            this->turnAround();
        } else {
            this->turnRight();
        }

        if (min == -1 || (right_val < min && right_val != -1)) {
            min = right_val;
            
            if (this->x_dir == 0 && this->y_dir == -1) {
                this->turnRight();
            } else if (this->x_dir == 1 && this->y_dir == 0) {
            } else if (this->x_dir == 0 && this->y_dir == 1) {
                this->turnLeft();
            } else {
                this->turnAround();
            }
        }

        if (min == -1 || (left_val < min && left_val != -1)) {
            min = left_val;
            
            if (this->x_dir == 0 && this->y_dir == -1) {
                this->turnLeft();
            } else if (this->x_dir == 1 && this->y_dir == 0) {
                this->turnAround();
            } else if (this->x_dir == 0 && this->y_dir == 1) {
                this->turnRight();
            } else {
            }
        }

        if (min == -1 || (down_val < min && down_val != -1)) {
            min = down_val;
            
            if (this->x_dir == 0 && this->y_dir == -1) {
                this->turnAround();
            } else if (this->x_dir == 1 && this->y_dir == 0) {
                this->turnRight();
            } else if (this->x_dir == 0 && this->y_dir == 1) {
            } else {
                this->turnLeft();
            }
        }

        // Maze::printmaze(mapped_matrix);
        // std::cout << "Currently at (" << curr_x << ", " << curr_y << ") and facing (" << (int) x_dir << ", " << (int) y_dir << ")\n";
        // int dummy;
        // std::cin >> dummy;
    }

    return res;
}

int main(int argc, char** argv) {
    Micromouse mouse {};

    std::vector<std::pair<int, int>> path1 = mouse.navigate_maze(maze, true);

    mouse.floodfill(1, 1);
    std::vector<std::pair<int, int>> path2 = mouse.navigate_maze(maze, false);

    // std::cout << "Path1 length: " << path1.size() << " and path2 length: " << path2.size() << "\n";

    // std::cout << "[";
    // for (auto e: path1) {
    //     std::cout << "(" << e.first << ", " << e.second << "), ";
    // }
    // std::cout << "]\n";

    // std::cout << "[";
    // for (auto e: path2) {
    //     std::cout << "(" << e.first << ", " << e.second << "), ";
    // }
    // std::cout << "]\n";

    return 0;
}
