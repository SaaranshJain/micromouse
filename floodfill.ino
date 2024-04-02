#include <iostream>
#include <queue>
#include <set>
#include <vector>
#include <string>
#include "floodfill.hpp"

Maze::Maze(char filename[50]) {
    FILE* fp = fopen(filename, "r");

    matrix = (int*) calloc(33*33, sizeof(int));
    int i = 0;

    fscanf(fp, "%d", &(matrix[i++]));

    while(!feof(fp)) {
        fscanf(fp, "%d", &(matrix[i++]));
    }

    fclose(fp);
}

void Maze::printmaze(int* matrix) {
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
}

void Micromouse::place_mouse(unsigned short x, unsigned short y) {
    curr_x = x;
    curr_y = y;
}

bool Micromouse::go_forward(Maze maze, unsigned short goal_x, unsigned short goal_y) {
    unsigned short to_see_ind = (this->curr_y + this->y_dir)*(2*16 + 1) + this->curr_x + this->x_dir;

    if (maze.matrix[to_see_ind] == -1) {
        this->mapped_matrix[to_see_ind] = -1;
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

std::vector<std::pair<int, int>> Micromouse::navigate_maze(Maze maze, bool to_goal) {
    std::vector<std::pair<int, int>> res;
    unsigned short goal_x = to_goal ? 15 : 1;
    unsigned short goal_y = to_goal ? 15 : 1;

    while (curr_x != goal_x || curr_y != goal_y) {
        bool forward_res = this->go_forward(maze, goal_x, goal_y);

        if (forward_res) {
            res.push_back(std::pair { curr_x, curr_y });
        }

        int up_val, right_val, left_val, down_val;
        up_val = this->mapped_matrix[(this->curr_y - 1)*(2*16 + 1) + this->curr_x];
        right_val = this->mapped_matrix[(this->curr_y)*(2*16 + 1) + this->curr_x + 1];
        left_val = this->mapped_matrix[(this->curr_y)*(2*16 + 1) + this->curr_x - 1];
        down_val = this->mapped_matrix[(this->curr_y + 1)*(2*16 + 1) + this->curr_x];
        
        int min = up_val;
        this->set_dir(0, -1);

        if (min == -1 || (right_val < min && right_val != -1)) {
            min = right_val;
            this->set_dir(1, 0);
        }

        if (min == -1 || (left_val < min && left_val != -1)) {
            min = left_val;
            this->set_dir(-1, 0);
        }

        if (min == -1 || (down_val < min && down_val != -1)) {
            min = down_val;
            this->set_dir(0, 1);
        }

        // Maze::printmaze(mapped_matrix);
        // std::cout << "Currently at (" << curr_x << ", " << curr_y << ") and facing (" << (int) x_dir << ", " << (int) y_dir << ")\n";
        // int dummy;
        // std::cin >> dummy;
    }

    return res;
}

int main(int argc, char** argv) {
    Maze maze { argv[1] };
    Micromouse mouse {};

    mouse.place_mouse(1, 1);
    std::vector<std::pair<int, int>> path1 = mouse.navigate_maze(maze, true);

    mouse.floodfill(1, 1);
    std::vector<std::pair<int, int>> path2 = mouse.navigate_maze(maze, false);

    std::cout << "Path1 length: " << path1.size() << " and path2 length: " << path2.size() << "\n";

    // std::cout << "[";
    // for (auto e: path1) {
    //     std::cout << "(" << e.first << ", " << e.second << "), ";
    // }
    // std::cout << "]\n";

    std::cout << "[";
    for (auto e: path2) {
        std::cout << "(" << e.first << ", " << e.second << "), ";
    }
    std::cout << "]\n";

    return 0;
}
