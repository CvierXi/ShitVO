#include <iostream>

#include "pc/simulate/simulator.h"

using namespace std;

int main(int argc, char** argv) {
    if (argc < 3) {
        cerr << "Usage: ./bin/demo_homo_vo ../config/default.yaml /your/dataset/path" << endl;
        return -1;
    }

    string config_file_path = argv[1];
    string dataset_path = argv[2];

    shared_ptr<Simulator> simulator = make_shared<Simulator>(config_file_path, dataset_path);
    simulator->runOdometry();

    cout << "Done!" << endl;

    return 0;
}
