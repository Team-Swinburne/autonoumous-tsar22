#pragma once
#include <iostream> // cout, cerr
#include <fstream> // ifstream
#include <sstream> // stringstream
#include <iomanip>
#include <vector>
#include <algorithm>
#include "Position.cpp"
using namespace std;

template<typename T> void printElement(T t, const int& width)
{
    const char separator    = ' ';
    cout << left << setw(width) << setfill(separator) << t;
}

class Yaml {
public:
    string location;
    string mode;
    float res;
    vector<float> pos;
    bool negate;
    float occ_thresh;
    float free_thresh;

    Yaml(string _filename) {
        ifstream infile(_filename);                         // file path
        if (infile.fail()) {
            cout << "Unable to open file!" << endl;
            cout << _filename << endl;
        }
        stringstream ss;
        string inputLine = "";

        while (!infile.eof()) {
            string data;
            getline(infile, data);
            read(data);
        }

        infile.close();
    }

    // read and assing data
    void read(string data) {
        std::string::iterator end_pos = std::remove(data.begin(), data.end(), ' ');
        data.erase(end_pos, data.end());
        vector<string> info;
        info = explode(":", data);
        if (info.at(0) == "image") {
            this->location = info.at(1);
        }
        else if (info.at(0) == "mode") {
            this->mode = info.at(1);
        }
        else if (info.at(0) == "resolution") {
            this->res = stof(info.at(1));
        }
        else if (info.at(0) == "origin") {
            string pos_xy = info.at(1);
            pos_xy = pos_xy.substr(1, pos.size() - 2);
            vector<string> pos_exp = explode(",", pos_xy);
            for (int i = 0; i < 3; i++) {
                this->pos.push_back(stof(pos_exp.at(i)));
            }
        }
        else if (info.at(0) == "negate") {
            if (info.at(1) == "0") negate = false;
            else if (info.at(1) == "1") negate = true;
        }
        else if (info.at(0) == "occupied_thresh") {
            occ_thresh = stof(info.at(1));
        }
        else if (info.at(0) == "free_thresh") {
            free_thresh = stof(info.at(1));
        }

    }

    void display() {
        cout << "Image Location: " << this->location << endl;
        cout << "Mode: " << this->mode << endl;
        cout << "Resolution: " << to_string(this->res) << endl;
        cout << "Origin: " << this->pos.at(0) << ", " << this->pos.at(1) << ", " << this->pos.at(2) << endl;
        cout << "Negate: ";
        if (this->negate) {
            cout << "1" << endl;
        }
        else {
            cout << "0" << endl;
        }
        cout << "Occupied Thresh: " << to_string(this->occ_thresh) << endl;
        cout << "Free Thresh: " << to_string(this->free_thresh) << endl;
    }

    // Split string by using delimiter
    vector<string> explode(const string& delimiter, const string& str) {
        vector<string> arr;

        int strleng = str.length();
        int delleng = delimiter.length();
        if (delleng == 0)
            return arr;//no change

        int i = 0;
        int k = 0;
        while (i < strleng)
        {
            int j = 0;
            while (i + j < strleng && j < delleng && str[i + j] == delimiter[j])
                j++;
            if (j == delleng)//found delimiter
            {
                arr.push_back(str.substr(k, i - k));
                i += delleng;
                k = i;
            }
            else
            {
                i++;
            }
        }
        arr.push_back(str.substr(k, i - k));
        return arr;
    }

};

class RealMap {
public:
    int rows, cols, size, greylevels;
    Position pos;
    string filetype;
    vector<vector<int>> matrix;
    Yaml data;  // now yaml file is int map class so it can access the file data

    RealMap();

    RealMap(string _filename) : data(_filename) {
        //data = Yaml(_filename);
        read(data.location);
    }
    void setpos(Position apos) {
        pos = apos;
        matrix[pos.y][pos.x] = 2;
    }

    void read(string filename) {
        // open stream in binary mode
        ifstream istr(filename, ios::in | ios::binary);
        if (istr.fail()) cout << "Unable to open file name: " << filename << endl;

        // parse header
        istr >> filetype >> cols >> rows >> greylevels;
        size = rows * cols;

        // check data
        cout << "filetype: " << filetype << endl;
        cout << "rows: " << rows << endl;
        cout << "cols: " << cols << endl;
        cout << "greylevels: " << greylevels << endl;
        cout << "size: " << size << endl;

        int fail_tracker = 0; // find which pixel failing on


        matrix = vector<vector<int>>(rows, vector<int>(cols));
        // every p5 file that I've read always get a useless pixel value 10 which I can't find in the ASCII version of the pgm file
        // so I assign that 10 to a useless variable to skip it
        unsigned char garbage = istr.get();

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                // read in binary char
                unsigned char t_ch = istr.get();
                // convert to integer
                int t_data = static_cast<int>(t_ch);
                // check if legal pixel
                if (t_data < 0 || t_data > greylevels) {
                    cout << "Failed on pixel: " << fail_tracker << endl;
                    cout << "Pixel value: " << t_data << endl;
                }
                // if passes add value to data array
                
                matrix[i][j] = convert(t_data);
                fail_tracker++;
            }
        }

        // close the stream
        istr.close();

    }
void ChangeMapWithPos(Position pos) {
    matrix[pos.y][pos.x] = 3;
}

void ChangeMap(int x, int y, int v) {
    int position[2];
    position[0] = x;
    position[1] = y;

    matrix[position[1]][position[0]] = v;
    }

    void print_map() {
        // write header
        // cout << "P2 " << "\n" << rows << " " << cols << "\n" << greylevels << endl;
        // write data
        string p = "";

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (matrix[i][j] == 1){
                    p = "*";
                }else if (matrix[i][j] == 0){
                    p = " ";
                } else if (matrix[i][j] == 2){
                    p = "@";
                } else if (matrix[i][j] == 3){
                    p = "%";
                } else if (matrix[i][j] == 4){
                    p = "o";
                } else if (matrix[i][j] == 5){
                    p = "+";
                }
                printElement(p, 1);
            }
            cout << endl;
        }
    }

    int convert(int v){
        float occ_thresh = data.occ_thresh;
        float free_thresh = data.free_thresh;

        if (((255 - v)/255.0) < free_thresh){
            return 0;
        }else{

            return 1;
        }
    }
};
/*
int main() {
    //Yaml yaml_file("test_map_2.yaml");
    RealMap pgm_map("5_laps_RTrack.yaml");

    //yaml_file.display();
    pgm_map.print_map();
} */

