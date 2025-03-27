#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

int main() {
    std::ifstream inputFile("run1_load1.csv");
    std::ofstream outputFile("output.csv");

    if (!inputFile.is_open() || !outputFile.is_open()) {
        std::cerr << "Error opening files.\n";
        return 1;
    }

    std::string line;
    bool isHeader = true;

    while (std::getline(inputFile, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;

        // Parse the CSV line
        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }

        // If it's the header row, write it directly
        if (isHeader) {
            isHeader = false;
            outputFile << line << "\n";
            continue;
        }

        // Modify the last column (ESC Input Current)
        if (row.size() == 9) {
            double escCurrent = std::stod(row[8]);
            escCurrent *= 1.0;  // Apply your formula here
            row[8] = std::to_string(escCurrent);
        }

        // Write to output CSV
        for (size_t i = 0; i < row.size(); ++i) {
            outputFile << row[i];
            std::cout<<row[i]<<"\t";
            if (i < row.size() - 1){outputFile << ",";}
                
                // std::cout<<",\t";
        }
        std::cout<<"\n";
        outputFile << "\n";
    }

    inputFile.close();
    outputFile.close();

    std::cout << "CSV processing complete.\n";
    return 0;
}
